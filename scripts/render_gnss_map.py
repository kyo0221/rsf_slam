#!/usr/bin/env python3
import argparse
import math
import os
import sys

import numpy as np
import rosbag2_py
import yaml
from PIL import Image
from rclpy.serialization import deserialize_message
from scipy import ndimage
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ideal_switch_odom import load_trajectory  # noqa: E402

UNKNOWN, FREE, OCCUPIED = 205, 254, 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument('georef')
    parser.add_argument('output')
    parser.add_argument('--window', type=float, default=10.0)
    parser.add_argument('--min-move', type=float, default=3.0)
    parser.add_argument('--step', type=int, default=2)
    parser.add_argument('--resolution', type=float, default=0.2)
    parser.add_argument('--max-range', type=float, default=25.0)
    parser.add_argument('--height', type=float, nargs=2, default=(-0.2, 1.6))
    parser.add_argument('--margin', type=float, default=30.0)
    parser.add_argument('--min-hits', type=int, default=2)
    parser.add_argument('--min-hit-ratio', type=float, default=0.15)
    parser.add_argument('--min-passes', type=int, default=4)
    parser.add_argument('--min-component', type=int, default=6)
    args = parser.parse_args()
    res = args.resolution
    poses = load_trajectory(args.bag, yaml.safe_load(open(args.georef)), args.window, args.min_move)
    xy = np.array([(p[0], p[1]) for p in poses.values() if np.isfinite(p[0])])
    origin = (math.floor((xy[:, 0].min() - args.margin) / res) * res, math.floor((xy[:, 1].min() - args.margin) / res) * res)
    shape = (math.ceil((xy[:, 1].max() + args.margin - origin[1]) / res), math.ceil((xy[:, 0].max() + args.margin - origin[0]) / res))
    hits = np.zeros(shape, np.int32)
    miss = np.zeros(shape, np.int32)
    H, W = shape

    def cells(x, y):
        c = ((x - origin[0]) / res).astype(np.int64)
        r = H - 1 - ((y - origin[1]) / res).astype(np.int64)
        ok = (c >= 0) & (c < W) & (r >= 0) & (r < H)
        return r[ok] * W + c[ok]

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/rsf/hokuyo_cloud2']))
    n = used = 0
    while reader.has_next():
        _, data, _ = reader.read_next()
        n += 1
        if n % args.step:
            continue
        msg = deserialize_message(data, PointCloud2)
        pose = poses.get(msg.header.stamp.sec * 10 ** 9 + msg.header.stamp.nanosec)
        if pose is None or not np.isfinite(pose[0]):
            continue
        p = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        r = np.hypot(p[:, 0], p[:, 1])
        keep = r > 0.7
        p, r = p[keep], r[keep]
        occ = (p[:, 2] > args.height[0]) & (p[:, 2] < args.height[1]) & (r < args.max_range)
        scale = np.minimum(1.0, args.max_range / r)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        dx = (c * p[:, 0] - s * p[:, 1]) * scale
        dy = (s * p[:, 0] + c * p[:, 1]) * scale
        rr = r * scale
        nseg = np.maximum(0, (rr - 0.15) / (res / 2)).astype(np.int64)
        total = int(nseg.sum())
        if total:
            ray = np.repeat(np.arange(len(rr)), nseg)
            k = np.arange(total) - np.repeat(np.cumsum(nseg) - nseg, nseg)
            f = (k + 0.5) * (res / 2) / rr[ray]
            miss.ravel()[np.unique(cells(pose[0] + dx[ray] * f, pose[1] + dy[ray] * f))] += 1
        if occ.any():
            hits.ravel()[np.unique(cells(pose[0] + dx[occ], pose[1] + dy[occ]))] += 1
        used += 1

    total = hits + miss
    ratio = hits / np.maximum(total, 1)
    k3 = np.ones((3, 3), bool)
    occupied = (hits >= args.min_hits) & (ratio >= args.min_hit_ratio)
    occupied = ndimage.binary_closing(occupied, structure=k3) | occupied
    label, num = ndimage.label(occupied, structure=k3)
    sizes = ndimage.sum(occupied, label, range(1, num + 1))
    small = np.isin(label, np.flatnonzero(sizes < args.min_component) + 1)
    occupied &= ~small
    free = (total >= args.min_passes) & (ratio < 0.1) & ~occupied
    free |= small & (miss >= 2)
    opened = ndimage.binary_opening(free, structure=k3)
    free = opened | (free & ndimage.binary_dilation(opened, structure=k3))
    free = ndimage.binary_fill_holes(free) & ~occupied | free
    surface = occupied & ndimage.binary_dilation(free, structure=k3)
    free &= ~surface
    img = np.full(shape, UNKNOWN, np.uint8)
    img[free] = FREE
    img[surface] = OCCUPIED
    Image.fromarray(img).save(args.output + '.pgm')
    yaml.safe_dump({
        'image': os.path.basename(args.output) + '.pgm', 'mode': 'trinary', 'resolution': res,
        'origin': [round(origin[0], 3), round(origin[1], 3), 0.0], 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.196}, open(args.output + '.yaml', 'w'), sort_keys=False)
    print(f'{used} scans, {W}x{H} cells, occupied {int(surface.sum())}, free {free.sum() * res * res:.0f} m2')


if __name__ == '__main__':
    main()
