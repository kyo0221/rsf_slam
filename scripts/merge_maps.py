#!/usr/bin/env python3
import argparse
import math
import os

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt

UNKNOWN, FREE = 205, 254


def load(path):
    meta = yaml.safe_load(open(path))
    image = np.array(Image.open(os.path.join(os.path.dirname(path), meta['image'])))
    return meta, image


def extent(meta, image):
    ox, oy = meta['origin'][:2]
    h, w = image.shape
    return ox, oy, ox + w * meta['resolution'], oy + h * meta['resolution']


def place(origin, shape, res, meta, image):
    out = np.full(shape, UNKNOWN, np.uint8)
    rows, cols = np.nonzero(image != UNKNOWN)
    x = meta['origin'][0] + (cols + 0.5) * meta['resolution']
    y = meta['origin'][1] + (image.shape[0] - 1 - rows + 0.5) * meta['resolution']
    c = np.clip(((x - origin[0]) / res).astype(int), 0, shape[1] - 1)
    r = np.clip(shape[0] - 1 - ((y - origin[1]) / res).astype(int), 0, shape[0] - 1)
    out[r, c] = image[rows, cols]
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('primary')
    parser.add_argument('secondary')
    parser.add_argument('output')
    parser.add_argument('--margin', type=float, default=6.0)
    args = parser.parse_args()
    pm, pi = load(args.primary)
    sm, si = load(args.secondary)
    res = pm['resolution']
    ex = [extent(pm, pi), extent(sm, si)]
    min_x, min_y = min(e[0] for e in ex), min(e[1] for e in ex)
    max_x, max_y = max(e[2] for e in ex), max(e[3] for e in ex)
    origin = (pm['origin'][0] - math.ceil((pm['origin'][0] - min_x) / res - 1e-9) * res,
              pm['origin'][1] - math.ceil((pm['origin'][1] - min_y) / res - 1e-9) * res)
    shape = (math.ceil((max_y - origin[1]) / res - 1e-9), math.ceil((max_x - origin[0]) / res - 1e-9))
    primary = place(origin, shape, res, pm, pi)
    secondary = place(origin, shape, res, sm, si)
    known = primary != UNKNOWN
    near = distance_transform_edt(~known) * res <= args.margin
    out = secondary.copy()
    out[near & (secondary < 128)] = UNKNOWN
    out[known] = primary[known]
    Image.fromarray(out).save(args.output + '.pgm')
    yaml.safe_dump({
        'image': os.path.basename(args.output) + '.pgm', 'mode': 'trinary', 'resolution': res,
        'origin': [round(origin[0], 3), round(origin[1], 3), 0.0], 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.196}, open(args.output + '.yaml', 'w'), sort_keys=False)
    print(f'{shape[1]}x{shape[0]} origin {origin} primary {known.sum()} secondary {(secondary != UNKNOWN).sum()} '
          f'suppressed {(near & (secondary < 128) & ~known).sum()}')


main()
