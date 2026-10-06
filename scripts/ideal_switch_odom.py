#!/usr/bin/env python3
import math

import numpy as np
import rclpy
import rclpy.executors
import rosbag2_py
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nmea_msgs.msg import Gpgga
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rsf_bag_visualizer.analysis import enu_from_latlon, rigid_align, simulate_switch
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster

LIO = '/rsf/lio_lidar_rate_odom'


def stamp_ns(stamp):
    return stamp.sec * 10 ** 9 + stamp.nanosec


def load_trajectory(bag, georef, win, min_move=8.0):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[LIO, '/rsf/nav_sat_fix', '/rsf/gpgga']))
    lio, nav, gga, stamps = [], [], [], []
    t0 = None
    while reader.has_next():
        topic, data, ts = reader.read_next()
        t0 = ts if t0 is None else t0
        t = (ts - t0) / 1e9
        if topic == LIO:
            m = deserialize_message(data, Odometry)
            q = m.pose.pose.orientation
            n = math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
            x, y, z, w = q.x / n, q.y / n, q.z / n, q.w / n
            lio.append((t, m.pose.pose.position.x, m.pose.pose.position.y,
                        math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
                        math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
                        math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))))
            stamps.append(stamp_ns(m.header.stamp))
        elif topic == '/rsf/nav_sat_fix':
            m = deserialize_message(data, NavSatFix)
            nav.append((t, m.latitude, m.longitude))
        else:
            gga.append((t, deserialize_message(data, Gpgga).gps_qual))
    L, N, Q = np.array(lio), np.array(nav), np.array(gga)
    lt, lx, ly = L[:, 0], L[:, 1], L[:, 2]
    e, n = enu_from_latlon(N[:, 1], N[:, 2], georef['lat0'], georef['lon0'])
    th, tx, ty = (georef['enu_to_map'][k] for k in ('theta_rad', 'tx', 'ty'))
    G = np.c_[math.cos(th) * e - math.sin(th) * n + tx, math.sin(th) * e + math.cos(th) * n + ty]
    qi = np.searchsorted(Q[:, 0], N[:, 0], side='right') - 1
    fix = np.isfinite(N[:, 1]) & (np.abs(N[:, 1]) > 1e-6) & (np.where(qi >= 0, Q[np.clip(qi, 0, None), 1], -1) == 4)
    sim, mode, *_ = simulate_switch(lt, lx, ly, N[:, 0], G, fix, win=win, min_move=min_move)
    te, GF = N[fix, 0], G[fix]
    LF = np.c_[np.interp(te, lt, lx), np.interp(te, lt, ly)]
    R = [rigid_align(LF, GF)[0]]
    for k in range(1, len(te)):
        sel = (te > te[k] - win) & (te <= te[k])
        R.append(rigid_align(LF[sel], GF[sel])[0] if sel.sum() >= 5 and np.hypot(*(GF[sel][-1] - GF[sel][0])) >= min_move else R[-1])
    R = np.array(R)
    k = np.clip(np.searchsorted(te, lt, side='right') - 1, 0, None)
    yaw = L[:, 3] + np.arctan2(R[k, 1, 0], R[k, 0, 0])
    return {int(s): (sim[i, 0], sim[i, 1], yaw[i], L[i, 4], L[i, 5], mode[i])
            for i, s in enumerate(stamps) if mode[i] >= 0}


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class IdealSwitchOdom(Node):

    def __init__(self):
        super().__init__('ideal_switch_odom')
        bag = self.declare_parameter('bag', '').value
        georef = yaml.safe_load(open(self.declare_parameter('georef', '').value))
        self.odom_frame = self.declare_parameter('odom_frame', 'rsf_odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.skip_lio = self.declare_parameter('skip_lio', False).value
        self.poses = load_trajectory(bag, georef, self.declare_parameter('window', 60.0).value,
                                     self.declare_parameter('min_move', 8.0).value)
        self.get_logger().info(f'loaded {len(self.poses)} poses')
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, LIO, self.on_lio, 100)

    def on_lio(self, msg):
        pose = self.poses.get(stamp_ns(msg.header.stamp))
        if pose is None or not np.isfinite(pose[0]) or (self.skip_lio and pose[5] == 1):
            return
        x, y, yaw, roll, pitch, _ = pose
        base = TransformStamped()
        base.header.stamp = msg.header.stamp
        base.header.frame_id = self.odom_frame
        base.child_frame_id = self.base_frame
        base.transform.translation.x, base.transform.translation.y = float(x), float(y)
        base.transform.rotation.z, base.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        sensor = TransformStamped()
        sensor.header.stamp = msg.header.stamp
        sensor.header.frame_id = self.base_frame
        sensor.child_frame_id = msg.child_frame_id
        r = sensor.transform.rotation
        r.x, r.y, r.z, r.w = quaternion(roll, pitch, 0.0)
        self.tf.sendTransform([base, sensor])


def main():
    rclpy.init()
    try:
        rclpy.spin(IdealSwitchOdom())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
