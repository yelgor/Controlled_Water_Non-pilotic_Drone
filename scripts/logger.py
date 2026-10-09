#!/usr/bin/env python3
"""Пише в results/log_<tag>.csv: ground truth, GPS, оцінку KF, уставки регулятора, тягу (20 Гц).
tag - параметр (launch ставить режим: square, circle, manual, ...; з use_gt:=true - square_gt).
Графіки: python3 scripts/plot.py [results/log_<tag>.csv]  (без аргументу - найсвіжіший лог)
"""
import csv
import math
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

from boat_model import yaw_from_quat

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COLUMNS = ['t', 'gt_x', 'gt_y', 'gt_psi', 'gt_u', 'gt_r',
           'kf_x', 'kf_y', 'kf_psi', 'kf_u', 'kf_r', 'kf_bias', 'kf_sx', 'kf_sy', 'kf_spsi',
           'gps_x', 'gps_y', 'gps_nis', 'u_ref', 'psi_ref', 'wp', 'xte',
           'seg_ax', 'seg_ay', 'seg_bx', 'seg_by', 'thrust_l', 'thrust_r']


def odom_fields(msg):
    p = msg.pose.pose
    return [p.position.x, p.position.y, yaw_from_quat(p.orientation),
            msg.twist.twist.linear.x, msg.twist.twist.angular.z]


class Logger(Node):
    def __init__(self):
        super().__init__('kater_logger')
        os.makedirs(os.path.join(ROOT, 'results'), exist_ok=True)
        tag = self.declare_parameter('tag', 'run').value
        self.path = os.path.join(ROOT, 'results', f'log_{tag}.csv')
        self.f = open(self.path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(COLUMNS)
        self.d = dict.fromkeys(COLUMNS, math.nan)
        self.gps_new = self.nis_new = False

        self.create_subscription(Odometry, '/kater/odom_gt', self.on_gt, 10)
        self.create_subscription(Odometry, '/kater/odom_kf', self.on_kf, 10)
        self.create_subscription(Float64, '/kater/kf/gyro_bias', lambda m: self.set(kf_bias=m.data), 10)
        self.create_subscription(PointStamped, '/kater/meas/gps', self.on_gps, 10)
        self.create_subscription(Float64MultiArray, '/kater/kf/gps_innov', self.on_innov, 10)
        self.create_subscription(Float64MultiArray, '/kater/ctrl/debug', self.on_dbg, 10)
        self.create_subscription(Float64, '/kater/thrust_left', lambda m: self.set(thrust_l=m.data), 10)
        self.create_subscription(Float64, '/kater/thrust_right', lambda m: self.set(thrust_r=m.data), 10)
        self.create_timer(0.05, self.write)
        self.get_logger().info(f'лог -> {self.path}')

    def set(self, **kw):
        self.d.update(kw)

    def on_gt(self, m):
        self.d.update(zip(['gt_x', 'gt_y', 'gt_psi', 'gt_u', 'gt_r'], odom_fields(m)))

    def on_kf(self, m):
        self.d.update(zip(['kf_x', 'kf_y', 'kf_psi', 'kf_u', 'kf_r'], odom_fields(m)))
        c = m.pose.covariance
        self.set(kf_sx=math.sqrt(c[0]), kf_sy=math.sqrt(c[7]), kf_spsi=math.sqrt(c[35]))

    def on_gps(self, m):
        self.set(gps_x=m.point.x, gps_y=m.point.y)
        self.gps_new = True

    def on_innov(self, m):
        self.set(gps_nis=m.data[2])
        self.nis_new = True

    def on_dbg(self, m):
        self.set(u_ref=m.data[0], psi_ref=m.data[2], wp=m.data[4], xte=m.data[5],
                 seg_ax=m.data[6], seg_ay=m.data[7], seg_bx=m.data[8], seg_by=m.data[9])

    def write(self):
        if math.isnan(self.d['gt_x']):
            return
        self.d['t'] = self.get_clock().now().nanoseconds * 1e-9
        row = dict(self.d)
        if not self.gps_new:  # GPS 5 Гц: пишемо лише нові точки
            row['gps_x'] = row['gps_y'] = math.nan
        if not self.nis_new:  # приходить від KF трохи пізніше за GPS - окремий прапорець
            row['gps_nis'] = math.nan
        self.gps_new = self.nis_new = False
        self.w.writerow([row[c] for c in COLUMNS])
        self.f.flush()


def main():
    rclpy.init()
    node = Logger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.f.close()


if __name__ == '__main__':
    main()
