#!/usr/bin/env python3
"""Розширений фільтр Калмана (EKF) для катера.

Стан: s = [x, y, psi, u, r, b]   (b - зсув гіроскопа, рад/с)
Прогноз: модель руху з boat_model.py + команди тяги /kater/thrust_left|right, 50 Гц.
Корекція: /kater/meas/gps (x, y), /kater/meas/compass (psi), /kater/meas/gyro (r + b).
Вихід: /kater/odom_kf (nav_msgs/Odometry; twist.linear.x = u, twist.angular.z = r),
       /kater/kf/gyro_bias (std_msgs/Float64),
       /kater/kf/gps_innov (Float64MultiArray [dx, dy, NIS]) - інновація GPS: вимірювання мінус прогноз.
         NIS = innov^T S^-1 innov; для узгодженого фільтра в середньому = 2 (2 виміри),
         95% значень < 5.99. Це можна рахувати і на реальному катері, де ground truth немає.
Ground truth фільтр не бачить.
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Float64MultiArray

from boat_model import D, IZ, M, N_R, N_RR, X_U, X_UU, quat_from_yaw, wrap

DT = 0.02
X, Y, PSI, U, R, B = range(6)
# Шум процесу (спектральна густина, на секунду): похибки моделі, хвилі, бічний дрейф
Q = np.diag([0.01, 0.01, 1e-4, 0.05, 0.5, 1e-6])


class Ekf(Node):
    def __init__(self):
        super().__init__('kater_kf')
        self.r_gps = self.declare_parameter('gps_sigma', 0.5).value ** 2
        self.r_compass = self.declare_parameter('compass_sigma', 0.05).value ** 2
        self.r_gyro = self.declare_parameter('gyro_sigma', 0.02).value ** 2

        self.s = np.zeros(6)
        self.P = np.diag([1.0, 1.0, 0.5, 0.1, 0.1, 0.01])
        self.initialized = False  # x, y, psi беруться з першого GPS / компаса
        self.got_compass = False
        self.t_left = self.t_right = 0.0

        self.create_subscription(Float64, '/kater/thrust_left', lambda m: setattr(self, 't_left', m.data), 10)
        self.create_subscription(Float64, '/kater/thrust_right', lambda m: setattr(self, 't_right', m.data), 10)
        self.create_subscription(PointStamped, '/kater/meas/gps', self.on_gps, 10)
        self.create_subscription(Vector3Stamped, '/kater/meas/compass', self.on_compass, 10)
        self.create_subscription(Imu, '/kater/meas/gyro', self.on_gyro, 10)
        self.pub = self.create_publisher(Odometry, '/kater/odom_kf', 10)
        self.pub_bias = self.create_publisher(Float64, '/kater/kf/gyro_bias', 10)
        self.pub_innov = self.create_publisher(Float64MultiArray, '/kater/kf/gps_innov', 10)
        self.create_timer(DT, self.step)

    # ---------- прогноз ----------
    def step(self):
        if not self.initialized:
            return
        s = self.s
        psi, u, r = s[PSI], s[U], s[R]
        thrust = self.t_left + self.t_right
        torque = D * (self.t_right - self.t_left)

        s_dot = np.array([
            u * math.cos(psi),
            u * math.sin(psi),
            r,
            (thrust + X_U * u + X_UU * u * abs(u)) / M,
            (torque + N_R * r + N_RR * r * abs(r)) / IZ,
            0.0,
        ])
        A = np.zeros((6, 6))
        A[X, PSI], A[X, U] = -u * math.sin(psi), math.cos(psi)
        A[Y, PSI], A[Y, U] = u * math.cos(psi), math.sin(psi)
        A[PSI, R] = 1.0
        A[U, U] = (X_U + 2 * X_UU * abs(u)) / M
        A[R, R] = (N_R + 2 * N_RR * abs(r)) / IZ
        F = np.eye(6) + A * DT

        self.s = s + s_dot * DT
        self.s[PSI] = wrap(self.s[PSI])
        self.P = F @ self.P @ F.T + Q * DT
        self.publish()

    # ---------- корекція ----------
    def update(self, z, h, H, R_meas, angle_idx=None):
        innov = np.atleast_1d(z - h)
        if angle_idx is not None:
            innov[angle_idx] = wrap(innov[angle_idx])
        S = H @ self.P @ H.T + R_meas
        S_inv = np.linalg.inv(S)
        K = self.P @ H.T @ S_inv
        self.s = self.s + K @ innov
        self.s[PSI] = wrap(self.s[PSI])
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_meas @ K.T  # форма Джозефа
        return innov, float(innov @ S_inv @ innov)

    def on_gps(self, msg):
        z = np.array([msg.point.x, msg.point.y])
        if not self.initialized:
            if not self.got_compass:
                return
            self.s[X], self.s[Y] = z
            self.initialized = True
            self.get_logger().info('EKF ініціалізовано')
            return
        H = np.zeros((2, 6))
        H[0, X] = H[1, Y] = 1.0
        innov, nis = self.update(z, self.s[[X, Y]], H, np.eye(2) * self.r_gps)
        self.pub_innov.publish(Float64MultiArray(data=[float(innov[0]), float(innov[1]), nis]))

    def on_compass(self, msg):
        if not self.got_compass:
            self.s[PSI] = msg.vector.z
            self.got_compass = True
            return
        H = np.zeros((1, 6))
        H[0, PSI] = 1.0
        self.update(np.array([msg.vector.z]), np.array([self.s[PSI]]), H,
                    np.array([[self.r_compass]]), angle_idx=0)

    def on_gyro(self, msg):
        if not self.initialized:
            return
        H = np.zeros((1, 6))
        H[0, R] = H[0, B] = 1.0
        self.update(np.array([msg.angular_velocity.z]), np.array([self.s[R] + self.s[B]]), H,
                    np.array([[self.r_gyro]]))

    def publish(self):
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = float(self.s[X]), float(self.s[Y])
        q = o.pose.pose.orientation
        q.x, q.y, q.z, q.w = quat_from_yaw(self.s[PSI])
        o.twist.twist.linear.x = float(self.s[U])
        o.twist.twist.angular.z = float(self.s[R])
        P = self.P
        o.pose.covariance[0], o.pose.covariance[1] = P[X, X], P[X, Y]
        o.pose.covariance[6], o.pose.covariance[7] = P[Y, X], P[Y, Y]
        o.pose.covariance[35] = P[PSI, PSI]
        o.twist.covariance[0], o.twist.covariance[35] = P[U, U], P[R, R]
        self.pub.publish(o)
        self.pub_bias.publish(Float64(data=float(self.s[B])))


def main():
    rclpy.init()
    try:
        rclpy.spin(Ekf())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
