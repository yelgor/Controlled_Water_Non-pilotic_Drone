#!/usr/bin/env python3
"""Штучні зашумлені сенсори з ground truth /kater/odom_gt.

    /kater/meas/gps      geometry_msgs/PointStamped   x, y у м (кадр odom), 5 Гц,  σ = gps_sigma
    /kater/meas/compass  geometry_msgs/Vector3Stamped z = курс, рад,          10 Гц, σ = compass_sigma
    /kater/meas/gyro     sensor_msgs/Imu              angular_velocity.z,     50 Гц, σ = gyro_sigma + зсув gyro_bias

Параметри (ros2 run ... --ros-args -p gps_sigma:=1.0): gps_sigma, compass_sigma, gyro_sigma, gyro_bias, seed.
"""
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

from boat_model import wrap, yaw_from_quat

GPS_PERIOD, COMPASS_PERIOD = 0.2, 0.1
SUNK_Z = -0.3  # м; на плаву z ≈ -0.038


class SensorsNoise(Node):
    def __init__(self):
        super().__init__('kater_sensors_noise')
        self.gps_sigma = self.declare_parameter('gps_sigma', 0.5).value
        self.compass_sigma = self.declare_parameter('compass_sigma', 0.05).value
        self.gyro_sigma = self.declare_parameter('gyro_sigma', 0.02).value
        self.gyro_bias = self.declare_parameter('gyro_bias', 0.03).value
        self.rng = np.random.default_rng(self.declare_parameter('seed', 1).value)

        self.pub_gps = self.create_publisher(PointStamped, '/kater/meas/gps', 10)
        self.pub_compass = self.create_publisher(Vector3Stamped, '/kater/meas/compass', 10)
        self.pub_gyro = self.create_publisher(Imu, '/kater/meas/gyro', 10)
        self.create_subscription(Odometry, '/kater/odom_gt', self.on_gt, 10)
        self.next_gps = self.next_compass = 0.0
        self.last_t = 0.0

    def on_gt(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t < self.last_t - 0.05:
            # Reset світу: час пішов назад. Без цього GPS/компас мовчали б, доки час не наздожене старий
            self.next_gps = self.next_compass = 0.0
        self.last_t = t
        p = msg.pose.pose.position
        if p.z < SUNK_Z:
            self.get_logger().error(
                f'Катер під водою (z = {p.z:.1f} м). Reset світу в Gazebo ламає плавучість '
                '(Buoyancy у gz-sim 8) - не тисни Reset, перезапусти ros2 launch',
                throttle_duration_sec=5.0)
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        n = self.rng.standard_normal

        gyro = Imu()
        gyro.header.stamp = msg.header.stamp
        gyro.header.frame_id = 'base_link'
        gyro.angular_velocity.z = msg.twist.twist.angular.z + self.gyro_bias + self.gyro_sigma * n()
        gyro.angular_velocity_covariance[8] = self.gyro_sigma ** 2
        self.pub_gyro.publish(gyro)  # 50 Гц, як odom_gt

        if t >= self.next_compass:
            self.next_compass = t + COMPASS_PERIOD
            c = Vector3Stamped()
            c.header.stamp = msg.header.stamp
            c.header.frame_id = 'odom'
            c.vector.z = wrap(yaw + self.compass_sigma * n())
            self.pub_compass.publish(c)

        if t >= self.next_gps:
            self.next_gps = t + GPS_PERIOD
            g = PointStamped()
            g.header.stamp = msg.header.stamp
            g.header.frame_id = 'odom'
            g.point.x = p.x + self.gps_sigma * n()
            g.point.y = p.y + self.gps_sigma * n()
            self.pub_gps.publish(g)


def main():
    rclpy.init()
    try:
        rclpy.spin(SensorsNoise())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
