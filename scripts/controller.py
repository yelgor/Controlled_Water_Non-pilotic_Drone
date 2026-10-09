#!/usr/bin/env python3
"""Автопілот: проходження маршруту двома PID-регуляторами.

    наведення LOS:   курс на точку на відрізку маршруту за lookahead м попереду
    PID швидкості:   T   = T_ff(u_ref) + PID(u_ref - u)          (T_ff - з моделі опору)
    PID курсу:       dT  = PID(psi_ref - psi), D-частина по виміряній r
    розподіл:        T_l = (T - dT)/2,  T_r = (T + dT)/2   (поворот має пріоритет при насиченні)
    задній хід:      на відрізку з dir = -1 курс = напрям на ціль + 180°, u_ref < 0

Стан бере з /kater/odom_kf (або /kater/odom_gt при use_gt:=true).
Параметри:
    mode       square | circle | figure8 | zigzag | fwd_back | random | steps | custom (див. routes.py)
    size, seed розмір маршруту, м; зерно для random
    waypoints  для mode:=custom - плоский список x1,y1,x2,y2,...;  loop - по колу
    u_ref, lookahead, use_gt
    speed_kp/ki/kd, heading_kp/ki/kd - коефіцієнти PID
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64, Float64MultiArray

from boat_model import T_MAX, X_U, X_UU, wrap, yaw_from_quat
from routes import STEPS, make_route

DT = 0.05
ACCEPT_RADIUS = 1.0
NAN = float('nan')


class Pid:
    def __init__(self, kp, ki, kd, i_limit):
        self.kp, self.ki, self.kd, self.i_limit = kp, ki, kd, i_limit
        self.i = 0.0

    def __call__(self, err, d_meas, dt):
        """d_meas - похідна виміряної величини (D по вимірюванню, без стрибків при зміні уставки)."""
        self.i = max(-self.i_limit, min(self.i_limit, self.i + err * dt))
        return self.kp * err + self.ki * self.i - self.kd * d_meas


class Controller(Node):
    def __init__(self):
        super().__init__('kater_controller')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.mode = p('mode', 'square')
        self.u_ref = p('u_ref', 1.0)
        self.lookahead = p('lookahead', 2.0)
        size = p('size', 10.0)
        seed = p('seed', 1)
        loop = p('loop', True)
        wp = self.declare_parameter('waypoints', Parameter.Type.DOUBLE_ARRAY).value
        use_gt = p('use_gt', False)

        self.speed_pid = Pid(p('speed_kp', 3.0), p('speed_ki', 1.0), p('speed_kd', 0.0), i_limit=2.0)
        self.heading_pid = Pid(p('heading_kp', 3.0), p('heading_ki', 0.2), p('heading_kd', 0.8), i_limit=1.0)

        if self.mode == 'custom':
            if not wp or len(wp) % 2:
                raise ValueError('mode:=custom потребує waypoints:="[x1, y1, x2, y2, ...]"')
            self.wps, self.loop = [(x, y, 1, 1.0) for x, y in zip(wp[0::2], wp[1::2])], loop
        elif self.mode != 'steps':
            self.wps, self.loop = make_route(self.mode, size, seed)

        self.state = None
        self.prev_wp = (0.0, 0.0)
        self.idx = 0
        self.done = False
        self.t0 = None

        topic = '/kater/odom_gt' if use_gt else '/kater/odom_kf'
        self.create_subscription(Odometry, topic, self.on_odom, 10)
        self.pub_l = self.create_publisher(Float64, '/kater/thrust_left', 10)
        self.pub_r = self.create_publisher(Float64, '/kater/thrust_right', 10)
        # [u_ref, u, psi_ref, psi, idx, xte, ax, ay, bx, by] - для логера/графіків
        self.pub_dbg = self.create_publisher(Float64MultiArray, '/kater/ctrl/debug', 10)
        self.create_timer(DT, self.step)
        gains = (f'PID швидкості {self.speed_pid.kp}/{self.speed_pid.ki}/{self.speed_pid.kd}, '
                 f'курсу {self.heading_pid.kp}/{self.heading_pid.ki}/{self.heading_pid.kd}')
        route = 'стрибки уставок' if self.mode == 'steps' else f'{len(self.wps)} точок'
        self.get_logger().info(f'режим {self.mode} ({route}), стан з {topic}, {gains}')

    def on_odom(self, msg):
        p = msg.pose.pose
        self.state = (p.position.x, p.position.y, yaw_from_quat(p.orientation),
                      msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def guidance(self, x, y):
        """LOS: -> (курс на точку попереду, напрям руху, множник швидкості, xte, відрізок) або None."""
        tx, ty, direction, speed = self.wps[self.idx]
        ax, ay = self.prev_wp
        seg = math.hypot(tx - ax, ty - ay)
        ex, ey = ((tx - ax) / seg, (ty - ay) / seg) if seg > 1e-6 else (1.0, 0.0)
        along = (x - ax) * ex + (y - ay) * ey          # проєкція на відрізок
        if math.hypot(tx - x, ty - y) < ACCEPT_RADIUS or along > seg:
            self.get_logger().info(f'точка {self.idx} ({tx:.1f}, {ty:.1f}) пройдена')
            self.prev_wp = (tx, ty)
            self.idx += 1
            if self.idx >= len(self.wps):
                if not self.loop:
                    return None
                self.idx = 0
            return self.guidance(x, y)
        xte = -(x - ax) * ey + (y - ay) * ex           # бокове відхилення від відрізка, >0 - зліва
        target = min(along + self.lookahead, seg)
        bearing = math.atan2(ay + ey * target - y, ax + ex * target - x)
        return bearing, direction, speed, xte, (ax, ay, tx, ty)

    def step(self):
        if self.state is None:
            return
        if self.done:
            self.send(0.0, 0.0)
            return
        x, y, psi, u, r = self.state
        if self.mode == 'steps':
            # чисті стрибки уставок, без LOS і без зниження швидкості в повороті
            t = self.get_clock().now().nanoseconds * 1e-9
            self.t0 = t if self.t0 is None else self.t0
            t_cycle = (t - self.t0) % sum(s[0] for s in STEPS)
            for dur, psi_deg, u_step in STEPS:
                if t_cycle < dur:
                    break
                t_cycle -= dur
            psi_ref, u_ref = math.radians(psi_deg), u_step
            e_psi = wrap(psi_ref - psi)
            xte, seg = NAN, (NAN,) * 4
        else:
            g = self.guidance(x, y)
            if g is None:
                self.done = True
                self.send(0.0, 0.0)
                self.get_logger().info('маршрут завершено')
                return
            bearing, direction, speed, xte, seg = g
            psi_ref = bearing if direction > 0 else wrap(bearing + math.pi)
            e_psi = wrap(psi_ref - psi)
            # на крутому повороті скидаємо швидкість - менший радіус
            u_ref = direction * speed * self.u_ref * max(0.3, math.cos(e_psi))

        t_ff = -(X_U * u_ref + X_UU * u_ref * abs(u_ref))
        thrust = t_ff + self.speed_pid(u_ref - u, 0.0, DT)
        d_thrust = self.heading_pid(e_psi, r, DT)

        d_thrust = max(-2 * T_MAX, min(2 * T_MAX, d_thrust))
        # пріоритет повороту: обмежуємо сумарну тягу, щоб обидва рушії лишились у [-T_MAX, T_MAX]
        lim = 2 * T_MAX - abs(d_thrust)
        thrust = max(-lim, min(lim, thrust))
        self.send((thrust - d_thrust) / 2, (thrust + d_thrust) / 2)
        self.pub_dbg.publish(Float64MultiArray(data=[
            u_ref, u, psi_ref, psi, float(self.idx), xte, *map(float, seg)]))

    def send(self, left, right):
        self.pub_l.publish(Float64(data=float(left)))
        self.pub_r.publish(Float64(data=float(right)))


def main():
    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.send(0.0, 0.0)  # зупинити рушії
        except Exception:
            pass


if __name__ == '__main__':
    main()
