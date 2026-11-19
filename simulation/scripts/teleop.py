#!/usr/bin/env python3
"""Ручне керування катером з клавіатури (диференційна тяга).

    w / s  (↑ / ↓)  - газ вперед / назад, крок 0.5 Н
    a / d  (← / →)  - кермо вліво / вправо, крок 1 Н різниці на рушій
    x               - кермо прямо (газ лишається)
    space           - стоп
    q               - вихід
Працює і в українській розкладці (ц/і/ф/в/ч/й).

Кермо має пріоритет: якщо газ + кермо не влазять у межу рушія, газ зменшується,
а різниця тяг (поворот) зберігається. Інакше на повному газі поворот зникав би.

Публікує /kater/thrust_left і /kater/thrust_right (std_msgs/Float64, Н).
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from boat_model import T_MAX

THROTTLE_STEP = 0.5
STEER_STEP = 1.0   # 1 Н -> ~27 град/с на місці, 5 Н -> ~80 град/с

KEYS = {
    'w': 'up', 'ц': 'up', '\x1b[A': 'up',
    's': 'down', 'і': 'down', 'ы': 'down', '\x1b[B': 'down',
    'a': 'left', 'ф': 'left', '\x1b[D': 'left',
    'd': 'right', 'в': 'right', '\x1b[C': 'right',
    'x': 'center', 'ч': 'center',
    ' ': 'stop',
    'q': 'quit', 'й': 'quit',
}


def clip(v, lim):
    return max(-lim, min(lim, v))


class Teleop(Node):
    def __init__(self):
        super().__init__('kater_teleop')
        self.pub_l = self.create_publisher(Float64, '/kater/thrust_left', 10)
        self.pub_r = self.create_publisher(Float64, '/kater/thrust_right', 10)
        self.throttle = 0.0
        self.steer = 0.0  # >0 - вліво (правий рушій тягне сильніше)

    def command(self, cmd):
        if cmd == 'up':
            self.throttle = clip(self.throttle + THROTTLE_STEP, T_MAX)
        elif cmd == 'down':
            self.throttle = clip(self.throttle - THROTTLE_STEP, T_MAX)
        elif cmd == 'left':
            self.steer = clip(self.steer + STEER_STEP, T_MAX)
        elif cmd == 'right':
            self.steer = clip(self.steer - STEER_STEP, T_MAX)
        elif cmd == 'center':
            self.steer = 0.0
        elif cmd == 'stop':
            self.throttle = self.steer = 0.0

    def publish(self):
        throttle = clip(self.throttle, T_MAX - abs(self.steer))  # пріоритет керма
        left, right = throttle - self.steer, throttle + self.steer
        self.pub_l.publish(Float64(data=left))
        self.pub_r.publish(Float64(data=right))
        print(f'\rгаз={self.throttle:+.1f}  кермо={self.steer:+.1f}  '
              f'L={left:+.2f} Н  R={right:+.2f} Н   ', end='', flush=True)


def read_key():
    """Один символ або escape-послідовність стрілки (\\x1b[A ...)."""
    key = sys.stdin.read(1)
    if key == '\x1b' and select.select([sys.stdin], [], [], 0.01)[0]:
        key += sys.stdin.read(2)
    return key


def main():
    rclpy.init()
    node = Teleop()
    settings = termios.tcgetattr(sys.stdin)
    print(__doc__)
    # дати DDS час знайти інші вузли
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.count_publishers('/kater/thrust_left') > 1:
        print('УВАГА: тягу вже публікує інший вузол (автопілот controller.py) - він перебиватиме\n'
              '       твої команди 20 разів на секунду. Для ручного керування запусти\n'
              '       ros2 launch ~/simulation/launch/sim.launch.py mode:=manual\n')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                cmd = KEYS.get(read_key())
                if cmd == 'quit':
                    break
                node.command(cmd)
            node.publish()  # 10 Гц, навіть без натискань
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        node.throttle = node.steer = 0.0
        node.publish()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
