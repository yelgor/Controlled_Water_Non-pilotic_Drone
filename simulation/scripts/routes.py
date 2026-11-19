"""Маршрути автопілота (параметр mode у controller.py).

Маршрут - список точок (x, y, dir, speed):
    dir   = +1 - йти носом вперед, -1 - кормою вперед (задній хід без розвороту)
    speed - множник до u_ref (1.0 - як задано)
Старт катера - (0, 0), ніс по +x. size - характерний розмір маршруту, м.

Режим steps - не маршрут, а таблиця стрибків уставок (для налаштування PID):
    (тривалість, с;  курс, град;  швидкість, м/с)
"""
import math

import numpy as np


def square(size):
    s = size
    return [(s, 0, 1, 1.0), (s, s, 1, 1.0), (0, s, 1, 1.0), (0, 0, 1, 1.0)], True


def circle(size, n=16):
    """Коло радіуса size/2 проти год. стрілки, старт у (0, 0) по дотичній."""
    rad = size / 2
    return [(rad * math.sin(a), rad - rad * math.cos(a), 1, 1.0)
            for a in np.linspace(2 * math.pi / n, 2 * math.pi, n)], True


def figure8(size, n=16):
    """Вісімка: коло проти год. стрілки над віссю x, потім за год. стрілкою під нею."""
    rad = size / 2
    angles = np.linspace(2 * math.pi / n, 2 * math.pi, n)
    up = [(rad * math.sin(a), rad - rad * math.cos(a), 1, 1.0) for a in angles]
    down = [(rad * math.sin(a), -rad + rad * math.cos(a), 1, 1.0) for a in angles]
    return up + down, True


def zigzag(size):
    """Змійка вперед по x (крок size/2, розмах ±size/3) і назад тим самим шляхом."""
    step, amp = size / 2, size / 3
    there = [(step * (i + 1), amp * (1 if i % 2 == 0 else -1), 1, 1.0) for i in range(4)]
    there.append((step * 5, 0, 1, 1.0))
    back = [(x, y, 1, 1.0) for x, y, _, _ in reversed(there[:-1])] + [(0, 0, 1, 1.0)]
    return there + back, True


def fwd_back(size):
    """Пряма вперед на size м, потім назад кормою вперед (без розвороту), далі по колу."""
    return [(size, 0, 1, 1.0), (0, 0, -1, 0.7)], True


def random_route(size, seed, n=40):
    """Випадкові точки в квадраті ±size, мінімум 3 м одна від одної, випадкова швидкість."""
    rng = np.random.default_rng(seed)
    pts, prev = [], (0.0, 0.0)
    while len(pts) < n:
        x, y = rng.uniform(-size, size, 2)
        if math.hypot(x - prev[0], y - prev[1]) < 3.0:
            continue
        pts.append((float(x), float(y), 1, float(rng.uniform(0.5, 1.5))))
        prev = (x, y)
    return pts, False


# Стрибки уставок: курс +90, -90, -90, -90 (поворот на 180 при повторі), потім швидкість.
# Траєкторія замикається: 10 м на схід, 10 на північ, 10 на схід, 10 на південь, 20 на захід.
STEPS = [
    (10, 0, 1.0),
    (10, 90, 1.0),
    (10, 0, 1.0),
    (10, -90, 1.0),
    (10, 180, 0.5),
    (10, 180, 1.5),
]

MODES = {
    'square': square,
    'circle': circle,
    'figure8': figure8,
    'zigzag': zigzag,
    'fwd_back': fwd_back,
}


def make_route(mode, size, seed):
    """-> (точки, по колу?)"""
    if mode == 'random':
        return random_route(size, seed)
    if mode not in MODES:
        raise ValueError(f'невідомий mode "{mode}"; є: {", ".join(list(MODES) + ["random", "steps", "custom"])}')
    return MODES[mode](size)
