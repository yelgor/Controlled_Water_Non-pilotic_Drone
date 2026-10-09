"""Спільні параметри катера (див. models/kater/model.sdf) і модель руху для KF / регулятора.

Площинна модель, зв'язана система (v ≈ 0):
    x_dot   = u cos(psi)            y_dot = u sin(psi)          psi_dot = r
    m*u_dot = (T_l + T_r) + X_U*u + X_UU*u|u|
    Iz*r_dot = D*(T_r - T_l) + N_R*r + N_RR*r|r|
"""
import math

M = 1.54          # кг, корпус + гвинти
IZ = 0.022        # кг*м^2
D = 0.035         # м, плече рушія від осі симетрії
X_U, X_UU = -1.0, -2.0
N_R, N_RR = -0.1, -0.1
T_MAX = 5.0       # Н, межа одного рушія


def wrap(a):
    """Кут у [-pi, pi)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    """(x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)

