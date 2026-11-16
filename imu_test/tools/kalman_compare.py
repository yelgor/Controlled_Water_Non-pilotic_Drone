#!/usr/bin/env python3
"""Простий фільтр Калмана для крену й нахилу і порівняння з Madgwick на вже записаному CSV.

    python3 tools/kalman_compare.py data/<файл>.csv

Стан для кожної осі: x = [кут, зміщення гіро b].
  Прогноз:  кут += (ω − b)·dt          (гіроскоп)
  Корекція: z = кут з акселерометра    (сила тяжіння)
Пише <ім'я>_kalman.png і друкує, наскільки Калман і Madgwick розходяться.
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from analyze_imu import load

SIG_GYRO = 0.2      # °/с, шум гіро (виміряно: 0,18–0,27)
SIG_BIAS = 0.002    # °/с за √с, як швидко «гуляє» зміщення (з Аллана, дуже мало)
SIG_ACC = 2.0       # °, наскільки не віримо куту з акселерометра (рух, вібрація)


def kalman_axis(omega, z, dt):
    x = np.array([z[0], 0.0])
    P = np.diag([1.0, 0.1])
    R = SIG_ACC ** 2
    out = np.empty((len(z), 2))
    for k in range(len(z)):
        d = dt[k]
        # 1. Прогноз: інтегруємо гіро, невизначеність росте
        F = np.array([[1, -d], [0, 1]])
        x = np.array([x[0] + (omega[k] - x[1]) * d, x[1]])
        Q = np.diag([(SIG_GYRO * d) ** 2, SIG_BIAS ** 2 * d])
        P = F @ P @ F.T + Q
        # 2. Корекція: порівнюємо з кутом з акселерометра
        y = z[k] - x[0]                 # нев'язка
        S = P[0, 0] + R
        K = P[:, 0] / S                 # коефіцієнт Калмана (скільки віримо виміру)
        x = x + K * y
        P = P - np.outer(K, P[0, :])
        out[k] = x
    return out


def main():
    meta, c = load(sys.argv[1])
    t = (c["t_us"] - c["t_us"][0]) / 1e6
    dt = np.diff(t, prepend=t[0] + (t[1] - t[0]))
    ax, ay, az = c["ax_g"], c["ay_g"], c["az_g"]
    roll_acc = np.degrees(np.arctan2(ay, az))
    pitch_acc = np.degrees(np.arctan2(-ax, np.sqrt(ay ** 2 + az ** 2)))
    kr = kalman_axis(c["gx_dps"], roll_acc, dt)
    kp = kalman_axis(c["gy_dps"], pitch_acc, dt)

    for name, k, m in [("roll", kr[:, 0], c["roll_deg"]), ("pitch", kp[:, 0], c["pitch_deg"])]:
        d = k - m
        print(f"{name}: Калман − Madgwick: середнє {d.mean():+.2f}°, СКВ {d.std():.2f}°, макс {np.abs(d).max():.2f}°")
    print(f"Оцінка залишкового зміщення гіро наприкінці: x {kr[-1,1]:+.3f}, y {kp[-1,1]:+.3f} °/с")

    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for a, name, acc, k, m in [(axs[0], "Крен (roll)", roll_acc, kr[:, 0], c["roll_deg"]),
                               (axs[1], "Нахил (pitch)", pitch_acc, kp[:, 0], c["pitch_deg"])]:
        a.plot(t, acc, color="0.75", lw=.5, label="лише акселерометр (шумить)")
        a.plot(t, m, "C0", lw=1, label="Madgwick (на платі)")
        a.plot(t, k, "C3--", lw=1, label="Калман (на ноутбуці)")
        a.set_ylabel("°"); a.set_title(name); a.grid(alpha=.3); a.legend(fontsize=8, loc="upper right")
    axs[2].plot(t, kr[:, 1], label="b_x"); axs[2].plot(t, kp[:, 1], label="b_y")
    axs[2].set(title="Зміщення гіро, яке оцінив Калман (залишок після калібрування)", xlabel="t, с", ylabel="°/с")
    axs[2].grid(alpha=.3); axs[2].legend(fontsize=8)
    fig.tight_layout()
    out = sys.argv[1][:-4] + "_kalman.png"
    fig.savefig(out, dpi=120)
    print("Графік:", out)


if __name__ == "__main__":
    main()
