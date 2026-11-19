#!/usr/bin/env python3
"""Траєкторія ходьби з IMU двома способами.

    python3 tools/walk_trajectory.py data/imu_..._walk.csv [--steps 40] [--step-len 0.7]

1. Наївний: акселерометр -> світова система (за кутами Madgwick) -> мінус g -> двічі інтегруємо.
2. Крокомір (PDR): кроки = піки модуля прискорення, напрям кроку = курс з гіро (yaw).
   --steps N: справжня кількість кроків (якщо порахував), тоді довжина кроку не потрібна для перевірки.
Пише <ім'я>_walk.png і <ім'я>_walk.txt поруч з CSV.
"""
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from analyze_imu import load

G = 9.81


def rot_body_to_world(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    R = np.empty((len(roll), 3, 3))
    R[:, 0, 0] = cy * cp; R[:, 0, 1] = cy * sp * sr - sy * cr; R[:, 0, 2] = cy * sp * cr + sy * sr
    R[:, 1, 0] = sy * cp; R[:, 1, 1] = sy * sp * sr + cy * cr; R[:, 1, 2] = sy * sp * cr - cy * sr
    R[:, 2, 0] = -sp;     R[:, 2, 1] = cp * sr;                R[:, 2, 2] = cp * cr
    return R


def lowpass(x, fs, fc):
    a = 1 - np.exp(-2 * np.pi * fc / fs)
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = y[i - 1] + a * (x[i] - y[i - 1])
    return y


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--steps", type=int, default=0, help="справжня кількість кроків")
    ap.add_argument("--step-len", type=float, default=0.7, help="довжина кроку, м")
    ap.add_argument("--still", type=float, default=4.0, help="скільки секунд на початку плата нерухома")
    ap.add_argument("--ref-bias", default="", help="офсет гіро з довгого статичного запису 'x,y,z' (°/с), "
                    "якщо калібрування на старті зіпсоване рухом")
    args = ap.parse_args()

    meta, c = load(args.csv)
    t = (c["t_us"] - c["t_us"][0]) / 1e6
    fs = 1.0 / np.median(np.diff(t))
    a_body = np.stack([c["ax_g"], c["ay_g"], c["az_g"]], 1)
    roll, pitch = np.radians(c["roll_deg"]), np.radians(c["pitch_deg"])
    yaw = np.unwrap(np.radians(c["yaw_deg"]))
    bias_note = "курс: Madgwick (калібрування з початку запису)"
    if args.ref_bias:
        # Калібрування на старті зіпсоване: повертаємо його офсет і віднімаємо правильний,
        # курс інтегруємо заново: psi' = (q sin(roll) + r cos(roll)) / cos(pitch)
        ref = np.array([float(x) for x in args.ref_bias.split(",")])
        cal = np.array([float(x) for x in meta["gyro_bias_dps"][:3]])
        w = np.radians(np.stack([c["gx_dps"], c["gy_dps"], c["gz_dps"]], 1) + (cal - ref))
        psi_dot = (w[:, 1] * np.sin(roll) + w[:, 2] * np.cos(roll)) / np.cos(pitch)
        yaw = yaw[0] + np.concatenate([[0], np.cumsum(0.5 * (psi_dot[1:] + psi_dot[:-1]) * np.diff(t))])
        bias_note = f"курс: перераховано з офсетом {ref} замість {cal} (°/с)"

    still = t < args.still
    g_meas = np.linalg.norm(a_body[still], axis=1).mean()      # 1,044 g у цього датчика
    a_body = a_body / g_meas                                    # грубе виправлення масштабу

    # ---- 1. Наївне подвійне інтегрування ----
    R = rot_body_to_world(roll, pitch, yaw)
    a_w = np.einsum("nij,nj->ni", R, a_body) * G
    a_w[:, 2] -= G
    a_w -= a_w[still].mean(0)                                  # прибрати залишок, виміряний у спокої
    dt = np.gradient(t)
    v = np.cumsum(a_w * dt[:, None], 0)
    p_naive = np.cumsum(v * dt[:, None], 0)

    # ---- 2. Крокомір ----
    an = lowpass(np.linalg.norm(a_body, axis=1), fs, 3.0) - 1.0
    thr = max(0.08, 0.4 * np.percentile(np.abs(an[~still]), 95)) if (~still).any() else 0.1
    steps = []
    last = -1.0
    for i in range(1, len(an) - 1):
        if an[i] > thr and an[i] >= an[i - 1] and an[i] > an[i + 1] and t[i] - last > 0.3:
            steps.append(i)
            last = t[i]
    steps = np.array(steps, int)
    L = args.step_len
    if args.steps and len(steps):
        L_note = f"довжина кроку {L} м (задано); кроків знайдено {len(steps)}, справжніх {args.steps}"
    else:
        L_note = f"довжина кроку {L} м (задано); кроків знайдено {len(steps)}"
    pos = [np.zeros(2)]
    for i in steps:
        pos.append(pos[-1] + L * np.array([np.cos(yaw[i]), np.sin(yaw[i])]))
    pos = np.array(pos)
    path = L * len(steps)
    close = np.linalg.norm(pos[-1] - pos[0])

    out = [
        f"Файл: {args.csv}",
        f"Тривалість {t[-1]:.1f} с, {fs:.0f} Гц; |a| у спокої = {g_meas:.4f} g (поділено на нього)",
        "",
        bias_note,
        "",
        "1. Подвійне інтегрування акселерометра:",
        f"   кінцева точка ({p_naive[-1,0]:+.1f}, {p_naive[-1,1]:+.1f}, {p_naive[-1,2]:+.1f}) м, "
        f"макс. відхилення {np.linalg.norm(p_naive[:, :2], axis=1).max():.1f} м",
        "",
        "2. Крокомір (кроки + курс з гіро):",
        f"   {L_note}",
        f"   довжина шляху {path:.1f} м, повна зміна курсу {np.degrees(yaw[-1]-yaw[0]):+.1f}°",
        f"   похибка замикання (кінець vs старт) {close:.2f} м = {100*close/max(path,1e-9):.1f} % шляху",
    ]
    report = "\n".join(out)
    print(report)
    base = args.csv[:-4]
    with open(base + "_walk.txt", "w") as f:
        f.write(report + "\n")

    fig, ax = plt.subplots(1, 3, figsize=(15, 4.6))
    ax[0].plot(p_naive[:, 0], p_naive[:, 1])
    ax[0].plot(0, 0, "go", label="старт")
    ax[0].set(title="1. Подвійне інтегрування", xlabel="x, м", ylabel="y, м")
    ax[0].axis("equal"); ax[0].grid(alpha=.3); ax[0].legend()
    ax[1].plot(pos[:, 0], pos[:, 1], "o-", ms=3)
    ax[1].plot(0, 0, "go", ms=8, label="старт")
    ax[1].plot(pos[-1, 0], pos[-1, 1], "rs", ms=8, label=f"кінець ({close:.1f} м від старту)")
    ax[1].set(title="2. Крокомір: кроки + курс гіро", xlabel="x, м", ylabel="y, м")
    ax[1].axis("equal"); ax[1].grid(alpha=.3); ax[1].legend(fontsize=8)
    ax[2].plot(t, an, lw=.6, label="|a| − 1 g (фільтр 3 Гц)")
    ax[2].plot(t[steps], an[steps], "rx", label=f"кроки: {len(steps)}")
    ax[2].axhline(thr, color="k", ls="--", lw=.7)
    ax2 = ax[2].twinx(); ax2.plot(t, np.degrees(yaw), "C2", lw=1); ax2.set_ylabel("курс, °", color="C2")
    ax[2].set(title="Виявлення кроків і курс", xlabel="t, с", ylabel="g"); ax[2].legend(fontsize=8, loc="upper left")
    fig.tight_layout()
    fig.savefig(base + "_walk.png", dpi=130)
    print(f"\nГрафік: {base}_walk.png\nЗвіт: {base}_walk.txt")


if __name__ == "__main__":
    main()
