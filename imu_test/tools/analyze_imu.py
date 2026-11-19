#!/usr/bin/env python3
"""Аналіз запису imu_logger.py: шум, зміщення, дрейф, варіація Аллана, кут повороту.

    python3 tools/analyze_imu.py data/imu_..._static_nxp.csv
    python3 tools/analyze_imu.py data/imu_..._turnz90.csv

Пише поруч з CSV:  <ім'я>_report.txt  і  <ім'я>_plots.png
Потрібні numpy і matplotlib:  sudo apt install python3-numpy python3-matplotlib
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

AX = ("x", "y", "z")


def load(path):
    meta, header, rows = {}, None, []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                parts = line[1:].strip().split(",")
                meta[parts[0]] = parts[1:]
                continue
            if line.startswith("t_us,"):
                header = line.split(",")
                continue
            p = line.split(",")
            if header and len(p) == len(header):
                try:
                    rows.append([float(x) for x in p])
                except ValueError:
                    pass
    d = np.array(rows)
    return meta, {c: d[:, i] for i, c in enumerate(header)}


def allan(rate, fs, n_tau=40):
    """Перекрита варіація Аллана для швидкості (°/с). Повертає tau, adev."""
    theta = np.cumsum(rate) / fs
    n = len(theta)
    ms = np.unique(np.logspace(0, np.log10(n // 9), n_tau).astype(int))
    taus, adev = [], []
    for m in ms:
        d = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        taus.append(m / fs)
        adev.append(np.sqrt(np.mean(d ** 2) / (2 * (m / fs) ** 2)))
    return np.array(taus), np.array(adev)


def unwrap_deg(a):
    return np.degrees(np.unwrap(np.radians(a)))


def main(path):
    meta, c = load(path)
    t = (c["t_us"] - c["t_us"][0]) / 1e6
    dt = np.diff(c["t_us"]) / 1e6
    dt = dt[(dt > 0) & (dt < 0.1)]
    fs = 1.0 / np.median(dt)
    dur = t[-1]
    out = []
    p = out.append

    p(f"Файл: {path}")
    p(f"IMU: {meta.get('imu', ['?'])[0]}, режим {meta.get('mode', ['?'])[0]}")
    p(f"Рядків {len(t)}, тривалість {dur:.1f} с ({dur/60:.1f} хв), частота {fs:.1f} Гц")
    gaps = int(np.sum(np.diff(c['t_us']) > 2.5e6 / fs))
    p(f"Пропуски в даних: {gaps}")
    if "gyro_bias_dps" in meta:
        b = [float(x) for x in meta["gyro_bias_dps"]]
        p(f"Зміщення з калібрування на старті b, °/с: x {b[0]:+.4f}  y {b[1]:+.4f}  z {b[2]:+.4f}")
    p("")

    p("ГІРОСКОП (після віднімання b)")
    p("вісь  залишк.середнє °/с  СКВ °/с   ARW °/√год  нестаб.зміщ. °/год (τ)   ∫ω dt, °")
    allan_res = {}
    for i, a in enumerate(AX):
        g = c[f"g{a}_dps"]
        tau, ad = allan(g, fs)
        allan_res[a] = (tau, ad)
        arw = np.interp(1.0, tau, ad) * 60            # σ(τ=1 с) · 60 -> °/√год
        k = int(np.argmin(ad))
        bi = ad[k] / 0.664 * 3600                      # мінімум кривої / 0,664 -> °/год
        ang = np.sum(g[1:] * np.diff(t))
        p(f"  {a}   {g.mean():+12.4f}       {g.std():7.4f}   {arw:8.3f}    {bi:8.1f} ({tau[k]:.0f} с)      {ang:+8.2f}")
    p("")

    p("АКСЕЛЕРОМЕТР, g")
    for a in AX:
        v = c[f"a{a}_g"]
        p(f"  {a}: середнє {v.mean():+.5f}, СКВ {v.std():.5f}  ({v.std()*9.81*1000:.1f} мм/с²)")
    norm = np.sqrt(c["ax_g"] ** 2 + c["ay_g"] ** 2 + c["az_g"] ** 2)
    p(f"  |a|: {norm.mean():.4f} g (ідеально 1,000; відхилення = похибка масштабу акселерометра)")
    p("")

    p("МАГНІТОМЕТР, гаус")
    mn = np.sqrt(c["mx_gauss"] ** 2 + c["my_gauss"] ** 2 + c["mz_gauss"] ** 2)
    for a in AX:
        v = c[f"m{a}_gauss"]
        p(f"  {a}: середнє {v.mean():+.4f}, СКВ {v.std():.4f}")
    p(f"  |m|: {mn.mean():.3f} гаус (поле Землі ~0,5; більше означає залізо поруч або без калібрування)")
    p("")

    p("КУТИ (Madgwick)")
    yaw = unwrap_deg(c["yaw_deg"])
    for name, v in (("roll", unwrap_deg(c["roll_deg"])), ("pitch", c["pitch_deg"]), ("yaw", yaw)):
        p(f"  {name:5s}: початок {v[0]:+8.3f}°, кінець {v[-1]:+8.3f}°, зміна {v[-1]-v[0]:+7.3f}°, "
          f"розмах {v.max()-v.min():6.3f}°")
    slope = np.polyfit(t, yaw, 1)[0] * 60
    p(f"  дрейф курсу (лінійна апроксимація): {slope:+.3f} °/хв = {slope*60:+.1f} °/год")
    p("")
    p(f"calc_us: середнє {c['calc_us'].mean():.1f}, макс {c['calc_us'].max():.0f} мкс "
      f"(бюджет одного кроку {1e6/fs:.0f} мкс)")
    p("")
    p("Як читати: ARW = білий шум гіро (менше краще); нестабільність зміщення = як сильно 'гуляє' b;")
    p("∫ω dt = на скільки градусів повернулась плата навколо осі за весь запис (для поворотів).")

    report = "\n".join(out)
    print(report)
    base = path[:-4] if path.endswith(".csv") else path
    with open(base + "_report.txt", "w") as f:
        f.write(report + "\n")

    fig, axs = plt.subplots(2, 2, figsize=(13, 8))
    w = max(1, int(fs))                                 # середні за 1 с, щоб графік читався
    n = len(t) // w * w
    tb = t[:n].reshape(-1, w).mean(1) / 60
    for a in AX:
        axs[0, 0].plot(tb, c[f"g{a}_dps"][:n].reshape(-1, w).mean(1), label=f"g{a}")
        axs[0, 1].loglog(*allan_res[a], "o-", ms=3, label=f"g{a}")
    axs[0, 0].set(title="Гіроскоп, середнє за 1 с", xlabel="хв", ylabel="°/с")
    axs[0, 1].set(title="Варіація Аллана", xlabel="τ, с", ylabel="σ, °/с")
    axs[0, 1].grid(True, which="both", alpha=0.3)
    axs[1, 0].plot(t / 60, yaw, label="yaw")
    axs[1, 0].plot(t / 60, unwrap_deg(c["roll_deg"]) - unwrap_deg(c["roll_deg"])[0], label="roll − roll₀")
    axs[1, 0].plot(t / 60, c["pitch_deg"] - c["pitch_deg"][0], label="pitch − pitch₀")
    axs[1, 0].set(title="Кути (зміна від початку)", xlabel="хв", ylabel="°")
    for a in AX:
        axs[1, 1].plot(tb, c[f"a{a}_g"][:n].reshape(-1, w).mean(1), label=f"a{a}")
    axs[1, 1].set(title="Акселерометр, середнє за 1 с", xlabel="хв", ylabel="g")
    for ax in axs.flat:
        ax.legend()
    fig.suptitle(path.split("/")[-1])
    fig.tight_layout()
    fig.savefig(base + "_plots.png", dpi=110)
    print(f"\nЗвіт: {base}_report.txt\nГрафіки: {base}_plots.png")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for p_ in sys.argv[1:]:
        main(p_)
