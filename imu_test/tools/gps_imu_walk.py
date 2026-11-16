#!/usr/bin/env python3
"""Прогулянка з IMU + GPS: злиття кроків і курсу гіро з GPS (маленький EKF).

    python3 tools/gps_imu_walk.py data/imu_..._gpswalk.csv [--dropout 60,30]

У CSV мають бути рядки GPS,t_us,lat,lon,sats,hdop,speed_kmh,course_deg (прошивка imu-test з GPS).

Стан EKF: [E, N, psi0, L]
  E, N  положення, м (схід, північ) від першої точки GPS
  psi0  поправка курсу: справжній курс = yaw гіро + psi0 (гіро не знає, де північ)
  L     довжина кроку, м
Прогноз на кожному кроці:   E += L cos(yaw + psi0),  N += L sin(yaw + psi0)
Оновлення на кожному GPS:   z = [E, N],  шум sigma = max(2 м, 2.5 * HDOP)
psi0 і L самі «вчаться» з того, куди GPS показує рух.

--dropout START,DUR: вимкнути GPS на DUR с з моменту START (як під мостом). Видно, скільки
набігає похибки на самому IMU і чи EKF повертається після відновлення сигналу.
Пише <ім'я>_gps.png і <ім'я>_gps.txt поруч з CSV.
"""
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from analyze_imu import load
from walk_trajectory import lowpass

R_EARTH = 6371000.0


def load_gps(path):
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith("GPS,"):
                p = line.strip().split(",")
                if len(p) >= 8:
                    try:
                        rows.append([float(x) for x in p[1:8]])
                    except ValueError:
                        pass
    return np.array(rows)  # t_us, lat, lon, sats, hdop, speed, course


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def detect_steps(t, a_body, still, fs):
    an = lowpass(np.linalg.norm(a_body, axis=1), fs, 3.0) - 1.0
    thr = max(0.08, 0.4 * np.percentile(np.abs(an[~still]), 95)) if (~still).any() else 0.1
    steps, last = [], -1.0
    for i in range(1, len(an) - 1):
        if an[i] > thr and an[i] >= an[i - 1] and an[i] > an[i + 1] and t[i] - last > 0.3:
            steps.append(i)
            last = t[i]
    return np.array(steps, int)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--step-len", type=float, default=0.7, help="початкова довжина кроку, м")
    ap.add_argument("--still", type=float, default=4.0, help="скільки секунд на початку стоїте на місці")
    ap.add_argument("--dropout", default="", help="START,DUR: вимкнути GPS на DUR с з моменту START (с від початку)")
    ap.add_argument("--no-vel", dest="vel", action="store_false",
                    help="не використовувати швидкість і курс GPS (тільки координати)")
    ap.add_argument("--min-sats", type=int, default=5)
    args = ap.parse_args()

    meta, c = load(args.csv)
    t0 = c["t_us"][0]
    t = (c["t_us"] - t0) / 1e6
    fs = 1.0 / np.median(np.diff(t))
    a_body = np.stack([c["ax_g"], c["ay_g"], c["az_g"]], 1)
    still = t < args.still
    a_body = a_body / np.linalg.norm(a_body[still], axis=1).mean()
    yaw = np.unwrap(np.radians(c["yaw_deg"]))
    steps = detect_steps(t, a_body, still, fs)
    st_t = t[steps]

    g = load_gps(args.csv)
    if len(g) < 5:
        raise SystemExit("У файлі майже немає рядків GPS. GPS під'єднаний і має фікс? (шукайте '# gps nofix' у файлі)")
    g = g[g[:, 3] >= args.min_sats]
    tg = (g[:, 0] - t0) / 1e6
    lat0, lon0 = np.radians(g[0, 1]), np.radians(g[0, 2])
    gE = (np.radians(g[:, 2]) - lon0) * np.cos(lat0) * R_EARTH
    gN = (np.radians(g[:, 1]) - lat0) * R_EARTH
    sig = np.maximum(2.0, 2.5 * g[:, 4])
    use = np.ones(len(g), bool)
    if args.dropout:
        d0, dd = (float(x) for x in args.dropout.split(","))
        use = ~((tg >= d0) & (tg < d0 + dd))

    # Початковий psi0: медіана (напрям руху GPS - курс гіро) по перших 6 відрізках по 5 с,
    # де пройдено > 4 м і курс гіро всередині відрізка майже не мінявся (ішли прямо)
    diffs = []
    for a in np.arange(tg[0], tg[-1] - 5, 2.5):
        i0, i1 = np.searchsorted(tg, [a, a + 5])
        i1 = min(i1, len(tg) - 1)
        if not use[i0] or np.hypot(gE[i1] - gE[i0], gN[i1] - gN[i0]) < 4.0:
            continue
        k = (t >= tg[i0]) & (t <= tg[i1])
        if np.ptp(yaw[k]) > np.radians(25):
            continue
        diffs.append(np.arctan2(gN[i1] - gN[i0], gE[i1] - gE[i0]) - yaw[k].mean())
        if len(diffs) >= 6:
            break
    if not diffs:
        raise SystemExit("Не знайшлося прямого відрізка GPS > 4 м за 5 с, курс не визначити")
    d = np.array(diffs)
    psi0 = np.angle(np.mean(np.exp(1j * d)))                     # круговий середній як старт
    psi0 = wrap(psi0 + np.median(wrap(d - psi0)))                # і медіана довкола нього
    # ---- Тільки IMU (кроки + гіро), стартовий курс і довжина кроку як у EKF до навчання ----
    pdr = [np.array([gE[0], gN[0]])]
    for i in steps:
        pdr.append(pdr[-1] + args.step_len * np.array([np.cos(yaw[i] + psi0), np.sin(yaw[i] + psi0)]))
    pdr = np.array(pdr)

    # ---- EKF ----
    x = np.array([gE[0], gN[0], psi0, args.step_len])
    P = np.diag([sig[0] ** 2, sig[0] ** 2, np.radians(20) ** 2, 0.2 ** 2])
    Q_step = np.diag([0.1 ** 2, 0.1 ** 2, np.radians(0.3) ** 2, 0.002 ** 2])  # на один крок
    H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], float)
    events = sorted([(t[i], 0, i) for i in steps] + [(tg[j], 1, j) for j in range(len(g))])
    track, nis, err_drop = [], [], []
    for te, kind, j in events:
        if kind == 0:  # крок
            th = yaw[j] + x[2]
            L = x[3]
            F = np.eye(4)
            F[0, 2], F[0, 3] = -L * np.sin(th), np.cos(th)
            F[1, 2], F[1, 3] = L * np.cos(th), np.sin(th)
            x[0] += L * np.cos(th)
            x[1] += L * np.sin(th)
            P = F @ P @ F.T + Q_step
            track.append((te, x[0], x[1]))
        else:          # GPS
            if not use[j]:
                err_drop.append(np.hypot(x[0] - gE[j], x[1] - gN[j]))
                continue
            Rm = np.eye(2) * sig[j] ** 2
            y = np.array([gE[j], gN[j]]) - H @ x
            S = H @ P @ H.T + Rm
            K = P @ H.T @ np.linalg.inv(S)
            nis.append(float(y @ np.linalg.solve(S, y)))
            x = x + K @ y
            x[2] = wrap(x[2])
            P = (np.eye(4) - K @ H) @ P
            # Швидкість і курс GPS (Доплер) точніші за різницю координат: коли йдемо, теж беремо їх
            if args.vel and g[j, 5] > 2.5:
                f_step = np.sum((st_t > te - 3.0) & (st_t <= te)) / 3.0   # кроків за секунду
                if f_step > 0.5:
                    # швидкість: v = L * f_step  ->  вчить довжину кроку
                    Hv = np.array([[0, 0, 0, f_step]])
                    yv = np.array([g[j, 5] / 3.6 - f_step * x[3]])
                    Sv = Hv @ P @ Hv.T + 0.3 ** 2
                    Kv = P @ Hv.T / Sv
                    x = x + (Kv * yv).ravel()
                    P = (np.eye(4) - Kv @ Hv) @ P
                    # курс: математичний курс = 90° - курс GPS (від півночі за годинниковою)
                    th = np.interp(te, t, yaw)
                    Hc = np.array([[0, 0, 1, 0]])
                    yc = np.array([wrap(np.radians(90 - g[j, 6]) - (th + x[2]))])
                    Sc = Hc @ P @ Hc.T + np.radians(15) ** 2
                    if yc[0] ** 2 / Sc[0, 0] < 9:           # відкидаємо явні викиди курсу GPS
                        Kc = P @ Hc.T / Sc
                        x = x + (Kc * yc).ravel()
                        x[2] = wrap(x[2])
                        P = (np.eye(4) - Kc @ Hc) @ P
            track.append((te, x[0], x[1]))
    track = np.array(track)

    # Довжина шляху за GPS: точки раз на 5 с, щоб не сумувати «тремтіння» нерухомого GPS
    sel = np.searchsorted(tg, np.arange(tg[0], tg[-1], 5.0))
    gps_len = np.sum(np.hypot(np.diff(gE[sel]), np.diff(gN[sel])))
    out = [
        f"Файл: {args.csv}",
        f"IMU {t[-1]:.0f} с, {fs:.0f} Гц, кроків {len(steps)}; GPS {len(g)} точок, "
        f"супутників {int(np.median(g[:, 3]))}, HDOP {np.median(g[:, 4]):.1f}",
        f"Шлях за GPS {gps_len:.0f} м, за кроками {len(steps) * args.step_len:.0f} м (крок {args.step_len} м)",
        "",
        "EKF навчився:",
        f"  довжина кроку {x[3]:.2f} м (було {args.step_len}) -> шлях за кроками {len(steps) * x[3]:.0f} м",
        f"  поправка курсу psi0 {np.degrees(x[2]):+.1f}° (старт {np.degrees(psi0):+.1f}°, "
        f"різниця = скільки гіро «з'їхав» за прогулянку)",
        f"  NIS середній {np.mean(nis):.2f} (норма ~2: якщо >> 2, шум GPS або кроку занижений)" if nis else "",
        f"  кінець: IMU-лише {np.hypot(*(pdr[-1] - [gE[-1], gN[-1]])):.1f} м від GPS, "
        f"EKF {np.hypot(track[-1, 1] - gE[-1], track[-1, 2] - gN[-1]):.1f} м",
    ]
    if args.dropout:
        out += ["", f"Провал GPS {args.dropout} с: {len(err_drop)} точок без GPS, "
                f"макс. відхилення EKF від GPS {max(err_drop) if err_drop else 0:.1f} м"]
    report = "\n".join(s for s in out if s is not None)
    print(report)
    base = args.csv[:-4]
    with open(base + "_gps.txt", "w") as f:
        f.write(report + "\n")

    fig, ax = plt.subplots(1, 2, figsize=(13, 5.5))
    ax[0].plot(gE, gN, "k.", ms=4, label="GPS")
    if args.dropout:
        ax[0].plot(gE[~use], gN[~use], "x", color="0.6", ms=5, label="GPS вимкнено (провал)")
    ax[0].plot(pdr[:, 0], pdr[:, 1], "C1-", lw=1, label="тільки IMU (кроки + гіро)")
    ax[0].plot(track[:, 1], track[:, 2], "C0-", lw=1.6, label="EKF: IMU + GPS")
    ax[0].plot(gE[0], gN[0], "go", ms=8, label="старт")
    ax[0].set(title="Траєкторія", xlabel="схід, м", ylabel="північ, м")
    ax[0].axis("equal"); ax[0].grid(alpha=.3); ax[0].legend(fontsize=8)
    # Курс з GPS: напрям зміщення за 5 с (по сусідніх точках раз на секунду це шум)
    dE, dN = np.diff(gE[sel]), np.diff(gN[sel])
    ok = np.hypot(dE, dN) > 3.0
    tc = 0.5 * (tg[sel][1:] + tg[sel][:-1])
    ax[1].plot(t, np.degrees(wrap(yaw + psi0)), "C1.", ms=1, label="курс гіро + стартова поправка")
    ax[1].plot(tc[ok], np.degrees(np.arctan2(dN, dE))[ok], "ko", ms=4, label="курс з руху GPS (за 5 с)")
    ax[1].set(title="Курс (математичний: 0° = схід, +90° = північ)", xlabel="t, с", ylabel="°", ylim=(-185, 185))
    ax[1].grid(alpha=.3); ax[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(base + "_gps.png", dpi=130)
    print(f"\nГрафік: {base}_gps.png\nЗвіт: {base}_gps.txt")


if __name__ == "__main__":
    main()
