#!/usr/bin/env python3
"""Машинка / катер з IMU + GPS: EKF без кроків, рух зі швидкості.

    python3 tools/gps_imu_drive.py data/imu_..._drive.csv [--dropout 60,20]

Плата має бути жорстко закріплена на машинці (як на катері). У CSV мають бути рядки GPS.

Стан EKF: [E, N, v, psi0]
  E, N  положення, м (схід, північ) від першої точки GPS
  v     швидкість уздовж курсу, м/с
  psi0  поправка курсу: справжній курс = yaw гіро + psi0 (гіро не знає, де північ, і повільно пливе)
Прогноз (10 Гц):   E += v cos(yaw + psi0) dt,   N += v sin(yaw + psi0) dt,   v і psi0 майже сталі
Оновлення від GPS: координати (шум max(2 м, 2.5*HDOP)), швидкість (0.3 м/с),
                   курс GPS, коли їдемо швидше 1,5 м/с (15°)
--dropout START,DUR: вимкнути GPS на DUR с (як під мостом): що буде на самому гіро + останній швидкості.
--start-end: ви повернулись у точку старту, тоді відстань кінець–старт = похибка (ground truth).
Пише <ім'я>_drive.png і <ім'я>_drive.txt поруч з CSV.
"""
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from analyze_imu import load
from gps_imu_walk import load_gps, wrap, R_EARTH


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--dropout", default="", help="START,DUR: вимкнути GPS на DUR с з моменту START (с від початку)")
    ap.add_argument("--start-end", action="store_true", help="фініш у точці старту: рахувати похибку замикання")
    ap.add_argument("--min-sats", type=int, default=5)
    ap.add_argument("--rate", type=float, default=10.0, help="частота прогнозу EKF, Гц")
    args = ap.parse_args()

    meta, c = load(args.csv)
    t0 = c["t_us"][0]
    t = (c["t_us"] - t0) / 1e6
    yaw = np.unwrap(np.radians(c["yaw_deg"]))

    g = load_gps(args.csv)
    if len(g) < 5:
        raise SystemExit("У файлі майже немає рядків GPS. GPS під'єднаний і має фікс?")
    g = g[g[:, 3] >= args.min_sats]
    tg = (g[:, 0] - t0) / 1e6
    lat0, lon0 = np.radians(g[0, 1]), np.radians(g[0, 2])
    gE = (np.radians(g[:, 2]) - lon0) * np.cos(lat0) * R_EARTH
    gN = (np.radians(g[:, 1]) - lat0) * R_EARTH
    gv = g[:, 5] / 3.6
    gcrs = np.radians(90.0 - g[:, 6])          # курс GPS (від півночі за годинниковою) -> математичний
    sig = np.maximum(2.0, 2.5 * g[:, 4])
    use = np.ones(len(g), bool)
    if args.dropout:
        d0, dd = (float(x) for x in args.dropout.split(","))
        use = ~((tg >= d0) & (tg < d0 + dd))

    # Початковий psi0: курс GPS мінус курс гіро там, де їдемо швидко й прямо
    yg = np.interp(tg, t, yaw)
    rate_ok = np.abs(np.gradient(yg, tg)) < np.radians(10)
    k = use & (gv > 1.5) & rate_ok
    if k.sum() < 3:
        raise SystemExit("Немає ділянок, де машинка їде прямо швидше 1,5 м/с: курс не визначити")
    d = (gcrs - yg)[k][:10]
    psi0_start = wrap(np.angle(np.mean(np.exp(1j * d))))

    # ---- EKF ----
    x = np.array([gE[0], gN[0], 0.0, psi0_start])
    P = np.diag([sig[0] ** 2, sig[0] ** 2, 1.0, np.radians(10) ** 2])
    q = np.array([0.05, 0.05, 1.0, np.radians(0.1)]) ** 2      # на 1 с: положення, швидкість (прискорення), курс
    dt = 1.0 / args.rate
    tp = np.arange(0, t[-1], dt)
    yp = np.interp(tp, t, yaw)
    events = sorted([(tp[i], 0, i) for i in range(len(tp))] + [(tg[j], 1, j) for j in range(len(g))])

    def update(x, P, H, y, R):
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        return x + K @ y, (np.eye(4) - K @ H) @ P, float(y @ np.linalg.solve(S, y))

    track, nis, err_drop, psi_hist = [], [], [], []
    for te, kind, j in events:
        if kind == 0:
            th = yp[j] + x[3]
            v = x[2]
            F = np.eye(4)
            F[0, 2], F[0, 3] = np.cos(th) * dt, -v * np.sin(th) * dt
            F[1, 2], F[1, 3] = np.sin(th) * dt, v * np.cos(th) * dt
            x[0] += v * np.cos(th) * dt
            x[1] += v * np.sin(th) * dt
            P = F @ P @ F.T + np.diag(q) * dt
            track.append((te, x[0], x[1], x[2]))
            psi_hist.append((te, x[3]))
            continue
        if not use[j]:
            err_drop.append(np.hypot(x[0] - gE[j], x[1] - gN[j]))
            continue
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], float)
        x, P, e = update(x, P, H, np.array([gE[j], gN[j]]) - x[:2], np.eye(2) * sig[j] ** 2)
        nis.append(e)
        H = np.array([[0, 0, 1, 0]], float)
        x, P, _ = update(x, P, H, np.array([gv[j] - x[2]]), np.eye(1) * 0.3 ** 2)
        if gv[j] > 1.5:
            th = np.interp(tg[j], t, yaw)
            H = np.array([[0, 0, 0, 1]], float)
            y = np.array([wrap(gcrs[j] - th - x[3])])
            S = P[3, 3] + np.radians(15) ** 2
            if y[0] ** 2 / S < 9:
                x, P, _ = update(x, P, H, y, np.eye(1) * np.radians(15) ** 2)
        x[3] = wrap(x[3])
    track = np.array(track)
    psi_hist = np.array(psi_hist)

    sel = np.searchsorted(tg, np.arange(tg[0], tg[-1], 5.0))
    gps_len = np.sum(np.hypot(np.diff(gE[sel]), np.diff(gN[sel])))
    drift = np.degrees(wrap(psi_hist[-1, 1] - psi_hist[0, 1]))
    out = [
        f"Файл: {args.csv}",
        f"IMU {t[-1]:.0f} с; GPS {len(g)} точок, супутників {int(np.median(g[:, 3]))}, HDOP {np.median(g[:, 4]):.1f}",
        f"Шлях за GPS {gps_len:.0f} м, макс. швидкість {gv.max():.1f} м/с",
        "",
        f"Поправка курсу psi0: старт {np.degrees(psi0_start):+.1f}°, кінець {np.degrees(x[3]):+.1f}° "
        f"-> гіро з'їхав на {drift:+.1f}° за {t[-1] / 60:.1f} хв",
        f"NIS середній {np.mean(nis):.2f} (норма ~2)",
        f"Кінець: EKF {np.hypot(track[-1, 1] - gE[-1], track[-1, 2] - gN[-1]):.1f} м від останньої точки GPS",
    ]
    if args.start_end:
        # Точка старту = середнє GPS, поки машинка ще стоїть (один відлік GPS шумить на 2–3 м)
        i_mv = np.argmax(gv > 1.0)
        s0 = np.array([gE[:max(i_mv, 1)].mean(), gN[:max(i_mv, 1)].mean()])
        out.append(f"Замикання (фініш = старт, старт усереднено по {max(i_mv, 1)} точках GPS): "
                   f"GPS {np.hypot(gE[-1] - s0[0], gN[-1] - s0[1]):.1f} м, "
                   f"EKF {np.hypot(track[-1, 1] - s0[0], track[-1, 2] - s0[1]):.1f} м")
    if args.dropout:
        out += [f"Провал GPS {args.dropout} с: {len(err_drop)} точок без GPS, "
                f"макс. відхилення EKF від GPS {max(err_drop) if err_drop else 0:.1f} м"]
    report = "\n".join(out)
    print(report)
    base = args.csv[:-4]
    with open(base + "_drive.txt", "w") as f:
        f.write(report + "\n")

    fig, ax = plt.subplots(1, 2, figsize=(13, 5.5))
    ax[0].plot(gE, gN, "k.", ms=4, label="GPS")
    if args.dropout:
        ax[0].plot(gE[~use], gN[~use], "x", color="0.6", ms=5, label="GPS вимкнено (провал)")
    ax[0].plot(track[:, 1], track[:, 2], "C0-", lw=1.6, label="EKF: гіро + GPS")
    ax[0].plot(gE[0], gN[0], "go", ms=8, label="старт")
    ax[0].set(title="Траєкторія", xlabel="схід, м", ylabel="північ, м")
    ax[0].axis("equal"); ax[0].grid(alpha=.3); ax[0].legend(fontsize=8)
    mv = gv > 1.5
    ax[1].plot(psi_hist[:, 0], np.degrees(wrap(np.interp(psi_hist[:, 0], t, yaw) + psi_hist[:, 1])),
               "C0.", ms=1, label="курс EKF (гіро + поправка)")
    ax[1].plot(tg[mv], np.degrees(wrap(gcrs[mv])), "k.", ms=4, label="курс GPS (коли їде > 1,5 м/с)")
    ax[1].set(title="Курс (математичний: 0° = схід, +90° = північ)", xlabel="t, с", ylabel="°", ylim=(-185, 185))
    ax[1].grid(alpha=.3); ax[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(base + "_drive.png", dpi=130)
    print(f"\nГрафік: {base}_drive.png\nЗвіт: {base}_drive.txt")


if __name__ == "__main__":
    main()
