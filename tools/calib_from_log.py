#!/usr/bin/env python3
"""Калібрування компаса з логу заїзду: катер робить кола на воді — скрипт рахує mag.off/scl X,Y.

    python3 tools/calib_from_log.py logNNN.csv            → показати результат і перевірку
    python3 tools/calib_from_log.py logNNN.csv --apply    → ще й записати в плату (set + save)

Потрібні колонки mx,my,mz,dx,dy,dz,cog (прошивка з листопада 2026).

Як це працює:
  1. Беремо рядки, де катер майже рівний (нахил < 8°) — на воді так майже завжди.
  2. Точки (mx, my) при обертанні лягають на коло/еліпс; центр = зміщення (залізо катера),
     сплюснутість = масштаб. Z не чіпаємо — на рівному катері він на курс майже не впливає
     (у FXOS8700 зміщення Z ~−1.3 Гс — власне зміщення чипа, лишаємо те, що в платі).
  3. Перевірка: на прямих ділянках зі швидкістю > 1 м/с курс компаса має збігатися з курсом
     руху за GPS. Різниця окремо для малого/середнього/великого газу — так видно вплив струму
     в силових дротах на компас.
"""
import csv
import glob
import math
import sys
from pathlib import Path

MAX_TILT = 0.14  # sin(8°)


def solve(a, b):
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-12:
            raise ValueError("вироджена система — катер не робив повних кіл?")
        m[c], m[p] = m[p], m[c]
        for r in range(n):
            if r != c:
                f = m[r][c] / m[c][c]
                for k in range(c, n + 1):
                    m[r][k] -= f * m[c][k]
    return [m[i][n] / m[i][i] for i in range(n)]


def fit_ellipse_xy(pts):
    """A x² + B y² + D x + E y = 1 (осі еліпса вздовж x/y), з попереднім центруванням."""
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    ata = [[0.0] * 4 for _ in range(4)]
    atb = [0.0] * 4
    for px, py in pts:
        x, y = px - cx, py - cy
        row = [x * x, y * y, x, y]
        for i in range(4):
            atb[i] += row[i]
            for j in range(4):
                ata[i][j] += row[i] * row[j]
    A, B, D, E = solve(ata, atb)
    if A <= 0 or B <= 0:
        raise ValueError("не еліпс — замало обертання")
    x0, y0 = -D / (2 * A), -E / (2 * B)
    g = 1 + A * x0 * x0 + B * y0 * y0
    rx, ry = math.sqrt(g / A), math.sqrt(g / B)
    avg = (rx + ry) / 2
    return (x0 + cx, y0 + cy), (avg / rx, avg / ry), avg


def heading(m, d, off, scl, offz, sclz):
    mx, my, mz = (m[0] - off[0]) * scl[0], (m[1] - off[1]) * scl[1], (m[2] - offz) * sclz
    dx, dy, dz = d
    roll = math.atan2(dy, dz)
    sr, cr = math.sin(roll), math.cos(roll)
    pitch = math.atan2(-dx, dy * sr + dz * cr)
    sp, cp = math.sin(pitch), math.cos(pitch)
    bx = mx * cp + my * sp * sr + mz * sp * cr
    by = mz * sr - my * cr
    return math.degrees(math.atan2(by, bx)) % 360


def angdiff(a, b):
    return (a - b + 180) % 360 - 180


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    apply = "--apply" in sys.argv
    offz = -1.3745  # замінюється значенням з плати при --apply
    sclz = 1.0
    if len(args) != 1:
        print(__doc__)
        return 1
    rows = list(csv.DictReader(Path(args[0]).open(encoding="utf-8")))
    if not rows or "mx" not in rows[0]:
        print("У лозі немає колонок mx/my/mz — його записала стара прошивка.")
        return 1

    data = []
    for r in rows:
        try:
            m = (float(r["mx"]), float(r["my"]), float(r["mz"]))
            d = (float(r["dx"]), float(r["dy"]), float(r["dz"]))
        except (ValueError, KeyError):
            continue
        if r.get("imu_ok") != "1" or (m == (0.0, 0.0, 0.0)):
            continue
        data.append({"m": m, "d": d, "thr": float(r["throttle"]), "spd": float(r["spd"]),
                     "cog": float(r["cog"]), "rud": float(r["rudder"]), "gps": r.get("gps_ok") == "1"})
    level = [p for p in data if abs(p["d"][0]) < MAX_TILT and abs(p["d"][1]) < MAX_TILT]
    print(f"рядків: {len(rows)}, з компасом: {len(data)}, рівних (нахил < 8°): {len(level)}")
    if len(level) < 200:
        print("Замало даних. Потрібно 2–3 повних кола.")
        return 1

    off, scl, radius = fit_ellipse_xy([p["m"][:2] for p in level])
    # Покриття: скільки секторів по 30° пройшов вектор поля навколо знайденого центру.
    sectors = {int(math.degrees(math.atan2(p["m"][1] - off[1], p["m"][0] - off[0])) % 360 // 30) for p in level}
    rr = [math.hypot((p["m"][0] - off[0]) * scl[0], (p["m"][1] - off[1]) * scl[1]) for p in level]
    mean_r = sum(rr) / len(rr)
    spread = math.sqrt(sum((x - mean_r) ** 2 for x in rr) / len(rr)) / mean_r * 100
    print(f"зміщення X/Y:  {off[0]:+.4f} {off[1]:+.4f} Гс")
    print(f"масштаб X/Y:   {scl[0]:.4f} {scl[1]:.4f}")
    print(f"горизонт. поле: {radius:.3f} Гс (поле Землі тут ≈ 0.2), розкид {spread:.1f} % (добре < 5)")
    print(f"покриття кола: {len(sectors)}/12 секторів (потрібно ≥ 10)")

    # Перевірка за GPS: прямо (кермо < 0.15), швидкість > 1 м/с, є курс GPS.
    bins = {"газ < 25%": [], "газ 25–45%": [], "газ > 45%": []}
    for p in data:
        if not p["gps"] or p["spd"] < 1.0 or p["cog"] < 0 or abs(p["rud"]) > 0.15:
            continue
        err = angdiff(heading(p["m"], p["d"], off, scl, offz, sclz), p["cog"])
        key = "газ < 25%" if p["thr"] < 0.25 else ("газ 25–45%" if p["thr"] < 0.45 else "газ > 45%")
        bins[key].append(err)
    print("перевірка: компас мінус курс GPS на прямих (> 1 м/с):")
    for k, v in bins.items():
        if len(v) < 10:
            print(f"  {k:11s}: замало точок ({len(v)})")
            continue
        mu = math.degrees(math.atan2(sum(math.sin(math.radians(e)) for e in v),
                                     sum(math.cos(math.radians(e)) for e in v)))
        sd = math.sqrt(sum(angdiff(e, mu) ** 2 for e in v) / len(v))
        print(f"  {k:11s}: середнє {mu:+.0f}°, розкид ±{sd:.0f}°  ({len(v)} точок)")
    print("  (однакове середнє на всіх газах = кут встановлення/схилення; різне = заважає струм)")

    problems = []
    if len(sectors) < 10:
        problems.append("катер не зробив повного кола")
    if spread > 8:
        problems.append("великий розкид — поле змінюється (струм, рух дротів?)")
    if not all(-2 < o < 2 for o in off) or not all(0.2 < k < 5 for k in scl):
        problems.append("значення поза межами параметрів")
    if problems:
        print("НЕ годиться для запису:\n  - " + "\n  - ".join(problems))
        return 1
    if not apply:
        print("Щоб записати в плату: додай --apply (плата по USB, пульт вимкнений).")
        return 0

    import time
    import serial
    port = (sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*")) + glob.glob("/dev/cu.usbmodem*") or [None])[0]
    if not port:
        print("плату не знайдено")
        return 1
    s = serial.Serial()
    s.port, s.baudrate, s.timeout, s.dtr, s.rts = port, 115200, 0.1, False, False
    s.open()
    time.sleep(4)
    s.reset_input_buffer()

    def cmd(c, wait=0.5):
        s.write((c + "\n").encode())
        end, out = time.time() + wait, b""
        while time.time() < end:
            out += s.read(4096)
        print(f">>> {c}\n{out.decode('utf-8', 'replace').strip()}")

    # sclz = 1: масштаб X/Y нормовано до 1 в середньому, Z має бути в тих самих одиницях.
    for name, val in (("offx", off[0]), ("offy", off[1]), ("sclx", scl[0]), ("scly", scl[1]), ("sclz", 1.0)):
        cmd(f"set mag.{name} {val:.4f}")
    cmd("save", 1.0)
    s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
