#!/usr/bin/env python3
"""Калібрування магнітометра на ноутбуці: сирі дані з плати → еліпсоїд → mag.off*/mag.scl* у плату.

    python3 tools/calib_mag.py [порт] [секунди]        (за замовчуванням: перший /dev/cu.usbmodem*, 60 с)

Поки йде запис — крутити плату/катер В УСІ БОКИ (повні оберти, нахили, догори дном). На відміну від
вбудованого `cal` (min/max), тут підбирається еліпсоїд за всіма точками, тож нахили не шкодять.
Результат записується в плату (`set` + `save`) лише якщо якість нормальна.

Потрібен pyserial (є в ~/.platformio/penv/bin/python). Плата перезапускається при відкритті порту —
тому все робиться в одному сеансі.
"""
import glob
import math
import sys
import time

import serial

EARTH_G = 0.5  # гаус, Україна: повне поле ≈ 0.5 Гс


def solve(a, b):
    """Розв'язати a·x = b (Гаусс з вибором головного елемента)."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-12:
            raise ValueError("вироджена система — замало різних положень")
        m[c], m[p] = m[p], m[c]
        for r in range(n):
            if r != c:
                f = m[r][c] / m[c][c]
                for k in range(c, n + 1):
                    m[r][k] -= f * m[c][k]
    return [m[i][n] / m[i][i] for i in range(n)]


def fit_ellipsoid(pts):
    """Еліпсоїд з осями вздовж x/y/z: A x² + B y² + C z² + D x + E y + F z = 1 (МНК).

    Спершу віднімаємо середнє: інакше при зміщенні, більшому за радіус (у FXOS8700 по Z ~1.3 Гс),
    права частина рівняння стає від'ємною і нормування на 1 ламається."""
    mx = [sum(p[i] for p in pts) / len(pts) for i in range(3)]
    ata = [[0.0] * 6 for _ in range(6)]
    atb = [0.0] * 6
    for px, py, pz in pts:
        x, y, z = px - mx[0], py - mx[1], pz - mx[2]
        row = [x * x, y * y, z * z, x, y, z]
        for i in range(6):
            atb[i] += row[i]
            for j in range(6):
                ata[i][j] += row[i] * row[j]
    A, B, C, D, E, F = solve(ata, atb)
    if min(A, B, C) <= 0:
        raise ValueError("не еліпсоїд — замало обертів або магніт поруч")
    c = (-D / (2 * A), -E / (2 * B), -F / (2 * C))  # центр відносно середнього
    g = 1 + A * c[0] ** 2 + B * c[1] ** 2 + C * c[2] ** 2
    off = (c[0] + mx[0], c[1] + mx[1], c[2] + mx[2])
    radii = (math.sqrt(g / A), math.sqrt(g / B), math.sqrt(g / C))
    avg = sum(radii) / 3
    scale = tuple(avg / r for r in radii)
    return off, scale, avg


def coverage(pts, off):
    """Скільки з 26 напрямків (куб 3×3×3 без центру) покрито точками — грубо «наскільки повна куля»."""
    seen = set()
    for x, y, z in pts:
        v = (x - off[0], y - off[1], z - off[2])
        n = math.sqrt(sum(c * c for c in v)) or 1
        key = tuple(int(round(c / n * 1.2)) for c in v)  # -1/0/+1 по кожній осі
        if key != (0, 0, 0):
            seen.add(key)
    return len(seen)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else (glob.glob("/dev/cu.usbmodem*") or [None])[0]
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    if not port:
        sys.exit("плату не знайдено (/dev/cu.usbmodem*)")
    s = serial.Serial()
    s.port, s.baudrate, s.timeout, s.dtr, s.rts = port, 115200, 0.1, False, False
    s.open()
    time.sleep(4.0)  # плата стартує після відкриття порту
    s.reset_input_buffer()

    def cmd(c, wait=1.0):
        s.write((c + "\n").encode())
        end, out = time.time() + wait, b""
        while time.time() < end:
            out += s.read(4096)
        txt = out.decode("utf-8", "replace").strip()
        print(f">>> {c}\n{txt}", flush=True)
        return txt

    print(f"Запис {dur:.0f} с — крутіть плату в усі боки...", flush=True)
    s.write(b"raw on\n")
    pts, buf, end = [], b"", time.time() + dur
    while time.time() < end:
        buf += s.read(4096)
        *lines, buf = buf.split(b"\n")
        for ln in lines:
            if ln.startswith(b"RAW,"):
                try:
                    v = [float(t) for t in ln[4:].decode().split(",")]
                    pts.append(tuple(v[:3]))
                except ValueError:
                    pass
    s.write(b"raw off\n")
    time.sleep(0.5)
    s.reset_input_buffer()
    print(f"Точок: {len(pts)}")
    if len(pts) < 300:
        sys.exit("замало точок — чи прошита плата з командою 'raw'?")

    off, scl, radius = fit_ellipsoid(pts)
    cal = [((x - off[0]) * scl[0], (y - off[1]) * scl[1], (z - off[2]) * scl[2]) for x, y, z in pts]
    mags = [math.sqrt(a * a + b * b + c * c) for a, b, c in cal]
    mean = sum(mags) / len(mags)
    spread = math.sqrt(sum((m - mean) ** 2 for m in mags) / len(mags)) / mean * 100
    cov = coverage(pts, off)
    print(f"зміщення, Гс:  {off[0]:+.4f} {off[1]:+.4f} {off[2]:+.4f}")
    print(f"масштаб:       {scl[0]:.4f} {scl[1]:.4f} {scl[2]:.4f}")
    print(f"радіус кулі:   {radius:.3f} Гс (поле Землі ≈ {EARTH_G} Гс)")
    print(f"розкид |B|:    {spread:.1f} %   (добре < 3 %, погано > 6 %)")
    print(f"покриття:      {cov}/26 напрямків (добре ≥ 18)")

    problems = []
    if spread > 6:
        problems.append("великий розкид — магніт/залізо поруч або рухали дроти")
    if cov < 12:
        problems.append("мало положень — крутіть у всі боки, включно з догори дном")
    if not all(-2 < o < 2 for o in off) or not all(0.2 < k < 5 for k in scl):
        problems.append("значення поза межами параметрів прошивки")
    if problems:
        print("НЕ записано в плату:\n  - " + "\n  - ".join(problems))
        s.close()
        return 1
    if cov < 18:
        print("Увага: покриття неповне, але записую — для курсу найважливіші горизонтальні оберти.")
    for name, val in zip(("offx", "offy", "offz", "sclx", "scly", "sclz"), (*off, *scl)):
        cmd(f"set mag.{name} {val:.4f}", 0.4)
    cmd("save")
    cmd("imu", 1.5)
    s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
