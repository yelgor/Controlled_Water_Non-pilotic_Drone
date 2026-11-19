#!/usr/bin/env python3
"""Калібрування акселерометра за 6 положеннями.

    python3 tools/calib_accel6.py data/imu_..._accel6.csv

Як записати: плата в коробці, кладемо коробку по черзі на 6 граней (Z вгору, Z вниз, X вгору,
X вниз, Y вгору, Y вниз), на кожній грані тримаємо нерухомо ~10 с, між гранями перевертаємо.
Скрипт сам знаходить спокійні відрізки, групує їх за тим, яка вісь дивиться вгору чи вниз, і рахує:
    зміщення  o = (a_вгору + a_вниз) / 2
    масштаб   k = 2 / (a_вгору − a_вниз)
    виправлене a = (a_сире − o) · k        (на кожній грані має стати рівно ±1 g)
Друкує рядки для main.cpp і перевірку |a| до/після.
"""
import sys
import numpy as np
from analyze_imu import load

WIN = 1.0          # с, вікно для пошуку спокою
STILL_STD = 0.01   # g, у спокої акселерометр тремтить менше за це
MIN_WIN = 3        # мінімум спокійних вікон на грань


def main(path):
    meta, c = load(path)
    t = (c["t_us"] - c["t_us"][0]) / 1e6
    fs = 1 / np.median(np.diff(t))
    a = np.stack([c["ax_g"], c["ay_g"], c["az_g"]], 1)
    n = int(WIN * fs)
    groups = {}                                   # (вісь, знак) -> список середніх
    for k in range(len(a) // n):
        w = a[k * n:(k + 1) * n]
        if w.std(0).max() > STILL_STD:
            continue                              # плату рухали
        m = w.mean(0)
        ax = int(np.argmax(np.abs(m)))
        if abs(m[ax]) < 0.8:
            continue                              # нахилена, не на грані
        groups.setdefault((ax, int(np.sign(m[ax]))), []).append(m)

    names = "XYZ"
    print(f"Файл: {path}\nЗнайдено спокійних вікон по {WIN:.0f} с:")
    for ax in range(3):
        for s in (+1, -1):
            print(f"  {names[ax]} {'вгору' if s > 0 else 'вниз '}: {len(groups.get((ax, s), []))}")
    missing = [f"{names[ax]}{'+' if s > 0 else '-'}" for ax in range(3) for s in (1, -1)
               if len(groups.get((ax, s), [])) < MIN_WIN]
    if missing:
        sys.exit(f"\nБракує граней (менше {MIN_WIN} с спокою): {', '.join(missing)}. Перезапишіть ці грані.")

    off, scl = np.zeros(3), np.ones(3)
    print("\nВісь   вгору, g   вниз, g   зміщення, g   масштаб")
    for ax in range(3):
        up = np.median([m[ax] for m in groups[(ax, 1)]])
        dn = np.median([m[ax] for m in groups[(ax, -1)]])
        off[ax] = (up + dn) / 2
        scl[ax] = 2 / (up - dn)
        print(f"  {names[ax]}   {up:+.4f}   {dn:+.4f}    {off[ax]:+.4f}      {scl[ax]:.4f}")

    print("\nПеревірка |a| на кожній грані (має бути 1,000):")
    for (ax, s), ms in sorted(groups.items()):
        ms = np.array(ms)
        before = np.linalg.norm(ms, axis=1).mean()
        after = np.linalg.norm((ms - off) * scl, axis=1).mean()
        print(f"  {names[ax]}{'+' if s > 0 else '-'}: до {before:.4f} g   після {after:.4f} g")

    print("\nВставити в src/main.cpp (замість нулів і одиниць):")
    print(f"static float accOffset[3] = {{{off[0]:.5f}f, {off[1]:.5f}f, {off[2]:.5f}f}};")
    print(f"static float accScale[3]  = {{{scl[0]:.5f}f, {scl[1]:.5f}f, {scl[2]:.5f}f}};")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1])
