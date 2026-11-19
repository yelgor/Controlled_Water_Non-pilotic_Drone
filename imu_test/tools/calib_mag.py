#!/usr/bin/env python3
"""Калібрування магнітометра (hard iron + soft iron по осях).

Запис: python3 tools/imu_logger.py --tag magcal
       ~60 с повільно крутіть плату в усі боки («вісімки», догори дриґом, на ребро),
       щоб вона подивилась у всі напрямки. Подалі від заліза, ноутбука й телефона.
       Плату кріпіть так, як вона стоятиме на машинці / катері, разом з усім, що поруч (батарея, дроти).
Аналіз: python3 tools/calib_mag.py data/imu_..._magcal.csv

Ідея: чисте поле Землі має однакову довжину в будь-якому положенні, тож точки (mx, my, mz)
лежать на сфері з центром у нулі. Залізо поруч зсуває центр (hard iron) і сплющує сферу в
еліпсоїд (soft iron). Підганяємо еліпсоїд a x² + b y² + c z² + d x + e y + f z = 1 і
перетворюємо його назад у сферу:  m_cal = (m - offset) * scale.
"""
import sys
import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from analyze_imu import load  # noqa: E402


def fit(m):
    x, y, z = m.T
    D = np.stack([x * x, y * y, z * z, x, y, z], 1)
    p, *_ = np.linalg.lstsq(D, np.ones(len(m)), rcond=None)
    a, b, c, d, e, f = p
    o = np.array([-d / (2 * a), -e / (2 * b), -f / (2 * c)])
    G = 1 + a * o[0] ** 2 + b * o[1] ** 2 + c * o[2] ** 2
    radii = np.sqrt(G / np.array([a, b, c]))
    return o, radii


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    path = sys.argv[1]
    meta, c = load(path)
    m = np.stack([c["mx_gauss"], c["my_gauss"], c["mz_gauss"]], 1)
    keep = np.r_[True, np.any(np.diff(m, axis=0) != 0, axis=1)]   # магнітометр повільніший: прибираємо повтори
    m = m[keep]
    if len(m) < 200 or np.all(m == 0):
        sys.exit("Замало даних магнітометра (у файлі нулі? прошивка має читати магнітометр)")

    o, radii = fit(m)
    if not np.all(np.isfinite(radii)):
        sys.exit("Еліпсоїд не підігнався: плата повертали замало. Повторіть, крутіть у всі боки.")
    r = radii.mean()
    scale = r / radii
    mc = (m - o) * scale
    n_raw = np.linalg.norm(m - m.mean(0), axis=1)
    n_cal = np.linalg.norm(mc, axis=1)

    # Покриття: які напрямки бачила плата (ділимо сферу на 26 секторів)
    u = mc / n_cal[:, None]
    dirs = np.array([[i, j, k] for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1) if (i, j, k) != (0, 0, 0)], float)
    dirs /= np.linalg.norm(dirs, axis=1)[:, None]
    seen = np.unique(np.argmax(u @ dirs.T, axis=1)).size

    print(f"Файл: {path}")
    print(f"Унікальних відліків магнітометра: {len(m)}")
    print(f"Покриття напрямків: {seen} з 26 секторів" + ("  (мало, крутіть більше)" if seen < 20 else ""))
    print()
    print("Вісь   зміщення, Гс   радіус, Гс   масштаб")
    for i, ax in enumerate("XYZ"):
        print(f"  {ax}    {o[i]:+8.4f}     {radii[i]:7.4f}    {scale[i]:7.4f}")
    print()
    print(f"Довжина поля після калібрування: {n_cal.mean():.3f} Гс, розкид {n_cal.std():.4f} Гс "
          f"({100 * n_cal.std() / n_cal.mean():.1f} %)")
    print("  Земля в Україні ~0,48–0,50 Гс. Сильно більше або розкид > 5 % = поруч залізо чи струм.")
    print()
    print("Вставити в src/main.cpp (замість нулів і одиниць):")
    print(f"static float magOffset[3] = {{{o[0]:.5f}f, {o[1]:.5f}f, {o[2]:.5f}f}};")
    print(f"static float magScale[3]  = {{{scale[0]:.5f}f, {scale[1]:.5f}f, {scale[2]:.5f}f}};")


if __name__ == "__main__":
    main()
