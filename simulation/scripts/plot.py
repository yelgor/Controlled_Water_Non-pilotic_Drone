#!/usr/bin/env python3
"""Графіки й метрики з results/log_<tag>.csv -> results/plot_<tag>.png.

    python3 scripts/plot.py                          # найсвіжіший лог
    python3 scripts/plot.py results/log_circle.csv   # конкретний

Що на графіках - див. README.md, розділ "Як читати графіки".
"""
import glob
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INK, GREY, BLUE, ORANGE = '#222222', '#9a9a9a', '#2a78d6', '#eb6834'
T_MAX = 5.0
STEP_PSI = np.radians(30)  # стрибок уставки курсу, від якого рахуємо перехідний процес
STEP_U = 0.3               # те саме для швидкості, м/с
WINDOW = 12.0              # макс. тривалість аналізу одного перехідного процесу, с


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def rms(a):
    a = a[~np.isnan(a)]
    return float(np.sqrt(np.mean(a ** 2))) if a.size else float('nan')


def step_responses(t, ref, meas, threshold, angle=False):
    """Стрибки уставки -> [(t від стрибка, нормована відповідь y, метрики)].
    y = 1 - похибка / величина стрибка: 0 - стан до стрибка, 1 - уставку досягнуто."""
    diff = (lambda a: wrap(a)) if angle else (lambda a: a)
    jumps = np.where(np.abs(diff(np.diff(ref))) > threshold)[0] + 1
    out = []
    for n, k in enumerate(jumps):
        size = diff(ref[k] - ref[k - 1])
        if angle and abs(size) > np.radians(150):  # 180° - напрям повороту невизначений
            continue
        end = jumps[n + 1] if n + 1 < len(jumps) else len(t)
        end = min(end, np.searchsorted(t, t[k] + WINDOW))
        if t[end - 1] - t[k] < 4.0:  # наступний стрибок надто близько - не встиг усталитись
            continue
        tt = t[k:end] - t[k]
        y = 1 - diff(ref[k:end] - meas[k:end]) / size
        out.append((tt, y, response_metrics(tt, y)))
    return out


def response_metrics(tt, y):
    """Перерегулювання %, час наростання 10-90% с, час встановлення в ±5% с."""
    over = max(0.0, float(np.max(y)) - 1) * 100
    i10, i90 = np.argmax(y >= 0.1), np.argmax(y >= 0.9)
    rise = tt[i90] - tt[i10] if y[i90] >= 0.9 else np.nan
    outside = np.where(np.abs(1 - y) > 0.05)[0]
    settle = tt[outside[-1]] if outside.size else 0.0
    if outside.size and outside[-1] == len(y) - 1:
        settle = np.nan  # так і не встановилось у ±5%
    return over, rise, settle


def mean_metrics(resp):
    m = np.array([r[2] for r in resp])
    return [np.nanmean(c) if np.any(~np.isnan(c)) else np.nan for c in m.T] if len(m) else [np.nan] * 3


def latest_log():
    logs = glob.glob(os.path.join(ROOT, 'results', 'log_*.csv'))
    if not logs:
        sys.exit('немає results/log_*.csv - спочатку запусти симуляцію')
    return max(logs, key=os.path.getmtime)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else latest_log()
    tag = os.path.basename(path)[4:-4] if os.path.basename(path).startswith('log_') else 'run'
    d = np.genfromtxt(path, delimiter=',', names=True)
    d = d[~np.isnan(d['kf_x'])]
    jumps = np.where(np.diff(d['t']) < 0)[0]
    if jumps.size:
        # після Reset світу катер тоне (плавучість у gz-sim 8 не переживає Reset) - дані без сенсу
        print(f'У лозі був Reset світу ({jumps.size} раз(и)) - малюю лише до першого')
        d = d[:jumps[0] + 1]
    t = d['t'] - d['t'][0]
    auto = np.any(~np.isnan(d['u_ref']))

    # ---------- похибки фільтра (відносно ground truth) ----------
    e_kf = np.hypot(d['kf_x'] - d['gt_x'], d['kf_y'] - d['gt_y'])
    g = ~np.isnan(d['gps_x'])
    e_gps = np.hypot(d['gps_x'][g] - d['gt_x'][g], d['gps_y'][g] - d['gt_y'][g])
    e_psi = np.degrees(wrap(d['kf_psi'] - d['gt_psi']))
    nis_ok = ~np.isnan(d['gps_nis'])
    nis = d['gps_nis'][nis_ok]

    # ---------- якість руху (по ground truth - як катер рухався насправді) ----------
    ex, ey = d['seg_bx'] - d['seg_ax'], d['seg_by'] - d['seg_ay']
    seg_len = np.hypot(ex, ey)
    with np.errstate(invalid='ignore', divide='ignore'):
        xte = (-(d['gt_x'] - d['seg_ax']) * ey + (d['gt_y'] - d['seg_ay']) * ex) / seg_len
    e_track_psi = np.degrees(wrap(d['psi_ref'] - d['gt_psi']))
    psi_steps = step_responses(t, d['psi_ref'], d['gt_psi'], STEP_PSI, angle=True) if auto else []
    u_steps = step_responses(t, d['u_ref'], d['gt_u'], STEP_U) if auto else []
    sat = np.mean((np.abs(d['thrust_l']) > T_MAX - 0.05) | (np.abs(d['thrust_r']) > T_MAX - 0.05)) * 100

    fig = plt.figure(figsize=(22, 13))
    gs = fig.add_gridspec(3, 4)
    ax_traj = fig.add_subplot(gs[0:2, 0])
    ax = {k: fig.add_subplot(gs[r, c]) for k, (r, c) in {
        'pos': (0, 1), 'psi_err': (0, 2), 'bias': (0, 3),
        'u': (1, 1), 'psi': (1, 2), 'steps': (1, 3),
        'xte': (2, 0), 'thrust': (2, 1), 'nis': (2, 2), 'text': (2, 3)}.items()}

    a = ax_traj
    if auto:
        segs = np.unique(np.column_stack([d['seg_ax'], d['seg_ay'], d['seg_bx'], d['seg_by']]), axis=0)
        segs = segs[~np.isnan(segs).any(axis=1)]
        for i, (x1, y1, x2, y2) in enumerate(segs):
            a.plot([x1, x2], [y1, y2], '--', color=INK, lw=1, alpha=0.6, label='маршрут' if i == 0 else None)
    a.plot(d['gps_x'][g], d['gps_y'][g], '.', ms=2, color=GREY, label='GPS (шум)')
    a.plot(d['gt_x'], d['gt_y'], color=INK, lw=2, label='ground truth')
    a.plot(d['kf_x'], d['kf_y'], color=BLUE, lw=1, label='EKF')
    a.plot(d['gt_x'][0], d['gt_y'][0], 'o', color=INK, ms=8, label='старт')
    a.set(title=f'Траєкторія ({tag})', xlabel='x, м', ylabel='y, м')
    a.set_aspect('equal', adjustable='datalim')
    a.legend(loc='best', fontsize=9)

    a = ax['pos']
    a.plot(t[g], e_gps, '.', ms=2, color=GREY, label=f'GPS, RMS {rms(e_gps):.2f} м')
    a.plot(t, e_kf, color=BLUE, label=f'EKF, RMS {rms(e_kf):.2f} м')
    a.plot(t, 2 * np.hypot(d['kf_sx'], d['kf_sy']), '--', color=BLUE, lw=0.8, label='EKF: 2σ (власна оцінка)')
    a.set(title='KF: похибка положення', xlabel='t, с', ylabel='м')
    a.legend(fontsize=9)

    a = ax['psi_err']
    two_sigma = 2 * np.degrees(d['kf_spsi'])
    a.fill_between(t, -two_sigma, two_sigma, color=BLUE, alpha=0.15, lw=0, label='±2σ (власна оцінка)')
    a.plot(t, e_psi, color=BLUE, lw=1, label=f'EKF - truth, RMS {rms(e_psi):.2f}°')
    a.set(title='KF: похибка курсу', xlabel='t, с', ylabel='град')
    a.legend(fontsize=9)

    a = ax['bias']
    a.axhline(0.03, color=INK, ls='--', lw=1, label='справжній зсув (sensors_noise.py)')
    a.plot(t, d['kf_bias'], color=BLUE, lw=1, label='оцінка EKF')
    a.set(title='KF: зсув гіроскопа', xlabel='t, с', ylabel='рад/с')
    a.legend(fontsize=9)

    a = ax['u']
    if auto:
        a.plot(t, d['u_ref'], '--', color=INK, lw=1, label='u_ref (уставка)')
    a.plot(t, d['gt_u'], color=INK, lw=1.5, label=f'u (truth), похибка RMS {rms(d["u_ref"] - d["gt_u"]):.2f} м/с'
           if auto else 'u (truth)')
    a.plot(t, d['kf_u'], color=BLUE, lw=0.8, label='u (EKF)')
    a.set(title='PID швидкості', xlabel='t, с', ylabel='м/с')
    a.legend(fontsize=9)

    a = ax['psi']
    psi_gt = np.unwrap(d['gt_psi'])  # без стрибків на ±180°; решту - відносно gt
    if auto:
        a.plot(t, np.degrees(psi_gt + wrap(d['psi_ref'] - psi_gt)), '--', color=INK, lw=1, label='ψ_ref (уставка)')
    a.plot(t, np.degrees(psi_gt), color=INK, lw=1.5, label=f'ψ (truth), похибка RMS {rms(e_track_psi):.1f}°'
           if auto else 'ψ (truth)')
    a.plot(t, np.degrees(psi_gt + wrap(d['kf_psi'] - psi_gt)), color=BLUE, lw=0.8, label='ψ (EKF)')
    a.set(title='PID курсу', xlabel='t, с', ylabel='град')
    a.legend(fontsize=9)

    a = ax['steps']
    a.axhspan(0.95, 1.05, color=INK, alpha=0.08, lw=0, label='±5% (встановлення)')
    a.axhline(1, color=INK, lw=0.8)
    for i, (tt, y, _) in enumerate(psi_steps):
        a.plot(tt, y, color=BLUE, lw=1, alpha=0.7, label='курс' if i == 0 else None)
    for i, (tt, y, _) in enumerate(u_steps):
        a.plot(tt, y, color=ORANGE, lw=1, alpha=0.7, label='швидкість' if i == 0 else None)
    a.set(title='Перехідні процеси: стрибки уставки (1 = досягнуто)', xlabel='t від стрибка, с',
          ylabel='частка стрибка', ylim=(-0.3, 1.6))
    if psi_steps or u_steps:
        a.legend(fontsize=9)
    else:
        a.text(0.5, 0.5, 'стрибків уставки немає', ha='center', transform=a.transAxes, color=GREY)

    a = ax['xte']
    if auto and np.any(~np.isnan(xte)):
        a.plot(t, xte, color=INK, lw=1, label=f'RMS {rms(xte):.2f} м, макс {np.nanmax(np.abs(xte)):.2f} м')
        a.axhline(0, color=INK, lw=0.5)
        a.legend(fontsize=9)
    else:
        a.text(0.5, 0.5, 'немає маршруту (steps / ручне)', ha='center', transform=a.transAxes, color=GREY)
    a.set(title='Бокове відхилення від маршруту (truth)', xlabel='t, с', ylabel='м, >0 - зліва')

    a = ax['thrust']
    a.plot(t, d['thrust_l'], color=BLUE, lw=0.8, label='T_l')
    a.plot(t, d['thrust_r'], color=ORANGE, lw=0.8, label='T_r')
    for s in (-T_MAX, T_MAX):
        a.axhline(s, color=INK, ls=':', lw=1)
    a.set(title=f'Тяга рушіїв (в межі ±5 Н {sat:.0f}% часу)', xlabel='t, с', ylabel='Н')
    a.legend(fontsize=9)

    a = ax['nis']
    a.plot(t[nis_ok], nis, '.', ms=2, color=GREY, label='NIS кожного GPS')
    if nis.size > 25:
        a.plot(t[nis_ok], np.convolve(nis, np.ones(25) / 25, mode='same'), color=BLUE, lw=1.5,
               label=f'ковзне середнє 5 с (усього {nis.mean():.2f})')
    a.axhline(2, color=INK, lw=1, label='очікуване середнє 2')
    a.axhline(5.99, color=INK, ls=':', lw=1, label=f'межа 95% (вище: {np.mean(nis > 5.99) * 100:.0f}%)')
    a.set(title='KF: інновації GPS (NIS) - можна і без truth', xlabel='t, с', ylabel='NIS', ylim=(0, 12))
    a.legend(fontsize=9)

    for a in [ax_traj, *ax.values()]:
        a.grid(alpha=0.3)

    po, pr, ps = mean_metrics(psi_steps)
    uo, ur, us = mean_metrics(u_steps)
    lines = [
        f'Прогін: {tag}, {t[-1]:.0f} с',
        '',
        'ФІЛЬТР КАЛМАНА',
        f'  положення RMS: GPS {rms(e_gps):.2f} м -> EKF {rms(e_kf):.2f} м',
        f'  курс RMS {rms(e_psi):.2f}°, швидкість RMS {rms(d["kf_u"] - d["gt_u"]):.3f} м/с',
        f'  NIS середнє {nis.mean():.2f} (норма ≈2), >5.99: {np.mean(nis > 5.99) * 100:.0f}% (норма ≈5%)',
        '',
        'РЕГУЛЯТОРИ',
    ]
    if auto:
        lines += [
            f'  стеження: швидкість RMS {rms(d["u_ref"] - d["gt_u"]):.2f} м/с, курс RMS {rms(e_track_psi):.1f}°',
            f'  бокове відхилення RMS {rms(xte):.2f} м' if np.any(~np.isnan(xte)) else '',
            f'  тяга в насиченні {sat:.0f}% часу',
            f'  курс, {len(psi_steps)} стрибків: перерегулювання {po:.0f}%,',
            f'     наростання {pr:.1f} с, встановлення {ps:.1f} с',
            f'  швидкість, {len(u_steps)} стрибків: перерегулювання {uo:.0f}%,',
            f'     наростання {ur:.1f} с, встановлення {us:.1f} с',
        ]
    else:
        lines.append('  ручне керування - уставок немає')
    a = ax['text']
    a.axis('off')
    a.text(0, 1, '\n'.join(lines), va='top', family='monospace', fontsize=10, transform=a.transAxes)

    fig.tight_layout()
    out = os.path.join(os.path.dirname(path), f'plot_{tag}.png')
    fig.savefig(out, dpi=100)
    print(out)
    print('\n'.join(line for line in lines if line))


if __name__ == '__main__':
    main()
