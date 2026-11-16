#!/usr/bin/env python3
"""Графіки заїзду з CSV-логу.

    python3 tools/plot_log.py log003.csv           → вікно з графіками
    python3 tools/plot_log.py log003.csv out.png   → зберегти в файл

Потрібно: pip install pandas matplotlib
"""
import sys

import matplotlib.pyplot as plt
import pandas as pd


def wrap180(s):
    return (s + 180.0) % 360.0 - 180.0


def main() -> int:
    if len(sys.argv) not in (2, 3):
        print(__doc__)
        return 1
    df = pd.read_csv(sys.argv[1])
    t = (df["t_ms"] - df["t_ms"].iloc[0]) / 1000.0
    hold = df["mode"] == "HOLD"

    fig, ax = plt.subplots(4, 1, sharex=True, figsize=(12, 11))

    ax[0].set_title("Курс: помилка (задано − факт), °")
    err = wrap180(df["hdg_sp"] - df["hdg"]).where(hold)
    ax[0].plot(t, err, label="помилка курсу")
    ax[0].axhline(0, color="k", lw=0.5)
    ax[0].legend(loc="upper right")

    ax[1].set_title("Курс: складові PID і кермо")
    for col in ("hdg_p", "hdg_i", "hdg_d", "rudder"):
        ax[1].plot(t, df[col].where(hold), label=col)
    ax[1].axhline(1, color="r", lw=0.5, ls="--")
    ax[1].axhline(-1, color="r", lw=0.5, ls="--")
    ax[1].legend(loc="upper right")

    ax[2].set_title("Швидкість, м/с")
    ax[2].plot(t, df["spd"], label="GPS")
    ax[2].plot(t, df["spd_sp"].where(df["spd_hold"] == 1), label="задано", ls="--")
    ax[2].legend(loc="upper right")

    ax[3].set_title("Швидкість: складові PID і газ")
    for col in ("spd_p", "spd_i", "spd_d", "spd_ff", "throttle"):
        ax[3].plot(t, df[col], label=col)
    ax[3].legend(loc="upper right")
    ax[3].set_xlabel("час, с")

    # Підсвітити, де був HOLD
    for a in ax:
        a.fill_between(t, 0, 1, where=hold, transform=a.get_xaxis_transform(),
                       color="green", alpha=0.07)
        a.grid(alpha=0.3)

    fig.tight_layout()
    if len(sys.argv) == 3:
        fig.savefig(sys.argv[2], dpi=120)
        print(f"Збережено {sys.argv[2]}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
