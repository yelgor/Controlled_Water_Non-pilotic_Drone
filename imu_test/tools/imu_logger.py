#!/usr/bin/env python3
"""Запис потоку CSV з ESP32-S3 (imu-test) у файл з міткою часу в назві.

Приклади:
    python3 tools/imu_logger.py                       # автопошук порту, пише до Ctrl+C
    python3 tools/imu_logger.py --tag static -d 600   # статика 10 хв
    python3 tools/imu_logger.py --tag turn90 --mag    # поворот, режим 9DOF

Файл: data/imu_YYYYmmdd_HHMMSS[_tag].csv. Рядки з '#' це коментарі прошивки
(зміщення гіро, режим). Для pandas: pd.read_csv(path, comment='#').
Після запису друкує короткий підсумок: частоту, середнє й СКВ гіро, дрейф курсу.
"""
import argparse
import glob
import math
import os
import sys
import time
from datetime import datetime

try:
    import serial
except ImportError:
    sys.exit("Потрібен pyserial:  python3 -m pip install --user pyserial  (або sudo apt install python3-serial)")

HEADER_START = "t_us,"


def find_port():
    ports = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
    if not ports:
        sys.exit("Порт не знайдено. Плата підключена? Перевірте: ls /dev/ttyACM* /dev/ttyUSB*")
    return ports[0]


def reset_board(ser):
    # Як esptool: EN (RTS) униз і вгору, GPIO0 (DTR) не чіпаємо. Працює через USB-UART міст.
    ser.dtr = False
    ser.rts = True
    time.sleep(0.1)
    ser.rts = False


def summarize(path, _print=print):
    cols = None
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            if line.startswith(HEADER_START):
                cols = line.strip().split(",")
                continue
            parts = line.strip().split(",")
            if cols and len(parts) == len(cols):
                try:
                    rows.append([float(x) for x in parts])
                except ValueError:
                    pass
    if not cols or len(rows) < 10:
        _print("Підсумок: замало даних")
        return
    idx = {c: i for i, c in enumerate(cols)}
    t = [r[idx["t_us"]] for r in rows]
    dur = (t[-1] - t[0]) / 1e6
    if dur <= 0:
        _print("Підсумок: некоректні часові мітки")
        return
    _print(f"\nПідсумок: {len(rows)} рядків, {dur:.1f} с, {len(rows) / dur:.1f} Гц")
    for ax in ("gx_dps", "gy_dps", "gz_dps"):
        v = [r[idx[ax]] for r in rows]
        m = sum(v) / len(v)
        sd = math.sqrt(sum((x - m) ** 2 for x in v) / len(v))
        _print(f"  {ax}: середнє {m:+.4f} °/с, СКВ {sd:.4f} °/с")
    # Дрейф курсу: різниця yaw кінця й початку з урахуванням переходу через ±180°
    yaw = [r[idx["yaw_deg"]] for r in rows]
    total = 0.0
    for a, b in zip(yaw, yaw[1:]):
        d = b - a
        d -= 360.0 * round(d / 360.0)
        total += d
    _print(f"  yaw: зміна {total:+.2f}°, дрейф {total / dur * 60:+.2f} °/хв")
    calc = [r[idx["calc_us"]] for r in rows]
    _print(f"  calc_us: середнє {sum(calc) / len(calc):.1f}, макс {max(calc):.0f}")
    gaps = sum(1 for a, b in zip(t, t[1:]) if b - a > 2.5 * dur * 1e6 / len(rows))
    if gaps:
        _print(f"  УВАГА: {gaps} пропусків у часових мітках (втрата рядків?)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", help="серійний порт (за замовчуванням перший /dev/ttyACM* або /dev/ttyUSB*)")
    ap.add_argument("-b", "--baud", type=int, default=921600)
    ap.add_argument("-d", "--duration", type=float, default=0, help="тривалість, с (0 = до Ctrl+C)")
    ap.add_argument("-t", "--tag", default="", help="назва сценарію в імені файлу: static, turn90, ...")
    ap.add_argument("-o", "--outdir", default="data")
    ap.add_argument("--every", type=int, default=1, help="писати кожен N-й рядок (10 = ~24 Гц замість 238)")
    ap.add_argument("--mag", action="store_true", help="перемкнути прошивку в 9DOF (з магнітометром)")
    ap.add_argument("--no-reset", action="store_true", help="не перезавантажувати плату на старті")
    args = ap.parse_args()

    port = args.port or find_port()
    os.makedirs(args.outdir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = f"imu_{stamp}" + (f"_{args.tag}" if args.tag else "") + ".csv"
    path = os.path.join(args.outdir, name)

    ser = serial.Serial(port, args.baud, timeout=1)
    if not args.no_reset:
        reset_board(ser)  # кожен файл починається з калібрування й заголовка
    else:
        ser.write(b"h")   # попросити заголовок у вже запущеної прошивки
    print(f"Порт {port} @ {args.baud}, запис у {path}")
    print("Плата має лежати нерухомо перші 7 с (калібрування). Ctrl+C зупиняє запис.")

    n = 0
    n_in = 0
    n_gps = 0
    header_seen = False
    mag_sent = False
    t_start = time.time()
    t_report = t_start
    n_report = 0
    try:
        with open(path, "w", buffering=1) as f:
            f.write(f"# logger_start,{datetime.now().isoformat(timespec='seconds')},port={port},tag={args.tag}\n")
            while True:
                if args.duration and header_seen and time.time() - t_start >= args.duration:
                    break
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                if line.startswith("#"):
                    print(line)
                    f.write(line + "\n")
                    continue
                if line.startswith(HEADER_START):
                    if args.mag and not mag_sent:
                        ser.write(b"m")  # прошивка надрукує новий заголовок з mode,9dof
                        mag_sent = True
                        continue
                    if not header_seen:
                        f.write(line + "\n")
                        header_seen = True
                        t_start = time.time()
                    continue
                if line.startswith("GPS,"):
                    if header_seen:
                        f.write(line + "\n")
                        n_gps += 1
                    continue
                if not header_seen:
                    continue  # дані до заголовка (шматок після ресету) пропускаємо
                if not line[0].isdigit():
                    print("\n" + line)  # службові повідомлення ядра ([E] Wire...), не дані
                    continue
                n_in += 1
                if (n_in - 1) % args.every:
                    continue
                f.write(line + "\n")
                n += 1
                now = time.time()
                if now - t_report >= 1.0:
                    rate = (n - n_report) / (now - t_report)
                    last = line.split(",")
                    angles = ",".join(last[10:13]) if len(last) >= 13 else "?"
                    print(f"\r{n:8d} рядків  {rate:6.1f} Гц  GPS {n_gps}  roll,pitch,yaw = {angles}      ", end="", flush=True)
                    t_report, n_report = now, n
    except KeyboardInterrupt:
        pass
    except serial.SerialException as e:
        print(f"\nПорт закрився: {e}")
    finally:
        ser.close()
    print(f"\nЗбережено {n} рядків IMU і {n_gps} рядків GPS у {path}")
    lines = []
    summarize(path, lambda *a: (print(*a), lines.append(" ".join(str(x) for x in a))))
    txt = path[:-4] + "_summary.txt"
    with open(txt, "w") as f:
        f.write("\n".join(lines).strip() + "\n")
    print(f"Підсумок збережено у {txt}")


if __name__ == "__main__":
    main()
