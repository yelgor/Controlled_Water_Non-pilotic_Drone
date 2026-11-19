#!/usr/bin/env python3
"""Злити лог заїзду з човна по USB.

    python3 tools/get_log.py list                  → список логів (порт знайде сам)
    python3 tools/get_log.py 3                     → log003.csv
    python3 tools/get_log.py all                   → усі логи з плати
    python3 tools/get_log.py /dev/ttyACM0 3        → явно вказати порт (Mac: /dev/cu.usbmodem…)

Чому лог качається кілька разів: прошивка має Serial.setTxTimeoutMs(20) (src/main.cpp),
і при `log dump` USB-CDC викидає шматки даних — один прохід дає лише 30–80 % рядків.
Тому скрипт повторює `log dump`, бере тільки цілі рядки і склеює їх за t_ms, поки обсяг не
зрівняється з розміром файлу на платі (з BEGIN_LOG). У flash дані цілі — губиться лише передача.

pyserial уже є в PlatformIO; інакше: pip install pyserial
Порт можна знайти командою: pio device list
"""
import glob
import re
import sys
import time

import serial

MAX_PASSES = 12      # скільки разів максимум перекачувати один лог
ENOUGH = 0.995       # частка розміру файлу, після якої вважаємо лог повним
MODES = {"DISARMED", "MANUAL", "HOLD", "FAILSAFE", "RTH", "AUTO"}


def find_port():
    ports = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*")) + glob.glob("/dev/cu.usbmodem*")
    return ports[0] if ports else None


def send(s, text):
    # Після паузи плата іноді губить перший символ команди — порожній рядок перед командою це прибирає.
    s.write(b"\n")
    time.sleep(0.1)
    s.reset_input_buffer()
    s.write((text + "\n").encode())


def read_until_quiet(s, quiet=1.0, total=5.0):
    out, end, last = [], time.time() + total, time.time()
    while time.time() < end and time.time() - last < quiet:
        line = s.readline().decode(errors="replace").rstrip()
        if line:
            out.append(line)
            last = time.time()
    return out


def dump_once(s, n):
    """Один прохід `log dump n` → (назва, розмір на платі, заголовок, рядки) або None."""
    send(s, f"log dump {n}")
    name, size, lines = None, 0, []
    deadline = time.time() + 10
    while time.time() < deadline:
        raw = s.readline()
        if not raw:
            continue
        deadline = time.time() + 10
        line = raw.decode(errors="replace").rstrip("\r\n")
        m = re.search(r"BEGIN_LOG (\S+) (\d+)", line)
        if m:
            name, size = m.group(1), int(m.group(2))
            continue
        if "END_LOG" in line:
            break
        if line.startswith("ERR"):
            print("  " + line)
            return None
        if name:
            lines.append(line)
    if not name:
        return None
    return name, size, lines


def is_header(line):
    return line.startswith("t_ms,")


def valid_row(line, ncols):
    f = line.split(",")
    if len(f) != ncols or f[1] not in MODES:
        return False
    try:
        int(f[0])
        for v in f[2:]:
            float(v)
    except ValueError:
        return False
    return True


def fetch(s, n):
    header, rows, name, size = None, {}, None, 0
    for p in range(1, MAX_PASSES + 1):
        r = dump_once(s, n)
        if r is None:
            if p == 1:
                return None
            continue
        name, size, lines = r
        for ln in lines:
            if header is None and is_header(ln):
                header = ln
        if header is None:
            continue
        ncols = header.count(",") + 1
        for ln in lines:
            if not is_header(ln) and valid_row(ln, ncols):
                rows.setdefault(int(ln.split(",", 1)[0]), ln)
        got = len(header) + 1 + sum(len(v) + 1 for v in rows.values())
        print(f"  {name}: прохід {p}, рядків {len(rows)}, {got}/{size} байт ({100 * got / max(size, 1):.0f} %)")
        if got >= ENOUGH * size:
            break
    if header is None:
        print(f"  {name}: не вдалося отримати заголовок")
        return None
    with open(name, "w", encoding="utf-8") as f:
        f.write(header + "\n")
        for t in sorted(rows):
            f.write(rows[t] + "\n")
    got = len(header) + 1 + sum(len(v) + 1 for v in rows.values())
    note = "" if got >= ENOUGH * size else "  ← НЕПОВНИЙ, запустіть ще раз"
    print(f"Збережено {name}: {len(rows)} рядків{note}")
    return name


def main() -> int:
    if len(sys.argv) == 2:
        port, what = find_port(), sys.argv[1]
        if not port:
            print("Плату не знайдено (/dev/ttyACM*). Підключена? Вкажіть порт: get_log.py /dev/ttyACM0 list")
            return 1
    elif len(sys.argv) == 3:
        port, what = sys.argv[1], sys.argv[2]
    else:
        print(__doc__)
        return 1
    print(f"Порт: {port}")
    s = serial.Serial()
    s.port, s.baudrate, s.timeout, s.dtr, s.rts = port, 115200, 2, False, False
    try:
        s.open()
    except serial.SerialException as e:
        print(f"Не відкрився порт: {e}\nЗакрийте pio device monitor (порт може тримати лише одна програма).")
        return 1
    with s:
        time.sleep(4)  # плата перезавантажується при відкритті порту — чекаємо старт
        s.reset_input_buffer()
        if what in ("list", "all"):
            send(s, "log list")
            listing = read_until_quiet(s)
            if what == "list":
                print("\n".join(listing))
                return 0
            nums = [int(m.group(1)) for ln in listing if (m := re.match(r"\s*(\d+)\s+\d+ байт", ln))]
            if not nums:
                print("Логів не знайдено:\n" + "\n".join(listing))
                return 1
            ok = sum(fetch(s, n) is not None for n in nums)
            print(f"Готово: {ok} з {len(nums)} логів")
            return 0 if ok == len(nums) else 1
        if fetch(s, int(what)) is None:
            print("Не отримав лог. Номер є в `list`? Пульт вимкнений (DISARM)?")
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
