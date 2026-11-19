#!/usr/bin/env python3
"""Калібрування компаса (магнітометра IMU) катера по USB.

Працює з основною прошивкою катера (гілка feature/pid-control, firmware/):
керує консольними командами `cal start` / `cal stop` / `save`, які вже є в прошивці.

    python3 tools/calib_imu.py                    # порт знайде сам, крутити 60 с
    python3 tools/calib_imu.py -p /dev/ttyACM0    # явно вказати порт
    python3 tools/calib_imu.py -t 90              # крутити 90 с
    python3 tools/calib_imu.py --no-save          # не записувати у flash (тільки перевірити)

Перед запуском:
  * катер зібраний повністю (батарея, дроти, мотор на місцях) — залізо поруч і є те,
    що ми калібруємо;
  * DISARM (тумблер ARM вниз), інакше прошивка відмовить;
  * подалі від ноутбука, телефона, металевих столів, арматури;
  * закрийте `pio device monitor`, бо він тримає порт.

Під час запису: тримайте катер РІВНО і повільно покрутіть 2–3 повні кола (360°).
Нахиляти НЕ треба: вбудований `cal` (min/max) від нахилів псується (docs/changes-2026-11-18.md).
Калібрування з нахилами (еліпсоїд) — tools/calib_mag.py; на воді з логу — tools/calib_from_log.py.
Enter — закінчити раніше.

pyserial уже є в PlatformIO (~/.platformio/penv/bin/python); інакше: pip install pyserial
"""
import argparse
import select
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("Немає pyserial: pip install pyserial  (або запустіть через ~/.platformio/penv/bin/python)")

BAUD = 115200


def find_port() -> str:
    ports = list(list_ports.comports())
    # ESP32-S3 з нативним USB (ARDUINO_USB_CDC_ON_BOOT=1) → /dev/ttyACM*, USB-UART міст → /dev/ttyUSB*
    for pref in ("/dev/ttyACM", "/dev/ttyUSB", "/dev/cu.usbmodem", "/dev/cu.usbserial", "COM"):
        for p in ports:
            if p.device.startswith(pref):
                return p.device
    sys.exit("Плату не знайдено. Підключіть USB або вкажіть порт: -p /dev/ttyACM0  (список: pio device list)")


def read_lines(s: serial.Serial, quiet: float = 0.6, total: float = 3.0) -> list[str]:
    """Читати рядки, поки плата не замовкне на `quiet` с (але не довше `total` с)."""
    lines, end, last = [], time.time() + total, time.time()
    while time.time() < end and time.time() - last < quiet:
        raw = s.readline()
        if raw:
            last = time.time()
            line = raw.decode(errors="replace").rstrip()
            if line:
                lines.append(line)
    return lines


def cmd(s: serial.Serial, text: str, **kw) -> list[str]:
    s.reset_input_buffer()
    s.write((text + "\n").encode())
    s.flush()
    return read_lines(s, **kw)


def enter_pressed() -> bool:
    try:
        r, _, _ = select.select([sys.stdin], [], [], 0)
    except (OSError, ValueError):  # Windows / немає stdin
        return False
    if r:
        sys.stdin.readline()
        return True
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description="Калібрування компаса катера через консоль прошивки")
    ap.add_argument("-p", "--port", help="серійний порт (за замовчуванням — знайти самому)")
    ap.add_argument("-t", "--time", type=int, default=60, help="скільки секунд крутити (60)")
    ap.add_argument("--no-save", action="store_true", help="не робити save у flash")
    args = ap.parse_args()

    port = args.port or find_port()
    print(f"Порт: {port}")
    s = serial.Serial()
    s.port, s.baudrate, s.timeout, s.dtr, s.rts = port, BAUD, 0.2, False, False
    try:
        s.open()
    except serial.SerialException as e:
        sys.exit(f"Не відкрився порт: {e}\nЗакрийте pio device monitor / інші програми на цьому порту.")

    with s:
        print("Чекаю старт плати (~4 с)...")
        time.sleep(4)  # плата перезавантажується при відкритті порту
        cmd(s, "tele off", total=0.8)  # прибрати живий CSV, щоб не заважав

        st = cmd(s, "status")
        if not st:
            print("Плата мовчить. Це прошивка катера (feature/pid-control)? Швидкість 115200?")
            return 1
        print("\n".join("  " + l for l in st))
        if any("компас: НЕМА" in l for l in st):
            print("\nУВАГА: прошивка не бачить компас. Перевірте командою `i2c` у моніторі порту.")
            if input("Все одно продовжити? [y/N] ").strip().lower() != "y":
                return 1

        r = cmd(s, "cal start")
        print("\n".join(r))
        if not any(l.startswith("OK") for l in r):
            print("Калібрування не запустилось (DISARM? тумблер ARM вниз)")
            return 1

        print(f"\n>>> КРУТІТЬ КАТЕР {args.time} с: рівно, 2–3 повні кола по горизонталі, без нахилів."
              " Enter — закінчити раніше.\n")
        t0 = time.time()
        try:
            while (left := args.time - (time.time() - t0)) > 0:
                if enter_pressed():
                    break
                print(f"\r  лишилось {left:4.0f} с ", end="", flush=True)
                s.read(s.in_waiting or 1)  # не давати буферу переповнюватись
        except KeyboardInterrupt:
            print("\nПерервано — все одно завершую калібрування")
        print()

        r = cmd(s, "cal stop")
        print("\n".join(r) if r else "Немає відповіді на 'cal stop'")
        if not any(l.startswith("OK") for l in r):
            print("\nНе вийшло. Крутіть довше і робіть повне коло, далі від заліза. Запустіть ще раз.")
            return 1

        if args.no_save:
            print("\n--no-save: калібрування діє до перезапуску, у flash не записано.")
        elif input("\nЗберегти у flash? [Y/n] ").strip().lower() in ("", "y", "yes", "т", "так"):
            print("\n".join(cmd(s, "save")))

        print("\nПараметри компаса зараз:")
        for name in ("mag.offx", "mag.offy", "mag.offz", "mag.sclx", "mag.scly", "mag.sclz", "mag.decl"):
            for l in cmd(s, f"get {name}", quiet=0.3, total=1.0):
                print("  " + l)
        print("\nПеревірка: у моніторі порту `status` — курс має мінятись рівно, повне коло = 360°.")
        print("Магнітне схилення (для України ~ +6..+9°): set mag.decl <градуси>  →  save")
    return 0


if __name__ == "__main__":
    sys.exit(main())
