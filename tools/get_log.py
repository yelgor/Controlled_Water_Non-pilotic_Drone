#!/usr/bin/env python3
"""Злити лог заїзду з човна по USB.

    python3 tools/get_log.py /dev/cu.usbserial-XXXX 3      → log003.csv
    python3 tools/get_log.py /dev/cu.usbserial-XXXX list   → список логів

pyserial уже є в PlatformIO; інакше: pip install pyserial
Порт можна знайти командою: pio device list
"""
import sys
import time

import serial


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 1
    port, what = sys.argv[1], sys.argv[2]
    with serial.Serial(port, 115200, timeout=2) as s:
        time.sleep(0.3)
        s.reset_input_buffer()
        if what == "list":
            s.write(b"log list\n")
            end = time.time() + 2
            while time.time() < end:
                line = s.readline().decode(errors="replace").rstrip()
                if line:
                    print(line)
            return 0

        s.write(f"log dump {int(what)}\n".encode())
        name, data = None, []
        deadline = time.time() + 10
        while time.time() < deadline:
            line = s.readline().decode(errors="replace")
            if not line:
                continue
            deadline = time.time() + 10
            if line.startswith("BEGIN_LOG"):
                name = line.split()[1]
                continue
            if line.startswith("END_LOG"):
                break
            if line.startswith("ERR"):
                print(line.strip())
                return 1
            if name:
                data.append(line)
        if not name:
            print("Не отримав лог. Човен підключений? DISARM? (log list)")
            return 1
        with open(name, "w", encoding="utf-8") as f:
            f.writelines(data)
        print(f"Збережено {name}: {len(data) - 1} рядків")
    return 0


if __name__ == "__main__":
    sys.exit(main())
