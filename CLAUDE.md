# Controlled Water Non-pilotic Drone

Водний дрон-аероглісер з автоматичним утриманням курсу і швидкості на PID-регуляторах.
Пілот керує з пульта ELRS; тумблер «HOLD» вмикає режим, у якому човен сам тримає
поточний курс (і, за бажанням, задану швидкість).

Поточний обсяг: тільки PID (курс + швидкість). Руху по точках і автоповернення поки немає.

## Апаратна частина

| Елемент | Модель | Підключення (див. `firmware/include/config.h`) |
|---|---|---|
| Контролер | ESP32-S3 N16R8 (16 МБ flash, 8 МБ PSRAM) | — |
| IMU + компас | LSM9DS1 або NXP FXOS8700+FXAS21002 (визначається автоматично) | I2C: SDA 8, SCL 9 |
| GPS | GY-GPS6MV2 (u-blox NEO-6M), 5 Гц | UART1: RX 16, TX 15 |
| Пульт | ELRS 2.4G приймач (BAYCKRC), протокол CRSF 420000 бод | UART2: RX 18, TX 17 |
| Рушій | Безщітковий мотор + **повітряний** пропелер (не реверсивний) | — |
| ESC | Skywalker 40A V2 (односпрямований, PWM 1000–2000 мкс) | GPIO 5 |
| Кермо | Серво в потоці повітря за пропелером | GPIO 4 |
| Живлення | UBEC | — |

Особливості:
- Аероглісер: кермо працює тільки коли пропелер дме. На малому газу інтеграл курсу заморожується.
- Курс береться з компаса (не з GPS), бо аероглісер ковзає боком.
- Компас треба калібрувати на зібраному човні (`cal start` / `cal stop`), модуль — подалі від мотора і силових дротів.
- Канали пульта: CH1 кермо, CH3 газ, CH5 ARM, CH6 HOLD.

## Структура

```
firmware/
  platformio.ini        env:esp32s3 (прошивка) + env:native (тести на ПК, Unity)
  partitions.csv        16 МБ: app 3 МБ + LittleFS ~13 МБ під логи
  include/config.h      піни, канали пульта, частоти
  src/                  код, що працює тільки на ESP32 (залізо)
    main.cpp            тільки збирає модулі разом
    drivers/            imu (LSM9DS1/FXOS8700), gps_neo6m, rc_crsf, actuators
    storage/            param_store (NVS), flight_log (LittleFS)
    console/            serial_console (команди з ноутбука)
  lib/                  логіка без заліза — тестується на ПК
    control/            pid, angle, heading_controller, speed_controller, mixer
    nav/                heading_math (компас з компенсацією нахилу), mag_calibration
    rc/                 crsf_parser
    core/               params, vehicle (режими, безпека, з'єднує регулятори)
    tuning/             command_parser, param_table
    telemetry/          csv (формат рядка логу)
  test/                 тести Unity, по папці на модуль
docs/pid-tuning.md      як налаштовувати PID на воді
tools/                  get_log.py (злити лог), plot_log.py (графіки)
```

Правило: усе, що можна перевірити без заліза, живе в `lib/` і має тести в `test/`.
`src/` — тонкий шар над залізом.

## Вимоги до PID

- реальний dt (з `millis()`), dt ≤ 0 → вихід не змінюється, великий dt → обрізається до `maxDt`, D скидається;
- derivative on measurement + НЧ-фільтр D;
- anti-windup (умовне інтегрування + обмеження інтеграла);
- обмеження виходу і slew rate;
- `reset()`, bumpless transfer (`initialize()`), зміна коефіцієнтів без ривка;
- NaN/Inf на вході → безпечний вихід;
- без динамічної пам'яті.

## Безпека

- `thr.max` (за замовчуванням 0.30) обмежує газ у всіх режимах;
- ARM тільки при газі на нулі; без ARM мотор не крутиться;
- втрата пульта > 0.5 с → газ 0, кермо в центр, DISARM;
- HOLD неможливий без компаса; без GPS швидкість береться зі стіка;
- watchdog 2 с.

## Правила роботи

- Відповіді й резюме — українською.
- Не комітити, не пушити і не прошивати залізо без дозволу користувача.
- Після змін: `pio test -e native` зелений і `pio run -e esp32s3` збирається.

## Команди

Усі з папки `firmware/`:

```bash
pio test -e native            # тести на ПК
pio run -e esp32s3            # зібрати прошивку
pio run -e esp32s3 -t upload  # прошити (тільки з дозволу!)
pio device monitor            # serial-консоль, 115200
```

Serial-консоль (на березі, по USB): `help`, `status`, `get`, `set <назва> <число>`,
`save`, `load`, `defaults`, `cal start|stop`, `log list|dump <n>|clear`, `tele on|off`, `i2c`.
Консоль — через вбудований USB чипа (`/dev/cu.usbmodem…`).

Логи: `python3 tools/get_log.py <порт> <n>` → `logNNN.csv`, `python3 tools/plot_log.py logNNN.csv`.
