#pragma once
// Розбір текстових команд з serial. Тільки парсинг — виконання в src/console.
//
//   help                 список команд
//   status               режим, сенсори, курс, швидкість
//   get [назва]          показати всі параметри або один
//   set <назва> <число>  змінити параметр (діє одразу, без перепрошивки)
//   save | load | defaults   зберегти у flash / прочитати / заводські
//   hdg <градуси>        змінити заданий курс (тільки в HOLD)
//   cal start | cal stop калібрування компаса
//   log list | log dump <n> | log clear
//   tele on | tele off   живий CSV у serial (для стенду)
//   i2c                  які пристрої бачить шина I2C (перевірка компаса)
//   servo                прогнати серво 1100→1500→1900→1500 мкс (тільки DISARMED)

#include <cstddef>

namespace tuning {

enum class CmdType {
    Empty,
    Help,
    Status,
    Get,
    Set,
    Save,
    Load,
    Defaults,
    Heading,
    CalStart,
    CalStop,
    LogList,
    LogDump,
    LogClear,
    TeleOn,
    TeleOff,
    I2cScan,
    ServoTest,
    BadArgs,
    Unknown,
};

struct Command {
    CmdType type = CmdType::Empty;
    char name[24] = {0};
    float value = 0.0f;
    bool hasName = false;
};

Command parseCommand(const char* line);

}  // namespace tuning
