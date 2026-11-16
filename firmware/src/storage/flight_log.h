#pragma once
// Лог заїздів у flash (LittleFS, ~13 МБ). Новий файл /logNNN.csv на кожен ARM.
// На березі: `log list`, `log dump N` (або tools/get_log.py), `log clear`.
//
// Запис у flash може блокувати на сотні мс і більше (стирання блоків, обслуговування
// LittleFS). Тому вся робота з файлом — в окремій задачі на ядрі 0, а цикл керування
// (ядро 1) тільки кладе рядки в чергу і НІКОЛИ не чекає. Черга повна → рядок
// відкидається (лічильник dropped()), керування не страждає.

#include <Arduino.h>
#include <FS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

class FlightLog {
public:
    // Монтує файлову систему і запускає задачу запису. Перший запуск: форматування
    // (до ~1 хв), пише про це в out. Повідомлення задачі («лог NNN: запис») — теж в out.
    bool begin(Print& out);

    // Неблокуючі — для циклу керування.
    bool start(const char* header);  // новий файл
    void write(const char* line);
    void stop();
    void flushIfDue(uint32_t) {}  // flush робить задача сама; лишено для сумісності
    bool active() const { return active_; }
    uint32_t dropped() const { return dropped_; }

    // Для консолі (на березі, DISARMED). Беруть м'ютекс файлової системи.
    // feed() викликається під час довгих операцій (watchdog): файлів може бути сотні.
    void list(Print& out, void (*feed)());
    bool dump(Print& out, unsigned index, void (*feed)());
    bool clear(void (*feed)());

private:
    enum class MsgType : uint8_t { Start, Line, Stop };
    static constexpr size_t kLineMax = 256;
    struct Msg {
        MsgType type;
        char text[kLineMax];
    };

    static void taskEntry(void* self);
    void taskLoop();
    void openNext(const char* header);
    void closeFile();
    bool send(MsgType type, const char* text);
    int nextIndex();
    static void pathFor(unsigned index, char* buf, size_t size);

    Print* out_ = nullptr;
    bool mounted_ = false;
    volatile bool active_ = false;
    volatile uint32_t dropped_ = 0;
    QueueHandle_t queue_ = nullptr;
    SemaphoreHandle_t fsLock_ = nullptr;

    // Тільки задача запису (під fsLock_):
    fs::File file_;
    uint32_t lastFlushMs_ = 0;
};
