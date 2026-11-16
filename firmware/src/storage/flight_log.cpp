#include "flight_log.h"

#include <LittleFS.h>

#include <cstdio>
#include <cstring>

namespace {
constexpr float kMaxUsage = 0.95f;  // не писати, якщо диск заповнений на 95%
constexpr uint32_t kFlushPeriodMs = 2000;
constexpr UBaseType_t kQueueDepth = 50;  // 5 с логу при 10 Гц, ~13 КБ RAM
constexpr uint32_t kTaskStack = 6144;
constexpr UBaseType_t kTaskPriority = 1;  // нижче за все важливе
constexpr BaseType_t kTaskCore = 0;       // цикл керування — на ядрі 1

int indexFromName(const char* name) {
    if (name[0] == '/') ++name;
    unsigned idx = 0;
    if (std::sscanf(name, "log%3u.csv", &idx) == 1) return static_cast<int>(idx);
    return -1;
}

class Lock {
public:
    // feed != nullptr: чекаємо шматками по 100 мс і годуємо watchdog (виклики з loopTask).
    explicit Lock(SemaphoreHandle_t m, void (*feed)() = nullptr) : m_(m) {
        while (xSemaphoreTake(m_, pdMS_TO_TICKS(100)) != pdTRUE) {
            if (feed) feed();
        }
    }
    ~Lock() { xSemaphoreGive(m_); }

private:
    SemaphoreHandle_t m_;
};
}  // namespace

void FlightLog::pathFor(unsigned index, char* buf, size_t size) {
    std::snprintf(buf, size, "/log%03u.csv", index);
}

bool FlightLog::begin(Print& out) {
    out_ = &out;
    mounted_ = LittleFS.begin(false);
    if (!mounted_) {
        out.println("      перший запуск: форматую пам'ять під логи, до 1 хв, не вимикай...");
        mounted_ = LittleFS.begin(true);
    }
    if (!mounted_) return false;

    fsLock_ = xSemaphoreCreateMutex();
    queue_ = xQueueCreate(kQueueDepth, sizeof(Msg));
    if (!fsLock_ || !queue_ ||
        xTaskCreatePinnedToCore(taskEntry, "flightLog", kTaskStack, this, kTaskPriority, nullptr,
                                kTaskCore) != pdPASS) {
        mounted_ = false;
        return false;
    }
    return true;
}

// ---------------- неблокуючий бік (цикл керування) ----------------

bool FlightLog::send(MsgType type, const char* text) {
    if (!queue_) return false;
    // Msg великий — тримаємо статично, щоб не роздувати стек loopTask.
    // send() викликається тільки з циклу керування, тож гонки немає.
    static Msg m;
    m.type = type;
    if (text) {
        std::strncpy(m.text, text, kLineMax - 1);
        m.text[kLineMax - 1] = '\0';
    } else {
        m.text[0] = '\0';
    }
    if (xQueueSend(queue_, &m, 0) != pdTRUE) {  // 0 = не чекати ніколи
        ++dropped_;
        return false;
    }
    return true;
}

bool FlightLog::start(const char* header) {
    if (!mounted_) return false;
    if (!send(MsgType::Start, header)) return false;
    active_ = true;
    return true;
}

void FlightLog::write(const char* line) {
    if (active_) send(MsgType::Line, line);
}

void FlightLog::stop() {
    if (!active_) return;
    active_ = false;
    // Stop не можна загубити: якщо черга повна — повторюємо коротко (рідкісний випадок, DISARM).
    for (int i = 0; i < 5 && !send(MsgType::Stop, nullptr); ++i) vTaskDelay(1);
}

// ---------------- задача запису (ядро 0) ----------------

void FlightLog::taskEntry(void* self) { static_cast<FlightLog*>(self)->taskLoop(); }

void FlightLog::taskLoop() {
    static Msg m;  // одна задача — статичний буфер безпечний
    for (;;) {
        const bool got = xQueueReceive(queue_, &m, pdMS_TO_TICKS(200)) == pdTRUE;
        Lock lock(fsLock_);
        if (got) {
            switch (m.type) {
                case MsgType::Start: openNext(m.text); break;
                case MsgType::Stop: closeFile(); break;
                case MsgType::Line:
                    if (file_ && file_.print(m.text) == 0) closeFile();  // диск заповнений
                    break;
            }
        }
        const uint32_t now = millis();
        if (file_ && now - lastFlushMs_ >= kFlushPeriodMs) {
            file_.flush();
            lastFlushMs_ = now;
        }
    }
}

int FlightLog::nextIndex() {
    int maxIdx = -1;
    fs::File root = LittleFS.open("/");
    if (!root) return 0;
    for (fs::File f = root.openNextFile(); f; f = root.openNextFile()) {
        const int idx = indexFromName(f.name());
        if (idx > maxIdx) maxIdx = idx;
    }
    return maxIdx + 1;
}

void FlightLog::openNext(const char* header) {
    closeFile();
    if (LittleFS.usedBytes() > kMaxUsage * LittleFS.totalBytes()) {
        if (out_) out_->println("лог: диск заповнений — log clear");
        return;
    }
    const int idx = nextIndex();
    if (idx > 999) {
        if (out_) out_->println("лог: забагато файлів — log clear");
        return;
    }
    char path[16];
    pathFor(static_cast<unsigned>(idx), path, sizeof(path));
    file_ = LittleFS.open(path, FILE_WRITE);
    if (!file_) {
        if (out_) out_->println("лог: не вдалося створити файл");
        return;
    }
    file_.print(header);
    lastFlushMs_ = millis();
    if (out_) out_->printf("лог %03d: запис\n", idx);
}

void FlightLog::closeFile() {
    if (file_) file_.close();
}

// ---------------- консоль (на березі) ----------------

void FlightLog::list(Print& out, void (*feed)()) {
    if (!mounted_) {
        out.println("ERR файлова система не змонтована");
        return;
    }
    Lock lock(fsLock_, feed);
    fs::File root = LittleFS.open("/");
    for (fs::File f = root.openNextFile(); f; f = root.openNextFile()) {
        const int idx = indexFromName(f.name());
        if (idx >= 0) out.printf("%3d  %8u байт\n", idx, static_cast<unsigned>(f.size()));
        if (feed) feed();
    }
    out.printf("зайнято %u з %u КБ, відкинуто рядків: %u\n",
               static_cast<unsigned>(LittleFS.usedBytes() / 1024),
               static_cast<unsigned>(LittleFS.totalBytes() / 1024),
               static_cast<unsigned>(dropped_));
}

bool FlightLog::dump(Print& out, unsigned index, void (*feed)()) {
    if (!mounted_) return false;
    Lock lock(fsLock_, feed);
    char path[16];
    pathFor(index, path, sizeof(path));
    fs::File f = LittleFS.open(path, FILE_READ);
    if (!f) return false;
    out.printf("BEGIN_LOG %s %u\n", path + 1, static_cast<unsigned>(f.size()));
    uint8_t buf[256];
    while (f.available()) {
        const size_t n = f.read(buf, sizeof(buf));
        out.write(buf, n);
        if (feed) feed();
    }
    out.println("END_LOG");
    return true;
}

bool FlightLog::clear(void (*feed)()) {
    if (!mounted_) return false;
    Lock lock(fsLock_, feed);
    closeFile();
    char paths[64][16];
    size_t count = 0;
    bool more = true;
    while (more) {  // по 64 файли за прохід, поки є що видаляти
        count = 0;
        fs::File root = LittleFS.open("/");
        for (fs::File f = root.openNextFile(); f && count < 64; f = root.openNextFile()) {
            const int idx = indexFromName(f.name());
            if (idx >= 0) pathFor(static_cast<unsigned>(idx), paths[count++], 16);
            if (feed) feed();
        }
        root.close();
        for (size_t i = 0; i < count; ++i) {
            LittleFS.remove(paths[i]);
            if (feed) feed();
        }
        more = count == 64;
    }
    return true;
}
