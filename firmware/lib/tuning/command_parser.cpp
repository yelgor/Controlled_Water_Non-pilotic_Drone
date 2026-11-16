#include "command_parser.h"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace tuning {

namespace {

constexpr size_t kMaxTokens = 4;
constexpr size_t kTokLen = 24;

// Розбиває рядок на слова (нижній регістр). Повертає кількість; -1 якщо слово задовге.
int tokenize(const char* line, char out[kMaxTokens][kTokLen]) {
    int n = 0;
    const char* s = line;
    while (*s) {
        while (*s && std::isspace(static_cast<unsigned char>(*s))) ++s;
        if (!*s) break;
        if (n >= static_cast<int>(kMaxTokens)) return -1;
        size_t len = 0;
        while (*s && !std::isspace(static_cast<unsigned char>(*s))) {
            if (len + 1 >= kTokLen) return -1;
            out[n][len++] = static_cast<char>(std::tolower(static_cast<unsigned char>(*s)));
            ++s;
        }
        out[n][len] = '\0';
        ++n;
    }
    return n;
}

bool parseFloat(const char* s, float& out) {
    char* end = nullptr;
    const float v = std::strtof(s, &end);
    if (end == s || *end != '\0' || !std::isfinite(v)) return false;
    out = v;
    return true;
}

bool eq(const char* a, const char* b) { return std::strcmp(a, b) == 0; }

}  // namespace

Command parseCommand(const char* line) {
    Command c;
    if (!line) return c;
    char t[kMaxTokens][kTokLen] = {};
    const int n = tokenize(line, t);
    if (n < 0) {
        c.type = CmdType::BadArgs;
        return c;
    }
    if (n == 0) return c;

    auto simple = [&](CmdType type) {
        c.type = (n == 1) ? type : CmdType::BadArgs;
        return c;
    };

    const char* cmd = t[0];
    if (eq(cmd, "help") || eq(cmd, "?")) return simple(CmdType::Help);
    if (eq(cmd, "status")) return simple(CmdType::Status);
    if (eq(cmd, "save")) return simple(CmdType::Save);
    if (eq(cmd, "load")) return simple(CmdType::Load);
    if (eq(cmd, "defaults")) return simple(CmdType::Defaults);
    if (eq(cmd, "i2c")) return simple(CmdType::I2cScan);
    if (eq(cmd, "servo")) return simple(CmdType::ServoTest);

    if (eq(cmd, "get")) {
        if (n > 2) { c.type = CmdType::BadArgs; return c; }
        c.type = CmdType::Get;
        if (n == 2) {
            std::strncpy(c.name, t[1], sizeof(c.name) - 1);
            c.hasName = true;
        }
        return c;
    }
    if (eq(cmd, "set")) {
        if (n != 3 || !parseFloat(t[2], c.value)) { c.type = CmdType::BadArgs; return c; }
        c.type = CmdType::Set;
        std::strncpy(c.name, t[1], sizeof(c.name) - 1);
        c.hasName = true;
        return c;
    }
    if (eq(cmd, "hdg")) {
        if (n != 2 || !parseFloat(t[1], c.value)) { c.type = CmdType::BadArgs; return c; }
        c.type = CmdType::Heading;
        return c;
    }
    if (eq(cmd, "cal")) {
        if (n == 2 && eq(t[1], "start")) c.type = CmdType::CalStart;
        else if (n == 2 && eq(t[1], "stop")) c.type = CmdType::CalStop;
        else c.type = CmdType::BadArgs;
        return c;
    }
    if (eq(cmd, "tele")) {
        if (n == 2 && eq(t[1], "on")) c.type = CmdType::TeleOn;
        else if (n == 2 && eq(t[1], "off")) c.type = CmdType::TeleOff;
        else c.type = CmdType::BadArgs;
        return c;
    }
    if (eq(cmd, "log")) {
        if (n == 2 && eq(t[1], "list")) c.type = CmdType::LogList;
        else if (n == 2 && eq(t[1], "clear")) c.type = CmdType::LogClear;
        else if (n == 3 && eq(t[1], "dump") && parseFloat(t[2], c.value) && c.value >= 0.0f &&
                 c.value == std::floor(c.value))
            c.type = CmdType::LogDump;
        else c.type = CmdType::BadArgs;
        return c;
    }
    c.type = CmdType::Unknown;
    return c;
}

}  // namespace tuning
