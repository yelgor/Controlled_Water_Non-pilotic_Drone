// ESP32-S3 + ELRS приймач (CRSF) + серво + мотор (ESC).
// + Обмеження керма від газу (чим більше газу, тим менший максимальний поворот).
// + Обмеження газу від керма (чим сильніше кермо вбік, тим менший максимальний газ).
//
// Серво сигнал  -> GPIO 4  (керує CH1, правий стік X)
// Мотор (ESC)   -> GPIO 5  (керує CH3, лівий стік Y / газ)
// Приймач TX    -> GPIO 17 (ESP отримує)
// Приймач RX    -> GPIO 18 (ESP відправляє)
// GND серво, ESC, приймача й ESP32 - спільний
//
// Команди монітора порту (115200):
//   c центр | a/d -/+10us | A/D -/+50us | l/h нижній/верхній ліміт
//   r режим пульта | m manual | ? допомога
// У MANUAL мотор завжди стоїть, а кермо ходить на повний хід (обмеження діє лише з пульта).
//
// ЯК КЕРУЄМО МОТОРОМ. ESC отримує такий самий ШІМ-сигнал, як серво: імпульс кожні 20 мс (50 Гц),
// довжина імпульсу 1000 us = стоп, 2000 us = повний газ. Зараз газ обмежено до MOTOR_MAX_US = 1500 (~50 %).
//
// ДВОСТОРОННЄ ОБМЕЖЕННЯ (захист від крену й перекидання). Обидва обмеження рахуються від
// ЗГЛАДЖЕНИХ виходів (motorUs, currentUs), тому змінюються плавно й не борються між собою:
//   1) Хід керма  = 1 − (1 − STEER_AT_FULL)  · fгазу      (газ росте -> кермо звужується)
//   2) Макс. газ  = 1 − (1 − THROTTLE_AT_FULL_STEER) · sкерма  (кермо росте -> газ обмежується)
// де fгазу = частка газу 0..1, sкерма = |відхилення керма| / повний хід 0..1.
// Разом вони утворюють безпечну «обвідну»: повний газ можливий лише при малому керму, а повний
// поворот лише на малому газі. Раптовий різкий поворот на повному газі не скидає газ миттєво,
// а знижує його з обмеженою швидкістю MOTOR_CAP_DECEL_US_PER_S (щоб не смикало катер).

#include <Arduino.h>
#include <ESP32Servo.h>

// ===================== НАЛАШТУВАННЯ =====================
constexpr int SERVO_PIN   = 4;
constexpr int MOTOR_PIN   = 5;
constexpr int CRSF_RX_PIN = 18;   // сюди йде TX приймача
constexpr int CRSF_TX_PIN = 17;   // сюди йде RX приймача

// --- Серво ---
constexpr int CENTER_US    = 1500;
constexpr int SERVO_MIN_US = 1250;
constexpr int SERVO_MAX_US = 1750;
constexpr int HARD_MIN_US  = 500;
constexpr int HARD_MAX_US  = 2500;
constexpr float SLEW_US_PER_S = 1000.0f;  // швидкість серво (повний хід ~0.5 с)
constexpr int  RC_CHANNEL = 1;
constexpr bool REVERSE    = false;

// --- Обмеження керма від газу ---
constexpr bool  SPEED_STEER_LIMIT = true;
constexpr float STEER_AT_FULL     = 0.4f;   // частка ходу керма на MOTOR_MAX_US (0.4 = 40 %)

// --- Обмеження газу від керма ---
constexpr bool  STEER_THROTTLE_LIMIT   = true;
constexpr float THROTTLE_AT_FULL_STEER = 0.5f;   // частка газу, коли кермо повністю вбік (0.5 = 50 %)
constexpr float MOTOR_CAP_DECEL_US_PER_S = 1200.0f;  // як швидко знижуємо газ через кермо

// --- Мотор (ESC) ---
constexpr int  MOTOR_CHANNEL = 3;        // CH3 = газ (лівий стік Y)
constexpr int  MOTOR_STOP_US = 1000;     // стоп
constexpr int  MOTOR_MAX_US  = 1500;     // ліміт газу для тестів, далі підніми до 2000
constexpr float MOTOR_SLEW_US_PER_S = 800.0f;  // швидкість розгону
constexpr int  MOTOR_ARM_BELOW = 300;    // мотор дозволяється, коли газ нижче цього (172..1811)
constexpr int  MOTOR_DEADBAND  = 30;     // мертва зона знизу

// --- Arm-перемикач (необов'язково) ---
constexpr bool USE_ARM_SWITCH = false;
constexpr int  ARM_CHANNEL    = 5;

// --- Failsafe / режими ---
constexpr uint32_t FAILSAFE_MS        = 500;
constexpr bool     FAILSAFE_GO_CENTER = true;
constexpr uint32_t MANUAL_WINDOW_MS   = 5000;
// ========================================================

constexpr uint32_t CRSF_BAUD = 420000;
constexpr uint8_t  CRSF_TYPE_RC_CHANNELS = 0x16;
constexpr int      CRSF_MIN = 172, CRSF_MID = 992, CRSF_MAX = 1811;

Servo servo;
Servo motor;
HardwareSerial &crsfSerial = Serial1;

enum Mode { MANUAL, RC };
Mode mode = MANUAL;

float currentUs = CENTER_US;
float targetUs  = CENTER_US;

float motorUs          = MOTOR_STOP_US;
float motorTarget      = MOTOR_STOP_US;   // після обмеження від керма
float motorStickTarget = MOTOR_STOP_US;   // те, що просить стік (до обмеження)
bool  motorArmed       = false;

uint16_t channels[16];
uint32_t lastFrameMs = 0;
bool     gotFrame    = false;

uint32_t lastCmdMs   = 0;
uint32_t lastPrintMs = 0;
uint32_t lastLoopUs  = 0;
uint32_t frameCount  = 0;
uint32_t crcErrors   = 0;
uint32_t rawBytes    = 0;

// ---------- CRSF ----------

uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *data++;
    for (int i = 0; i < 8; i++) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
    }
  }
  return crc;
}

void unpackChannels(const uint8_t *p) {
  uint32_t acc = 0;
  int bits = 0, idx = 0;
  for (int i = 0; i < 22; i++) {
    acc |= (uint32_t)p[i] << bits;
    bits += 8;
    while (bits >= 11 && idx < 16) {
      channels[idx++] = acc & 0x7FF;
      acc >>= 11;
      bits -= 11;
    }
  }
}

void readCrsf() {
  static uint8_t buf[64];
  static uint8_t pos = 0;

  while (crsfSerial.available()) {
    uint8_t b = crsfSerial.read();
    rawBytes++;

    if (pos == 0) {
      if (b == 0xC8 || b == 0xEE || b == 0xEA || b == 0xEC) buf[pos++] = b;
      continue;
    }
    if (pos == 1) {
      if (b < 2 || b > 62) { pos = 0; continue; }
      buf[pos++] = b;
      continue;
    }

    buf[pos++] = b;
    uint8_t frameLen = buf[1] + 2;
    if (pos < frameLen) continue;

    uint8_t type = buf[2];
    uint8_t crc  = buf[frameLen - 1];
    if (crc8(&buf[2], buf[1] - 1) == crc) {
      if (type == CRSF_TYPE_RC_CHANNELS && buf[1] == 24) {
        unpackChannels(&buf[3]);
        lastFrameMs = millis();
        gotFrame = true;
        frameCount++;
      }
    } else {
      crcErrors++;
    }
    pos = 0;
  }
}

bool linkIsOk() {
  return gotFrame && (millis() - lastFrameMs < FAILSAFE_MS);
}

int crsfToUs(uint16_t v) {
  v = constrain(v, CRSF_MIN, CRSF_MAX);
  if (REVERSE) v = CRSF_MIN + CRSF_MAX - v;
  if (v < CRSF_MID) return map(v, CRSF_MIN, CRSF_MID, SERVO_MIN_US, CENTER_US);
  return map(v, CRSF_MID, CRSF_MAX, CENTER_US, SERVO_MAX_US);
}

// Яку частку ходу керма дозволяємо при поточному газі: 1.0 на стопі, STEER_AT_FULL на MOTOR_MAX_US
float steerScale() {
  if (!SPEED_STEER_LIMIT) return 1.0f;
  float f = (motorUs - MOTOR_STOP_US) / (float)(MOTOR_MAX_US - MOTOR_STOP_US);
  f = constrain(f, 0.0f, 1.0f);
  return 1.0f - (1.0f - STEER_AT_FULL) * f;
}

// Яку частку газу дозволяємо при поточному керму: 1.0 при прямому, THROTTLE_AT_FULL_STEER при повному
float throttleScale() {
  if (!STEER_THROTTLE_LIMIT) return 1.0f;
  float half = (float)(SERVO_MAX_US - CENTER_US);
  float s = fabsf(currentUs - CENTER_US) / half;
  s = constrain(s, 0.0f, 1.0f);
  return 1.0f - (1.0f - THROTTLE_AT_FULL_STEER) * s;
}

// ---------- Серво ----------

void setTarget(float us) {
  targetUs = constrain(us, (float)SERVO_MIN_US, (float)SERVO_MAX_US);
}

void printHelp() {
  Serial.println(F("\n--- Команди ---"));
  Serial.println(F("c центр | a/d -/+10us | A/D -/+50us | l/h нижній/верхній ліміт"));
  Serial.println(F("r режим пульта | m manual | ? допомога"));
}

void handleSerialCmd() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;
    lastCmdMs = millis();

    switch (c) {
      case 'c': mode = MANUAL; setTarget(CENTER_US); break;
      case 'a': mode = MANUAL; setTarget(targetUs - 10); break;
      case 'd': mode = MANUAL; setTarget(targetUs + 10); break;
      case 'A': mode = MANUAL; setTarget(targetUs - 50); break;
      case 'D': mode = MANUAL; setTarget(targetUs + 50); break;
      case 'l': mode = MANUAL; setTarget(SERVO_MIN_US); break;
      case 'h': mode = MANUAL; setTarget(SERVO_MAX_US); break;
      case 'm': mode = MANUAL; Serial.println(F("MANUAL (тримаю позицію, мотор стоїть)")); break;
      case 'r': mode = RC;     Serial.println(F("РЕЖИМ ПУЛЬТА (опусти газ вниз, щоб мотор заармився)")); break;
      case '?': printHelp(); break;
      default: continue;
    }
    if (mode == MANUAL) {
      Serial.printf("target = %d us\n", (int)targetUs);
    }
  }
}

void updateRcTarget() {
  if (!linkIsOk()) {
    if (FAILSAFE_GO_CENTER) setTarget(CENTER_US);
    return;
  }

  if (USE_ARM_SWITCH && channels[ARM_CHANNEL - 1] < CRSF_MID) {
    setTarget(CENTER_US);
    return;
  }

  // відхилення від центру стискаємо залежно від газу
  float us = crsfToUs(channels[RC_CHANNEL - 1]);
  setTarget(CENTER_US + (us - CENTER_US) * steerScale());
}

// ---------- Мотор ----------

void updateMotorTarget() {
  bool armSwitchOk = !USE_ARM_SWITCH || channels[ARM_CHANNEL - 1] >= CRSF_MID;

  // Не RC, нема зв'язку або arm вимкнено -> стоп і розарм
  if (mode != RC || !linkIsOk() || !armSwitchOk) {
    motorArmed = false;
    motorStickTarget = MOTOR_STOP_US;
    motorTarget = MOTOR_STOP_US;
    return;
  }

  uint16_t t = channels[MOTOR_CHANNEL - 1];

  // Заарм тільки коли газ опущений
  if (!motorArmed) {
    if (t <= MOTOR_ARM_BELOW) motorArmed = true;
    else { motorStickTarget = MOTOR_STOP_US; motorTarget = MOTOR_STOP_US; return; }
  }

  if (t <= CRSF_MIN + MOTOR_DEADBAND) {
    motorStickTarget = MOTOR_STOP_US;
  } else {
    motorStickTarget = map(constrain(t, CRSF_MIN + MOTOR_DEADBAND, CRSF_MAX),
                           CRSF_MIN + MOTOR_DEADBAND, CRSF_MAX,
                           MOTOR_STOP_US, MOTOR_MAX_US);
  }

  // стеля газу від поточного повороту керма
  float cap = MOTOR_STOP_US + (MOTOR_MAX_US - MOTOR_STOP_US) * throttleScale();
  motorTarget = min(motorStickTarget, cap);
}

// ---------- Плавний вивід ----------

void updateOutputs() {
  uint32_t nowUs = micros();
  float dt = (nowUs - lastLoopUs) / 1e6f;
  lastLoopUs = nowUs;
  if (dt > 0.1f) dt = 0.1f;

  // серво
  float maxStep = SLEW_US_PER_S * dt;
  float diff = targetUs - currentUs;
  if (fabsf(diff) <= maxStep) currentUs = targetUs;
  else currentUs += (diff > 0 ? maxStep : -maxStep);
  servo.writeMicroseconds((int)(currentUs + 0.5f));

  // мотор: розгін плавний; зупинка стіком миттєва; зниження через кермо плавне
  if (motorTarget <= motorUs) {
    bool stickIsBinding = (motorTarget >= motorStickTarget);   // обмежує саме стік, а не кермо
    if (stickIsBinding) {
      motorUs = motorTarget;
    } else {
      motorUs -= MOTOR_CAP_DECEL_US_PER_S * dt;
      if (motorUs < motorTarget) motorUs = motorTarget;
    }
  } else {
    motorUs += MOTOR_SLEW_US_PER_S * dt;
    if (motorUs > motorTarget) motorUs = motorTarget;
  }
  motor.writeMicroseconds((int)(motorUs + 0.5f));
}

void printStatus() {
  uint32_t now = millis();
  if (now - lastPrintMs < 200) return;
  lastPrintMs = now;
  if (mode != RC) return;

  Serial.printf("RC link:%s ch%d=%4u servo=%4d (хід %3d%%) | ch%d=%4u motor=%4d (стеля %3d%%) %s | frames=%lu crcErr=%lu\n",
                linkIsOk() ? "OK " : "LOST",
                RC_CHANNEL,
                gotFrame ? channels[RC_CHANNEL - 1] : 0,
                (int)(currentUs + 0.5f),
                (int)(steerScale() * 100 + 0.5f),
                MOTOR_CHANNEL,
                gotFrame ? channels[MOTOR_CHANNEL - 1] : 0,
                (int)(motorUs + 0.5f),
                (int)(throttleScale() * 100 + 0.5f),
                motorArmed ? "ARM" : "SAFE",
                (unsigned long)frameCount, (unsigned long)crcErrors);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  for (int i = 0; i < 16; i++) channels[i] = CRSF_MID;
  channels[MOTOR_CHANNEL - 1] = CRSF_MIN;   // газ внизу до першого кадру

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo.setPeriodHertz(50);
  motor.setPeriodHertz(50);

  // Спочатку безпечні значення: серво в центр, мотор на стоп
  servo.attach(SERVO_PIN, HARD_MIN_US, HARD_MAX_US);
  servo.writeMicroseconds(CENTER_US);
  motor.attach(MOTOR_PIN, HARD_MIN_US, HARD_MAX_US);
  motor.writeMicroseconds(MOTOR_STOP_US);

  crsfSerial.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);

  lastCmdMs = millis();
  lastLoopUs = micros();

  Serial.println(F("\n=== ESP32-S3 серво + мотор + ELRS (CRSF) ==="));
  Serial.printf("Серво: центр %d us, ліміти %d..%d us, швидкість %d us/с\n",
                CENTER_US, SERVO_MIN_US, SERVO_MAX_US, (int)SLEW_US_PER_S);
  Serial.printf("Мотор: стоп %d us, макс %d us (CH%d)\n", MOTOR_STOP_US, MOTOR_MAX_US, MOTOR_CHANNEL);
  if (SPEED_STEER_LIMIT)
    Serial.printf("Кермо від газу: 100%% ходу на стопі, %d%% на макс. газі\n", (int)(STEER_AT_FULL * 100));
  if (STEER_THROTTLE_LIMIT)
    Serial.printf("Газ від керма: 100%% при прямому, %d%% при повному повороті\n", (int)(THROTTLE_AT_FULL_STEER * 100));
  Serial.printf("MANUAL-вікно %lu с, далі режим пульта (CH%d серво)\n",
                (unsigned long)(MANUAL_WINDOW_MS / 1000), RC_CHANNEL);
  Serial.println(F("Мотор стоїть, поки газ не опущений і не ввімкнено режим пульта."));
  printHelp();
}

void loop() {
  readCrsf();
  handleSerialCmd();

  if (mode == MANUAL && millis() - lastCmdMs > MANUAL_WINDOW_MS) {
    mode = RC;
    Serial.println(F("Авто-перехід у РЕЖИМ ПУЛЬТА"));
  }

  if (mode == RC) updateRcTarget();
  updateMotorTarget();
  updateOutputs();
  printStatus();
}
