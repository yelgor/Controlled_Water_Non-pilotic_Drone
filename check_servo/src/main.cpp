// ESP32-S3 + RDS3115MG: тест серво через монітор порту (115200).
// Серво сигнал -> GPIO 4. Живлення серво - окреме 5-7.2V, GND спільний!
//
// Команди (натискай клавішу, Enter не треба):
//   c центр | a/d -/+10us | A/D -/+50us | l/h до нижнього/верхнього ліміту
//   s старт/стоп повільного гойдання між лімітами
//   w розширити ліміти на 50us з кожного боку | n звузити на 50us
//   число + Enter (наприклад 1600) = поставити імпульс у us
//   ? допомога

#include <Arduino.h>
#include <ESP32Servo.h>

constexpr int SERVO_PIN   = 4;
constexpr int CENTER_US   = 1500;
constexpr int HARD_MIN_US = 500;      // абсолютні межі
constexpr int HARD_MAX_US = 2500;
constexpr float SLEW_US_PER_S   = 800.0f;  // швидкість руху
constexpr uint32_t SWEEP_PAUSE_MS = 600;   // пауза на краях при гойданні

Servo servo;
int   minUs = 1000;                    // поточні безпечні ліміти
int   maxUs = 2000;
float currentUs = CENTER_US;
float targetUs  = CENTER_US;

bool     sweeping = false;
bool     sweepUp  = true;
bool     waiting  = false;
uint32_t reachedMs = 0;
uint32_t lastLoopUs = 0;

String numBuf;

void printTarget() {
  Serial.printf("target = %d us  (ліміти %d..%d)\n", (int)targetUs, minUs, maxUs);
}

void setTarget(float us) {
  targetUs = constrain(us, (float)minUs, (float)maxUs);
}

void printHelp() {
  Serial.println(F("\n--- Команди ---"));
  Serial.println(F("c центр | a/d -/+10us | A/D -/+50us | l/h нижній/верхній ліміт"));
  Serial.println(F("s гойдання | w розширити ліміти | n звузити ліміти"));
  Serial.println(F("число + Enter = імпульс у us (напр. 1600) | ? допомога"));
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c >= '0' && c <= '9') { numBuf += c; continue; }

    if (c == '\n' || c == '\r') {
      if (numBuf.length()) {
        int v = numBuf.toInt();
        numBuf = "";
        sweeping = false;
        setTarget(v);
        if (v != (int)targetUs) Serial.printf("%d поза лімітами, обмежено\n", v);
        printTarget();
      }
      continue;
    }

    numBuf = "";   // будь-яка літера скидає недовведене число

    switch (c) {
      case 'c': sweeping = false; setTarget(CENTER_US); printTarget(); break;
      case 'a': sweeping = false; setTarget(targetUs - 10); printTarget(); break;
      case 'd': sweeping = false; setTarget(targetUs + 10); printTarget(); break;
      case 'A': sweeping = false; setTarget(targetUs - 50); printTarget(); break;
      case 'D': sweeping = false; setTarget(targetUs + 50); printTarget(); break;
      case 'l': sweeping = false; setTarget(minUs); printTarget(); break;
      case 'h': sweeping = false; setTarget(maxUs); printTarget(); break;
      case 's':
        sweeping = !sweeping;
        waiting = false;
        if (sweeping) { sweepUp = true; setTarget(maxUs); Serial.println(F("ГОЙДАННЯ: ввімкнено")); }
        else          { Serial.println(F("ГОЙДАННЯ: вимкнено")); }
        break;
      case 'w':
        minUs = max(HARD_MIN_US, minUs - 50);
        maxUs = min(HARD_MAX_US, maxUs + 50);
        printTarget();
        break;
      case 'n':
        if (maxUs - minUs > 300) { minUs += 50; maxUs -= 50; }
        setTarget(targetUs);
        printTarget();
        break;
      case '?': printHelp(); break;
      default: break;
    }
  }
}

void updateSweep() {
  if (!sweeping || currentUs != targetUs) return;
  if (!waiting) { waiting = true; reachedMs = millis(); return; }
  if (millis() - reachedMs > SWEEP_PAUSE_MS) {
    sweepUp = !sweepUp;
    setTarget(sweepUp ? maxUs : minUs);
    waiting = false;
    printTarget();
  }
}

void updateServo() {
  uint32_t nowUs = micros();
  float dt = (nowUs - lastLoopUs) / 1e6f;
  lastLoopUs = nowUs;
  if (dt > 0.1f) dt = 0.1f;

  float maxStep = SLEW_US_PER_S * dt;
  float diff = targetUs - currentUs;
  if (fabsf(diff) <= maxStep) currentUs = targetUs;
  else currentUs += (diff > 0 ? maxStep : -maxStep);

  servo.writeMicroseconds((int)(currentUs + 0.5f));
}

void setup() {
  Serial.begin(115200);
  delay(300);

  ESP32PWM::allocateTimer(0);
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, HARD_MIN_US, HARD_MAX_US);
  servo.writeMicroseconds(CENTER_US);   // одразу центр

  lastLoopUs = micros();

  Serial.println(F("\n=== Тест серво RDS3115MG ==="));
  Serial.printf("Центр %d us, ліміти %d..%d us\n", CENTER_US, minUs, maxUs);
  Serial.println(F("Серво у центрі. Тяги ще не чіпляй!"));
  printHelp();
}

void loop() {
  handleSerial();
  updateSweep();
  updateServo();
}