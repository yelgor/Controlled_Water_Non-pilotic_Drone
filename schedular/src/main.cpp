#include <Arduino.h>
#include <Wire.h>

#define LSM9DS1_AG_ADDR 0x6B
#define SDA_PIN 8
#define SCL_PIN 9

#define WHO_AM_I_XG  0x0F
#define CTRL_REG1_G  0x10
#define STATUS_REG_AG 0x17 
#define OUT_X_L_G    0x18   
#define CTRL_REG6_XL 0x20
#define OUT_X_L_XL   0x28   

// ============================================================================
// 1. АРХІТЕКТУРА СКЕДУЛЕРА (Time Budgeting)
// ============================================================================

typedef enum {
  PRIORITY_REALTIME = 0,
  PRIORITY_HIGH,
  PRIORITY_MEDIUM
} taskPriority_e;

typedef struct {
  const char *name;
  bool (*taskFunc)(void);
  uint32_t desiredPeriodUs;
  uint32_t lastExecutedUs;
  taskPriority_e priority;
  uint32_t avgExecTimeUs;
  bool enabled;
  uint32_t missedDeadlines;
  bool deadlineFlagged;
} task_t;

// ============================================================================
// 2. СИНХРОНІЗАЦІЯ ТА ГЛОБАЛЬНІ ЗМІННІ
// ============================================================================

typedef struct {
  float ax, ay, az;
  float gx, gy, gz;
  uint32_t imuRunCount;
  uint32_t avgExecUs;
  uint32_t missedDeadlines;
  float cpuLoadPct; 
  bool imuHealthy;
} telemetryData_t;

QueueHandle_t telemetryQueue; 

int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
float axG, ayG, azG;
float gxDps, gyDps, gzDps;
uint32_t imuRunCount = 0;

bool imuHealthy = true;
uint32_t imuErrorCounter = 0;
uint32_t cpuBusyTimeAccumulatorUs = 0;

float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
float accelBiasX = 0.0f, accelBiasY = 0.0f, accelBiasZ = 0.0f;

float kalmanAngleX = 0.0f;
float kalmanAngleY = 0.0f;

// ============================================================================
// 3. РОБОТА З ЗАЛІЗОМ (IMU)
// ============================================================================

void lsmInit() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(2); 

  Wire.beginTransmission(LSM9DS1_AG_ADDR);
  Wire.write(CTRL_REG1_G);
  Wire.write(0xC0); // ODR ~952 Hz
  Wire.endTransmission(true);

  Wire.beginTransmission(LSM9DS1_AG_ADDR);
  Wire.write(CTRL_REG6_XL);
  Wire.write(0xC0); // ODR ~952 Hz
  Wire.endTransmission(true);
}

bool isImuDataReady() {
  Wire.beginTransmission(LSM9DS1_AG_ADDR);
  Wire.write(STATUS_REG_AG);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)LSM9DS1_AG_ADDR, (uint8_t)1, (uint8_t)true) != 1) return false;
  
  uint8_t status = Wire.read();
  return (status & 0x03) == 0x03; 
}

bool readRawIMU(int16_t &gx, int16_t &gy, int16_t &gz, int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(LSM9DS1_AG_ADDR);
  Wire.write(OUT_X_L_G);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)LSM9DS1_AG_ADDR, (uint8_t)6, (uint8_t)true) != 6) return false;
  gx = Wire.read() | (Wire.read() << 8); gy = Wire.read() | (Wire.read() << 8); gz = Wire.read() | (Wire.read() << 8);

  Wire.beginTransmission(LSM9DS1_AG_ADDR);
  Wire.write(OUT_X_L_XL);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)LSM9DS1_AG_ADDR, (uint8_t)6, (uint8_t)true) != 6) return false;
  ax = Wire.read() | (Wire.read() << 8); ay = Wire.read() | (Wire.read() << 8); az = Wire.read() | (Wire.read() << 8);

  return true;
}

void calibrateIMU() {
  const int samples = 500;
  long sumGx = 0, sumGy = 0, sumGz = 0, sumAx = 0, sumAy = 0, sumAz = 0;
  int validSamples = 0;

  for (int i = 0; i < samples; i++) {
    uint32_t timeoutMillis = millis();
    
    // ДОДАНО: Захист від вічного зависання при калібруванні
    while (!isImuDataReady()) {
      if (millis() - timeoutMillis > 50) {
        Serial.println("Calibration timeout! Sensor disconnected?");
        return; 
      }
      delayMicroseconds(500); // Використовуємо 500мкс замість 100 для зменшення навантаження
    }
    
    int16_t gx, gy, gz, ax, ay, az;
    if (readRawIMU(gx, gy, gz, ax, ay, az)) {
      sumGx += gx; sumGy += gy; sumGz += gz; sumAx += ax; sumAy += ay; sumAz += az;
      validSamples++;
    }
  }

  if (validSamples > 0) {
    gyroBiasX = (float)sumGx / validSamples; gyroBiasY = (float)sumGy / validSamples; gyroBiasZ = (float)sumGz / validSamples;
    accelBiasX = (float)sumAx / validSamples; accelBiasY = (float)sumAy / validSamples; accelBiasZ = ((float)sumAz / validSamples) - 16384.0f;
    Serial.println("Calibration OK!");
  } else {
    Serial.println("Calibration FAILED!");
  }
}

// ============================================================================
// 4. ТАСКИ (ЯДРО 1)
// ============================================================================

bool taskIMU(void) {
  static uint32_t lastRecoveryAttemptMs = 0;

  // Якщо сенсор відпав — кожну секунду пробуємо повторно ініціалізувати його регістри
  if (!imuHealthy) {
    uint32_t nowMs = millis();
    if (nowMs - lastRecoveryAttemptMs > 1000) {
      lastRecoveryAttemptMs = nowMs;
      lsmInit(); 
    }
  }

  static uint8_t notReadyStreak = 0; 

  if (!isImuDataReady()) {
    notReadyStreak++;
    if (notReadyStreak > 5) {
      notReadyStreak = 0;
      if (++imuErrorCounter > 10) imuHealthy = false; 
      return true; 
    }
    return false; 
  }

  notReadyStreak = 0; 
  
  int16_t localGx, localGy, localGz, localAx, localAy, localAz;
  if (!readRawIMU(localGx, localGy, localGz, localAx, localAy, localAz)) {
    if (++imuErrorCounter > 10) imuHealthy = false;
    return true;
  }

  imuErrorCounter = 0;
  imuHealthy = true;

  gxDps = ((float)localGx - gyroBiasX) / 131.0f;
  gyDps = ((float)localGy - gyroBiasY) / 131.0f;
  gzDps = ((float)localGz - gyroBiasZ) / 131.0f;
  axG = ((float)localAx - accelBiasX) / 16384.0f;
  ayG = ((float)localAy - accelBiasY) / 16384.0f;
  azG = ((float)localAz - accelBiasZ) / 16384.0f;

  imuRunCount++;
  return true;
}

bool taskKalman(void) {
  if (!imuHealthy) return true;
  kalmanAngleX += gxDps * 0.001f;
  kalmanAngleY += gyDps * 0.001f;
  return true;
}

bool taskPID(void) {
  return true;
}

bool taskMotorOut(void) {
  return true;
}

// ============================================================================
// 5. ДИСПЕТЧЕР З TIME BUDGETING
// ============================================================================

#define NUM_TASKS 4
task_t tasks[NUM_TASKS] = {
  { "IMU",       taskIMU,       2000, 0, PRIORITY_REALTIME, 0, true, 0, false }, 
  { "Kalman",    taskKalman,    2000, 0, PRIORITY_REALTIME, 0, true, 0, false }, // Змінено з 1000 на 2000
  { "PID",       taskPID,       2000, 0, PRIORITY_HIGH,     0, true, 0, false },
  { "MotorOut",  taskMotorOut,  2000, 0, PRIORITY_HIGH,     0, true, 0, false },
};

void schedulerRun(void) {
  uint32_t now = micros();
  task_t *selected = NULL;
  int bestDynamicPriority = 999;

  uint32_t timeToNextIMU = 0;
  uint32_t imuElapsed = now - tasks[0].lastExecutedUs;
  if (imuElapsed < tasks[0].desiredPeriodUs) {
    timeToNextIMU = tasks[0].desiredPeriodUs - imuElapsed;
  }

  for (int i = 0; i < NUM_TASKS; i++) {
    if (!tasks[i].enabled) continue;
    uint32_t elapsed = now - tasks[i].lastExecutedUs;

    if (tasks[i].lastExecutedUs != 0 && elapsed >= (tasks[i].desiredPeriodUs * 2)) {
      if (!tasks[i].deadlineFlagged) { tasks[i].missedDeadlines++; tasks[i].deadlineFlagged = true; }
    }

    if (elapsed >= tasks[i].desiredPeriodUs) {
      
      if (i != 0 && tasks[i].avgExecTimeUs > 0) {
        if ((tasks[i].avgExecTimeUs + 50) > timeToNextIMU) {
          continue; 
        }
      }

      int ageBoost = (elapsed - tasks[i].desiredPeriodUs) / tasks[i].desiredPeriodUs;
      int currentDynPriority = (int)tasks[i].priority - ageBoost;
      if (currentDynPriority < 0) currentDynPriority = 0;

      if (selected == NULL || currentDynPriority < bestDynamicPriority) {
        selected = &tasks[i];
        bestDynamicPriority = currentDynPriority;
      }
    }
  }

  if (selected != NULL) {
    uint32_t start = micros();
    bool isCompleted = selected->taskFunc(); 
    uint32_t executionTime = micros() - start;

    cpuBusyTimeAccumulatorUs += executionTime;

    if (isCompleted) {
      // ДОДАНО: Оновлюємо поточний час після виконання, бо задача могла тривати довго
      now = micros(); 
      
      selected->avgExecTimeUs = (selected->avgExecTimeUs * 9 + executionTime) / 10;
      uint32_t missedBy = (selected->lastExecutedUs == 0) ? 0 : (now - selected->lastExecutedUs);

      if (selected->lastExecutedUs == 0 || missedBy > selected->desiredPeriodUs * 3) {
        selected->lastExecutedUs = now;
      } else {
        selected->lastExecutedUs += selected->desiredPeriodUs;
      }
      selected->deadlineFlagged = false;
    }
  }

  static uint32_t lastQueueSend = 0;
  now = micros(); // Оновлюємо перед перевіркою черги
  if (now - lastQueueSend >= 500000) { 
    lastQueueSend = now;
    
    // Перевірка чи чергу взагалі створено (на випадок помилки ініціалізації)
    if (telemetryQueue != NULL) {
      telemetryData_t msg;
      msg.ax = axG; msg.ay = ayG; msg.az = azG;
      msg.gx = gxDps; msg.gy = gyDps; msg.gz = gzDps;
      msg.imuRunCount = imuRunCount;
      imuRunCount = 0;
      msg.avgExecUs = tasks[0].avgExecTimeUs;
      msg.missedDeadlines = tasks[0].missedDeadlines;
      msg.imuHealthy = imuHealthy;

      msg.cpuLoadPct = ((float)cpuBusyTimeAccumulatorUs / 500000.0f) * 100.0f; 
      cpuBusyTimeAccumulatorUs = 0; 

      // xQueueSendToBack (не блокуємось, якщо черга повна)
      xQueueSend(telemetryQueue, &msg, 0); 
    }
  }
}

// ============================================================================
// 6. ТЕЛЕМЕТРІЯ (ЯДРО 0)
// ============================================================================

void taskTelemetryFreeRTOS(void *pvParameters) {
  telemetryData_t msg;
  for (;;) {
    if (xQueueReceive(telemetryQueue, &msg, portMAX_DELAY) == pdTRUE) {
      Serial.printf("Status: %s | Core 1 Load: %.1f%% | IMU Freq: %.0f Hz | Missed: %u\n",
                    msg.imuHealthy ? "OK" : "ERR", 
                    msg.cpuLoadPct,
                    (float)msg.imuRunCount / 0.5f, 
                    msg.missedDeadlines);
      Serial.printf("Accel (g):   X:%6.2f  Y:%6.2f  Z:%6.2f\n", msg.ax, msg.ay, msg.az);
      Serial.printf("Gyro (deg/s): X:%6.1f  Y:%6.1f  Z:%6.1f\n", msg.gx, msg.gy, msg.gz);
      Serial.println("----------------------------------------------------------");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  telemetryQueue = xQueueCreate(10, sizeof(telemetryData_t));
  if (telemetryQueue == NULL) {
    Serial.println("Error: Failed to create FreeRTOS queue!");
    while(1) delay(100); // Зупинка, якщо немає пам'яті
  }
  
  lsmInit();
  Serial.println("Calibrating IMU... Keep boat still!");
  calibrateIMU();
  
  // Запускаємо телеметрію на Ядрі 0
  xTaskCreatePinnedToCore(taskTelemetryFreeRTOS, "Telemetry", 4096, NULL, 1, NULL, 0);
}

void loop() {
  schedulerRun();
}