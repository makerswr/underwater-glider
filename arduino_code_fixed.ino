/*
 * Fixed: Stepper + MPU6050 (DMP) — Polling fallback for INT on non-interrupt pin
 * - Protects against FIFO packet-size > buffer overflow (uses 128-byte buffer)
 * - Reads dmpPacketSize once in setup()
 * - Limits packet drain loop to avoid pathological infinite loops
 * - Adds lightweight heartbeat so you can see device is alive even if no sensor output
 * - FIXED: JSON output format to match web interface expectations
 *
 * Wiring (as you reported):
 *  SDA -> D20
 *  SCL -> D21
 *  VCC -> 5V (or 3.3V depending on module)
 *  GND -> GND
 *  INT -> D4 (polling if D4 not interrupt-capable)
 *
 * Offsets applied:
 *   XAccel: -4361
 *   YAccel: -639
 *   ZAccel: 1569
 *   XGyro:  -3
 *   YGyro:  0
 *   ZGyro:  -13
 */

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

// Pins
const int stepPin = 3;
const int dirPin  = 2;
const int mpuInterruptPin = 4; // 현재 회로: D4 (외부 인터럽트가 아니면 폴링 모드)

// Motor
const int STEPS_PER_REV = 200;
volatile long stepsToGo = 0;

// Additional actuators
bool rudderActive = false;
bool elevatorActive = false;
bool aileronActive = false;
bool ballastActive = false;

// MPU object
MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus = 0;

// Make buffer big enough for larger DMP packets (safe margin)
uint8_t fifoBuffer[128]; // <= 128 bytes
uint16_t dmpPacketSize = 0; // cached packet size (from mpu.dmpGetFIFOPacketSize())

Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

unsigned long lastSensorSendTime = 0;
const long sensorSendInterval = 50; // ms (20 Hz)

// heartbeat
unsigned long lastHeartbeat = 0;
const long heartbeatInterval = 5000; // 5s

#define RAD_TO_DEG 57.295779513082320876f

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } // wait for Serial if needed
  Serial.println(F("=== Setup start ==="));

  // motor pins
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  // I2C (Mega: SDA=20, SCL=21)
  Wire.begin();
  Wire.setClock(400000);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  pinMode(mpuInterruptPin, INPUT); // if needed try INPUT_PULLUP

  Serial.print(F("MPU connection test: "));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection FAILED"));

  Serial.println(F("Initializing DMP..."));
  uint8_t devStatus = mpu.dmpInitialize();
  Serial.print(F("dmpInitialize() returned: "));
  Serial.println(devStatus);

  // Apply offsets (from your calibration)
  mpu.setXAccelOffset(-4361);
  mpu.setYAccelOffset(-639);
  mpu.setZAccelOffset(1569);

  mpu.setXGyroOffset(-3);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(-13);

  if (devStatus == 0) {
    Serial.println(F("DMP Initialization successful"));
    mpu.setDMPEnabled(true);

    // cache packet size once (avoid calling repeatedly and ensure consistency)
    dmpPacketSize = mpu.dmpGetFIFOPacketSize();
    Serial.print(F("DMP packet size (cached) = "));
    Serial.println(dmpPacketSize);

    int irq = digitalPinToInterrupt(mpuInterruptPin); // -1 if not interrupt-capable
    if (irq != -1) {
      attachInterrupt(irq, dmpDataReady, RISING);
      Serial.print(F("Interrupt attached on pin "));
      Serial.print(mpuInterruptPin);
      Serial.print(F(" (irq "));
      Serial.print(irq);
      Serial.println(F(")"));
    } else {
      Serial.print(F("Pin "));
      Serial.print(mpuInterruptPin);
      Serial.println(F(" is NOT external-interrupt-capable. Using POLLING for FIFO."));
    }

    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(") - will use ACCEL fallback / polling if possible"));
    dmpReady = false;
  }

  Serial.println(F("=== Setup done ==="));
}

void loop() {
  // 1) Serial command handling
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      // Motor commands (only when not moving)
      if (stepsToGo == 0 && (cmd.charAt(0) == '+' || cmd.charAt(0) == '-' || isDigit(cmd.charAt(0)))) {
        int8_t sign = 1;
        if (cmd.charAt(0) == '-') { sign = -1; cmd.remove(0,1); }
        else if (cmd.charAt(0) == '+') { cmd.remove(0,1); }

        bool isRev = false;
        if (cmd.length() > 0 && (cmd.charAt(0) == 'r' || cmd.charAt(0) == 'R')) {
          isRev = true; cmd.remove(0,1);
        }

        long val = cmd.toInt();
        if (val > 0) {
          stepsToGo = val * (isRev ? STEPS_PER_REV : 1);
          bool dirFwd = (sign > 0);
          digitalWrite(dirPin, dirFwd ? HIGH : LOW);
          Serial.print(F("MOTOR CMD accepted - stepsToGo: "));
          Serial.println(stepsToGo);
        } else {
          Serial.println(F("Invalid motor command (value <= 0)"));
        }
      }
      // Actuator commands
      else if (cmd.equalsIgnoreCase("rudder_on")) {
        rudderActive = true;
        Serial.println(F("RUDDER activated"));
      }
      else if (cmd.equalsIgnoreCase("rudder_off")) {
        rudderActive = false;
        Serial.println(F("RUDDER deactivated"));
      }
      else if (cmd.equalsIgnoreCase("elevator_on")) {
        elevatorActive = true;
        Serial.println(F("ELEVATOR activated"));
      }
      else if (cmd.equalsIgnoreCase("elevator_off")) {
        elevatorActive = false;
        Serial.println(F("ELEVATOR deactivated"));
      }
      else if (cmd.equalsIgnoreCase("aileron_on")) {
        aileronActive = true;
        Serial.println(F("AILERON activated"));
      }
      else if (cmd.equalsIgnoreCase("aileron_off")) {
        aileronActive = false;
        Serial.println(F("AILERON deactivated"));
      }
      else if (cmd.equalsIgnoreCase("ballast_on")) {
        ballastActive = true;
        Serial.println(F("BALLAST activated"));
      }
      else if (cmd.equalsIgnoreCase("ballast_off")) {
        ballastActive = false;
        Serial.println(F("BALLAST deactivated"));
      }
      else if (cmd.equalsIgnoreCase("all_off")) {
        rudderActive = false;
        elevatorActive = false;
        aileronActive = false;
        ballastActive = false;
        Serial.println(F("All actuators deactivated"));
      }
      else {
        Serial.print(F("Unknown command: "));
        Serial.println(cmd);
      }
    }
  }

  // 2) Stepper
  if (stepsToGo > 0) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
    stepsToGo--;
  }

  // 3) Sensor read & send
  readAndSendSensorData();

  // 4) Heartbeat if nothing else printed for a while
  if (millis() - lastHeartbeat >= heartbeatInterval) {
    lastHeartbeat = millis();
    Serial.println(F("[HEARTBEAT] alive"));
  }
}

void readAndSendSensorData() {
  if (millis() - lastSensorSendTime < sensorSendInterval) return;
  lastSensorSendTime = millis();

  // 센서 데이터 변수들 (웹 인터페이스 형식에 맞춤)
  float accX = 0.0f, accY = 0.0f, accZ = 0.0f;
  float gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f;
  float temperature = 25.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  float roll = 0.0f;
  float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
  const char* source = "NONE";

  if (dmpReady) {
    // If interrupt mode is available and an interrupt occurred
    if (mpuInterrupt) {
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();
      uint16_t fifoCount = mpu.getFIFOCount();
      uint16_t packetSize = dmpPacketSize; // use cached size

      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // overflow
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow! Reset FIFO."));
      } else if (packetSize > 0 && fifoCount >= packetSize) {
        // safe guard: ensure packetSize fits buffer
        if (packetSize > sizeof(fifoBuffer)) {
          Serial.print(F("ERROR: packetSize > fifoBuffer ("));
          Serial.print(packetSize);
          Serial.print(F(" > "));
          Serial.print(sizeof(fifoBuffer));
          Serial.println(F("). Skipping DMP read."));
        } else {
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          // Normalize quaternion to avoid invalid slerp behavior on frontend
          {
            float n = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
            if (n > 0.00001f) {
              q.w /= n; q.x /= n; q.y /= n; q.z /= n;
            }
            // Optional: force hemisphere to avoid sign flips
            if (q.w < 0) { q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z; }
          }
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          
          // DMP에서 가속도와 자이로 데이터 추출 (올바른 방법)
          VectorInt16 aa, gy;
          mpu.dmpGetAccel(&aa, fifoBuffer);
          mpu.dmpGetGyro(&gy, fifoBuffer);
          
          // 단위 변환
          accX = (float)aa.x / 16384.0f * 9.81f; // m/s²
          accY = (float)aa.y / 16384.0f * 9.81f;
          accZ = (float)aa.z / 16384.0f * 9.81f;
          
          gyroX = (float)gy.x / 131.0f; // °/s
          gyroY = (float)gy.y / 131.0f;
          gyroZ = (float)gy.z / 131.0f;
          
          pitch = ypr[1] * RAD_TO_DEG;
          yaw = ypr[0] * RAD_TO_DEG;
          roll = ypr[2] * RAD_TO_DEG;
          qw = q.w; qx = q.x; qy = q.y; qz = q.z;
          source = "DMP";
        }
      }
    }

    // Polling fallback: check FIFOCount and read latest packet(s)
    if (strcmp(source, "DMP") != 0) {
      uint16_t fifoCount = mpu.getFIFOCount();
      uint16_t packetSize = dmpPacketSize; // cached
      if (packetSize > 0 && fifoCount >= packetSize) {
        if (packetSize > sizeof(fifoBuffer)) {
          Serial.print(F("ERROR: packetSize > fifoBuffer ("));
          Serial.print(packetSize);
          Serial.print(F(" > "));
          Serial.print(sizeof(fifoBuffer));
          Serial.println(F("). Skipping DMP read."));
        } else {
          // Drain up to maxDrain iterations to avoid pathological long loops
          int maxDrain = 10;
          int drains = 0;
          while (fifoCount >= packetSize && drains < maxDrain) {
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount = mpu.getFIFOCount();
            drains++;
          }
          // Use last-read packet in fifoBuffer
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          // Normalize quaternion to avoid invalid slerp behavior on frontend
          {
            float n = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
            if (n > 0.00001f) {
              q.w /= n; q.x /= n; q.y /= n; q.z /= n;
            }
            // Optional: force hemisphere to avoid sign flips
            if (q.w < 0) { q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z; }
          }
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          
          // DMP에서 가속도와 자이로 데이터 추출 (올바른 방법)
          VectorInt16 aa, gy;
          mpu.dmpGetAccel(&aa, fifoBuffer);
          mpu.dmpGetGyro(&gy, fifoBuffer);
          
          // 단위 변환
          accX = (float)aa.x / 16384.0f * 9.81f; // m/s²
          accY = (float)aa.y / 16384.0f * 9.81f;
          accZ = (float)aa.z / 16384.0f * 9.81f;
          
          gyroX = (float)gy.x / 131.0f; // °/s
          gyroY = (float)gy.y / 131.0f;
          gyroZ = (float)gy.z / 131.0f;
          
          pitch = ypr[1] * RAD_TO_DEG;
          yaw = ypr[0] * RAD_TO_DEG;
          roll = ypr[2] * RAD_TO_DEG;
          qw = q.w; qx = q.x; qy = q.y; qz = q.z;
          source = "DMP-POLL";
        }
      }
    }
  }

  // ACCEL fallback when no DMP value
  if (strcmp(source, "NONE") == 0) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);

    // Raw 값을 실제 물리량으로 변환
    accX = (float)ax / 16384.0f * 9.81f; // m/s²
    accY = (float)ay / 16384.0f * 9.81f;
    accZ = (float)az / 16384.0f * 9.81f;
    
    gyroX = (float)gx / 131.0f; // °/s
    gyroY = (float)gy / 131.0f;
    gyroZ = (float)gz / 131.0f;
    
    // 간단한 pitch/roll 계산 (yaw는 자력계 없이 불가)
    pitch = atan2(-accX, sqrt(accY*accY + accZ*accZ)) * RAD_TO_DEG;
    roll  = atan2(accY, accZ) * RAD_TO_DEG;
    yaw   = 0.0f;
    source = "ACCEL";
  }

  // 온도 센서 읽기
  temperature = mpu.getTemperature() / 340.0f + 36.53f;

  // depth example (실제로는 압력 센서에서 읽어야 함)
  float depth = (float)random(0, 500) / 10.0f;

  // Determine glider state based on all actuators
  String gliderState = "IDLE";
  if (stepsToGo > 0) {
    gliderState = "MOTOR_MOVING";
  } else if (rudderActive || elevatorActive || aileronActive) {
    gliderState = "CONTROL_ACTIVE";
  } else if (ballastActive) {
    gliderState = "BALLAST_ACTIVE";
  }

  // 웹 인터페이스와 호환되는 JSON 출력
  Serial.print("{\"accX\":"); Serial.print(accX, 2);
  Serial.print(",\"accY\":"); Serial.print(accY, 2);
  Serial.print(",\"accZ\":"); Serial.print(accZ, 2);
  Serial.print(",\"gyroX\":"); Serial.print(gyroX, 2);
  Serial.print(",\"gyroY\":"); Serial.print(gyroY, 2);
  Serial.print(",\"gyroZ\":"); Serial.print(gyroZ, 2);
  Serial.print(",\"temperature\":"); Serial.print(temperature, 1);
  Serial.print(",\"pitch\":"); Serial.print(pitch, 2);
  Serial.print(",\"yaw\":"); Serial.print(yaw, 2);
  Serial.print(",\"roll\":"); Serial.print(roll, 2);
  Serial.print(",\"qw\":"); Serial.print(qw, 6);
  Serial.print(",\"qx\":"); Serial.print(qx, 6);
  Serial.print(",\"qy\":"); Serial.print(qy, 6);
  Serial.print(",\"qz\":"); Serial.print(qz, 6);
  Serial.print(",\"depth\":"); Serial.print(depth, 2);
  Serial.print(",\"glider_state\":\""); Serial.print(gliderState);
  Serial.print("\",\"actuators\":{");
  Serial.print("\"motor\":"); Serial.print(stepsToGo > 0 ? "true" : "false");
  Serial.print(",\"rudder\":"); Serial.print(rudderActive ? "true" : "false");
  Serial.print(",\"elevator\":"); Serial.print(elevatorActive ? "true" : "false");
  Serial.print(",\"aileron\":"); Serial.print(aileronActive ? "true" : "false");
  Serial.print(",\"ballast\":"); Serial.print(ballastActive ? "true" : "false");
  Serial.print("},\"source\":\""); Serial.print(source);
  Serial.println("\"}");
}
