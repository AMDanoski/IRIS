// Andrew Reynolds
// works to communicate with pi and receive a set rpm command
// SET: RPM$ - to turn on
// SET: 0 - to turn off
// Motors can be set to any rpm [1000, 9000] during operation
// On startup each motor should start at the same time and spinup at the same rate

#include <Arduino.h>
#include <avr/wdt.h>
#include <PinChangeInterrupt.h>

// ================= USER SETTINGS =================
float targetRPMs[4] = {0, 0, 0, 0};

const unsigned long RPM_SAMPLE_MS   = 50;   // can adjust
const unsigned long SERIAL_INTERVAL = 150;  // can adjust
const unsigned int  MIN_EDGE_US     = 250;  // can adjust (may effect encoder accuracy)

// ================= RAMP CONTROL =================
unsigned long motorStartTimes[4] = {0, 0, 0, 0};
bool motorStarted[4]             = {false, false, false, false};
const unsigned long RAMP_DURATION = 6000;
const unsigned long MOTOR_DELAY   = 8000;
bool initialRunupDone = false;

const unsigned long DUTY_RAMP_MS = 15000;

// PID constants
float Kp = 0.06, Ki = 0.0002, Kd = 0.021;

// ================= PIN SETUP =================
const int PWM_PINS[4]   = {3, 5, 6, 9};
const int ENC_A_PINS[4] = {2, 12, 10, A0};

// ================= STATE =================
volatile unsigned long encoderCounts[4]  = {0};
volatile unsigned long lastEdgeMicros[4] = {0};

unsigned long prevCounts[4]  = {0};
float sampledRPM[4]          = {0};

float pwmDuties[4]   = {0};
float integrals[4]   = {0};
float lastErrors[4]  = {0};

unsigned long lastRPMSample = 0;
unsigned long lastSerial    = 0;

unsigned long dutyRampStart[4] = {0};

bool allMotorsRunning = false;

// ================= SERIAL =================
char serialBuffer[256];
uint8_t serialIndex = 0;
float servo_cmd = 0;

// =====================================================
// NEW: SMOOTHED RPM TARGETS
// =====================================================
float rampTargetRPM[4] = {0};
const float MAX_RPM_STEP = 0.015;  // max rpm increase per 50ms sample

void smoothRPMTargets() {
  for (int i = 0; i < 4; i++) {
    float diff = targetRPMs[i] - rampTargetRPM[i];
    if (fabs(diff) <= MAX_RPM_STEP) {
      rampTargetRPM[i] = targetRPMs[i];
    } else {
      rampTargetRPM[i] += (diff > 0 ? MAX_RPM_STEP : -MAX_RPM_STEP);
    }
  }
}

// =====================================================
// ENCODER INTERRUPTS
// =====================================================
inline void countEdge(uint8_t i) {
  unsigned long now = micros();
  if ((uint16_t)(now - lastEdgeMicros[i]) > MIN_EDGE_US) {
    lastEdgeMicros[i] = now;
    encoderCounts[i]++;
  }
}

void ISR_M1() { if (digitalRead(2))  countEdge(0); }
void ISR_M2() { if (digitalRead(12)) countEdge(1); }
void ISR_M3() { if (digitalRead(10)) countEdge(2); }
void ISR_M4() { if (digitalRead(A0)) countEdge(3); }

// =====================================================
// COMPUTE RPM
// =====================================================
float computeRPM(uint8_t i, unsigned long deltaCount) {
  const float CPR = 12.0;
  float revs = deltaCount / CPR;
  return revs * (60000.0 / RPM_SAMPLE_MS);
}

// =====================================================
// PID CONTROL
// =====================================================
void updateMotorPID(int idx, float rampedRPM) {
  float rpm = sampledRPM[idx];
  float error = rampedRPM - rpm;

  integrals[idx] += error * (RPM_SAMPLE_MS / 1000.0);
  float derivative = (error - lastErrors[idx]) / (RPM_SAMPLE_MS / 1000.0);
  lastErrors[idx] = error;

  float raw = Kp * error + Ki * integrals[idx] + Kd * derivative;
  pwmDuties[idx] += raw / max(rampedRPM, 1.0f);
  pwmDuties[idx] = constrain(pwmDuties[idx], 0, 1);

  unsigned long now = millis();
  if (dutyRampStart[idx] > 0 && (now - dutyRampStart[idx]) < DUTY_RAMP_MS) {
    float rampFrac = (now - dutyRampStart[idx]) / (float)DUTY_RAMP_MS;
    rampFrac = constrain(rampFrac, 0.0f, 1.0f);
    analogWrite(PWM_PINS[idx], pwmDuties[idx] * 255 * rampFrac);
  } else {
    analogWrite(PWM_PINS[idx], pwmDuties[idx] * 255);
    dutyRampStart[idx] = 0;
  }
}

// =====================================================
// RAMP CONTROL
// =====================================================
float rampRPM(int idx, unsigned long now) {
  if (!motorStarted[idx]) return 0;
  unsigned long elapsed = now - motorStartTimes[idx];
  if (elapsed >= RAMP_DURATION) return targetRPMs[idx];
  return targetRPMs[idx] * (elapsed / (float)RAMP_DURATION);
}

// =====================================================
// SERIAL COMMANDS
// =====================================================
void parseCommand(const char* line) {
  if (strncmp(line, "SET:", 4) != 0) return;
  float rpm_cmd = atof(line + 4);
  servo_cmd = rpm_cmd;

  if (rpm_cmd <= 0) {
    for (int i = 0; i < 4; i++) {
      targetRPMs[i] = 0;
      rampTargetRPM[i] = 0;
      pwmDuties[i]  = 0;
      sampledRPM[i] = 0;
      motorStarted[i] = false;
      analogWrite(PWM_PINS[i], 0);
    }
    allMotorsRunning = false;
    initialRunupDone = false;
    return;
  }

  for (int i = 0; i < 4; i++) targetRPMs[i] = rpm_cmd;

  unsigned long now = millis();

  if (!initialRunupDone) {
    motorStartTimes[0] = now;
    motorStarted[0] = true;
    dutyRampStart[0] = now;

    motorStartTimes[2] = now;
    motorStarted[2] = true;
    dutyRampStart[2] = now;

    motorStarted[1] = false;
    motorStarted[3] = false;
    dutyRampStart[1] = 0;
    dutyRampStart[3] = 0;

    initialRunupDone = true;
    allMotorsRunning = false;
    return;
  }

  allMotorsRunning = true;
}

// =====================================================
// FAST SERIAL INPUT
// =====================================================
void handleSerial() {
    while (Serial.available()) {
        char c = Serial.read();

        // flush buffer if too old / too long
        if (serialIndex >= sizeof(serialBuffer) - 1) {
            serialIndex = 0;
        }

        if (c == '\n' || c == '\r') {
            serialBuffer[serialIndex] = '\0';
            parseCommand(serialBuffer);
            serialIndex = 0;

            // OPTIONAL: flush unused waiting characters
            while (Serial.available()) Serial.read();

        } else {
            serialBuffer[serialIndex++] = c;
        }
    }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  wdt_enable(WDTO_2S);
  Serial.begin(115200);
  delay(200);

  for (int i = 0; i < 4; i++) {
    pinMode(PWM_PINS[i], OUTPUT);
    analogWrite(PWM_PINS[i], 0);
  }

  pinMode(2,  INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(A0, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(2), ISR_M1, RISING);
  attachPCINT(digitalPinToPCINT(12), ISR_M2, RISING);
  attachPCINT(digitalPinToPCINT(10), ISR_M3, RISING);
  attachPCINT(digitalPinToPCINT(A0), ISR_M4, RISING);
}

// =====================================================
// LOOP  (with target RPM smoothing added)
// =====================================================
void loop() {
  wdt_reset();
  handleSerial();

  unsigned long now = millis();

  // NEW: smooth target RPM transitions to prevent voltage sag
  smoothRPMTargets();

  // ===== GROUPED STARTUP =====
  if (!allMotorsRunning) {
    if ((motorStarted[0] || motorStarted[2]) && !(motorStarted[1] && motorStarted[3])) {
      unsigned long groupAstart = motorStartTimes[0];
      if (now - groupAstart >= MOTOR_DELAY) {
        if (!motorStarted[1] && targetRPMs[1] > 0) {
          motorStartTimes[1] = now;
          motorStarted[1] = true;
          dutyRampStart[1] = now;
        }
        if (!motorStarted[3] && targetRPMs[3] > 0) {
          motorStartTimes[3] = now;
          motorStarted[3] = true;
          dutyRampStart[3] = now;
        }
        if (!motorStarted[0] && targetRPMs[0] > 0) {
        motorStartTimes[0] = now;
        motorStarted[0] = true;
        dutyRampStart[0] = now;
        }
      }
      if (!motorStarted[2] && targetRPMs[2] > 0) {
        motorStartTimes[2] = now;
        motorStarted[2] = true;
        dutyRampStart[2] = now;
      }
    }

    bool done = true;
    for (int i = 0; i < 4; i++) {
      if (!motorStarted[i]) done = false;
    }
    if (done) allMotorsRunning = true;
  }

  // ===== RPM SAMPLING & PID =====
  if (servo_cmd > 0 && now - lastRPMSample >= RPM_SAMPLE_MS) {
    lastRPMSample = now;
    for (int i = 0; i < 4; i++) {
      noInterrupts();
      unsigned long countNow = encoderCounts[i];
      interrupts();

      unsigned long delta = countNow - prevCounts[i];
      prevCounts[i] = countNow;
      sampledRPM[i] = computeRPM(i, delta);

      if (motorStarted[i]) {
        updateMotorPID(i, rampTargetRPM[i]);  // <-- USE SMOOTHED TARGET HERE
      }
    }
  }

  // ===== SERIAL OUTPUT =====
  if (now - lastSerial >= SERIAL_INTERVAL) {
    lastSerial = now;
    Serial.print(F("Motor RPM: "));
    for (int i = 0; i < 4; i++) {
      Serial.print((int)sampledRPM[i]);
      if (i < 3) Serial.write(',');
    }
    Serial.write('\n');

  }
}