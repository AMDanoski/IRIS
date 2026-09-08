// Andrew Reynolds
// Project ODIN - Capstone
// Dec. 2025

/* ============================================================
   SERIAL COMMAND REFERENCE — GIMBALS + LINEAR ACTUATORS
   ============================================================

   The system accepts two types of commands over Serial:

   ------------------------------------------------------------
   1)  SET:  (GIMBAL ROTATION COMMAND)
   ------------------------------------------------------------
   Format:
       SET:w1,w2,w3,w4

   Where:
       w1..w4 = angular velocities in radians per second
                for the 4 gimbal motors:

                Motor Indexing:
                  0 = Gimbal 1
                  1 = Gimbal 2
                  2 = Gimbal 3
                  3 = Gimbal 4

   Meaning:
     - The firmware converts radians/sec → RPM automatically.
     - Smooth acceleration filtering is applied (soft start / stop).
     - Directions follow the sign of the given rad/s value.
     - A value near 0 stops that gimbal motor.
     - If *all* gimbals are at zero, the driver bank is disabled
       to reduce heat and power draw.

   Example:
       SET:0.0,1.57,-1.57,0.0

       → Gimbal 2 spins CCW at 1.57 rad/s
       → Gimbal 3 spins CW at 1.57 rad/s
       → Gimbal 1 & 4 stop


   ------------------------------------------------------------
   2)  LIN:  (LINEAR ACTUATOR POSITION COMMAND)
   ------------------------------------------------------------
   Format:
       LIN:p1,p2,p3,p4

   Where:
       p1..p4 = desired actuator lengths in millimeters

                Motor Indexing:
                  lin0 = Motor 4  (driver idx 4)
                  lin1 = Motor 5  (driver idx 5)
                  lin2 = Motor 6  (driver idx 6)
                  lin3 = Motor 7  (driver idx 7)

   Meaning:
     - The firmware converts mm → microsteps using:
            microstepsPerMM = 1596.0
     - Each actuator moves toward its commanded mm setpoint:
         • Larger mm  → extend away from the home limit
         • Smaller mm → retract back toward the home limit
     - Positions are always relative to the homed zero point.
     - Motion is step-timed in the main loop (non-blocking).


   Example:
       LIN:25.0,40.0,10.0,50.0

       → Extend actuators to those mm lengths relative to zero.


   ------------------------------------------------------------
   HOMING + CALIBRATION PROCESS (Automatic in Setup)
   ------------------------------------------------------------

   On startup, the firmware automatically performs:

       1. Calibrate StallGuard for Linear Motors (4–7)
       2. Home Linear Motors (4–7)
       3. Calibrate StallGuard for Gimbal Motors (0–3)
       4. Home Gimbal Motors (0–3)
       5. Enter command-waiting main loop

   StallGuard calibration collects SG baseline values for each
   motor by sampling sg_result() while moving both directions.

   Homing drives each motor toward its mechanical stop until
   StallGuard detects a stall, then backs off a fixed distance
   and sets software position to zero.
  ************************************************************
 *  STALLGUARD SAFETY LIMIT
 *
 *  Each motor has a maximum allowed travel distance during homing.
 *  If StallGuard never triggers before this threshold, the motor
 *  immediately stops and homing fails safely.
    Adjust these numbers for your hardware.


   ------------------------------------------------------------
   TELEMETRY / DATA OUTPUT
   ------------------------------------------------------------
   The firmware periodically sends:

       DATA:a0,a1,a2,a3,l0,l1,l2,l3

   Where:
       a0..a3 = gimbal absolute angles in radians (0–2π)
       l0..l3 = target linear actuator lengths (mm)

   This is used by the Pi for live monitoring.


   ------------------------------------------------------------
   MOTOR INDEXING SUMMARY
   ------------------------------------------------------------

   drivers[0] = Gimbal 1
   drivers[1] = Gimbal 2
   drivers[2] = Gimbal 3
   drivers[3] = Gimbal 4
   drivers[4] = Linear Actuator 1
   drivers[5] = Linear Actuator 2
   drivers[6] = Linear Actuator 3
   drivers[7] = Linear Actuator 4

   DIR_PINS, STEP_PINS, CS_PINS mirror this index order.


   ------------------------------------------------------------
   IMPORTANT NOTES
   ------------------------------------------------------------

   - SET: affects only motors 0–3.
   - LIN: affects only motors 4–7.
   - Commands must end with '\n' or '\r'.
   - Serial buffer is automatically cleared after each command.
   - Firmware avoids TX overflow using Serial.availableForWrite().
   - All motors must be homed before they will follow SET/LIN.
   - sgBaseline[] is indexed per motor — no overlap occurs
     between gimbal and linear calibration ranges.

   ============================================================
*/


#include <TMCStepper.h>
#include <SPI.h>
#include <digitalWriteFast.h>

// =======================================================
// MEGA PIN MAPPING (8× TMC5160)
// =======================================================

// === Stepper Motors ===

#define EN_A_PIN 7

// Gimbal Motor 1
#define DIR1_PIN 22
#define STEP1_PIN 23
#define CS1_PIN 24

// Gimbal Motor 2
#define DIR2_PIN 25
#define STEP2_PIN 26
#define CS2_PIN 27

// Gimbal Motor 3
#define DIR3_PIN 28
#define STEP3_PIN 29
#define CS3_PIN 30

// Gimbal Motor 4
#define DIR4_PIN 31
#define STEP4_PIN 32
#define CS4_PIN 33


// === Linear Actuators ===

#define EN_B_PIN 6
// Lin Motor 1
#define DIR5_PIN 34
#define STEP5_PIN 35
#define CS5_PIN 36

// Lin Motor 2
#define DIR6_PIN 37
#define STEP6_PIN 38
#define CS6_PIN 39

// Lin Motor 3
#define DIR7_PIN 40
#define STEP7_PIN 41
#define CS7_PIN 42

// Lin Motor 4
#define DIR8_PIN 43
#define STEP8_PIN 44
#define CS8_PIN 45


// Hardware SPI (Mega)
#define MOSI_PIN 51
#define MISO_PIN 50
#define SCK_PIN 52

#define R_SENSE 0.075f

// =======================================================
// MOTOR CONSTANTS
// =======================================================
const float fullStepDeg = 1.8f;
const int microsteps = 16;
const float degPerMicro = fullStepDeg / microsteps;
const float microstepsPerRev = 200.0f * microsteps;

// radians/sec → RPM = (rad/s * 60) / (2π)
const float RADS_TO_RPM = 60.0f / (2.0f * PI);

// Optional offsets for all motors; follow index guide at the top
float angleOffset[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

// =======================================================
// DRIVERS
// =======================================================
TMC5160Stepper driver1(CS1_PIN, R_SENSE);
TMC5160Stepper driver2(CS2_PIN, R_SENSE);
TMC5160Stepper driver3(CS3_PIN, R_SENSE);
TMC5160Stepper driver4(CS4_PIN, R_SENSE);
TMC5160Stepper driver5(CS5_PIN, R_SENSE);
TMC5160Stepper driver6(CS6_PIN, R_SENSE);
TMC5160Stepper driver7(CS7_PIN, R_SENSE);
TMC5160Stepper driver8(CS8_PIN, R_SENSE);

TMC5160Stepper* drivers[8] = { &driver1, &driver2, &driver3, &driver4, &driver5, &driver6, &driver7, &driver8 };


int DIR_PINS[8] = { DIR1_PIN, DIR2_PIN, DIR3_PIN, DIR4_PIN, DIR5_PIN, DIR6_PIN, DIR7_PIN, DIR8_PIN };

int STEP_PINS[8] = { STEP1_PIN, STEP2_PIN, STEP3_PIN, STEP4_PIN, STEP5_PIN, STEP6_PIN, STEP7_PIN, STEP8_PIN };

int CS_PINS[8] = { CS1_PIN, CS2_PIN, CS3_PIN, CS4_PIN, CS5_PIN, CS6_PIN, CS7_PIN, CS8_PIN };

String serialCmd;

// =======================================================
// STALLGUARD HOMING FUNCTIONS
// =======================================================

/************************************************************
 *  STALLGUARD SAFETY LIMIT
 *
 *  Each motor has a maximum allowed travel distance during homing.
 *  If StallGuard never triggers before this threshold, the motor
 *  immediately stops and homing fails safely.
 *
 *  Example values:
 *      • Gimbals:  8000 steps max
 *      • Linears: 200000 steps max
 *
 *  Adjust these numbers for your hardware.
 ************************************************************/
long maxHomingTravel[8] = {
  8000, 8000, 8000, 8000,         // gimbals 0–3
  200000, 200000, 200000, 200000  // linears 4–7
};

// --- Gimbal Homing Parameters ---
const int stallThresholds[8] = { 300, 300, 300, 300, 300, 300, 300, 300 };  // lower = more sensitive

const int stepPulseUs = 5;   // step high time
const int stepDelayUs = 50;  // delay between steps (controls speed)

// Gimbal motors (0–3)
const int gimbalStepPulseUs = 5;
const int gimbalStepDelayUs = 500;

// Linear actuators (4–7)
const int linearStepPulseUs = 3;
const int linearStepDelayUs = 100;

// Calibration pulses (µs)
const int gimbalCalPulseUs = 5;
const int gimbalCalDelayUs = 500;

const int linearCalPulseUs = 3;
const int linearCalDelayUs = 100;


const int backoffSteps[8] = { 50, 1500, 1500, 10, 10, 10, 10, 10 };  // how far to back off after stall (per motor)
bool spinDirection[4] = { false, true, true, false };                // initial turn direction for homing
int sgBaseline[4] = { 200, 300, 300, 300 };                          // default backup values
int homingCurrent[8] = { 300, 300, 300, 300, 300, 300, 300, 300 };
const int stallSensitivity = 50;  // can be tuned

unsigned long lastGimbalStepMicros[4] = { 0, 0, 0, 0 };
unsigned long lastLinearStepMicros[4] = { 0, 0, 0, 0 };
unsigned long gimbalStepIntervalMicros[4] = { 900, 900, 900, 900 };
unsigned long linearStepIntervalMicros[4] = { 900, 900, 900, 900 };


// FUYU FSK30J linear actuator calibration
const float microstepsPerMM = 1596.0f;
const float mmPerMicrostep = 1.0f / microstepsPerMM;

long motorPos[4] = { 0, 0, 0, 0 };   // current microstep position
long targetPos[4] = { 0, 0, 0, 0 };  // commanded target (microsteps)

unsigned long lastStepMicros[4] = { 0, 0, 0, 0 };
unsigned long stepIntervalMicros[4] = { 900, 900, 900, 900 };  // move speed control

// =======================================================
// Motor State Tracking
// =======================================================
float targetRPM[4] = { 0, 0, 0, 0 };
float smoothRPM[4] = { 0, 0, 0, 0 };


float alpha = 0.05f;  // smoothing factor

bool motorEnabled[4] = { false, false, false, false };

long motorPosition[4] = { 0, 0, 0, 0 };
bool linActive = false;  // true when a new LIN command is received


// =======================================================
// SETUP
// =======================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  // === CS pin setup ===
  for (int i = 0; i < 8; i++) {
    pinMode(CS_PINS[i], OUTPUT);
    digitalWrite(CS_PINS[i], HIGH);
    delay(10);
  }
  delay(200);

  // === SPI setup ===
  // SPI.setRX(MISO_PIN); // Mega automatically assigns spi pins
  // SPI.setTX(MOSI_PIN);
  // SPI.setSCK(SCK_PIN);
  SPI.begin();
  delay(200);

  // === Enable drivers ===
  pinMode(EN_A_PIN, OUTPUT);
  pinMode(EN_B_PIN, OUTPUT);
  digitalWrite(EN_A_PIN, LOW);
  digitalWrite(EN_B_PIN, LOW);
  delay(20);

  // === Driver init ===
  for (int i = 0; i < 8; i++) {
    pinMode(DIR_PINS[i], OUTPUT);
    pinMode(STEP_PINS[i], OUTPUT);

    drivers[i]->begin();
    delay(20);
    drivers[i]->toff(4);
    drivers[i]->blank_time(24);
    drivers[i]->microsteps(microsteps);
    drivers[i]->rms_current(300);
    drivers[i]->en_pwm_mode(true);
    drivers[i]->pwm_freq(3);
    drivers[i]->pwm_autoscale(true);
    drivers[i]->intpol(true);
  }


  //Serial.println("\n=== STEP 1: CALIBRATE LINEAR ACTUATORS ===");
  for (int i = 4; i < 8; i++) {
    calibrateSG(i, true);
    delay(20);
  }

  //Serial.println("\n=== STEP 2: HOME LINEAR ACTUATORS ===");
  for (int i = 4; i < 8; i++) {
    homeLINMotor(i);
    delay(20);
  }

  //Serial.println("\n=== STEP 3: CALIBRATE GIMBAL MOTORS ===");
  for (int i = 0; i < 4; i++) {
    calibrateSG(i, false);
    delay(20);
  }

  //Serial.println("\n=== STEP 4: HOME GIMBAL MOTORS ===");
  for (int i = 0; i < 4; i++) {
    homeSingleMotor(i, true);
    delay(300);
  }

  Serial.println("\n=== ALL MOTORS CALIBRATED + HOMED ===\n");
}

// =======================================================
// MAIN LOOP
// =======================================================
void loop() {
  handleSerial();
  smoothRates();
  runMotors();
  runLinearMotors();  // only steps when linActive = true
  sendAngles();
}


// =======================================================
// SERIAL INPUT FORMAT:  SET: rate1,rate2,rate3,rate4
// (rates in radians per second)
// =======================================================
void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialCmd.length() > 0) {
        parseCommand(serialCmd);
        serialCmd = "";
      }
    } else {
      serialCmd += c;
    }
  }
}


void parseCommand(String cmd) {
  cmd.trim();

  bool isSet = cmd.startsWith("SET:");
  bool isLin = cmd.startsWith("LIN:");

  if (!isSet && !isLin) return;

  cmd.remove(0, 4);

  float w1, w2, w3, w4;
  int n = sscanf(cmd.c_str(), "%f,%f,%f,%f", &w1, &w2, &w3, &w4);

  if (n == 4) {
    if (isSet) {
      targetRPM[0] = w1 * RADS_TO_RPM;
      targetRPM[1] = w2 * RADS_TO_RPM;
      targetRPM[2] = w3 * RADS_TO_RPM;
      targetRPM[3] = w4 * RADS_TO_RPM;
    }

    if (isLin) {
      targetPos[0] = (long)(w1 * microstepsPerMM);
      targetPos[1] = (long)(w2 * microstepsPerMM);
      targetPos[2] = (long)(w3 * microstepsPerMM);
      targetPos[3] = (long)(w4 * microstepsPerMM);
      linActive = true;
    }
  }
}

// =======================================================
// ACCELERATION SMOOTHING
// =======================================================
void smoothRates() {
  const float accel = 50.0f;  // RPM change per second
  const float dt = 0.02f;     // 20 ms timestep

  bool allZero = true;

  for (int i = 0; i < 4; i++) {
    float error = targetRPM[i] - smoothRPM[i];

    // Limit the rate of change
    float maxChange = accel * dt;

    if (error > maxChange) error = maxChange;
    if (error < -maxChange) error = -maxChange;

    smoothRPM[i] += error;

    if (fabs(smoothRPM[i]) > 0.01f)
      allZero = false;
  }

  if (allZero) {
    digitalWrite(EN_A_PIN, HIGH);
    for (int i = 0; i < 4; i++) motorEnabled[i] = false;
    return;
  }

  digitalWrite(EN_A_PIN, LOW);

  for (int i = 0; i < 4; i++) {
    float rpm = smoothRPM[i];

    if (fabs(rpm) < 0.01f) {
      motorEnabled[i] = false;
      continue;
    }

    motorEnabled[i] = true;
    digitalWrite(DIR_PINS[i], rpm > 0 ? HIGH : LOW);

    float revsPerSec = fabs(rpm) / 60.0f;
    float microstepsPerSec = revsPerSec * microstepsPerRev;

    stepIntervalMicros[i] = (unsigned long)(1e6f / microstepsPerSec);
    if (stepIntervalMicros[i] < 200) stepIntervalMicros[i] = 200;
    if (stepIntervalMicros[i] > 20000) stepIntervalMicros[i] = 20000;
  }
}


// =======================================================
// MOTOR STEPPING
// =======================================================
void runMotors() {
  unsigned long now = micros();

  for (int i = 0; i < 4; i++) {
    if (!motorEnabled[i]) continue;

    if (now - lastGimbalStepMicros[i] >= stepIntervalMicros[i]) {
      lastGimbalStepMicros[i] = now;

      digitalWriteFast(STEP_PINS[i], HIGH);
      delayMicroseconds(gimbalStepPulseUs);
      digitalWriteFast(STEP_PINS[i], LOW);

      motorPosition[i] += (digitalRead(DIR_PINS[i]) ? 1 : -1);
    }
  }
}


// =======================================================
// SEND ANGLES (degrees, 0–360°) converted to rad/sec
// =======================================================
void sendAngles() {
    static unsigned long last = 0;

    if (millis() - last < 50) return;  // 20 Hz update
    last = millis();

    // --- SAFETY CHECK: avoid TX overflow ---
    if (Serial.availableForWrite() < 64) return;

    // --- Compute gimbal angles in radians ---
    float a[4];
    for (int i = 0; i < 4; i++) {
        a[i] = motorPosition[i] * degPerMicro + angleOffset[i];

        // Normalize to [0, 360)
        while (a[i] < 0) a[i] += 360.0f;
        while (a[i] >= 360) a[i] -= 360.0f;

        // Convert to radians
        a[i] *= PI / 180.0f;
    }

    // --- Print DATA ---
    char out[100];
    sprintf(out,
            "DATA:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
            a[0], a[1], a[2], a[3],
            targetPos[0], targetPos[1], targetPos[2], targetPos[3]);

    Serial.write(out);
}


// =======================================================
// HOME SINGLE GIMBAL MOTOR
// =======================================================
void homeSingleMotor(int idx, bool homeDirLow) {
  TMC5160Stepper* drv = drivers[idx];

  // Set homing direction
  digitalWriteFast(DIR_PINS[idx], homeDirLow ? LOW : HIGH);

  // Configure driver for StallGuard homing
  drv->rms_current(homingCurrent[idx]);
  drv->microsteps(16);
  drv->en_pwm_mode(false);  // disable stealthChop for SG
  drv->diag1_stall(true);

  long stepCount = 0;

  while (true) {
    // Step pulse
    digitalWriteFast(STEP_PINS[idx], HIGH);
    delayMicroseconds(stepPulseUs);
    digitalWriteFast(STEP_PINS[idx], LOW);
    stepCount++;

    // Update software motor position
    motorPosition[idx] += (homeDirLow ? -1 : 1);

    // Safety limit
    if (stepCount > maxHomingTravel[idx]) {
      Serial.print("ERROR: Gimbal motor ");
      Serial.print(idx);
      Serial.println(" exceeded safe homing travel! Stopping.");
      break;
    }

    // StallGuard detection
    uint16_t sg = drv->sg_result();
    if (sg < sgBaseline[idx] - stallSensitivity) {
      Serial.print("Stall detected on gimbal motor ");
      Serial.println(idx);
      break;
    }
  }

  // Back off from mechanical stop
  digitalWriteFast(DIR_PINS[idx], homeDirLow ? HIGH : LOW);
  for (int b = 0; b < backoffSteps[idx]; b++) {
    digitalWriteFast(STEP_PINS[idx], HIGH);
    delayMicroseconds(stepPulseUs);
    digitalWriteFast(STEP_PINS[idx], LOW);

    // Update position
    motorPosition[idx] += (homeDirLow ? 1 : -1);
  }

  // Reset position to zero
  motorPosition[idx] = 0;
  Serial.print("Gimbal motor ");
  Serial.print(idx);
  Serial.println(" homed.");
}



// =======================================================
// CALIBRATE SG BASELINE FOR EACH MOTOR
// Motor moves +500 steps then -500 steps
// Averages StallGuard readings for a reliable baseline
// =======================================================
void calibrateSG(int idx, bool isGimbal) {
  TMC5160Stepper* drv = drivers[idx];
  int DIR_PIN = DIR_PINS[idx];
  int STEP_PIN = STEP_PINS[idx];

  int pulseUs = isGimbal ? gimbalCalPulseUs : linearCalPulseUs;
  int delayUs = isGimbal ? gimbalCalDelayUs : linearCalDelayUs;

  drv->TCOOLTHRS(0xFFFFF);
  drv->sgt(10);
  drv->diag1_stall(true);
  drv->en_pwm_mode(false);

  long samples = 0;
  long sgSum = 0;

  // --- Move forward 500 steps ---
  digitalWrite(DIR_PIN, HIGH);
  for (int i = 0; i < 1000; i++) {
    digitalWriteFast(STEP_PIN, HIGH);
    delayMicroseconds(pulseUs);
    digitalWriteFast(STEP_PIN, LOW);

    uint16_t sg = drv->sg_result();
    Serial.println(sg);
    sgSum += sg;
    samples++;
  }

  // --- Move backward 500 steps ---
  digitalWrite(DIR_PIN, LOW);
  for (int i = 0; i < 500; i++) {
    digitalWriteFast(STEP_PIN, HIGH);
    delayMicroseconds(pulseUs);
    digitalWriteFast(STEP_PIN, LOW);

    uint16_t sg = drv->sg_result();
    Serial.println(sg);
    sgSum += sg;
    samples++;
  }

  int avgSG = sgSum / samples;
  sgBaseline[idx] = avgSG;

  drv->en_pwm_mode(true);
  drv->pwm_autoscale(true);
  delay(200);
}




void homeAllMotors() {
  digitalWrite(EN_A_PIN, LOW);

  // --- calibrate all motors before homing ---
  Serial.println("\n--- CALIBRATING ALL MOTORS FOR SG BASELINES ---");
  for (int i = 0; i < 4; i++) {
    calibrateSG(i, true);
  }
  Serial.println("--- SG CALIBRATION COMPLETE ---\n");

  // --- Now perform homing with calibrated thresholds ---
  for (int i = 0; i < 4; i++) {
    homeSingleMotor(i, true);
    delay(500);
  }
}



// ---------------------------------------------------
// HOME LIN MOTOR (sequential)
// ---------------------------------------------------
void homeLINMotor(int idx) {
  TMC5160Stepper* drv = drivers[idx];

  digitalWriteFast(DIR_PINS[idx], HIGH);

  drv->rms_current(homingCurrent[idx]);
  drv->microsteps(16);
  drv->en_pwm_mode(false);
  drv->diag1_stall(true);

  long stepCount = 0;
  int lowCount = 0;  // <<< ADDED: StallGuard stability counter

  while (true) {
    // Step pulse
    digitalWriteFast(STEP_PINS[idx], HIGH);
    delayMicroseconds(stepPulseUs);
    digitalWriteFast(STEP_PINS[idx], LOW);
    delayMicroseconds(stepDelayUs);
    stepCount++;

    // Update software position
    motorPos[idx - 4] -= 1;  // idx 4–7 maps to motorPos[0–3]

    // Safety limit
    if (stepCount > maxHomingTravel[idx]) {
      Serial.print("ERROR: Linear actuator ");
      Serial.print(idx);
      Serial.println(" exceeded safe homing travel! Stopping.");
      break;
    }

    // StallGuard detection
    uint16_t sg = drv->sg_result();
    Serial.println(sg);

    // <<< UPDATED BLOCK ONLY >>>
    if (stepCount > 500) {  // ignore SG before 500 steps
      if (sg < (sgBaseline[idx] - 50)) {
        lowCount++;  // must be low several times
      } else {
        lowCount = 0;  // reset if it bounces high
      }

      if (lowCount >= 5) {  // stall must be stable for 5 reads
        Serial.print("Stall detected on linear actuator ");
        Serial.println(idx);
        break;
      }
    }
    // <<< END UPDATED BLOCK >>>
  }

  // Back off
  digitalWriteFast(DIR_PINS[idx], HIGH);
  for (int b = 0; b < backoffSteps[idx]; b++) {
    digitalWriteFast(STEP_PINS[idx], HIGH);
    delayMicroseconds(stepPulseUs);
    digitalWriteFast(STEP_PINS[idx], LOW);
    delayMicroseconds(stepDelayUs);

    // Update software position
    motorPos[idx - 4] += 1;
  }

  // Reset zero
  motorPos[idx - 4] = 0;
  Serial.print("Linear actuator ");
  Serial.print(idx);
  Serial.println(" homed.");
}


void runLinearMotors() {
    if (!linActive) return;  // do nothing unless LIN command received

    bool allDone = true;
    unsigned long now = micros();

    for (int i = 0; i < 4; i++) {
        if (motorPos[i] == targetPos[i]) continue;

        allDone = false;  // at least one actuator still moving

        if (now - lastLinearStepMicros[i] >= linearStepIntervalMicros[i]) {
            lastLinearStepMicros[i] = now;

            bool dir = (targetPos[i] > motorPos[i]);
            digitalWrite(DIR_PINS[i + 4], dir ? HIGH : LOW);

            digitalWriteFast(STEP_PINS[i + 4], HIGH);
            delayMicroseconds(linearStepPulseUs);
            digitalWriteFast(STEP_PINS[i + 4], LOW);

            motorPos[i] += dir ? 1 : -1;
        }
    }

    if (allDone) linActive = false;  // stop moving once all reach target
}

