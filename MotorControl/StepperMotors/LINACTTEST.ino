// =======================================================
// 4-MOTOR TMC5160 SEQUENTIAL HOMING (1 → 2 → 3 → 4)
// =======================================================

#include <TMCStepper.h>
#include <SPI.h>
#include <digitalWriteFast.h>

// ---------------- Pin Definitions ----------------
#define EN_PIN 7

#define DIR1_PIN 8
#define STEP1_PIN 9
#define CS1_PIN 17

#define DIR2_PIN 10
#define STEP2_PIN 11
#define CS2_PIN 20

#define DIR3_PIN 12
#define STEP3_PIN 13
#define CS3_PIN 21

#define DIR4_PIN 14
#define STEP4_PIN 15
#define CS4_PIN 22

// Shared SPI
#define MOSI_PIN 19
#define MISO_PIN 16
#define SCK_PIN 18

#define R_SENSE 0.075f

// ---------------- Driver Objects ----------------
TMC5160Stepper driver1(CS1_PIN, R_SENSE);
TMC5160Stepper driver2(CS2_PIN, R_SENSE);
TMC5160Stepper driver3(CS3_PIN, R_SENSE);
TMC5160Stepper driver4(CS4_PIN, R_SENSE);

TMC5160Stepper* drivers[4] = { &driver1, &driver2, &driver3, &driver4 };

int DIR_PINS[4] = { DIR1_PIN, DIR2_PIN, DIR3_PIN, DIR4_PIN };
int STEP_PINS[4] = { STEP1_PIN, STEP2_PIN, STEP3_PIN, STEP4_PIN };
int CS_PINS[4] = { CS1_PIN, CS2_PIN, CS3_PIN, CS4_PIN };

// FUYU FSK30J linear actuator calibration
const float microstepsPerMM = 1596.0f;
const float mmPerMicrostep = 1.0f / microstepsPerMM;

long motorPos[4] = {0,0,0,0};        // current microstep position
long targetPos[4] = {0,0,0,0};       // commanded target (microsteps)

unsigned long lastStepMicros[4] = {0,0,0,0};
unsigned long stepIntervalMicros = 900;  // move speed control

float targetLinS[4] = {0,0,0,0}; // commanded mm from Pi

// StallGuard thresholds
int stallThresholds[4] = {120, 100, 80, 120};

// Homing constants
const int stepPulseUs = 3;
const int stepDelayUs = 100;
const int backoffSteps = 180;

// =======================================================
// ------------------- SERIAL HANDLING --------------------
// =======================================================

String serialCmd = "";

// Read incoming serial commands
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

// Parse command format:
// LIN: mm1,mm2,mm3,mm4
void parseCommand(String cmd) {
  cmd.trim();

  if (!cmd.startsWith("LIN:")) return;  // ignore other commands

  cmd.remove(0, 4);

  float s1, s2, s3, s4;
  int n = sscanf(cmd.c_str(), "%f,%f,%f,%f", &s1, &s2, &s3, &s4);
  if (n != 4) return;

  targetLinS[0] = s1;
  targetLinS[1] = s2;
  targetLinS[2] = s3;
  targetLinS[3] = s4;

  // Convert mm → microsteps
  for (int i = 0; i < 4; i++)
    targetPos[i] = (long)(targetLinS[i] * microstepsPerMM);

  // Enable only on valid command
  digitalWrite(EN_PIN, LOW);
}



// ---------------------------------------------------
// SETUP
// ---------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n--- 4-MOTOR SEQUENTIAL HOMING ---");

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); // disabled until real command arrives

  // Setup SPI pins
  SPI.setRX(MISO_PIN);
  SPI.setTX(MOSI_PIN);
  SPI.setSCK(SCK_PIN);
  SPI.begin();

  // Setup drivers
  for (int i = 0; i < 4; i++) {
    pinMode(DIR_PINS[i], OUTPUT);
    pinMode(STEP_PINS[i], OUTPUT);

    pinMode(CS_PINS[i], OUTPUT);
    digitalWrite(CS_PINS[i], HIGH);

    drivers[i]->begin();
    drivers[i]->toff(4);
    drivers[i]->blank_time(24);
    drivers[i]->microsteps(16);
    drivers[i]->rms_current(800);
    drivers[i]->en_pwm_mode(false);
    drivers[i]->pwm_autoscale(true);
    drivers[i]->intpol(true);
    drivers[i]->TCOOLTHRS(0xFFFFF);
    drivers[i]->sgt(15);
  }

  delay(1000);

  // Sequential homing
  for (int i = 0; i < 4; i++) homeMotor(i);

  delay(1000);

  // Sequential homing second attempt
  for (int i = 0; i < 4; i++) homeMotor(i);

  Serial.println("All linear actuators homed.");
}

// =======================================================
// -------------------- HOMING ROUTINE --------------------
// =======================================================

void homeMotor(int m) {
  Serial.print("Homing motor ");
  Serial.println(m + 1);

  long stepCount = 0;
  unsigned long lastStep = 0;

  digitalWrite(DIR_PINS[m], HIGH);

  while (true) {
    unsigned long now = micros();

    if (now - lastStep >= stepDelayUs) {
      lastStep = now;
      digitalWriteFast(STEP_PINS[m], HIGH);
      delayMicroseconds(stepPulseUs);
      digitalWriteFast(STEP_PINS[m], LOW);
      stepCount++;
    }

    static int poll = 0;
    poll++;
    if (poll < 50) continue;
    poll = 0;

    digitalWrite(CS_PINS[m], LOW);
    uint16_t sg = drivers[m]->sg_result();
    digitalWrite(CS_PINS[m], HIGH);

    if (sg < stallThresholds[m] && stepCount > 50) break;
  }

  // Back off
  digitalWrite(DIR_PINS[m], LOW);
  for (int s = 0; s < backoffSteps; s++) {
    digitalWriteFast(STEP_PINS[m], HIGH);
    delayMicroseconds(stepPulseUs);
    digitalWriteFast(STEP_PINS[m], LOW);
    delayMicroseconds(stepDelayUs);
  }

  motorPos[m] = 0;  // zero position
}

// =======================================================
// -------------------- MOTOR STEPPING --------------------
// =======================================================

void moveMotors() {
  unsigned long now = micros();

  for (int i = 0; i < 4; i++) {
    long error = targetPos[i] - motorPos[i];

    if (abs(error) < 2) continue;

    // Set direction
    digitalWriteFast(DIR_PINS[i], error > 0 ? HIGH : LOW);

    if (now - lastStepMicros[i] >= stepIntervalMicros) {
      lastStepMicros[i] = now;
      digitalWriteFast(STEP_PINS[i], HIGH);
      delayMicroseconds(stepPulseUs);
      digitalWriteFast(STEP_PINS[i], LOW);
      motorPos[i] += (error > 0 ? 1 : -1);
    }
  }
}

// =======================================================
// -------------------- POSITION REPORT -------------------
// =======================================================

void sendLengths() {
  static unsigned long last = 0;
  if (millis() - last < 50) return;
  last = millis();

  float mm[4];
  for (int i = 0; i < 4; i++)
    mm[i] = motorPos[i] * mmPerMicrostep;

  Serial.printf("LINPOS:%.2f,%.2f,%.2f,%.2f\n",
                mm[0], mm[1], mm[2], mm[3]);
}

// =======================================================
// ------------------------- LOOP -------------------------
// =======================================================

void loop() {
  handleSerial();
  moveMotors();
  sendLengths();
}




// =============================================================

// test below for pi readability - no 1280 overflow



/*
  4x Linear Actuators Controller (Arduino Mega)
  - TMC5160 drivers (shared SPI)
  - LIN: mm1,mm2,mm3,mm4  -> move each actuator to target mm
  - 1596 microsteps per mm
  - Fast serial parser (no String) modeled on provided example
  - Motors enabled only while moving; disabled when all targets reached
  - Watchdog enabled to recover from hangs
*/

#include <TMCStepper.h>
#include <SPI.h>
#include <digitalWriteFast.h>
#include <avr/wdt.h>    // watchdog

// --------------------------- PINS --------------------------------
#define EN_PIN   7

#define DIR1_PIN 8
#define STEP1_PIN 9
#define CS1_PIN  17

#define DIR2_PIN 10
#define STEP2_PIN 11
#define CS2_PIN  20

#define DIR3_PIN 12
#define STEP3_PIN 13
#define CS3_PIN  21

#define DIR4_PIN 14
#define STEP4_PIN 15
#define CS4_PIN  22

// Shared SPI (Mega)
#define MOSI_PIN 19
#define MISO_PIN 16
#define SCK_PIN  18

#define R_SENSE 0.075f

// ------------------------ TMC OBJECTS -----------------------------
TMC5160Stepper drv1(CS1_PIN, R_SENSE);
TMC5160Stepper drv2(CS2_PIN, R_SENSE);
TMC5160Stepper drv3(CS3_PIN, R_SENSE);
TMC5160Stepper drv4(CS4_PIN, R_SENSE);

TMC5160Stepper* drivers[4] = { &drv1, &drv2, &drv3, &drv4 };

int DIR_PINS[4]  = { DIR1_PIN, DIR2_PIN, DIR3_PIN, DIR4_PIN };
int STEP_PINS[4] = { STEP1_PIN, STEP2_PIN, STEP3_PIN, STEP4_PIN };
int CS_PINS[4]   = { CS1_PIN, CS2_PIN, CS3_PIN, CS4_PIN };

// --------------------- POSITION CONVERSION -----------------------
const float MICROSTEPS_PER_MM = 1596.0f;
const float MM_PER_MICROSTEP = 1.0f / MICROSTEPS_PER_MM;

// step counters (microsteps)
volatile long motorPos[4]     = { 0, 0, 0, 0 };  // current microstep counts
long targetPos[4]             = { 0, 0, 0, 0 };  // target microstep counts
unsigned long lastStepMicros[4] = { 0, 0, 0, 0 }; // last step time per axis

// per-axis dynamic step interval (us) => controls speed; can be changed externally
unsigned long stepIntervalMicros[4] = { 900, 900, 900, 900 }; // default speed

// limits for the interval (you may tune)
const unsigned long STEP_INTERVAL_MIN_US = 120;   // fastest (smallest interval)
const unsigned long STEP_INTERVAL_MAX_US = 20000; // slowest (largest interval)

// homing parameters
const int HOMING_STEP_PULSE_US = 3;
const int HOMING_STEP_DELAY_US = 50;
int stallThresholds[4] = { 120, 100, 80, 120 };  // indivudual motor tuning
const int HOMING_BACKOFF_STEPS = 180; 

// ----------------------- SERIAL PARSING --------------------------
#define SERIAL_BUFFER_SIZE 128
char serialBuffer[SERIAL_BUFFER_SIZE];
size_t serialIndex = 0;

// Flag: set to true when a valid LIN: command was received and motors enabled
volatile bool linCommanded = false;

// temporary storage for parsed mm targets (float)
float targetMM[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

// reporting timing
const unsigned long REPORT_INTERVAL_MS = 100; // how often to send LINPOS
unsigned long lastReportMillis = 0;

// watch dog interval - use 2s
// call wdt_reset() regularly in loop

// -------------------- FORWARD DECLARATIONS -----------------------
void parseCommand(const char* line);
void handleSerialFast();
void moveMotorsNonBlocking();
void sendPositions();
void homeAllSequential();
void homeSingleMotor(int m);

// ------------------------ SETUP --------------------------------
void setup() {
  // enable watchdog (2s)
  wdt_enable(WDTO_2S);

  Serial.begin(115200);
  delay(200);
  Serial.println(F("\nLinear actuator controller booting..."));

  // SPI config (shared)
  SPI.setRX(MISO_PIN);
  SPI.setTX(MOSI_PIN);
  SPI.setSCK(SCK_PIN);
  SPI.begin();

  // enable pin (start disabled)
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); // active LOW -> HIGH means disabled

  // pins + driver setup
  for (int i = 0; i < 4; ++i) {
    pinMode(DIR_PINS[i], OUTPUT);
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(CS_PINS[i], OUTPUT);
    digitalWrite(CS_PINS[i], HIGH);

    drivers[i]->begin();
    drivers[i]->toff(4);
    drivers[i]->blank_time(24);
    drivers[i]->microsteps(16);
    drivers[i]->rms_current(900);      // tune as needed
    drivers[i]->TCOOLTHRS(0xFFFFF);
    drivers[i]->pwm_autoscale(true);
    drivers[i]->sgt(10);
  }

  delay(200);
  Serial.println(F("Drivers configured. Beginning homing..."));

  // sequential homing; motors will be zeroed
  homeAllSequential();

  Serial.println(F("Homing complete. Waiting for LIN: commands."));
}

// ------------------------ MAIN LOOP -----------------------------
void loop() {
  // reset watchdog
  wdt_reset();

  // fast serial reader (non-blocking)
  handleSerialFast();

  // non-blocking motion
  moveMotorsNonBlocking(); // keep in loop, does not activate if the command is not recieved

  // periodic reporting
  unsigned long nowMs = millis();
  if ((nowMs - lastReportMillis) >= REPORT_INTERVAL_MS) {
    lastReportMillis = nowMs;
    sendPositions();
    Serial.flush(); // help keep Pi side in sync (blocks until TX buffer empty)
  }
}

// ------------------- SERIAL FAST HANDLER ------------------------
// reads bytes from Serial, builds a C-string (no String), calls parseCommand on newline
void handleSerialFast() {
  int avail = Serial.available();
  while (avail-- > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialIndex > 0) {
        serialBuffer[serialIndex] = '\0';
        // parse input line
        parseCommand(serialBuffer);
        // reset index
        serialIndex = 0;
      }
    } else {
      if (serialIndex < (SERIAL_BUFFER_SIZE - 1)) {
        serialBuffer[serialIndex++] = c;
      } else {
        // overflow: reset to avoid partial/wrong parse
        serialIndex = 0;
      }
    }
  }
}

// -------------------- COMMAND PARSER ---------------------------
// Accepts: LIN: mm1,mm2,mm3,mm4
// Example: "LIN: 12.34,0.0,3.0,100.5"
void parseCommand(const char* line) {
  // quick prefix check
  if (strncmp(line, "LIN:", 4) != 0) return;

  // pointer to after prefix
  const char* p = line + 4;

  // simple parse: read four floats separated by commas.
  // We use a robust but straightforward parser to avoid heavy library calls.
  for (int i = 0; i < 4; ++i) {
    // skip spaces
    while (*p == ' ' || *p == '\t') ++p;

    // parse float (handles optional sign and decimal)
    bool neg = false;
    if (*p == '-') { neg = true; ++p; }
    long intPart = 0;
    while (*p >= '0' && *p <= '9') {
      intPart = intPart * 10 + (*p - '0');
      ++p;
    }
    float value = (float)intPart;

    if (*p == '.') {
      ++p;
      float place = 0.1f;
      while (*p >= '0' && *p <= '9') {
        value += (*p - '0') * place;
        place *= 0.1f;
        ++p;
      }
    }

    if (neg) value = -value;
    targetMM[i] = value;

    // skip spaces after number
    while (*p == ' ' || *p == '\t') ++p;

    // if comma, skip it and continue
    if (*p == ',') ++p;
    else {
      // if missing comma but not last, parsing may be malformed - bail out silently
      if (i < 3) return;
    }
  }

  // valid parse: convert mm -> microsteps
  for (int i = 0; i < 4; ++i) {
    // clamp to reasonable range if needed (optional)
    // example: if you know travel limits, clamp here.
    long steps = (long)roundf(targetMM[i] * MICROSTEPS_PER_MM);
    targetPos[i] = steps;
  }

  // enable motors and indicate commanded state
  digitalWrite(EN_PIN, LOW); // active LOW
  linCommanded = true;
}

// ------------------------ MOTION --------------------------------
// Non-blocking: each axis steps independently based on stepIntervalMicros[].
// When all axes reach their targets, disable EN_PIN and clear linCommanded.
void moveMotorsNonBlocking() {
  // If not commanded, nothing to do (motors remain disabled)
  if (!linCommanded) return;

  unsigned long now = micros();
  bool anyMoving = false;

  for (int i = 0; i < 4; ++i) {
    long cur = motorPos[i];
    long tgt = targetPos[i];
    long err = tgt - cur;

    if (err == 0) continue; // axis at target

    anyMoving = true;

    // set direction pin
    if (err > 0) digitalWriteFast(DIR_PINS[i], HIGH);
    else digitalWriteFast(DIR_PINS[i], LOW);

    // enforce min/max interval bounds for safety
    unsigned long iv = stepIntervalMicros[i];
    if (iv < STEP_INTERVAL_MIN_US) iv = STEP_INTERVAL_MIN_US;
    if (iv > STEP_INTERVAL_MAX_US) iv = STEP_INTERVAL_MAX_US;

    if ((now - lastStepMicros[i]) >= iv) {
      lastStepMicros[i] = now;

      // single step pulse
      digitalWriteFast(STEP_PINS[i], HIGH);
      // short pulse width
      delayMicroseconds(3);
      digitalWriteFast(STEP_PINS[i], LOW);

      // update position
      motorPos[i] += (err > 0 ? 1 : -1);
    }
  }

  // if no axis is moving anymore, disable drivers and clear command flag
  if (!anyMoving) {
    digitalWrite(EN_PIN, HIGH); // disable (active LOW)
    linCommanded = false;
  }
}

// ----------------------- POSITION REPORT ------------------------
// Reports current mm positions in this format:
// LINPOS:xx.xx,yy.yy,zz.zz,aa.aa
void sendPositions() {
  float mm[4];
  for (int i = 0; i < 4; ++i) mm[i] = motorPos[i] * MM_PER_MICROSTEP;

  // send using printf style to minimize allocations
  Serial.print("LINPOS:");
  Serial.print(mm[0], 2); Serial.print(',');
  Serial.print(mm[1], 2); Serial.print(',');
  Serial.print(mm[2], 2); Serial.print(',');
  Serial.print(mm[3], 2);
  Serial.print('\n');
}

// ------------------------ HOMING --------------------------------
// Sequential homing using StallGuard 
void homeAllSequential() {
  // ensure drivers enabled for homing
  digitalWrite(EN_PIN, LOW);

  for (int i = 0; i < 4; ++i) {
    // put driver into spreadCycle for more reliable stall detect
    drivers[i]->en_pwm_mode(false);
    drivers[i]->pwm_autoscale(true);
    drivers[i]->TCOOLTHRS(0xFFFFF);
    drivers[i]->sgt(12);

    homeSingleMotor(i);

    // restore to stealthChop if desired (keeps quiet)
    drivers[i]->en_pwm_mode(true);
    drivers[i]->pwm_autoscale(true);
    delay(200);
  }

  // after homing, disable until a LIN: command arrives
  digitalWrite(EN_PIN, HIGH);
  linCommanded = false;
}

void homeSingleMotor(int m) {
  Serial.print(F("Homing axis "));
  Serial.println(m + 1);

  long stepCount = 0;
  unsigned long lastStep = 0;

  // direction toward home (adjust if your hardware requires the opposite)
  digitalWrite(DIR_PINS[m], HIGH);

  while (true) {
    unsigned long now = micros();

    if ((now - lastStep) >= HOMING_STEP_DELAY_US) {
      lastStep = now;
      digitalWriteFast(STEP_PINS[m], HIGH);
      delayMicroseconds(HOMING_STEP_PULSE_US);
      digitalWriteFast(STEP_PINS[m], LOW);
      stepCount++;
    }

    // poll StallGuard periodically
    static int poll = 0;
    poll++;
    if (poll < 50) continue;
    poll = 0;

    digitalWrite(CS_PINS[m], LOW);
    uint16_t sg = drivers[m]->sg_result();
    digitalWrite(CS_PINS[m], HIGH);

    Serial.print(F("SG: "));
    Serial.println(sg);

    if (sg < stallThresholds[m] && stepCount > 50) {
      Serial.println(F("Stall detected, backing off..."));
      break;
    }
  }

  // back off
  digitalWrite(DIR_PINS[m], LOW);
  for (int s = 0; s < HOMING_BACKOFF_STEPS; ++s) {
    digitalWriteFast(STEP_PINS[m], HIGH);
    delayMicroseconds(HOMING_STEP_PULSE_US);
    digitalWriteFast(STEP_PINS[m], LOW);
    delayMicroseconds(HOMING_STEP_DELAY_US);
  }

  // set software zero
  motorPos[m] = 0;
  targetPos[m] = 0;

  Serial.print(F("Axis "));
  Serial.print(m + 1);
  Serial.println(F(" homed."));
}
