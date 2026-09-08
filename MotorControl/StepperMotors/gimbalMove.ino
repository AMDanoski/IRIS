#include <TMCStepper.h>
#include <SPI.h>
#include <digitalWriteFast.h>

// =======================================================
// PICO PIN MAPPING (4× TMC5160)
// =======================================================
#define EN_PIN 7

// Motor 1 pins
#define DIR1_PIN 10
#define STEP1_PIN 11
#define CS1_PIN 20

// Motor 2 pins
#define DIR2_PIN 8  
#define STEP2_PIN 9
#define CS2_PIN 17

// Motor 3 pins
#define DIR3_PIN 14
#define STEP3_PIN 15
#define CS3_PIN 22

// Motor 4 pins
#define DIR4_PIN 12
#define STEP4_PIN 13 
#define CS4_PIN 21

// Shared SPI0
#define MOSI_PIN 19
#define MISO_PIN 16
#define SCK_PIN 18

#define R_SENSE 0.075f

// =======================================================
// MOTOR CONSTANTS
// =======================================================
const float fullStepDeg = 1.8f;
const int microsteps = 32;
const float degPerMicro = fullStepDeg / microsteps;
const float microstepsPerRev = 200.0f * microsteps;

// radians/sec → RPM = (rad/s * 60) / (2π)
const float RADS_TO_RPM = 60.0f / (2.0f * PI);

// Optional offsets
float angleOffset[4] = { 0, 0, 0, 0 };

// =======================================================
// DRIVERS
// =======================================================
TMC5160Stepper driver1(CS1_PIN, R_SENSE);
TMC5160Stepper driver2(CS2_PIN, R_SENSE);
TMC5160Stepper driver3(CS3_PIN, R_SENSE);
TMC5160Stepper driver4(CS4_PIN, R_SENSE);

TMC5160Stepper* drivers[4] = { &driver1, &driver2, &driver3, &driver4 };

// =======================================================
// STALLGUARD HOMING FUNCTIONS
// =======================================================

// --- User-adjustable homing parameters ---
const int stallThreshold = 300;                        // lower = more sensitive
const int stepPulseUs = 5;                             // step high time
const int stepDelayUs = 250;                          // delay between steps (controls speed)
const int backoffSteps[4] = { 50, 3200, 3200, 50 };     // how far to back off after stall (per motor)
bool spinDirection[4] = { false, true, true, false };  // initial turn direction for homing
int sgBaseline[4] = { 200, 300, 300, 300 };            // default backup values



// =======================================================
// STATE
// =======================================================
float targetRPM[4] = { 0, 0, 0, 0 };
float smoothRPM[4] = { 0, 0, 0, 0 };

float alpha = 0.05f;  // smoothing factor

bool motorEnabled[4] = { false, false, false, false };

unsigned long lastStepMicros[4] = { 0, 0, 0, 0 };
unsigned long stepIntervalMicros[4] = { 0, 0, 0, 0 };

long motorPosition[4] = { 0, 0, 0, 0 };

int DIR_PINS[4] = { DIR1_PIN, DIR2_PIN, DIR3_PIN, DIR4_PIN };
int STEP_PINS[4] = { STEP1_PIN, STEP2_PIN, STEP3_PIN, STEP4_PIN };

String serialCmd;

// =======================================================
// SETUP
// =======================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(CS1_PIN, OUTPUT);
  pinMode(CS2_PIN, OUTPUT);
  pinMode(CS3_PIN, OUTPUT);
  pinMode(CS4_PIN, OUTPUT);

  digitalWrite(CS1_PIN, HIGH);
  digitalWrite(CS2_PIN, HIGH);
  digitalWrite(CS3_PIN, HIGH);
  digitalWrite(CS4_PIN, HIGH);
  delay(200);

  SPI.setRX(MISO_PIN);
  SPI.setTX(MOSI_PIN);
  SPI.setSCK(SCK_PIN);
  delay(200);
  SPI.begin();
  delay(200);

  Serial.println("Testing SPI to Motor 1...");
  driver1.begin();
  delay(20);

  uint32_t gstat = driver1.GSTAT();
  Serial.printf("Motor1 GSTAT = 0x%08lX\n", gstat);


  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  for (int i = 0; i < 4; i++) {
    pinMode(DIR_PINS[i], OUTPUT);
    pinMode(STEP_PINS[i], OUTPUT);

    // other driver options available but not used from the tmc5160 library
    drivers[i]->begin();
    delay(20);
    drivers[i]->toff(4);
    drivers[i]->blank_time(24);  //Chopper blanking time (higher = quieter, lower = faster response)
    drivers[i]->microsteps(microsteps);
    drivers[i]->rms_current(300);
    drivers[i]->en_pwm_mode(true);
    drivers[i]->pwm_freq(3);
    drivers[i]->pwm_autoscale(true);
    drivers[i]->intpol(true);
  }
  delay(50);

  Serial.println("4-Motor TMC5160 Controller Ready.");

  delay(200);  // allow drivers to stabilize
  homeAllMotors();
  const int stepDelayUs = 250;
  playStarTrekIntro(0); // pin definition neeeds to change
}

// =======================================================
// MAIN LOOP
// =======================================================
void loop() {
  handleSerial();
  smoothRates();
  runMotors();
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

  if (!cmd.startsWith("SET:")) return;
  //if (!cmd.startsWith("LIN:")) return;

  cmd.remove(0, 4);

  float w1, w2, w3, w4;  // will need a s1, s2, s3, s4 for the lin act)
  int n = sscanf(cmd.c_str(), "%f,%f,%f,%f", &w1, &w2, &w3, &w4);

  if (n == 4) {
    targetRPM[0] = w1 * RADS_TO_RPM;
    targetRPM[1] = w2 * RADS_TO_RPM;
    targetRPM[2] = w3 * RADS_TO_RPM;
    targetRPM[3] = w4 * RADS_TO_RPM;
  }
}

// =======================================================
// ACCELERATION SMOOTHING
// =======================================================
void smoothRates() {
  bool allZero = true;

  for (int i = 0; i < 4; i++) {
    smoothRPM[i] = smoothRPM[i] + alpha * (targetRPM[i] - smoothRPM[i]);

    if (fabs(smoothRPM[i]) > 0.01f)
      allZero = false;
  }

  if (allZero) {
    digitalWrite(EN_PIN, HIGH);
    for (int i = 0; i < 4; i++) motorEnabled[i] = false;
    return;
  }

  digitalWrite(EN_PIN, LOW);

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

    if (now - lastStepMicros[i] >= stepIntervalMicros[i]) {
      lastStepMicros[i] = now;

      digitalWriteFast(STEP_PINS[i], HIGH);
      delayMicroseconds(5);
      digitalWriteFast(STEP_PINS[i], LOW);


      motorPosition[i] += (digitalRead(DIR_PINS[i]) ? 1 : -1);
    }
  }
}

// =======================================================
// SEND ANGLES (degrees, 0–360°)
// =======================================================
void sendAngles() {
  static unsigned long last = 0;
  if (millis() - last < 50) return;  // <-- 50ms (20Hz)
  last = millis();

  // --- NEW BUFFER SAFETY CHECK ---
  if (Serial.availableForWrite() < 64) return;  // avoid overflow if TX buffer is full

  float a[4];
  for (int i = 0; i < 4; i++) {
    // Compute angle in degrees
    a[i] = motorPosition[i] * degPerMicro + angleOffset[i];

    // Normalize to [0, 360)
    while (a[i] < 0) a[i] += 360.0f;
    while (a[i] >= 360) a[i] -= 360.0f;

    // Convert to radians
    a[i] *= PI / 180.0f;
  }
  // printing to the pi:
  // first 4 digits are gimbal angle
  // last 4 digits are linear actuator position in mm from origin
  Serial.printf("DATA:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", a[0], a[1], a[2], a[3], a[0], a[1], a[2], a[3]);
}

void homeSingleMotor(int idx, bool homeDirLow) {
  const int stepDelayUs = 500;
  TMC5160Stepper* drv = drivers[idx];
  int DIR_PIN = DIR_PINS[idx];
  int STEP_PIN = STEP_PINS[idx];

  Serial.printf("\n--- HOMING MOTOR %d ---\n", idx + 1);

  // *** THE IMPORTANT FIX ***
  drv->en_pwm_mode(false);     // Disable StealthChop (MUST be OFF for StallGuard)
  drv->pwm_autoscale(false);
  drv->pwm_grad(0);
  drv->pwm_ofs(0);

  // StallGuard setup
  drv->TCOOLTHRS(0xFFFFF);
  drv->sgt(10);
  drv->diag1_stall(true);
  drv->en_pwm_mode(false);  // use spreadCycle for more reliable stall detect

  // Move toward stop
  digitalWrite(DIR_PIN, spinDirection[idx] ? LOW : HIGH);
  long stepCount = 0;

  while (true) {
    digitalWriteFast(STEP_PIN, HIGH);
    delayMicroseconds(stepPulseUs);
    digitalWriteFast(STEP_PIN, LOW);
    delayMicroseconds(stepDelayUs);
    stepCount++;

    uint16_t sg = drv->sg_result();

    // Optional debug print every 100 steps
    if (stepCount % 100 == 0) {
      Serial.printf("M%d Step %ld | SG_RESULT: %u\n | sgBaseline: %u\n", idx + 1, stepCount, sg, sgBaseline[idx] );
    }

    if (sg < (sgBaseline[idx] * 0.30f) && stepCount > 300) {
      Serial.printf("M%d STALL DETECTED at step %ld (SG=%u)\n", idx + 1, stepCount, sg);
      break;
    }
  }

  // Back off a little
  Serial.printf("M%d backing off %d steps...\n", idx + 1, backoffSteps[idx]);
  digitalWrite(DIR_PIN, homeDirLow ? HIGH : LOW);
  for (int i = 0; i < backoffSteps[idx]; i++) {
    digitalWriteFast(STEP_PIN, HIGH);
    delayMicroseconds(stepPulseUs);
    digitalWriteFast(STEP_PIN, LOW);
    delayMicroseconds(stepDelayUs);
  }

  // Set software zero
  motorPosition[idx] = 0;
  angleOffset[idx] = 0;
  Serial.printf("M%d homing complete. Position zeroed.\n", idx + 1);

  // Return driver to StealthChop mode for normal use
  drv->en_pwm_mode(true);
  drv->pwm_autoscale(true);
}


// =======================================================
// CALIBRATE SG BASELINE FOR EACH MOTOR
// Motor moves +500 steps then -500 steps
// Averages StallGuard readings for a reliable baseline
// =======================================================
void calibrateSG(int idx) {
  TMC5160Stepper* drv = drivers[idx];
  int DIR_PIN = DIR_PINS[idx];
  int STEP_PIN = STEP_PINS[idx];

  Serial.printf("\n--- CALIBRATING SG MOTOR %d ---\n", idx + 1);

  drv->TCOOLTHRS(0xFFFFF);
  drv->sgt(10);
  drv->diag1_stall(true);
  drv->en_pwm_mode(false);  // make SG more reliable

  long samples = 0;
  long sgSum = 0;

  // --- Move forward 500 steps ---
  digitalWrite(DIR_PIN, HIGH);
  for (int i = 0; i < 500; i++) {
    digitalWriteFast(STEP_PIN, HIGH);
    delayMicroseconds(stepPulseUs);
    digitalWriteFast(STEP_PIN, LOW);
    delayMicroseconds(stepDelayUs);

    uint16_t sg = drv->sg_result();
    sgSum += sg;
    samples++;
  }

  // --- Move backward 500 steps ---
  digitalWrite(DIR_PIN, LOW);
  for (int i = 0; i < 500; i++) {
    digitalWriteFast(STEP_PIN, HIGH);
    delayMicroseconds(stepPulseUs);
    digitalWriteFast(STEP_PIN, LOW);
    delayMicroseconds(stepDelayUs);

    uint16_t sg = drv->sg_result();
    sgSum += sg;
    samples++;
  }

  // --- Compute baseline ---
  int avgSG = sgSum / samples;
  sgBaseline[idx] = avgSG;

  Serial.printf("M%d SG baseline = %d\n", idx + 1, avgSG);

  // restore mode used later
  drv->en_pwm_mode(true);
  drv->pwm_autoscale(true);

  delay(200);
}


void homeAllMotors() {
  digitalWrite(EN_PIN, LOW);

  // --- NEW: calibrate all motors before homing ---
  Serial.println("\n--- CALIBRATING ALL MOTORS FOR SG BASELINES ---");
  for (int i = 0; i < 4; i++) {
    calibrateSG(i);
  }
  Serial.println("--- SG CALIBRATION COMPLETE ---\n");

  // --- Now perform homing with calibrated thresholds ---
  for (int i = 0; i < 4; i++) {
    homeSingleMotor(i, true);
    delay(500);
  }

  Serial.println("\nAll motors homed successfully.\n");
}




//====================

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0

// Melody (unchanged)
int melody[] = {
  NOTE_D4, -8, NOTE_G4, 16, NOTE_C5, -4, 
  NOTE_B4, 8, NOTE_G4, -16, NOTE_E4, -16, NOTE_A4, -16,
  NOTE_D5, 2,
};

int tempo = 80;

/* ---------------------------------------------------
          FUNCTION YOU CAN CALL ANYWHERE
----------------------------------------------------*/
void playStarTrekIntro(int buzzer) {

  int notes = sizeof(melody) / sizeof(melody[0]) / 2;
  int wholenote = (60000 * 4) / tempo;
  int divider = 0;
  int noteDuration = 0;

  for (int i = 0; i < notes * 2; i += 2) {

    divider = melody[i + 1];
    if (divider > 0) {
      noteDuration = wholenote / divider;
    } else {
      noteDuration = (wholenote / abs(divider)) * 1.5;
    }

    tone(buzzer, melody[i], noteDuration * 0.9);
    delay(noteDuration);
    noTone(buzzer);
  }
}




