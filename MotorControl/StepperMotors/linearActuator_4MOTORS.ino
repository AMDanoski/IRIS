// // =======================================================
// // 4-MOTOR TMC5160 SEQUENTIAL HOMING (1 → 2 → 3 → 4)
// // =======================================================

// #include <TMCStepper.h>
// #include <SPI.h>
// #include <digitalWriteFast.h>

// // ---------------- Pin Definitions ----------------
// #define EN_PIN 7

// #define DIR1_PIN 8
// #define STEP1_PIN 9
// #define CS1_PIN 17

// #define DIR2_PIN 10
// #define STEP2_PIN 11
// #define CS2_PIN 20

// #define DIR3_PIN 12
// #define STEP3_PIN 13
// #define CS3_PIN 21

// #define DIR4_PIN 14
// #define STEP4_PIN 15
// #define CS4_PIN 22

// // Shared SPI
// #define MOSI_PIN 19
// #define MISO_PIN 16
// #define SCK_PIN 18

// #define R_SENSE 0.075f

// // ---------------- Driver Objects ----------------
// TMC5160Stepper driver1(CS1_PIN, R_SENSE);
// TMC5160Stepper driver2(CS2_PIN, R_SENSE);
// TMC5160Stepper driver3(CS3_PIN, R_SENSE);
// TMC5160Stepper driver4(CS4_PIN, R_SENSE);

// TMC5160Stepper* drivers[4] = { &driver1, &driver2, &driver3, &driver4 };

int DIR_PINS[4] = { DIR1_PIN, DIR2_PIN, DIR3_PIN, DIR4_PIN };
int STEP_PINS[4] = { STEP1_PIN, STEP2_PIN, STEP3_PIN, STEP4_PIN };
int CS_PINS[4] = { CS1_PIN, CS2_PIN, CS3_PIN, CS4_PIN };

// ---------------- Homing Parameters ----------------
const int stepPulseUs = 3;
const int stepDelayUs = 100;
int stallThresholds[4] = { 120, 100, 80, 120}; // can be tuned for each motor {1, 2, 3, 4}
const int backoffSteps = 180;

// ---------------------------------------------------
// SETUP
// ---------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n--- 4-MOTOR SEQUENTIAL HOMING ---");

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

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

  delay(2000);

  // Home motors sequentially; runs twice with delay
  for (int i = 0; i < 4; i++) {
    homeSingleMotor(i);
  }
  delay(2000);
    for (int i = 0; i < 4; i++) {
    homeSingleMotor(i);
    delay(2000);
  }

  Serial.println("All motors homed sequentially.");
}

// ---------------------------------------------------
// HOME ONE MOTOR (sequential)
// ---------------------------------------------------
void homeSingleMotor(int m) {
  Serial.print("\nHoming motor ");
  Serial.println(m + 1);

  long stepCount = 0;
  unsigned long lastStep = 0;

  // Set direction toward home
  digitalWrite(DIR_PINS[m], HIGH);

  while (true) {
    unsigned long now = micros();

    // Step motor
    if (now - lastStep >= stepDelayUs) {
      lastStep = now;

      digitalWriteFast(STEP_PINS[m], HIGH);
      delayMicroseconds(stepPulseUs);
      digitalWriteFast(STEP_PINS[m], LOW);

      stepCount++;
    }

    // Poll StallGuard every 50 steps
    static int poll = 0;
    poll++;
    if (poll < 50) continue;
    poll = 0;

    digitalWrite(CS_PINS[m], LOW);
    uint16_t sg = drivers[m]->sg_result();
    digitalWrite(CS_PINS[m], HIGH);

    Serial.print("M");
    Serial.print(m + 1);
    Serial.print(" SG=");
    Serial.println(sg);

    // Stall detected
    if (sg > 0 && sg < stallThresholds[m] && stepCount > 50) {
      Serial.print("Motor ");
      Serial.print(m + 1);
      Serial.println(" homed.");
      break;
    }
  }

  // -----------------------
  // BACKOFF
  // -----------------------
  Serial.print("Backing off motor ");
  Serial.println(m + 1);

  digitalWrite(DIR_PINS[m], LOW);

  for (int s = 0; s < backoffSteps; s++) {
    digitalWriteFast(STEP_PINS[m], HIGH);
    delayMicroseconds(stepPulseUs);
    digitalWriteFast(STEP_PINS[m], LOW);
    delayMicroseconds(stepDelayUs);
  }

  Serial.print("Motor ");
  Serial.print(m + 1);
  Serial.println(" finished.");
}

// ---------------------------------------------------
// LOOP
// ---------------------------------------------------
void loop() {
  //should be reading for serial and handle
}
