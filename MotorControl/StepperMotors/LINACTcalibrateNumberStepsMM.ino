// this code calibrates the number of steps per mm of a linear actuator using a stepper motor 
#include <TMCStepper.h>
#include <SPI.h>

// --- Pin Definitions ---
#define EN_PIN     7
#define DIR_PIN    8
#define STEP_PIN   9
#define CS_PIN     17
#define MOSI_PIN   19
#define MISO_PIN   16
#define SCK_PIN    18

#define R_SENSE    0.075f

// --- TMC5160 Driver (hardware SPI) ---
TMC5160Stepper driver(CS_PIN, R_SENSE);

// --- Motion Settings ---
const int stepPulseUs = 3;       // Step pulse width
const int stepDelayUs = 20;      // Delay between steps (speed)
const int backoffSteps = 500;    // Steps to reverse after stall
const int stallThreshold = 80;   // SG_RESULT threshold for stall detection

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("\n--- TMC5160 Linear Actuator Travel Calibration ---");

  // --- Pin setup ---
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT);
  pinMode(SCK_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW); // Enable driver

  // --- SPI + Driver setup ---
  SPI.begin();
  driver.begin();

  driver.toff(5);
  driver.blank_time(24);
  driver.microsteps(16);
  driver.rms_current(600);    // Enough torque to move fully
  driver.en_pwm_mode(false);  // SpreadCycle (StallGuard only works here)
  driver.pwm_autoscale(false);
  driver.intpol(true);
  driver.TCOOLTHRS(150000);
  driver.sgt(10);             // Medium StallGuard sensitivity

  Serial.println("Driver initialized.\n");

  // Step 1: Home actuator
  homeActuator();

  // Step 2: Move to far end and count steps
  //moveToEndAndCount();

}

void homeActuator() {
  Serial.println("Homing actuator...");

  digitalWrite(DIR_PIN, LOW); // Move toward home
  long steps = 0;

  while (true) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepPulseUs);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelayUs);
    steps++;

    uint16_t sg = driver.sg_result();
    Serial.println(sg);

    if (sg < stallThreshold && steps > 50) {
      Serial.println("Home stall detected!");
      break;
    }
  }

  // --- Back off slightly ---
  Serial.println("Backing off from home...");
  digitalWrite(DIR_PIN, HIGH); // Move away from stop
  for (int i = 0; i < backoffSteps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepPulseUs);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelayUs);
  }

  driver.XACTUAL(0); // Reset position to zero
  Serial.println("Position set to 0 (home).\n");
}

void moveToEndAndCount() {
  Serial.println("Moving toward far end...");

  digitalWrite(DIR_PIN, HIGH); // Move opposite direction (away from home)
  long stepCount = 0;

  while (true) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepPulseUs);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelayUs);
    stepCount++;

    uint16_t sg = driver.sg_result();
    Serial.println(sg);

    // Stall detected at far end
    if (sg < stallThreshold && stepCount > 200) {
      Serial.println("\nFar-end stall detected!");
      break;
    }
  }

  Serial.print("Total travel steps: ");
  Serial.println(stepCount);

  // --- Return home for verification ---
  Serial.println("Returning home...");
  digitalWrite(DIR_PIN, LOW); // Back toward zero

  while (true) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepPulseUs);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelayUs);

    uint16_t sg = driver.sg_result();
    if (sg < stallThreshold) {
      Serial.println("Home reached again.");
      break;
    }
  }

  digitalWrite(EN_PIN, HIGH); // Disable driver
  Serial.println("\nCalibration complete!");
}

void loop() {
  // Nothing in loop
}