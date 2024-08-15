#include <Arduino.h>
// Install AccelStepper: Tools > Manage libraries > Search for and install "AccelStepper"
#include <AccelStepper.h>

#define DEBUG 0

// SETTINGS
const int EXTRUDER_RPM = 60;  // rotations per minute of stepper

// PINS

// Comm pins
const int DI_ROBOT_RUN_FORWARD_PIN = 2;
const int DI_ROBOT_RUN_BACKWARDS_PIN = 4;

const int DO_DIR_PIN = 7;   // DIR - Direction
const int DO_STEP_PIN = 9;  // STP/PUL - Step, Pulse

// NEMA 17HS1070-C5X: 1.8 degrees step angle
const float STEP_ANGLE_DEGREES = 1.8;

// microsteps per rev => microstep multiplier:
// steps_per_rev_microstepping => steps_per_rev_microstepping/steps_per_rev = microstep multiplier
// e.g:
// 400 => 400/200 = 2
// or (steps_per_rev/microsteps_per_rev)^-1 = microstep_multiplier
const int MICROSTEP_MULTIPLIER = 8;

// NEMA 17HS1070-C5X: 5:1
const float GEAR_RATIO = 5;

const bool STEPPER_INVERT_DIR = false;

// "Speeds of more than 1000 steps per second are unreliable." --accelstepper docs
// But it still seems fine
const float MAX_STEPS_PER_SEC = 5000;

// Create a new instance of the AccelStepper class:
AccelStepper g_stepper = AccelStepper(AccelStepper::DRIVER, DO_STEP_PIN, DO_DIR_PIN);

// FUNCTIONS

float rpm_to_steps_per_sec(float rpm, float step_angle_degrees, float microstep_multiplier = 1.0, float gear_ratio = 1.0) {
  float steps_per_rev = 360.0 / step_angle_degrees;

  steps_per_rev = steps_per_rev * microstep_multiplier;
  steps_per_rev = steps_per_rev * gear_ratio;

  float steps_per_min = rpm * steps_per_rev;

  float steps_per_sec = steps_per_min / 60.;

  return steps_per_sec;
}

const int STEPS_PER_SEC = round(rpm_to_steps_per_sec(EXTRUDER_RPM, STEP_ANGLE_DEGREES, MICROSTEP_MULTIPLIER, GEAR_RATIO));

void setup() {
  //g_stepper.setEnablePin(O_PIN_ENABLE);
  g_stepper.setPinsInverted(/* directionInvert */ STEPPER_INVERT_DIR,
                            /* stepInvert */ false,
                            /* enableInvert */ true);
  // Set the maximum speed in steps per second:
  g_stepper.setMaxSpeed(MAX_STEPS_PER_SEC);

  // setup robot input pin
  pinMode(DI_ROBOT_RUN_FORWARD_PIN, INPUT_PULLUP);
  pinMode(DI_ROBOT_RUN_BACKWARDS_PIN, INPUT_PULLUP);

  // led
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

#if DEBUG == 1
  Serial.begin(9600);
#endif
}

void loop() {

  bool forwards = digitalRead(DI_ROBOT_RUN_FORWARD_PIN) == HIGH;
  bool backwards = digitalRead(DI_ROBOT_RUN_BACKWARDS_PIN) == HIGH;

  int direction = 0;  // Defaults to 0

  if (forwards && !backwards) {
    direction = 1;
  } else if (!forwards && backwards) {
    direction = -1;
  }

#if DEBUG == 1
  Serial.print("Dir:");
  Serial.print(direction);
#endif

  if (direction != 0) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  g_stepper.setSpeed(STEPS_PER_SEC * direction);

#if DEBUG == 1
  Serial.print(",");
  Serial.print("Speed:");
  Serial.println(g_stepper.speed());
#endif

  g_stepper.runSpeed();
}
