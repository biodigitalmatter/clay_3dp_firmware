#include <Arduino.h>
#include <AccelStepper.h>

// PINS
const int DO_STEP_PIN = 9;
const int DO_DIR_PIN = 7;
const int DI_ROBOT_STEPPER_FORWARDS_PIN = 2;

// const int DI_ROBOT_ESTOP_PIN = 2;
// const int DO_STEPPER_ENABLE_PIN = 2;

// MOTOR

const int EXTRUDER_RPM = 20;

// NEMA 17HS1070-C5X: 1.8 degrees step angle
const float STEP_ANGLE_DEGREES = 1.8;

// microsteps per rev => microstep multiplier:
// steps_per_rev_microstepping => steps_per_rev_microstepping/steps_per_rev = microstep multiplier
// e.g:
// 400 => 400/200 = 2
// or (steps_per_rev/microsteps_per_rev)^-1 = microstep_multiplier
const int MICROSTEP_MULTIPLIER = 8;

// NEMA 17HS1070-C5X: 5:1
const float GEAR_RATIO = 1 / 5;

// "Speeds of more than 1000 steps per second are unreliable." --accelstepper docs
// But it still seems fine
const float MAX_STEPS_PER_SEC = 1000;

const bool STEPPER_INVERT_DIRECTION = false;

AccelStepper g_stepper;

float rpm_to_steps_per_sec(float rpm, float step_angle_degrees, int microstep_multiplier = 1, float gear_ratio = 1.0) {
  float steps_per_rev = 360.0 / step_angle_degrees;
  steps_per_rev = steps_per_rev * microstep_multiplier;
  steps_per_rev = steps_per_rev * gear_ratio;

  float steps_per_min = rpm * steps_per_rev;

  float steps_per_sec = steps_per_min / 6.0;

  return steps_per_sec;
}

void setup() {
  g_stepper = AccelStepper(AccelStepper::DRIVER, DO_STEP_PIN, DO_DIR_PIN);
  // g_stepper.setEnablePin(O_PIN_ENABLE);
  g_stepper.setPinsInverted(/* directionInvert */ STEPPER_INVERT_DIRECTION,
                            /* stepInvert */ false,
                            /* enableInvert */ true);

  float steps_per_sec = rpm_to_steps_per_sec(EXTRUDER_RPM, STEP_ANGLE_DEGREES, MICROSTEP_MULTIPLIER, GEAR_RATIO);

  g_stepper.setMaxSpeed(MAX_STEPS_PER_SEC);
  g_stepper.setSpeed(steps_per_sec);

  pinMode(DI_ROBOT_STEPPER_FORWARDS_PIN, INPUT_PULLUP);
}

void loop() {
  if (digitalRead(DI_ROBOT_STEPPER_FORWARDS_PIN))
    g_stepper.runSpeed();
}
