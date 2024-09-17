#include <Arduino.h>
#include <Controllino.h>

//#define USE_STEPPER_ENABLE_PIN

const uint8_t DI_ROBOT_RUN_PIN = CONTROLLINO_SCREW_TERMINAL_ANALOG_ADC_IN_00; // A0 PC0 23 Analog 0
const uint8_t DI_ROBOT_DIR_PIN = CONTROLLINO_SCREW_TERMINAL_DIGITAL_ADC_IN_01; // A1 PC1 24 Analog 1

const uint8_t DO_DRIVER_PWR_PIN = CONTROLLINO_SCREW_TERMINAL_RELAY_00; // COMMON TO 24V
const uint8_t DO_DRIVER_DIR_RELAY = CONTROLLINO_SCREW_TERMINAL_RELAY_02; // COMMON TO GND
// Stepper motor pins
#ifdef USE_STEPPER_ENABLE_PIN
const int DO_NC_ENABLE_PIN = CONTROLLINO_PIN_HEADER_DIGITAL_OUT_01; // 5 PD5  9 Digital 1
#endif
const int DO_STEP_PIN = CONTROLLINO_PIN_HEADER_DIGITAL_OUT_02; // 6 PD6 10 Digital 2
const int DO_DIR_PIN = CONTROLLINO_PIN_HEADER_DIGITAL_OUT_03; // 7 PD7 11 Digital 3

void setup() {
  // setup output pins
  pinMode(DO_DRIVER_PWR_PIN, OUTPUT);
  pinMode(DO_DRIVER_DIR_RELAY, OUTPUT);

  // setup robot input pins
  pinMode(DI_ROBOT_RUN_PIN, INPUT_PULLUP);
  pinMode(DI_ROBOT_DIR_PIN, INPUT_PULLUP);
}

void loop() {
  digitalWrite(DO_DRIVER_PWR_PIN, digitalRead(DI_ROBOT_RUN_PIN));
  digitalWrite(DO_DRIVER_DIR_RELAY, !digitalRead(DI_ROBOT_DIR_PIN));
}
