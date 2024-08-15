#include <Arduino.h>
#include <Controllino.h>  /* Usage of CONTROLLINO library allows you to use CONTROLLINO_xx aliases in your sketch. */

const uint8_t DI_ROBOT_RUN_PIN = CONTROLLINO_SCREW_TERMINAL_ANALOG_ADC_IN_00;
const uint8_t DI_ROBOT_DIR_PIN = CONTROLLINO_SCREW_TERMINAL_DIGITAL_ADC_IN_01;

const uint8_t DO_DRIVER_PWR_PIN = CONTROLLINO_SCREW_TERMINAL_RELAY_00; // COMMON TO 24V
const uint8_t DO_DRIVER_DIR_RELAY = CONTROLLINO_SCREW_TERMINAL_RELAY_02; // COMMON TO GND

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
