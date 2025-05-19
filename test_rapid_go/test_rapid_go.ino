#include <Arduino.h>

const uint8_t NUM_SPD_BITS = 8;

const pin_size_t DI_ROBOT_SPD0_PIN = CONTROLLINO_MICRO_AI2;
const pin_size_t DI_ROBOT_SPD1_PIN = CONTROLLINO_MICRO_AI3;
const pin_size_t DI_ROBOT_SPD2_PIN = CONTROLLINO_MICRO_AI4;
const pin_size_t DI_ROBOT_SPD3_PIN = CONTROLLINO_MICRO_AI5;
const pin_size_t DI_ROBOT_SPD4_PIN = CONTROLLINO_MICRO_DI0;
const pin_size_t DI_ROBOT_SPD5_PIN = CONTROLLINO_MICRO_DI1;
const pin_size_t DI_ROBOT_SPD6_PIN = CONTROLLINO_MICRO_DI2;
const pin_size_t DI_ROBOT_SPD7_PIN = CONTROLLINO_MICRO_DI3;

pin_size_t ROBOT_SPD_PINS [] = {
  DI_ROBOT_SPD0_PIN,
  DI_ROBOT_SPD1_PIN,
  DI_ROBOT_SPD2_PIN,
  DI_ROBOT_SPD3_PIN,
  DI_ROBOT_SPD4_PIN,
  DI_ROBOT_SPD5_PIN,
  DI_ROBOT_SPD6_PIN,
  DI_ROBOT_SPD7_PIN,
};


char* ROBOT_SPD_PIN_NAMES [] = {
"AI2",
"AI3",
"AI4",
"AI5",
"DI0",
"DI1",
"DI2",
"DI3",
};

// this is default on the controllino but I'm setting it just to be sure
const uint8_t AI_RESOLUTION = 23;

// Calculate threshold value for 7V in a 0-24V range with 23-bit resolution
// 7V comes from spec from ABB, I think
const uint32_t AI_DIGITAL_THRESHOLD = (7.0 / 24.0) * ((1UL << AI_RESOLUTION) - 1);

void setup() {
  Serial.begin(115200);
  // wait since Controllino Micro uses software USB not hardware USB-to-serial
  while (!Serial);
  Serial.println("Starting...");

  analogReadResolution(AI_RESOLUTION);

  // setup robot spd input pins
  for (pin_size_t pin : ROBOT_SPD_PINS) {
    setDigitalThreshold(pin, AI_DIGITAL_THRESHOLD); // noop for digital inputs
    pinMode(pin, INPUT);
  }
}


void printDIs() {
  for (int i = 0; i<NUM_SPD_BITS; i++ ) {
    Serial.print("pin_name:");
    Serial.print(ROBOT_SPD_PIN_NAMES[i]);
    Serial.print(",");
    Serial.print("state:");
    Serial.println(digitalRead(ROBOT_SPD_PINS[i]));
    Serial.print(",");
  }
}

uint8_t readSpdInputs() {
  uint8_t rawValue = 0;

  for (int i = 0; i < NUM_SPD_BITS; i++) {
    rawValue |= (digitalRead(ROBOT_SPD_PINS[i]) << i);
  }
  return rawValue;
}

int8_t readSignedSpdInputs() {
  // Convert to signed int8_t
  // Values 128-255 will become -128 to -1
  return (int8_t)readSpdInputs();
}

uint32_t g_prev_millis = 0;
uint32_t g_cur_millis = 0;
uint8_t g_spd_reading = 0;

void loop() {
  g_cur_millis = millis();

  if (g_cur_millis - g_prev_millis >= 500) {
    g_prev_millis = g_cur_millis;

    printDIs();

    g_spd_reading = readSpdInputs();

    Serial.print("g_spdReading:");
    Serial.println(g_spd_reading);
    Serial.print(",signed:");
    Serial.println(readSignedSpdInputs());
  }
}
