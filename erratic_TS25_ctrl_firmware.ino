#include <Arduino.h>
#include <ODriveMCPCAN.hpp>

//#define DEBUG 1
#define DEBUG_GInputs 0

const uint32_t SPEED_SEND_INTERVAL = 10;

const float_t EXTRUSION_SPD_RPS_MIN = -30.0;
const float_t EXTRUSION_SPD_RPS_MAX = 30.0;

const pin_size_t DI_ROBOT_SPD0_PIN = CONTROLLINO_MICRO_AI2;
const pin_size_t DI_ROBOT_SPD1_PIN = CONTROLLINO_MICRO_AI3;
const pin_size_t DI_ROBOT_SPD2_PIN = CONTROLLINO_MICRO_AI4;
const pin_size_t DI_ROBOT_SPD3_PIN = CONTROLLINO_MICRO_AI5;
const pin_size_t DI_ROBOT_SPD4_PIN = CONTROLLINO_MICRO_DI0;
const pin_size_t DI_ROBOT_SPD5_PIN = CONTROLLINO_MICRO_DI1;
const pin_size_t DI_ROBOT_SPD6_PIN = CONTROLLINO_MICRO_DI2;
const pin_size_t DI_ROBOT_SPD7_PIN = CONTROLLINO_MICRO_DI3;

const uint8_t NUM_SPD_BITS = 8;

pin_size_t ROBOT_SPD_PINS[] = {
  DI_ROBOT_SPD0_PIN,
  DI_ROBOT_SPD1_PIN,
  DI_ROBOT_SPD2_PIN,
  DI_ROBOT_SPD3_PIN,
  DI_ROBOT_SPD4_PIN,
  DI_ROBOT_SPD5_PIN,
  DI_ROBOT_SPD6_PIN,
  DI_ROBOT_SPD7_PIN,
};


// this is default on the controllino but I'm setting it just to be sure
const uint8_t AI_RESOLUTION = 23;

// Calculate threshold value for 7V in a 0-24V range with 23-bit resolution
// 7V comes from spec from ABB, I think
const uint32_t AI_DIGITAL_THRESHOLD = (7.0 / 24.0) * ((1UL << AI_RESOLUTION) - 1);

// CAN bus baudrate. Make sure this matches for every device on the bus
const uint32_t CAN_BAUDRATE = 125000;

// ODrive node_id for g_odrv0
const uint32_t ODRV0_NODE_ID = 0;

// vendored/modified from controllino_rp2\hardware\rp2040
MCP2515Class& g_can_intf = CAN;

void printAndHalt(const char* message) {
#ifdef DEBUG
  Serial.println(message);
#endif
  while (true);
}

// Instantiate ODrive objects
ODriveCAN g_odrv0(wrap_can_intf(g_can_intf), ODRV0_NODE_ID);  // Standard CAN message ID
ODriveCAN* g_odrives[] = { &g_odrv0 };                        // Make sure all ODriveCAN instances are accounted for here

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Keep some application-specific user data for every ODrive.
ODriveUserData g_odrv0_user_data;

static inline void receiveCallback(int packet_size) {
  if (packet_size > 8) {
    return;  // not supported
  }
  CanMsg msg = { .id = (uint32_t)g_can_intf.packetId(), .len = (uint8_t)packet_size };
  g_can_intf.readBytes(msg.buffer, packet_size);
  onCanMessage(msg);
}

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* g_odrv0_user_data = static_cast<ODriveUserData*>(user_data);
  g_odrv0_user_data->last_heartbeat = msg;
  g_odrv0_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* g_odrv0_user_data = static_cast<ODriveUserData*>(user_data);
  g_odrv0_user_data->last_feedback = msg;
  g_odrv0_user_data->received_feedback = true;
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  for (auto odrive : g_odrives) {
    onReceive(msg, *odrive);
  }
}

bool setupCan() {
  SPI1.setRX(PIN_SPI1_MISO);
  SPI1.setTX(PIN_SPI1_MOSI);
  SPI1.setSCK(PIN_SPI1_SCK);
  // configure and initialize the CAN bus interface
  //CAN.setClockFrequency(MCP2515_CLK_HZ);
  if (!g_can_intf.begin(CAN_BAUDRATE)) {
    return false;
  }
  g_can_intf.onReceive(receiveCallback);
  return true;
}

void setup() {
#ifdef DEBUG
  Serial.begin(115200);

  // wait since Controllino Micro uses software USB not hardware USB-to-serial */
  while (!Serial);
  Serial.println("Starting...");
#endif

  delay(200);

  analogReadResolution(AI_RESOLUTION);

  // setup robot spd input pins
  for (pin_size_t pin : ROBOT_SPD_PINS) {
    setDigitalThreshold(pin, AI_DIGITAL_THRESHOLD);  // noop for digital inputs
    pinMode(pin, INPUT);
  }

  // Register callbacks for the heartbeat and encoder feedback messages
  g_odrv0.onFeedback(onFeedback, &g_odrv0_user_data);
  g_odrv0.onStatus(onHeartbeat, &g_odrv0_user_data);

  if (!setupCan()) printAndHalt("CAN failed to initialize: reset required");

#ifdef DEBUG
  Serial.println("Waiting for ODrive...");
#endif
  while (!g_odrv0_user_data.received_heartbeat) {
    pumpEvents(g_can_intf);
    delay(100);
  }

#ifdef DEBUG
  Serial.println("found ODrive");

  // request bus voltage and current (1sec timeout)
  Serial.println("attempting to read bus voltage and current");
#endif

  Get_Bus_Voltage_Current_msg_t vbus;

  if (!g_odrv0.getBusVI(vbus, 1000)) {
    printAndHalt("vbus request failed!");
  }

#ifdef DEBUG
  Serial.print("DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  Serial.println("Enabling closed loop control...");
#endif

  while (g_odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    g_odrv0.clearErrors();
    delay(1);
    g_odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    // Pump events for 150ms. This delay is needed for two reasons;
    // 1. If there is an error condition, such as missing DC power, the ODrive might
    //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
    //    on the first heartbeat response, so we want to receive at least two
    //    heartbeats (100ms default interval).
    // 2. If the bus is congested, the setState command won't get through
    //    immediately but can be delayed.
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(g_can_intf);
    }
  }
#ifdef DEBUG
  Serial.println("ODrive running!");
#endif
}

#if DEBUG_GInputs == 1
uint8_t g_debug_spd_counter_with_overflow = 0;
uint32_t g_debug_spd_prev_millis = 0;
const uint16_t DEBUG_SPD_INTERVAL = 100;

uint8_t readSpdInputs() {
  uint32_t debug_spd_cur_millis = millis();
  if (debug_spd_cur_millis - g_debug_spd_prev_millis > DEBUG_SPD_INTERVAL) {
    g_debug_spd_prev_millis = debug_spd_cur_millis;
    // Incrementing counter with expected overflow at 255
    g_debug_spd_counter_with_overflow++;
  }
  return g_debug_spd_counter_with_overflow;
}
#else
uint8_t readSpdInputs() {
  uint8_t rawValue = 0;

  for (int i = 0; i < NUM_SPD_BITS; i++) {
    rawValue |= (digitalRead(ROBOT_SPD_PINS[i]) << i);
  }
  return rawValue;
}
#endif

int8_t readSignedSpdInputs() {
  // Convert to signed int8_t
  // Values 128-255 will become -128 to -1
  return (int8_t)readSpdInputs();
}

uint32_t g_prev_millis = 0;
uint32_t g_cur_millis = 0;
uint8_t g_spd_reading = 0;

void loop() {
  pumpEvents(g_can_intf);  // This is required on some platforms to handle incoming feedback CAN messages

  g_cur_millis = millis();

  if (g_cur_millis - g_prev_millis >= SPEED_SEND_INTERVAL) {
    g_prev_millis = g_cur_millis;

    g_spd_reading = readSignedSpdInputs();


    g_odrv0.setVelocity(map(g_spd_reading,
                            -127,
                            127,
                            EXTRUSION_SPD_RPS_MIN, EXTRUSION_SPD_RPS_MAX));

#ifdef DEBUG
    Serial.print("g_spdReading:");
    Serial.print(g_spd_reading);
    Serial.print(",");
    Serial.print("g_odrv0-vel:");
    if (g_odrv0_user_data.received_feedback) {
      Get_Encoder_Estimates_msg_t feedback = g_odrv0_user_data.last_feedback;
      g_odrv0_user_data.received_feedback = false;

      Serial.println(feedback.Vel_Estimate);
    } else {
      Serial.println("ERROR");
    }
#endif
  }
}
