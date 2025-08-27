#include <ModbusMaster.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <mcp2515.h>

// === Analog Input สำหรับเซนเซอร์แรงดัน 12V ===
#define ANALOG_IN_PIN    A0
#define REF_VOLTAGE      4.75
#define ADC_RESOLUTION   1024.0
#define R1               30000.0
#define R2               7500.0

// === RS485 MODBUS SETUP ===
#define MAX485_DE_RE 4
SoftwareSerial rs485Serial(2, 3);  // RX, TX → RO, DI
ModbusMaster node;  // สำหรับ PZEM-017 (48V)

// === CAN SETUP ===
MCP2515 mcp2515(10);
struct can_frame canMsg;

// === CONFIG ===
#define VOLTAGE_FULL_48V   51
#define VOLTAGE_CUTOFF_48V 42

#define VOLTAGE_FULL_12V   14.6
#define VOLTAGE_CUTOFF_12V 10.5

unsigned long lastTime = 0;
const unsigned long interval = 1000;

void preTransmission() {
  digitalWrite(MAX485_DE_RE, HIGH);
}

void postTransmission() {
  digitalWrite(MAX485_DE_RE, LOW);
}

void setup() {
  Serial.begin(9600);
  rs485Serial.begin(9600);

  pinMode(MAX485_DE_RE, OUTPUT);
  digitalWrite(MAX485_DE_RE, LOW);

  node.begin(1, rs485Serial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.println("✅ Start reading 48V Modbus + 12V Analog + CAN Bus");
}

void loop() {
  unsigned long now = millis();
  if (now - lastTime >= interval) {
    lastTime = now;

    readBattery48V();
    readBattery12V();
  }
}

void readBattery48V() {
  uint8_t result = node.readInputRegisters(0x0000, 6);

  if (result == node.ku8MBSuccess) {
    float voltage = node.getResponseBuffer(0) / 100.0;
    float current = node.getResponseBuffer(1) / 100.0;
    float power   = node.getResponseBuffer(2) / 10.0;
    float energy  = node.getResponseBuffer(3) / 1000.0;

    float batteryPercent = ((voltage - VOLTAGE_CUTOFF_48V) / (VOLTAGE_FULL_48V - VOLTAGE_CUTOFF_48V)) * 100.0;
    batteryPercent = constrain(batteryPercent, 0.0, 100.0);

    Serial.println("\n🔋 [48V Battery]");
    Serial.print("Voltage: "); Serial.print(voltage); Serial.println(" V");
    Serial.print("Current: "); Serial.print(current); Serial.println(" A");
    Serial.print("Battery %: "); Serial.print(batteryPercent); Serial.println(" %");

    sendCanInt16(0x11, voltage);        // Voltage
    sendCanInt16(0x12, current * 100);        // Current
    sendCanInt16(0x13, (int16_t)batteryPercent); // Battery %

  } else {
    Serial.print("❌ Failed to read PZEM017 (48V), Error Code: ");
    Serial.println(result);
  }
}

void readBattery12V() {
  int adc_value = analogRead(ANALOG_IN_PIN);
  float voltage_adc = ((float)adc_value * REF_VOLTAGE) / ADC_RESOLUTION;
  float voltage = voltage_adc * (R1 + R2) / R2;

  float batteryPercent = ((voltage - VOLTAGE_CUTOFF_12V) / (VOLTAGE_FULL_12V - VOLTAGE_CUTOFF_12V)) * 100.0;
  batteryPercent = constrain(batteryPercent, 0.0, 100.0);

  Serial.println("\n🔋 [12V Battery]");
  Serial.print("ADC Value: "); Serial.println(adc_value);
  Serial.print("Voltage: "); Serial.print(voltage); Serial.println(" V");
  Serial.print("Battery %: "); Serial.print(batteryPercent); Serial.println(" %");

  sendCanInt16(0x10, voltage * 100);        // Voltage
  sendCanInt16(0x09, round(batteryPercent)); // Battery %
}

void sendCanInt16(uint16_t can_id, int16_t value) {
  
  canMsg.can_id  = can_id;
  canMsg.can_dlc = 2;
  canMsg.data[0] = value >> 8;
  canMsg.data[1] = value & 0xFF;

  if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print("📡 Sent CAN ID: 0x");
    Serial.print(can_id, HEX);
    Serial.print(" | Value: ");
    Serial.println(value);
  } else {
    Serial.println("❌ Failed to send CAN message");
  }
}
