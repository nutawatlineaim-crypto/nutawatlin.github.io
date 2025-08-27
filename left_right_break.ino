#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(10);  // CS = Pin 10

// ==== INPUT PIN CONFIG ====
#define RIGHT_VOLT_PIN A0  // ไฟเลี้ยวขวา
#define LEFT_VOLT_PIN  A1  // ไฟเลี้ยวซ้าย
#define BRAKE_SWITCH_PIN 5 // สวิตช์เบรก

// ==== ไฟกระพริบแยกอิสระ ====
unsigned long lastBlinkTimeLeft = 0;
unsigned long lastBlinkTimeRight = 0;
bool blinkStateLeft = false;      // ใช้สลับ true/false เพื่อกระพริบไฟเลี้ยวซ้าย
bool blinkStateRight = false;     // ใช้สลับ true/false เพื่อกระพริบไฟเลี้ยวขวา
bool brakePressed = false;        // เก็บสถานะว่ากดเบรกอยู่หรือไม่
bool lastBrakeState = false;      // ใช้ตรวจจับการเปลี่ยนแปลงสถานะเบรก
bool lastRightState = false;      // เก็บสถานะล่าสุดของไฟเลี้ยวขวา
bool lastLeftState = false;       // เก็บสถานะล่าสุดของไฟเลี้ยวซ้าย


const unsigned long blinkInterval = 500; // กระพริบทุก 500 ms


void setup() {
  Serial.begin(9600);

  pinMode(RIGHT_VOLT_PIN, INPUT);
  pinMode(LEFT_VOLT_PIN, INPUT);
  pinMode(BRAKE_SWITCH_PIN, INPUT_PULLUP);

  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Serial.println("✅ CAN Bus Initialized (Slave)");
}

void loop() {
  checkCanMessages();

  // ==== เบรก ====
  brakePressed = (digitalRead(BRAKE_SWITCH_PIN) == LOW);
  if (brakePressed != lastBrakeState) {
    lastBrakeState = brakePressed;
    sendCanMessage(0x03, brakePressed ? 1 : 0);  // ส่งเบรก
    Serial.println(brakePressed ? "🛑 Brake Pressed" : "✅ Brake Released");
  }

  // ==== ไฟเลี้ยวขวา ====
  float rightVolt = analogRead(RIGHT_VOLT_PIN) * (5.0 / 1023.0);
  Serial.print("A1 (Right) Volt: ");
  Serial.println(rightVolt, 2);

  if (rightVolt > 4.0) {
    if (millis() - lastBlinkTimeRight >= blinkInterval) {
      lastBlinkTimeRight = millis();
      blinkStateRight = !blinkStateRight;
      sendCanMessage(0x07, blinkStateRight ? 1 : 0);
      Serial.println(blinkStateRight ? "➡️ Right Blink ON" : "➡️ Right Blink OFF");
    }
    lastRightState = true;
  } else if (rightVolt < 0.5 && lastRightState) {
    lastRightState = false;
    blinkStateRight = false;
    sendCanMessage(0x07, 0);  // ปิดแน่นอน
    Serial.println("➡️ Right Indicator OFF");
  }

  // ==== ไฟเลี้ยวซ้าย ====
  float leftVolt = analogRead(LEFT_VOLT_PIN) * (5.0 / 1023.0);
  Serial.print("A0 (Left) Volt: ");
  Serial.println(leftVolt, 2);

  if (leftVolt > 4.0) {
    if (millis() - lastBlinkTimeLeft >= blinkInterval) {
      lastBlinkTimeLeft = millis();
      blinkStateLeft = !blinkStateLeft;
      sendCanMessage(0x08, blinkStateLeft ? 1 : 0);
      Serial.println(blinkStateLeft ? "⬅️ Left Blink ON" : "⬅️ Left Blink OFF");
    }
    lastLeftState = true;
  } else if (leftVolt < 0.5 && lastLeftState) {
    lastLeftState = false;
    blinkStateLeft = false;
    sendCanMessage(0x08, 0);  // ปิดแน่นอน
    Serial.println("⬅️ Left Indicator OFF");
  }
}


void checkCanMessages() {
  struct can_frame canMsg;

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print("📥 Received CAN ID: 0x");
    Serial.println(canMsg.can_id, HEX);

    if (canMsg.can_id == 0x400) {
      Serial.println("🔄 Master Request Received");

      sendCanMessage(0x03, brakePressed ? 1 : 0);       // เบรก
      sendCanMessage(0x08, lastLeftState ? 1 : 0);       // ไฟเลี้ยวซ้าย
      sendCanMessage(0x07, lastRightState ? 1 : 0);      // ไฟเลี้ยวขวา
    }
  }
}

void sendCanMessage(uint32_t canID, uint8_t value) {
  struct can_frame canMsg;
  canMsg.can_id = canID;
  canMsg.can_dlc = 2;
   uint8_t payload = (value == 1) ? 0xFF : 0x00;  // สวยงามขึ้น
  canMsg.data[0] = 0x00;
  canMsg.data[1] = payload;

  if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print("📡 Sent CAN ID: 0x");
    Serial.print(canID, HEX);
    Serial.print(" | Value: ");
    Serial.println(value);
  } else {
    Serial.println("❌ Failed to send CAN message");
  }
}
