#include <Wire.h>
#include <SPI.h>
#include <mcp2515.h>



// HW040 Pin Definitions
#define HW040_CLK_PIN 2
#define HW040_DT_PIN  3
#define HW040_SW_PIN  4

// Rotary Encoder Constants
#define DIRECTION_CW 0
#define DIRECTION_CCW 1
#define PPR 80 // Adjusted PPR to match your encoder
#define MAX_DEGREES 540
#define MIN_DEGREES -540

// MCP2515 CAN Configuration
MCP2515 mcp2515(10);
struct can_frame canMsgMPU;
struct can_frame canMsgHW040;



// HW040 Rotary Encoder Variables
volatile int hw040_counter = 0;
int hw040_direction = DIRECTION_CW;
float hw040_degrees = 0.0;
unsigned long lastTimeMPU = 0;
unsigned long lastTimeHW040 = 0;

void calculateDegrees() {
  hw040_degrees = (360.0 / PPR) * hw040_counter;

  // Clamp degrees to the range [-540, 540]
  if (hw040_degrees > MAX_DEGREES) {
    hw040_degrees = MAX_DEGREES;
    hw040_counter = (MAX_DEGREES * PPR) / 360;
  } else if (hw040_degrees < MIN_DEGREES) {
    hw040_degrees = MIN_DEGREES;
    hw040_counter = (MIN_DEGREES * PPR) / 360;
  }
}

void isr_rotaryEncoder() {
  static bool last_CLK_state = LOW;
  bool current_CLK_state = digitalRead(HW040_CLK_PIN);
  bool current_DT_state = digitalRead(HW040_DT_PIN);

  if (current_CLK_state != last_CLK_state && current_CLK_state == HIGH) { // Rising edge detection
    if (current_DT_state == LOW) {
      hw040_counter++;
      hw040_direction = DIRECTION_CW; // Clockwise
    } else {
      hw040_counter--;
      hw040_direction = DIRECTION_CCW; // Counter-clockwise
    }
    calculateDegrees();

    // Debug output immediately
    Serial.print("HW040 Direction: ");
    Serial.print(hw040_direction == DIRECTION_CW ? "Clockwise" : "Counter-clockwise");
    Serial.print(" | Counter: ");
    Serial.print(hw040_counter);
    Serial.print(" | Degrees: ");
    Serial.println(hw040_degrees);
  }
  last_CLK_state = current_CLK_state;
}

void setup() {
  // Initialize Serial
  Serial.begin(9600);

  

  // Initialize Pins for HW040 Rotary Encoder
  pinMode(HW040_CLK_PIN, INPUT);
  pinMode(HW040_DT_PIN, INPUT);
  pinMode(HW040_SW_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(HW040_CLK_PIN), isr_rotaryEncoder, CHANGE);

  // Initialize MCP2515 CAN Controller
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  

  // Set CAN Message Properties for HW040
  canMsgHW040.can_id = 0x02;
  canMsgHW040.can_dlc = 2; // Sending 2 bytes for HW040 degrees
}

void loop() {
  

  // Send HW040 Data Every 1000 ms
  if (millis() - lastTimeHW040 >= 1000) {
    lastTimeHW040 = millis();

    canMsgHW040.data[0] = (int16_t)(hw040_degrees) >> 8;  // MSB
    canMsgHW040.data[1] = (int16_t)(hw040_degrees) & 0xFF; // LSB

    if (mcp2515.sendMessage(&canMsgHW040) == MCP2515::ERROR_OK) {
      Serial.println("HW040 CAN message sent!");
    } else {
      Serial.println("Error sending HW040 CAN message!");
    }
  }

  delay(1000); // Short delay for debounce
}