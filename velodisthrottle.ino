#include <SPI.h>
#include <mcp2515.h>
#include <math.h>

#define ENCODER_PIN 2
#define THROTTLE_PIN A0         
#define CAN_ID_VELOCITY 0x05
#define CAN_ID_DISTANCE 0x06

#define CAN_ID_THROTTLE 0x04    

volatile int pulseCount = 0;
unsigned long lastPulseTime = 0;
const int debounceTime = 5;

const int pulsePerRevolution = 50;
const float wheelCircumference = 1.57;  // meters

unsigned long lastTime = 0;
const unsigned long interval = 1000;

float totalDistance = 0.0;
float latestVelocity = 0.0;
bool lastThrottleState = false;

MCP2515 mcp2515(10);

void setup() {
    Serial.begin(9600);
    pinMode(ENCODER_PIN, INPUT_PULLUP);
    pinMode(THROTTLE_PIN, INPUT);

    SPI.begin();
    mcp2515.reset();
    if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) == MCP2515::ERROR_OK) {
        Serial.println("✅ CAN Bus initialized (500kbps, 8MHz)");
    } else {
        Serial.println("❌ CAN Bus initialization failed.");
        while (1);
    }
    mcp2515.setNormalMode();
}

void loop() {
    static int lastState = HIGH;
    int currentState = digitalRead(ENCODER_PIN);
    unsigned long currentTime = millis();

    if (lastState == HIGH && currentState == LOW && (currentTime - lastPulseTime > debounceTime)) {
        pulseCount++;
        lastPulseTime = currentTime;
    }
    lastState = currentState;

    // === อ่านค่าคันเร่ง Curtis ===
    float throttleVolt = analogRead(THROTTLE_PIN) * (5.0 / 1023.0);
    bool throttlePressed = (throttleVolt > 1.0);  // เงื่อนไข > 1.0V = ON

    if (throttlePressed != lastThrottleState) {
        lastThrottleState = throttlePressed;

        sendCanMessage(CAN_ID_THROTTLE, throttlePressed ? 1 : 0);

        Serial.print("Throttle Voltage: ");
        Serial.print(throttleVolt, 2);
        Serial.print(" V → Status: ");
        Serial.println(throttlePressed ? "ON (1)" : "OFF (0)");
    }

    // === ส่งความเร็ว/ระยะทุก 1 วินาที ===
    if (currentTime - lastTime >= interval) {
        float RPM = (float)pulseCount / pulsePerRevolution * 60.0;
        float velocity_mps = (RPM / 60.0) * wheelCircumference;
        float velocity_kmh = velocity_mps * 3.6;
        float distance_km = velocity_kmh * (interval / 3600000.0);
        totalDistance += distance_km;
        latestVelocity = velocity_kmh;

        Serial.print("Velocity (km/h): "); Serial.print(velocity_kmh);
        Serial.print(" | Distance (km): "); Serial.println(totalDistance);

        sendCanMessage(CAN_ID_VELOCITY, velocity_kmh);
        sendCanMessage(CAN_ID_DISTANCE, totalDistance*100);

        pulseCount = 0;
        lastTime = currentTime;
    }

    
}

void sendCanMessage(uint32_t canID, float value) {
    struct can_frame frame;
    frame.can_id = canID;
    frame.can_dlc = 2;

    int16_t scaledValue = (int16_t)round(value); 

    frame.data[0] = scaledValue >> 8;
    frame.data[1] = scaledValue & 0xFF;

    if (mcp2515.sendMessage(&frame) == MCP2515::ERROR_OK) {
        Serial.print("📡 Sent CAN ID: 0x");
        Serial.print(canID, HEX);
        Serial.print(" | Scaled Value: ");
        Serial.println(scaledValue);
    } else {
        Serial.println("❌ Error sending CAN message");
    }
}


