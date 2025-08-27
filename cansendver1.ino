#include <TFMPlus.h>
#include <Servo.h>
#include <mcp2515.h>
#include <SPI.h>
#include "printf.h"

#define BUZZER 6
#define SERVO_PIN 9
#define BUZZER_INTERVAL 500  
#define MOVING_AVG_SIZE 5


TFMPlus tfmP;
Servo myservo;
MCP2515 mcp2515(10);

// ตัวแปร LiDAR
int16_t tfDist = 0;
int lidar_buffer[MOVING_AVG_SIZE] = {0};
int lidar_index = 0;

// ตัวแปร Buzzer
bool buzzerState = HIGH;
unsigned long previousMillisBuzzer = 0;

// ตัวแปร Servo
int servoAngle = 60;
bool isIncreasing = true;
unsigned long previousServoMillis = 0;
const unsigned long servoInterval = 30;
bool servoPaused = false;
unsigned long servoPauseTime = 0;

// ตัวแปรระยะสำหรับการแจ้งเตือน (อัปเดตจาก DWIN ผ่านตัวรับ)
int thresholdDistanceLidar = 10;

// ตัวแปรสำหรับพิมพ์ค่าทุก 5 วินาที
unsigned long previousPrintMillis = 0;
const unsigned long printInterval = 5000; // 5 วินาที (5000ms)

// ตัวแปรเก็บค่าที่ส่งล่าสุด
int lastSentDistance = -1;
int lastSentAngle = -1;

// ตัวแปรเก็บค่าที่พิมพ์ล่าสุด
int lastPrintedDistance = -1;
int lastPrintedAngle = -1;

// ตัวแปรตรวจสอบว่าเซอร์โวกำลังหมุนอยู่หรือไม่
bool isServoMoving = true; 
//ควบตุมให้สถานะ Buzzer ทำงานได้ต่อเนื่อง
bool shouldBlinkBuzzer = false;

// ตั้งค่า LiDAR
void setupLiDAR() {
    Serial.begin( 115200);   // Intialize terminal serial port
    delay(20);               // Give port time to initalize
    printf_begin();          // Initialize printf.
    printf("\r\nTFMPlus Library Example - 10SEP2021\r\n");  // say 'hello'

    Serial.begin( 115200);  // Initialize TFMPLus device serial port.
    delay(20);               // Give port time to initalize
    tfmP.begin( &Serial);   // Initialize device library object and...
                             // pass device serial port to the object.

    // Send some example commands to the TFMini-Plus
    // - - Perform a system reset - - - - - - - - - - -
    printf( "Soft reset: ");
    if( tfmP.sendCommand( SOFT_RESET, 0))
    {
        printf( "passed.\r\n");
    }
    else tfmP.printReply();
  
    delay(500);  // added to allow the System Rest enough time to complete

  // - - Display the firmware version - - - - - - - - -
    printf( "Firmware version: ");
    if( tfmP.sendCommand( GET_FIRMWARE_VERSION, 0))
    {
        printf( "%1u.", tfmP.version[ 0]); // print three single numbers
        printf( "%1u.", tfmP.version[ 1]); // each separated by a dot
        printf( "%1u\r\n", tfmP.version[ 2]);
    }
    else tfmP.printReply();
    // - - Set the data frame-rate to 20Hz - - - - - - - -
    printf( "Data-Frame rate: ");
    if( tfmP.sendCommand( SET_FRAME_RATE, FRAME_20))
    {
        printf( "%2uHz.\r\n", FRAME_20);
    }
    else tfmP.printReply();
  delay(500);            // And wait for half a second.
}


void setup() {
    Serial.begin(115200);
    setupLiDAR();

    // ตั้งค่า CAN Bus
    SPI.begin();
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    Serial.println("✅ CAN Bus Transmitter initialized (500kbps, 8MHz)");

    // ตั้งค่า Servo
    myservo.attach(SERVO_PIN);
    myservo.write(servoAngle);

    // ตั้งค่า Buzzer
    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, HIGH);
}

void loop() {
    moveServo();
    readLidarContinuously(); 
    processCANMessage(); 
    printDataEvery5Seconds();
    blinkBuzzer();
   
    
}

// ฟังก์ชันหมุน Servo ไป-กลับ 120 องศา
void moveServo() {
    // if (servoPaused) {
    //     if (millis() - servoPauseTime >= 1000) {  // รอ 1 วินาที
    //         servoPaused = false;
    //         isServoMoving = true;  // ✅ เซอร์โวเริ่มทำงานอีกครั้ง
    //     } else {
    //         isServoMoving = false; // ✅ เซอร์โวหยุด
    //         return;  // หยุดทำงานของ Servo
    //     }
    // }

    unsigned long currentMillis = millis();
    if (currentMillis - previousServoMillis >= servoInterval) {
        previousServoMillis = currentMillis;
        myservo.write(servoAngle);

        if (isIncreasing) {
            servoAngle+=2;
            if (servoAngle >= 120) isIncreasing = false;
        } else {
            servoAngle-=2;
            if (servoAngle <= 30) isIncreasing = true;
        }
    }
}

void processCANMessage() {
    struct can_frame canMsg;
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        switch (canMsg.can_id) {
            case 0x203:  // threshold update
                thresholdDistanceLidar = (canMsg.data[0] << 8) | canMsg.data[1];
                Serial.print("🔹 Received Threshold: ");
                Serial.println(thresholdDistanceLidar);
                break;
            case 0x310:  // Master request
                Serial.println("📡 ✅ Received request from Master (Sender1)");
                checkLidar();  // ✅ ส่งค่าถ้าระยะ < threshold
                break;
        }
    }
}


// ฟังก์ชันคำนวณค่าเฉลี่ยของ LiDAR
int calculateMovingAverage(int *buffer) {
    int sum = 0;
    for (int i = 0; i < MOVING_AVG_SIZE; i++) {
        sum += buffer[i];
    }
    return sum / MOVING_AVG_SIZE;
}

int latestDistance = 0;
void readLidarContinuously() {
    if (tfmP.getData(tfDist)) {
        lidar_buffer[lidar_index] = tfDist;
        lidar_index = (lidar_index + 1) % MOVING_AVG_SIZE;
        latestDistance = calculateMovingAverage(lidar_buffer);  // 👈 เก็บค่าไว้
    }
}

// ฟังก์ชันตรวจจับระยะทางด้วย LiDAR
void checkLidar() {
    int avgDist = calculateMovingAverage(lidar_buffer);
    
    // ตรวจสอบว่าระยะต่ำกว่าค่าที่ตั้งไว้ไหม
    if (avgDist < thresholdDistanceLidar) {
        
        if (!shouldBlinkBuzzer) {  // หาก buzzer ยังไม่ได้กระพริบ
            shouldBlinkBuzzer = true;  // เปิด Buzzer
            sendCanMessage(0x200, avgDist);  // ส่งค่าระยะ
            sendCanMessage(0x201, servoAngle); // ส่งค่ามุม
        }
    } else {
        if (shouldBlinkBuzzer) {  // หาก buzzer กำลังกระพริบ
            shouldBlinkBuzzer = false;  // ปิด Buzzer
            digitalWrite(BUZZER, HIGH);
        }
    }
}


// ฟังก์ชันทำให้ Buzzer กระพริบ
void blinkBuzzer() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisBuzzer >= BUZZER_INTERVAL) {
        previousMillisBuzzer = currentMillis;
        buzzerState = !buzzerState;
        
        if (shouldBlinkBuzzer) {
            digitalWrite(BUZZER, buzzerState);  // ทำให้ Buzzer กระพริบ
        }
    }
}

// 📡 ฟังก์ชันส่งข้อมูลผ่าน CAN Bus
void sendCanMessage(int canID, int value) {
    // ✅ ถ้าค่าเหมือนเดิม ไม่ต้องส่งซ้ำ
    if ((canID == 0x200 && value == lastSentDistance) || (canID == 0x201 && value == lastSentAngle)) {
        return;
    }

    // ✅ อัปเดตค่าที่ส่งล่าสุด
    if (canID == 0x200) lastSentDistance = value;
    if (canID == 0x201) lastSentAngle = value;

    struct can_frame canMsg;
    canMsg.can_id = canID;
    canMsg.can_dlc = 2;
    canMsg.data[0] = highByte(value);
    canMsg.data[1] = lowByte(value);

    Serial.print("📡 Sending CAN ID: 0x");
    Serial.print(canID, HEX);
    Serial.print(" | Value: ");
    Serial.println(value);

    for (int attempt = 0; attempt < 3; attempt++) {
        if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {
            Serial.println("✅ CAN Message Sent Successfully");
            return;
        } else {
            Serial.println("❌ Error Sending CAN Message, Retrying...");
            mcp2515.reset();
            delay(10);
        }
    }
    Serial.println("🚨 Failed to send CAN Message after 3 attempts.");
}

// ✅ **ฟังก์ชันพิมพ์ค่าทุก 5 วินาที**
void printDataEvery5Seconds() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousPrintMillis >= printInterval) {
        previousPrintMillis = currentMillis;

        Serial.println("⏳ [INFO] Printing data every 5 seconds...");
        Serial.println("=====================================");
        
        if (lastPrintedDistance != lastSentDistance || lastPrintedAngle != lastSentAngle) {
            Serial.print("📏 Distance: ");
            Serial.print(lastSentDistance);
            Serial.print(" cm | 🎯 Servo Angle: ");
            Serial.print(lastSentAngle);
            Serial.println("°");
            lastPrintedDistance = lastSentDistance;
            lastPrintedAngle = lastSentAngle;
        } else {
            Serial.println("🚨 No valid data received from CAN Bus yet.");
        }
        
        Serial.println("=====================================");
    }
}
