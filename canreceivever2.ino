#include <mcp2515.h>
#include <SPI.h>
#include <SoftwareSerial.h>

// DWIN Protocol
#define Rx_SIZE             12
#define Tx_SIZE              8
#define DWIN_HD_BC_SIZE      3

#define DWIN_FM_BC           2
#define DWIN_FM_VPH          4
#define DWIN_FM_VPL          5

#define DWIN_FM_TPH          7
#define DWIN_FM_TPL          8

#define DWIN_FM_DTH          6
#define DWIN_FM_DTL          7

uint8_t dwin_tx_data[8] = { 0x5A, 0xA5, 0x05, 0x82, 0x51, 0x00, 0x00, 0x00 }; // Frame for sending data
byte dwin_rx_data[Rx_SIZE]; // Receive buffer
uint8_t index = 0; // byte's index of DWIN frame 
uint8_t byte_count = Rx_SIZE; // Maximun receive size of DWIN frame
boolean rx_completed = false; // Receive completed flag
boolean sw_flag = LOW;
const unsigned int vp_LidarB_led = 0x5000; //กำหนด Address led lidar behind
const unsigned int vp_LidarF_led = 0x5010; //กำหนด Address led lidar front
const unsigned int vp_DistB = 0x5020; //ระยะจากตัวหลัง
const unsigned int vp_angleB = 0x5030; //มุมจากตัวหลัง
const unsigned int vp_angleF = 0x5040; //มุมะจากตัวหน้า
const unsigned int vp_DistF = 0x5200; //ระยะจากตัวหน้า

// VP Address สำหรับการตั้งค่าปุ่มเพิ่ม/ลดของ LiDAR 1 และ LiDAR 2
const unsigned int vp_thresholdLidar1_increase = 0x6004;
const unsigned int vp_thresholdLidar1_decrease = 0x6005;
const unsigned int vp_thresholdLidar2_increase = 0x6006;
const unsigned int vp_thresholdLidar2_decrease = 0x6007;

// VP Address สำหรับแสดงค่าระยะการแจ้งเตือนปัจจุบัน
const unsigned int vp_thresholdLidar1_display = 0x6000;  // VP สำหรับแสดงระยะการแจ้งเตือนของ LiDAR 1
const unsigned int vp_thresholdLidar2_display = 0x6002;  // VP สำหรับแสดงมุมการแจ้งเตือนของ LiDAR 2

#define PAGE_SWITCH_VP 0x0084  // VP Address สำหรับการเปลี่ยนหน้าในจอ DWIN
#define PAGE_0 0x0000          // หน้าเริ่มต้น
#define PAGE_1 0x0001          // หน้าที่แสดงเมื่อ avgDist1 < thresholdDistanceLidar1
#define PAGE_2 0x0002          // หน้าที่แสดงเมื่อ avgDist2 < thresholdDistanceLidar2

MCP2515 mcp2515(10); // ใช้ CS ที่ขา 10 สำหรับ CAN Bus
SoftwareSerial dwinSerial(2, 3); // ใช้ขา 3 (TX) และ 2 (RX) สำหรับ DWIN Display

// ตัวแปรเก็บค่าล่าสุดที่ได้รับจาก CAN Bus
int16_t latestDistanceB = -1;  // ค่าเริ่มต้นเป็น -1 เพื่อบ่งบอกว่าไม่มีค่ามาก่อน
int16_t latestAngleB = -1;     // ค่าเริ่มต้นเป็น -1 เช่นกัน
int16_t latestDistanceF = -1;  // ค่าเริ่มต้นเป็น -1 เพื่อบ่งบอกว่าไม่มีค่ามาก่อน
int16_t latestAngleF = -1;     // ค่าเริ่มต้นเป็น -1 เช่นกัน

int thresholdDistanceLidar1 = 10;
int thresholdDistanceLidar2 = 10;

unsigned long lastRequestTime = 0;
const unsigned long requestInterval = 300;

// สำหรับเช็ค timeout ของ DistanceB (0x200)
unsigned long lastReceivedDistanceBTime = 0;
bool isLidarB_LedOn = false;

// สำหรับเช็ค timeout ของ DistanceF (0x204)
unsigned long lastReceivedDistanceFTime = 0;
bool isLidarF_LedOn = false;

const unsigned long distanceTimeout = 500; // 3 วินาที

void setup() {
    Serial.begin(115200);
    dwinSerial.begin(115200);
    delay(1000);

    SPI.begin();
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);  // ตั้งค่าความเร็วเป็น 500kbps
    mcp2515.setNormalMode();
    Serial.println("✅ Master Receiver Ready");  // แสดงค่าที่ตรงกับการตั้งค่า

    // ตรวจสอบสถานะของ CAN Bus
    uint8_t canStatus = mcp2515.getStatus();
    Serial.print("🟢 CAN Bus Status: 0x");
    Serial.println(canStatus, HEX);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastRequestTime >= requestInterval) {
    lastRequestTime = currentMillis;

    // Request from Sender1
    sendRequestToSlave(0x310); delay(10);

    // Request from Sender2
    sendRequestToSlave(0x311);
  }
  receiveCanData();
  receiveThresholdFromDWIN();
  
  checkDistanceTimeout();  // ✅ ตรวจ timeout เพื่อปิดไฟเมื่อไม่มีข้อมูล
}

void sendRequestToSlave(uint16_t requestID) {
  struct can_frame reqMsg;
  reqMsg.can_id = requestID;
  reqMsg.can_dlc = 1;
  reqMsg.data[0] = 0x01;
  mcp2515.sendMessage(&reqMsg);
  Serial.print("📤 Request sent to slave ID: 0x");
  Serial.println(requestID, HEX);
}

// ฟังก์ชันรับข้อมูลจาก CAN Bus
void receiveCanData() {
    struct can_frame canMsg;

    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print("📥 ID: 0x"); Serial.print(canMsg.can_id, HEX);
    Serial.print(" | Data: "); Serial.println((canMsg.data[0] << 8) | canMsg.data[1]);

        // ตรวจสอบ CAN ID และแสดงข้อมูล
        switch (canMsg.can_id) {
            case 0x200:  // ค่า DistanceB
{
    int16_t newDistanceB = (canMsg.data[0] << 8) | canMsg.data[1];
    if (newDistanceB != latestDistanceB) {
        latestDistanceB = newDistanceB;
        Serial.print("📏 DistanceB: ");
        Serial.println(latestDistanceB);
        TxDWIN(vp_DistB, latestDistanceB);
        lastReceivedDistanceBTime = millis();  // ✅ บันทึกเวลา

        // ✅ ตรวจระยะเฉพาะเมื่อมีการเปลี่ยน
        if (latestDistanceB < thresholdDistanceLidar1) {
            if (!isLidarB_LedOn) {
                TxDWIN(vp_LidarB_led, 1);
                isLidarB_LedOn = true;
            }
            switchPage(PAGE_1);
        }
    }
}
break;


            case 0x201:  // ค่า Angle
                latestAngleB = (canMsg.data[0] << 8) | canMsg.data[1];
                Serial.print("🎯 AngleB: ");
                Serial.println(latestAngleB);

                // ส่งข้อมูลไปยัง DWIN หากค่า angle มีการเปลี่ยนแปล'
                TxDWIN(vp_angleB, latestAngleB); 
                
                 
                break;

            case 0x204:  // ค่า DistanceF
               latestDistanceF = (canMsg.data[0] << 8) | canMsg.data[1];
               Serial.print("📏 DistanceF: ");
               Serial.println(latestDistanceF);

              TxDWIN(vp_DistF, latestDistanceF);
              lastReceivedDistanceFTime = millis();  // ✅ บันทึกเวลา

              if (latestDistanceF < thresholdDistanceLidar1) {
                 if (!isLidarF_LedOn) {
                 TxDWIN(vp_LidarF_led, 1);  // ✅ เปิดไฟ
                 isLidarF_LedOn = true;
              }
              switchPage(PAGE_2);
               }
           break;


            case 0x205:  // ค่า Angle
                latestAngleF = (canMsg.data[0] << 8) | canMsg.data[1];
                Serial.print("🎯 AngleF: ");
                Serial.println(latestAngleF);
                // ส่งข้อมูลไปยัง DWIN หากค่า angle มีการเปลี่ยนแปลง
                TxDWIN(vp_angleF, latestAngleF); 
                break;      

            default:
                Serial.println("❓ [WARNING] Unknown CAN ID Received");
                break;
        }

    } 
} 


void receiveThresholdFromDWIN() {
  uint8_t buffer[2];
  while (dwinSerial.available() >= 2) {
        dwinSerial.readBytes(buffer, 2);
        int vp = (buffer[0] << 8) | buffer[1];

        bool thresholdUpdated = false;

        // ตรวจสอบปุ่มเพิ่ม/ลดของ LiDAR 1
        if (vp == vp_thresholdLidar1_increase) {
            thresholdDistanceLidar1 += 10;
            TxDWIN(vp_thresholdLidar1_display, thresholdDistanceLidar1);
            thresholdUpdated = true;
        } else if (vp == vp_thresholdLidar1_decrease) {
            thresholdDistanceLidar1 -= 10;
            TxDWIN(vp_thresholdLidar1_display, thresholdDistanceLidar1);
            thresholdUpdated = true;
        }

        // ตรวจสอบปุ่มเพิ่ม/ลดของ LiDAR 2
        if (vp == vp_thresholdLidar2_increase) {
            thresholdDistanceLidar2 += 10;
            TxDWIN(vp_thresholdLidar2_display, thresholdDistanceLidar2);
            thresholdUpdated = true;
        } else if (vp == vp_thresholdLidar2_decrease) {
            thresholdDistanceLidar2 -= 10;
            TxDWIN(vp_thresholdLidar2_display, thresholdDistanceLidar2);
            thresholdUpdated = true;
        }

        // ส่ง threshold กลับไปยัง Sender (เฉพาะเมื่อมีการเปลี่ยนแปลงจริง)
        if (thresholdUpdated) {
            sendThresholdBackToSender(thresholdDistanceLidar1, 0x203);  // LiDAR1
            sendThresholdBackToSender(thresholdDistanceLidar2, 0x208);  // LiDAR2
        }
  }
}


// ฟังก์ชันส่งค่า threshold กลับไปยังตัวส่งผ่าน CAN Bus
void sendThresholdBackToSender(int thresholdValue, uint16_t canID) {
    struct can_frame canMsg;
    canMsg.can_id = canID;
    canMsg.can_dlc = 2;
    canMsg.data[0] = highByte(thresholdValue);
    canMsg.data[1] = lowByte(thresholdValue);

    if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {
        Serial.print("✅ Sent threshold ");
        Serial.print(thresholdValue);
        Serial.print(" to sender ID: 0x");
        Serial.println(canID, HEX);
    } else {
        Serial.println("❌ Error sending threshold to sender.");
    }
}

void switchPage(uint16_t pageNumber) {
    uint8_t txData[10] = {
        0x5A, 0xA5,             // ส่วนหัวของคำสั่ง
        0x07,                   // ความยาวข้อมูล
        0x82,                   // คำสั่งเขียน
        highByte(PAGE_SWITCH_VP), lowByte(PAGE_SWITCH_VP), // VP Address 0x0084
        0x5A, 0x01,             // คำสั่งเปลี่ยนหน้า 0x5A01
        highByte(pageNumber), lowByte(pageNumber)          // หมายเลขหน้าที่ต้องการแสดง
    };
    dwinSerial.write(txData, sizeof(txData));  // ส่งคำสั่งไปยังจอ DWIN
    dwinSerial.flush();
}

// 📡 **ฟังก์ชันส่งข้อมูลไปยัง DWIN**
void TxDWIN(uint16_t vp, uint16_t value) {
    uint8_t txData[8] = { 
        0x5A, 0xA5, // Header
        0x05,       // Frame Length
        0x82,       // Write Command
        highByte(vp), lowByte(vp),
        highByte(value), lowByte(value)
    };
    Serial.print("📡 Sending to DWIN | VP: 0x");
    Serial.print(vp, HEX);
    Serial.print(" | Value: ");
    Serial.println(value);

    dwinSerial.write(txData, sizeof(txData));
    dwinSerial.flush();
}
void checkDistanceTimeout() {
    unsigned long now = millis();

    // 🔸 เช็คฝั่ง B
    if ((now - lastReceivedDistanceBTime > distanceTimeout) && isLidarB_LedOn) {
        TxDWIN(vp_LidarB_led, 0);  // ปิดไฟ
        isLidarB_LedOn = false;
        Serial.println("⚠️ DistanceB timeout: turning off LED");
    }

    // 🔸 เช็คฝั่ง F
    if ((now - lastReceivedDistanceFTime > distanceTimeout) && isLidarF_LedOn) {
        TxDWIN(vp_LidarF_led, 0);  // ปิดไฟ
        isLidarF_LedOn = false;
        Serial.println("⚠️ DistanceF timeout: turning off LED");
    }
}
