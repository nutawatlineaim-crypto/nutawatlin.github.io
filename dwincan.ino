#include <Wire.h>
#include <SPI.h>
#include <mcp2515.h>
#include <ezButton.h>
#include <SD.h>

#define SD_CS      4
#define CAN_CS     9
#define USE_SD_CARD true

#define STEERING_RATIO 16.0

MCP2515 mcp2515(9);
struct can_frame canMsg;

#define Rx_SIZE 12
byte dwin_rx_data[Rx_SIZE];
boolean rx_completed = false;

const unsigned int vp_rotaryRAD      = 0x5500;
const unsigned int vp_vehicleRAD     = 0x5600;
const unsigned int vp_animationFrame = 0x7000;
const unsigned int vp_wheelFrame     = 0x6000;
const unsigned int vp_brake          = 0x4000;
const unsigned int vp_throttle       = 0x4010;
const unsigned int vp_distance       = 0x4030;
const unsigned int vp_speed          = 0x4020;
const unsigned int vp_turn_right     = 0x5010;
const unsigned int vp_turn_left      = 0x5020;
const unsigned int vp_volt12         = 0x5040;
const unsigned int vp_volt48         = 0x5050;
const unsigned int vp_current48      = 0x4200;
const unsigned int vp_battery48      = 0x6300;
const unsigned int bat12_Frame      = 0x5030;





int16_t last_distance = 0;
int16_t last_speed = 0;

void deselectAllDevices() {
  digitalWrite(SD_CS, HIGH);
  digitalWrite(CAN_CS, HIGH);
}

void selectDevice(uint8_t cs_pin) {
  deselectAllDevices();
  delayMicroseconds(5);
  digitalWrite(cs_pin, LOW);
  delayMicroseconds(5);
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(115200);
  Serial3.begin(115200);
  SPI.begin();


  pinMode(CAN_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  deselectAllDevices();

  // Init CAN
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  selectDevice(CAN_CS);
  if (mcp2515.reset() != MCP2515::ERROR_OK ||
      mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK ||
      mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println("❌ MCP2515 Init Failed");
    while (1);
  }
  deselectAllDevices();
  SPI.endTransaction();
  Serial.println("✅ MCP2515 Initialized");

  // Init SD Card
  if (USE_SD_CARD) {
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    selectDevice(SD_CS);
    Serial.println("🔧 Initializing SD card...");
    if (!SD.begin(4)) {
      Serial.println("❌ SD Card Init Failed");
    } else {
      Serial.println("✅ SD Card Initialized");
      if (!SD.exists("log.csv")) {
        File dataFile = SD.open("log.csv", FILE_WRITE);
        if (dataFile) {
          dataFile.println("Timestamp,Steering Angle,Wheel Angle,Distance,Speed");
          dataFile.flush();
          dataFile.close();
          Serial.println("✅ log.csv created.");
        } else {
          Serial.println("❌ Failed to create log.csv");
        }
      }
    }
    deselectAllDevices();
    SPI.endTransaction();
  }

  Serial.println("System Initialized...");
}

void logDataToSD(int16_t steeringAngle, float wheelAngle, int16_t distance, int16_t speed) {
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  selectDevice(SD_CS);
  File dataFile = SD.open("log.csv", FILE_WRITE);
  if (dataFile) {
    if (dataFile.size() == 0) {
      dataFile.println("Timestamp,Steering Angle,Wheel Angle,Distance,Speed");
    }
    dataFile.print(__DATE__);
    dataFile.print(" ");
    dataFile.print(__TIME__);
    dataFile.print(",");
    dataFile.print(steeringAngle);
    dataFile.print(",");
    dataFile.print(wheelAngle, 2);
    dataFile.print(",");
    dataFile.print(distance);
    dataFile.print(",");
    dataFile.println(speed);
    dataFile.close();
  } else {
    Serial.println("❌ Failed to open log.csv");
  }
  deselectAllDevices();
  SPI.endTransaction();
}

void loop() {
  static unsigned long lastSendTime = 0;
  static unsigned long lastSDLogTime = 0;
  unsigned long now = millis();

  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  selectDevice(CAN_CS);
  bool gotData = (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK);
  deselectAllDevices();
  SPI.endTransaction();

  if (gotData) {
    if (canMsg.can_id == 0x02 && canMsg.can_dlc == 2) {
      int16_t steering_wheel_angle = (canMsg.data[0] << 8) | canMsg.data[1];
      int16_t normalized_angle = steering_wheel_angle % 1080;
      if (normalized_angle > 540) normalized_angle -= 1080;
      if (normalized_angle < -540) normalized_angle += 1080;

      int frame;
      if (normalized_angle >= -162) {
        frame = map(normalized_angle, -162, 540, 1, 40);
      } else {
        frame = map(normalized_angle, -540, -180, 61, 41);
      }
      frame = constrain(frame, 1, 63);

      int wheel_frame = map(normalized_angle / STEERING_RATIO, -33.75, 33.75, 1, 68);
      wheel_frame = constrain(wheel_frame, 1, 68);

      if (now - lastSendTime >= 200) {
        lastSendTime = now;

        TxDWIN(vp_rotaryRAD, steering_wheel_angle);
        delay(5);
        TxDWIN(vp_animationFrame, frame);
        delay(5);
        TxDWIN(vp_vehicleRAD, (int16_t)(steering_wheel_angle / STEERING_RATIO * 100));
        delay(5);
        TxDWIN(vp_wheelFrame, wheel_frame);
        delay(5);

        int16_t val0 = map(normalized_angle + 540, 0, 1080, 0, 270);
        Graph_DWIN(0, val0);
        
        Serial.print("Steering Wheel Angle: ");
        Serial.println(steering_wheel_angle);
        
      }

      if (USE_SD_CARD && now - lastSDLogTime >= 1000) {
        lastSDLogTime = now;
        logDataToSD(steering_wheel_angle, steering_wheel_angle / STEERING_RATIO, last_distance, last_speed);
      }
    }

    if (canMsg.can_id == 0x03 && canMsg.can_dlc == 2) {
      int16_t brake_value = (canMsg.data[0] << 8) | canMsg.data[1];
      TxDWIN(vp_brake, constrain(brake_value, 0, 1));
    }

    if (canMsg.can_id == 0x04 && canMsg.can_dlc == 2) {
      int16_t throttle_value = (canMsg.data[0] << 8) | canMsg.data[1];
      TxDWIN(vp_throttle, constrain(throttle_value, 0, 1));
    }

    if (canMsg.can_id == 0x06 && canMsg.can_dlc == 2) {
      int16_t distance_value = (canMsg.data[0] << 8) | canMsg.data[1];
      last_distance = distance_value;
      TxDWIN(vp_distance, distance_value);
      int16_t val2 = map(distance_value , 0, 6, 0, 4);
      Graph_DWIN2(2, val2);
      Serial.println(distance_value);
    }

    if (canMsg.can_id == 0x05 && canMsg.can_dlc == 2) {
      int16_t speed_value = (canMsg.data[0] << 8) | canMsg.data[1];
      last_speed = speed_value;
      TxDWIN(vp_speed, speed_value);
      int16_t val1 = map(speed_value , 0, 20, 0, 400);
      Graph_DWIN1(1, val1);
      
    }
   

    if (canMsg.can_id == 0x07 && canMsg.can_dlc == 2) {
      int16_t turn_right_value = (canMsg.data[0] << 8) | canMsg.data[1];
      TxDWIN(vp_turn_right, constrain(turn_right_value, 0, 1));
}

    if (canMsg.can_id == 0x08 && canMsg.can_dlc == 2) {
      int16_t turn_left_value = (canMsg.data[0] << 8) | canMsg.data[1];
      TxDWIN(vp_turn_left, constrain(turn_left_value, 0, 1));
      
}
  if (canMsg.can_id == 0x09 && canMsg.can_dlc == 2) {
    // อ่านค่าเปอร์เซ็นต์แบตเตอรี่ที่ส่งมา
    int16_t battPercent = (canMsg.data[0] << 8 | canMsg.data[1]);  // ค่าเปอร์เซ็นต์แบตเตอรี่ (0-100%)
    
    int iconId = 0; // ตัวแปรสำหรับกำหนดค่า Icon ID

    // การเลือกไอคอนตามค่าเปอร์เซ็นต์แบตเตอรี่
    if (battPercent > 75) {
        iconId = 1;  // Icon 14 สำหรับ 100% (เขียว)
    } else if (battPercent > 50) {
        iconId = 2;  // Icon 16 สำหรับ 75% (เหลือง)
    } else if (battPercent > 25) {
        iconId = 3;  // Icon 17 สำหรับ 50% (ส้ม)
    } else {
        iconId = 4;  // Icon 18 สำหรับ 25% หรือแย่กว่า (แดง)
    }

    // ส่ง Icon ID ไปยัง DWIN Display
    TxDWIN(bat12_Frame, iconId);  // ส่ง Icon ID ไปที่ DWIN Display

   
}


  if (canMsg.can_id == 0x11 && canMsg.can_dlc == 2) {
      int16_t volt48_value = (canMsg.data[0] << 8) | canMsg.data[1];
      TxDWIN(vp_volt48, constrain(volt48_value, 0, 1));
      int16_t val3 = map(volt48_value , 0, 50, 0,350);
      Graph_DWIN3(3 , val3);
      
      
}
   if (canMsg.can_id == 0x12 && canMsg.can_dlc == 2) {
      int16_t cur48_value = (canMsg.data[0] << 8) | canMsg.data[1];
      TxDWIN(vp_current48, cur48_value);
      int16_t val4 = map(cur48_value , 0, 30, 0,5);
      Graph_DWIN4(6,val4);
      
}
  
  
  if (canMsg.can_id == 0x13 && canMsg.can_dlc == 2) {
      int16_t battery48_value = (canMsg.data[0] << 8) | canMsg.data[1];
      TxDWIN(vp_battery48, battery48_value);
      
}


  }
  
  RxDWIN();
  delay(10);
}

void TxDWIN(unsigned int vp, int16_t value) {
  uint8_t dwin_tx_data[8] = {
    0x5A, 0xA5, 0x05, 0x82,
    highByte(vp), lowByte(vp),
    highByte(value), lowByte(value)
  };
  Serial2.write(dwin_tx_data, 8);
  Serial2.flush();
}






void Graph_DWIN(unsigned int channel, int16_t value) {  //ch0
  uint8_t graph_data[16] = {
    0x5A, 0xA5, 0x0D, 0x82,
    0x03, 0x10,
    0x5A, 0xA5,
    0x01, (uint8_t)channel,
    0x00, 0x02,
    highByte((uint16_t)value), lowByte((uint16_t)value),
    0x00, 0x00
  };
  
  Serial3.write(graph_data, 16);
  Serial3.flush();
  
  
}

void Graph_DWIN1(unsigned int channel, int16_t value) {  //ch1
  uint8_t graph_data[16] = {
    0x5A, 0xA5, 0x0D, 0x82,
    0x03, 0x10,
    0x5A, 0xA5,
    0x01, (uint8_t)channel,
    0x01, 0x02,
    highByte((uint16_t)value), lowByte((uint16_t)value),
    0x00, 0x00
  };
  
  Serial3.write(graph_data, 16);
  Serial3.flush();
  
}

void Graph_DWIN2(unsigned int channel, int16_t value) {  //ch2
  uint8_t graph_data[16] = {
    0x5A, 0xA5, 0x0D, 0x82,
    0x03, 0x10,
    0x5A, 0xA5,
    0x01, (uint8_t)channel,
    0x02, 0x02,
    highByte((uint16_t)value), lowByte((uint16_t)value),
    0x00, 0x00
  };
  
  Serial3.write(graph_data, 16);
  Serial3.flush();
  
}


void Graph_DWIN3(unsigned int channel, int16_t value) {  //ch3
  uint8_t graph_data[16] = {
    0x5A, 0xA5, 0x0D, 0x82,
    0x03, 0x10,
    0x5A, 0xA5,
    0x01, (uint8_t)channel,
    0x03, 0x02,
    highByte((uint16_t)value), lowByte((uint16_t)value),
    0x00, 0x00
  };
  
  Serial3.write(graph_data, 16);
  Serial3.flush();
  
}

void Graph_DWIN4(unsigned int channel, int16_t value) {  //ch4
  uint8_t graph_data[16] = {
    0x5A, 0xA5, 0x0D, 0x82,
    0x03, 0x10,
    0x5A, 0xA5,
    0x01, (uint8_t)channel,
    0x06, 0x02,
    highByte((uint16_t)value), lowByte((uint16_t)value),
    0x00, 0x00
  };
  
  Serial3.write(graph_data, 16);
  Serial3.flush();
  
  
}


void RxDWIN() {
  static uint8_t index = 0;
  while (Serial2.available()) {
    dwin_rx_data[index] = Serial2.read();
    if (index == 0 && dwin_rx_data[0] != 0x5A) {
      index = 0;
      continue;
    }
    index++;
    if (index >= Rx_SIZE) {
      index = 0;
      rx_completed = true;
      Serial.println("Received full packet from DWIN!");

      uint16_t vp = (dwin_rx_data[4] << 8) | dwin_rx_data[5];
      uint16_t value = (dwin_rx_data[6] << 8) | dwin_rx_data[7];

      // Serial.print("VP: 0x"); Serial.print(vp, HEX);
      // Serial.print(" | Value: "); Serial.println(value);
    }
  }
}