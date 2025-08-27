#define ENCODER_PIN 2

volatile long pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
const unsigned int debounceTime = 1; // ลด debounce time

void setup() {
  
  Serial.begin(9600);
  pinMode(ENCODER_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countPulse, CHANGE);
  Serial.println("🌀 หมุนล้อ 1 รอบ แล้วกด Enter ใน Serial Monitor");
}

void loop() {
  if (Serial.available()) {
    Serial.read();

    Serial.print("Pulse Count for 1 full rotation: ");
    Serial.println(pulseCount);

    pulseCount = 0;
    Serial.println("🌀 หมุนอีก 1 รอบ แล้วกด Enter อีกครั้ง");
  }
}


void countPulse() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTime > debounceTime) {
    pulseCount++;
    lastPulseTime = currentTime;
  }
}
