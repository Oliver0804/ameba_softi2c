#include <Arduino.h>

#define SDA_PIN 9
#define SCL_PIN 10


void i2c_start() {
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(10);
  digitalWrite(SCL_PIN, LOW);
}

void i2c_stop() {
  digitalWrite(SDA_PIN, LOW);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SDA_PIN, HIGH);
}

bool i2c_write_bit(bool bit) {
  digitalWrite(SDA_PIN, bit);
  delayMicroseconds(10);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(10);
  bool ack = !digitalRead(SDA_PIN);
  digitalWrite(SCL_PIN, LOW);
  return ack;
}

bool i2c_write_byte(uint8_t byte) {
  for (int i = 7; i >= 0; i--) {
    i2c_write_bit((byte >> i) & 1);
  }
  
  return i2c_write_bit(1); // 讀取ACK
}

uint8_t i2c_read_byte(bool ack = true) {
  uint8_t byte = 0;
  pinMode(SDA_PIN, INPUT);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(10);
    byte |= digitalRead(SDA_PIN) << i;
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(10);
  }
  pinMode(SDA_PIN, OUTPUT);
  i2c_write_bit(!ack); 
  return byte;
}

void i2c_write(uint8_t address, uint8_t reg, uint8_t data) {
  i2c_start();
  i2c_write_byte(address << 1);
  i2c_write_byte(reg);
  i2c_write_byte(data);
  i2c_stop();
}

uint8_t i2c_read(uint8_t address, uint8_t reg) {
  i2c_start();
  i2c_write_byte(address << 1);
  i2c_write_byte(reg);
  i2c_start(); 
  i2c_write_byte((address << 1) | 1); 
  uint8_t data = i2c_read_byte(false); 
  i2c_stop();
  return data;
}

void setup() {
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);

  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);

  Serial.begin(115200);
  Serial.println("I2C Scanner");
}

void loop() {
  byte error, address;
  int nDevices;

  Serial.println("掃描中...");

  nDevices = 0;
  for (address = 1; address < 127; address++ ) {
    i2c_start();
    error = !i2c_write_byte(address << 1); // I2C位址需要左移一位才是正確的位址

    i2c_stop();

    if (error == 0) {
      Serial.print("找到I2C設備位址: ");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    }
  }

  if (nDevices == 0) {
    Serial.println("未找到I2C設備");
  } else {
    Serial.println("掃描完成");
  }

  delay(5000);
}
