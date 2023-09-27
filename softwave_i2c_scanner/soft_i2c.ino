
#define ICM42605_ADDR 0x68
#define ACCEL_XOUT_H   0x1F
#define ACCEL_XOUT_L   0x20
#define ACCEL_YOUT_H   0x21
#define ACCEL_YOUT_L   0x22
#define ACCEL_ZOUT_H   0x23
#define ACCEL_ZOUT_L   0x24

void i2c_start() {
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(DELAY_TIME);
  digitalWrite(SCL_PIN, LOW);
}

void i2c_stop() {
  digitalWrite(SDA_PIN, LOW);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(DELAY_TIME);
  digitalWrite(SDA_PIN, HIGH);
}

void i2c_write_bit(bool bit) {
  digitalWrite(SDA_PIN, bit);
  delayMicroseconds(DELAY_TIME);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(DELAY_TIME);
  digitalWrite(SCL_PIN, LOW);
}

bool i2c_write_byte(uint8_t byte) {
  for (int i = 7; i >= 0; i--) {
    i2c_write_bit((byte >> i) & 1);
  }
  
  // 讀取ACK
  pinMode(SDA_PIN, INPUT);  // 設置SDA為輸入模式來讀取acknowledge位
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(DELAY_TIME);
  bool ack = !digitalRead(SDA_PIN);  // 如果SDA是低，那麼ack為true
  digitalWrite(SCL_PIN, LOW);
  pinMode(SDA_PIN, OUTPUT);  // 設回輸出模式以進行後續傳輸
  delayMicroseconds(DELAY_TIME);
  
  return ack;
}


uint8_t i2c_read_byte(bool ack = true) {
  uint8_t byte = 0;

  pinMode(SDA_PIN, INPUT);
  digitalWrite(SCL_PIN, LOW);  // 確保開始時SCL是LOW

  for (int i = 7; i >= 0; i--) {
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(DELAY_TIME);
    byte |= digitalRead(SDA_PIN) << i;
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(DELAY_TIME);
  }
  
  pinMode(SDA_PIN, OUTPUT);
  i2c_write_bit(!ack);  // 發送ACK或NACK

  //Serial.println(byte);
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
