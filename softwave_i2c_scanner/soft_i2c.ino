
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

bool i2c_write_bit(bool bit) {
  digitalWrite(SDA_PIN, bit);
  delayMicroseconds(DELAY_TIME);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(DELAY_TIME);
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
    Serial.println(i);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(DELAY_TIME);
    byte |= digitalRead(SDA_PIN) << i;
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(DELAY_TIME);
  }
  pinMode(SDA_PIN, OUTPUT);
  i2c_write_bit(!ack); 
  Serial.println(byte,BIN);
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
