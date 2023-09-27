#include <Arduino.h>
#define ICM42605_ADDR 0x68
#define DELAY_TIME 100
#define SDA_PIN 9
#define SCL_PIN 10

void scan_ICM42605_registers() {
  Serial.println("開始掃描ICM42605的寄存器：");
  
  for (uint8_t regAddr = 0x00; regAddr <= 0x7F; regAddr++) {
    uint8_t regValue = i2c_read(ICM42605_ADDR, regAddr);

    Serial.print("寄存器0x");
    if (regAddr < 16) Serial.print("0");  // 確保格式正確
    Serial.print(regAddr, HEX);
    Serial.print(": 0x");
    if (regValue < 16) Serial.print("0");  // 確保格式正確
    Serial.println(regValue, HEX);
    delay(10);  // 稍微延遲以確保串口不會被淹沒
  }

  Serial.println("掃描完畢！");
}

bool initialize_ICM42605() {
  // 確認WHO_AM_I
  uint8_t who_am_i = i2c_read(ICM42605_ADDR, 0x75);
  if (who_am_i != 0x42) {
    return false;
  }

  // 重設裝置
  i2c_write(ICM42605_ADDR, 0x80, 0x01); // PWR_MGMT0 寄存器: 重設
  delay(50); // 等待重設完成

  // 設定電源和時鐘
  i2c_write(ICM42605_ADDR, 0x80, 0x00); // PWR_MGMT0 寄存器: 設定模式為正常

  // 設定陀螺儀和加速度計的範圍、數據速率等，這裡只是一個基本示例
  // 實際應用中可能需要其他設定
  i2c_write(ICM42605_ADDR, 0x36, 0x03); // GYRO_CONFIG0: 設定為2000 dps
  i2c_write(ICM42605_ADDR, 0x37, 0x03); // ACCEL_CONFIG0: 設定為16g

  return true;
}

void setup() {
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);

  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);

  Serial.begin(115200);
  Serial.println("I2C Scanner");
  
  // 確保ICM42605初始化正確
  //if (!initialize_ICM42605()) {
  //  Serial.println("ICM42605初始化失敗！");
  //  while (1);
  //}
   //scan_ICM42605_registers();
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
