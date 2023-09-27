#include <Arduino.h>

#include "icm42605.h"

#define SDA_PIN 9
#define SCL_PIN 10
#define DELAY_TIME 10
//ICM42605
#define ICM42605_ADDR 0x68
ICM42605 icm; // 宣告ICM42605物件

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
  // 確認 WHO_AM_I
  uint8_t who_am_i = i2c_read(ICM42605_ADDR, 0x75);
  if (who_am_i != 0x42) {
    return false;
  }
   // 初始化ICM42605
  icm.init(AFS_2G, GFS_500DPS, 0x01, 0x01);

  // 計算零偏值
  float accelBias[3] = {0, 0, 0};
  float gyroBias[3] = {0, 0, 0};
  icm.offsetBias(accelBias, gyroBias);
  
  Serial.println("Accelerometer Offset:");
  Serial.print("X: "); Serial.println(accelBias[0]);
  Serial.print("Y: "); Serial.println(accelBias[1]);
  Serial.print("Z: "); Serial.println(accelBias[2]);

  Serial.println("Gyroscope Offset:");
  Serial.print("X: "); Serial.println(gyroBias[0]);
  Serial.print("Y: "); Serial.println(gyroBias[1]);
  Serial.print("Z: "); Serial.println(gyroBias[2]);

  return true;
}


void soft_i2c_scan(){
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
void setup() {
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);

  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);

  Serial.begin(115200);
  Serial.println("I2C Scanner");
  
  if (!initialize_ICM42605()) {
    Serial.println("ICM42605初始化失敗！");
    while (1);
  }
  // scan_ICM42605_registers();
}

void loop() {
  int16_t data[7];
  icm.readData(data);  // 讀取ICM42605的資料

  float temperature = data[0];
  float ax = data[1] * icm.getAres(AFS_2G);
  float ay = data[2] * icm.getAres(AFS_2G);
  float az = data[3] * icm.getAres(AFS_2G);
  float gx = data[4] * icm.getGres(GFS_500DPS);
  float gy = data[5] * icm.getGres(GFS_500DPS);
  float gz = data[6] * icm.getGres(GFS_500DPS);

  Serial.print("Temperature: "); Serial.println(temperature);
  Serial.print("Accel X: "); Serial.println(ax);
  Serial.print("Accel Y: "); Serial.println(ay);
  Serial.print("Accel Z: "); Serial.println(az);
  Serial.print("Gyro X: "); Serial.println(gx);
  Serial.print("Gyro Y: "); Serial.println(gy);
  Serial.print("Gyro Z: "); Serial.println(gz);
  Serial.println("----------------------------");

  delay(10);  // 延遲1秒
}
