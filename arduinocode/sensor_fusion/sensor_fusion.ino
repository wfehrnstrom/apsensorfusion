#include <Wire.h>
#include "sensor_fusion.h";

#define PWR_MGMT_1 107
#define GYRO_CONFIG 27
#define CONFIG 26
#define INT_STATUS 58

#define GYRO_FULL_SCALE 0b00011000
#define CONFIG_FULL_BANDWITH 0b00000111

void readAccelerometer(int *buf){
  
}

void setup(){
  Serial.begin(115200);
  Wire.begin();
  uint8_t write_buf[1] = {0};
  writeReg(PWR_MGMT_1, buf, 1);
  write_buf[0] = GYRO_FULL_SCALE;
  writeReg(GYRO_CONFIG, buf, 1);
  write_buf[0] = CONFIG_FULL_BANDWITH;
  writeReg(CONFIG, CONFIG_FULL_BANDWITH, 1);
  uint8_t read_buf[1];
  readReg(PWR_MGMT_1, buf, 1);
  Serial.println(buf[0], BIN);
  readReg(GYRO_CONFIG, buf, 1);
  Serial.println(buf[0], BIN);
  readReg(CONFIG, buf, 1);
  Serial.println(buf[0], BIN);
}

void loop(){
  
}

