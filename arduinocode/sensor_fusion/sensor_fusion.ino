#include <Wire.h>
#include <sensor_fusion.h>

#define PWR_MGMT_1 107
#define GYRO_CONFIG 27
#define CONFIG 26
#define INT_STATUS 58

#define PWR_MGMT_WAKE 0b00000000
#define GYRO_FULL_SCALE 0b00011000
#define CONFIG_FULL_BANDWITH 0b00000000

#define ACCEL_X_1 59
#define ACCEL_X_2 60
#define ACCEL_Y_1 61
#define ACCEL_Y_2 62
#define ACCEL_Z_1 63
#define ACCEL_Z_2 64

#define GYRO_X_1 67
#define GYRO_X_2 68
#define GYRO_Y_1 69
#define GYRO_Y_2 70
#define GYRO_Z_1 71
#define GYRO_Z_2 72

#define ACCEL_LSB_SENSITIVITY 16384
#define GYRO_LSB_SENSITIVITY 16.4

uint8_t accel_x_1_buf[1];
uint8_t accel_x_2_buf[1];
uint8_t accel_y_1_buf[1];
uint8_t accel_y_2_buf[1];
uint8_t accel_z_1_buf[1];
uint8_t accel_z_2_buf[1];
uint8_t gyro_x_1_buf[1];
uint8_t gyro_x_2_buf[1];
uint8_t gyro_y_1_buf[1];
uint8_t gyro_y_2_buf[1];
uint8_t gyro_z_1_buf[1];
uint8_t gyro_z_2_buf[1];

//TODO: Should we use double?
float accel_x; // g's
float accel_y;
float accel_z;
float gyro_x; // degrees/sec
float gyro_y;
float gyro_z;

void readAccelerometer(int *buf) {
  
}

void readGyroscope(int *buf){
  
}

bool readReady() {
  uint8_t int_status[1];
  readReg(INT_STATUS, int_status, 1);
  // if the lsb of INT_STATUS has been set, then 
  if(int_status[0] & (0b00000001) == 1){
    return true;
  }
  return false;
}

void setup(){
  Serial.begin(115200);
  Wire.begin();
  
  uint8_t write_buf[1];
  write_buf[0] = PWR_MGMT_WAKE;
  writeReg(PWR_MGMT_1, write_buf, 1);
  write_buf[0] = GYRO_FULL_SCALE;
  writeReg(GYRO_CONFIG, write_buf, 1);
  write_buf[0] = CONFIG_FULL_BANDWITH;
  writeReg(CONFIG, write_buf, 1);

  uint8_t read_buf[1];
  readReg(PWR_MGMT_1, read_buf, 1);
  Serial.println(read_buf[0], BIN);
  readReg(GYRO_CONFIG, read_buf, 1);
  Serial.println(read_buf[0], BIN);
  readReg(CONFIG, read_buf, 1);
  Serial.println(read_buf[0], BIN);
}

void loop() {
  readAccelAndGyro();
//  Serial.println("Accel: " + String(accel_x) + ", " + String(accel_y) + ", " + String(accel_z));
  Serial.println("Gyro: " + String(gyro_x) + ", " + String(gyro_y) + ", " + String(gyro_z));
  delay(200);
}

void readAccelAndGyro() {
  if (readReady()) {
    readReg(ACCEL_X_1, accel_x_1_buf, 1);
    readReg(ACCEL_X_2, accel_x_2_buf, 1);
    readReg(ACCEL_Y_1, accel_y_1_buf, 1);
    readReg(ACCEL_Y_2, accel_y_2_buf, 1);
    readReg(ACCEL_Z_1, accel_z_1_buf, 1);
    readReg(ACCEL_Z_2, accel_z_2_buf, 1);
    accel_x = (int)((accel_x_1_buf[0] << 8) | accel_x_2_buf[0]) / (float)(ACCEL_LSB_SENSITIVITY);
    accel_y = (int)((accel_y_1_buf[0] << 8) | accel_y_2_buf[0]) / (float)(ACCEL_LSB_SENSITIVITY);
    accel_z = (int)((accel_z_1_buf[0] << 8) | accel_z_2_buf[0]) / (float)(ACCEL_LSB_SENSITIVITY);

    readReg(GYRO_X_1, gyro_x_1_buf, 1);
    readReg(GYRO_X_2, gyro_x_2_buf, 1);
    readReg(GYRO_Y_1, gyro_y_1_buf, 1);
    readReg(GYRO_Y_2, gyro_y_2_buf, 1);
    readReg(GYRO_Z_1, gyro_z_1_buf, 1);
    readReg(GYRO_Z_2, gyro_z_2_buf, 1);
    gyro_x = (int)((gyro_x_1_buf[0] << 8) | gyro_x_2_buf[0]) / (float)(GYRO_LSB_SENSITIVITY);
    gyro_y = (int)((gyro_y_1_buf[0] << 8) | gyro_y_2_buf[0]) / (float)(GYRO_LSB_SENSITIVITY);
    gyro_z = (int)((gyro_z_1_buf[0] << 8) | gyro_z_2_buf[0]) / (float)(GYRO_LSB_SENSITIVITY);
  }
}