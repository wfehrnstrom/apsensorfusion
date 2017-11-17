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

int accel_x;
int accel_y;
int accel_z;
int gyro_x;
int gyro_y;
int gyro_z;

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
}

//TODO Values printed seem too big (~2^14 when staying still. That only leaves room for a max of 2g's, 2^15-1)
void readAccelAndGyro() {
  if (readReady()) {
    readReg(ACCEL_X_1, accel_x_1_buf, 1);
    readReg(ACCEL_X_2, accel_x_2_buf, 1);
    readReg(ACCEL_Y_1, accel_y_1_buf, 1);
    readReg(ACCEL_Y_2, accel_y_2_buf, 1);
    readReg(ACCEL_Z_1, accel_z_1_buf, 1);
    readReg(ACCEL_Z_2, accel_z_2_buf, 1);
    accel_x = (accel_x_1_buf[0] << 8) | accel_x_2_buf[0];
    accel_y = (accel_y_1_buf[0] << 8) | accel_y_2_buf[0];
    accel_z = (accel_z_1_buf[0] << 8) | accel_z_2_buf[0];

    readReg(GYRO_X_1, gyro_x_1_buf, 1);
    readReg(GYRO_X_2, gyro_x_2_buf, 1);
    readReg(GYRO_Y_1, gyro_y_1_buf, 1);
    readReg(GYRO_Y_2, gyro_y_2_buf, 1);
    readReg(GYRO_Z_1, gyro_z_1_buf, 1);
    readReg(GYRO_Z_2, gyro_z_2_buf, 1);
    gyro_x = (gyro_x_1_buf[0] << 8) | gyro_x_2_buf[0];
    gyro_y = (gyro_y_1_buf[0] << 8) | gyro_y_2_buf[0];
    gyro_z = (gyro_z_1_buf[0] << 8) | gyro_z_2_buf[0];
  }
}