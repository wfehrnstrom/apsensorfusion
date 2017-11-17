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

float accel_x_bias;
float accel_y_bias;
float accel_z_bias;
float gyro_x_bias;
float gyro_y_bias;
float gyro_z_bias;

unsigned long lastTime;
vector gyro_unit;
vector * gyro_unit_ptr;
vector acc_unit;
vector * acc_unit_ptr;
quaternion q;
quaternion * q_ptr;
float angular_accel;

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
  acc_unit_ptr = &acc_unit;
  gyro_unit_ptr = &gyro_unit;
  q_ptr = &q;
  uint8_t write_buf[1];
  write_buf[0] = PWR_MGMT_WAKE;
  writeReg(PWR_MGMT_1, write_buf, 1);
  write_buf[0] = GYRO_FULL_SCALE;
  writeReg(GYRO_CONFIG, write_buf, 1);
  write_buf[0] = CONFIG_FULL_BANDWITH;
  writeReg(CONFIG, write_buf, 1);

  //uint8_t read_buf[1];
  //readReg(PWR_MGMT_1, read_buf, 1);
  //Serial.println(read_buf[0], BIN);
  //readReg(GYRO_CONFIG, read_buf, 1);
  //Serial.println(read_buf[0], BIN);
  //readReg(CONFIG, read_buf, 1);
  //Serial.println(read_buf[0], BIN);
  lastTime = millis();
}

void loop() {
    unsigned long curTime = millis();
    unsigned long timePassed = curTime - lastTime;
    
    //calculateBias();
    readAccelAndGyro(true);
    float theta += angular_accel*timePassed;
    quaternion_create(gyro_unit_ptr, theta, result);
    vector gyroResult;
    vector * gyro_result_ptr = &gyroResult;
    quaternion_rotate(gyro_unit_ptr, result, gyro_result_ptr);
    printUnit();
//  Serial.println("Accel: " + String(accel_x) + ", " + String(accel_y) + ", " + String(accel_z));
//  Serial.println("Gyro: " + String(gyro_x) + ", " + String(gyro_y) + ", " + String(gyro_z));
  delay(1000);
}
void printUnit(){
  Serial.print(acc_unit.x);
  Serial.print(" ");
  Serial.print(acc_unit.y);
  Serial.print(" ");
  Serial.print(acc_unit.z);
  Serial.print(" ");
  Serial.print(gyro_unit.x);
  Serial.print(" ");
  Serial.print(gyro_unit.y);
  Serial.print(" ");
  Serial.print(gyro_unit.z);
}
bool readAccelAndGyro(bool correctForBias) {
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
    vector acc;
    vector * acc_ptr = &acc;
    acc.x = accel_x;
    acc.y = accel_y;
    acc.z = accel_z;
    vector_normalize(acc_ptr, acc_unit_ptr);
    readReg(GYRO_X_1, gyro_x_1_buf, 1);
    readReg(GYRO_X_2, gyro_x_2_buf, 1);
    readReg(GYRO_Y_1, gyro_y_1_buf, 1);
    readReg(GYRO_Y_2, gyro_y_2_buf, 1);
    readReg(GYRO_Z_1, gyro_z_1_buf, 1);
    readReg(GYRO_Z_2, gyro_z_2_buf, 1);
    gyro_x = (int)((gyro_x_1_buf[0] << 8) | gyro_x_2_buf[0]) / (float)(GYRO_LSB_SENSITIVITY);
    gyro_y = (int)((gyro_y_1_buf[0] << 8) | gyro_y_2_buf[0]) / (float)(GYRO_LSB_SENSITIVITY);
    gyro_z = (int)((gyro_z_1_buf[0] << 8) | gyro_z_2_buf[0]) / (float)(GYRO_LSB_SENSITIVITY);
    vector gyro;
    vector gyro_ptr = &gyro;
    gyro.x = gyro_x;
    gyro.y = gyro_y;
    gyro.z = gyro_z;
    angular_accel = vector_normalize(gyro_ptr, gyro_unit_ptr);
    if (correctForBias) {
      accel_x -= accel_x_bias;
      accel_y -= accel_y_bias;
      accel_z -= accel_z_bias;
      gyro_x -= gyro_x_bias;
      gyro_y -= gyro_y_bias;
      gyro_z -= gyro_z_bias;
    }
    return true;
  }
  return false;
}

void calculateBias() {
  float accel_x_avg = 0;
  float accel_y_avg = 0;
  float accel_z_avg = 0;
  float gyro_x_avg = 0;
  float gyro_y_avg = 0;
  float gyro_z_avg = 0;
  const int numSamples = 100;
  for (int i = 0; i < numSamples; i++) {
    while(!readAccelAndGyro(false)) {
      delay(10);
    }
    accel_x_avg += accel_x;
    accel_y_avg += accel_y;
    accel_z_avg += accel_z;
    gyro_x_avg += gyro_x;
    gyro_y_avg += gyro_y;
    gyro_z_avg += gyro_z;
    delay(200);
  }
  accel_x_avg /= numSamples;
  accel_y_avg /= numSamples;
  accel_z_avg /= numSamples;
  gyro_x_avg /= numSamples;
  gyro_y_avg /= numSamples;
  gyro_z_avg /= numSamples;
  Serial.print("accel_x_bias = " + String(accel_x_avg) + ";");
  Serial.print("accel_y_bias = " + String(accel_y_avg) + ";");
  Serial.print("accel_z_bias = " + String(accel_z_avg-1) + ";");
  Serial.print("gyro_x_bias = " + String(gyro_x_avg) + ";");
  Serial.print("gyro_y_bias = " + String(gyro_y_avg) + ";");
  Serial.print("gyro_z_bias = " + String(gyro_z_avg) + ";");
}

