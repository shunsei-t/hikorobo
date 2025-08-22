#include <Servo.h>
#include <Wire.h>
#include <SSCI_BME280.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <MadgwickAHRS.h>
#include "util.h"
#include "hz_sleep.h"

Servo sv_ele, sv_ail_l, sv_ail_r;
SSCI_BME280 bme280;

LSM6DS3 IMU(I2C_MODE, 0x6A);  //I2C device address 0x6A
HzSleep r(MEASURING_FREQ);//ループ周期を調整するクラス
Madgwick m_;

pos3d_t gyr_ = { 0 };
pos3d_t acc_ = { 0 };
pos3d_t ang_ = { 0 };
pos3d_t ang_offset_ = { 0 };
double mes_time_ = 1.0 / (double) MEASURING_FREQ * 1000.0;
float integral_roll = 0.0;
float integral_pitch = 0.0;
void read_gyr();
void read_acc();

#define LOOP1_TIME_MS 100
unsigned long loop1_last_time;
void loop1();

volatile unsigned long edge_h_;
volatile unsigned long edge_l_;
volatile unsigned long du;
volatile bool state_auto;
volatile bool last_state_;
void propo_cb();

void setup() {
  Serial.begin(9600);

  Wire.begin();
  bme280.setMode(I2C_ADDR, OSRS_T, OSRS_P, OSRS_H, BME280MODE, T_SB, FILTER, SPI3W_EN);
  bme280.readTrim();

  pinMode(PIN_ELE, OUTPUT);
  pinMode(PIN_AIL_L, OUTPUT);
  pinMode(PIN_AIL_R, OUTPUT);

  sv_ele.attach(PIN_ELE);
  sv_ail_l.attach(PIN_AIL_L);
  sv_ail_r.attach(PIN_AIL_R);

  loop1_last_time = millis();

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  pinMode(PIN_PIT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_PIT), propo_cb, CHANGE);

  IMU.settings.gyroRange = 2000;
  IMU.settings.accelRange = 4;  
  while (IMU.begin() != 0) {
    Serial.print("Begin IMU...");
  }

  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G,  0x8C);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x8A);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL7_G,  0x00);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL8_XL, 0x09);
  // 測定周波数  
  m_.begin((int)MEASURING_FREQ);
}

void loop() {

  if (state_auto) {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, LOW);

    digitalWrite(PIN_SW, HIGH);
  } else {
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);

    digitalWrite(PIN_SW, LOW);
  }

  double temp_act, press_act, hum_act;
  bme280.readData(&temp_act, &press_act, &hum_act);

  read_acc();
  read_gyr();
  m_.updateIMU(gyr_.x, gyr_.y, gyr_.z, acc_.x, acc_.y, acc_.z);

  ang_.x = m_.getRoll();
  ang_.y = m_.getPitch();

  // pid
  float proportional_roll = TARGET_ROLL - ang_.x;
  float proportional_pitch = TARGET_PITCH - ang_.y;
  integral_roll = constrain(integral_roll, -LIMIT_INTEGRAL_ROLL, LIMIT_INTEGRAL_ROLL);
  integral_pitch = constrain(integral_pitch, -LIMIT_INTEGRAL_PITCH, LIMIT_INTEGRAL_PITCH);

  float pid_roll  = ang_.x*KP_ROLL  + gyr_.x*KD_ROLL  + integral_roll;
  float pid_pitch = ang_.y*KP_PITCH + gyr_.y*KD_PITCH + integral_pitch;
  
  integral_roll  -= proportional_roll*KI_ROLL;
  integral_pitch -= proportional_pitch*KI_PITCH;

  // write servo angle
  static int i = 0;
  i++;
  if(i>=SERVO_SKIP){
    sv_ail_l.write(constrain(pid_pitch + 90 + OFFSET_SERVO_AIL_L, 0, 180));
    sv_ail_r.write(constrain(-pid_pitch + 90 + OFFSET_SERVO_AIL_R, 0, 180));
    sv_ele.write(constrain(pid_roll + 90 + OFFSET_SERVO_AIL_R, 0, 180));
    i = 0;
  }

  // loop1
  if (millis() - loop1_last_time > LOOP1_TIME_MS) {
    // Serial.println(press_act);
    // bme280.readData(&temp_act, &press_act, &hum_act);
    // sv_ail_l.write(90);
    // sv_ail_r.write(90);
  }

  r.sleep();
}

void propo_cb(){
  bool state = digitalRead(PIN_PIT);

  if (state > last_state_) {
    edge_h_ = micros();
  }
  if (state < last_state_) {
    edge_l_ = micros();
  }

  if (edge_h_ < edge_l_) {
    du = edge_l_ - edge_h_;
    state_auto = du > PULSE_WIDTH;
  }

  last_state_ = state;
}

void read_gyr() {
  gyr_.x = IMU.readFloatGyroX();
  gyr_.y = IMU.readFloatGyroY();
  gyr_.z = IMU.readFloatGyroZ();
}

void read_acc() {
  acc_.x = IMU.readFloatAccelX();
  acc_.y = IMU.readFloatAccelY();
  acc_.z = IMU.readFloatAccelZ();
}