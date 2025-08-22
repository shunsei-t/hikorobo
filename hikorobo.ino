#include <Servo.h>
#include <Wire.h>
#include <SSCI_BME280.h>
#include "util.h"

Servo sv_ele, sv_ail_l, sv_ail_r;
SSCI_BME280 bme280;

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

  // loop1
  if (millis() - loop1_last_time > LOOP1_TIME_MS) {
    // Serial.println(press_act);
    // bme280.readData(&temp_act, &press_act, &hum_act);
    sv_ail_l.write(90);
    sv_ail_r.write(90);
  }
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
