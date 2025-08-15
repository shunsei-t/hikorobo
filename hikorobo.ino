#include <Servo.h>

#define PIN_PROPO 5
#define PIN_SERVO 1
#define PIN_MULTI 7 //6じゃなくて7になっている
#define THRE_PROPO 100
#define SYS_DURATION 1200 //loopが重くなると値が変わる可能性
#define SERIAL_PRINT_MS 100

Servo sv;

unsigned long last_time;
unsigned long edge_h;
unsigned long edge_l;
bool last_state;
bool state_auto;

void setup() {
  Serial.begin(9600);
  pinMode(PIN_PROPO, INPUT);
  pinMode(PIN_SERVO, OUTPUT);
  pinMode(PIN_MULTI, OUTPUT);

  sv.attach(PIN_SERVO);

  // default value
  last_time = millis();
  edge_h = micros();
  edge_l = micros();
  last_state = false;
  state_auto = false;
}

void loop() {
  bool state = analogRead(PIN_PROPO) > THRE_PROPO;

  if(state > last_state) {
    edge_h = micros();
  }
  if(state < last_state) {
    edge_l = micros();
  }

  if (edge_h < edge_l){
    unsigned long du = edge_l - edge_h;
    state_auto = du > SYS_DURATION;
  }

  if (state_auto) {
    digitalWrite(PIN_MULTI, HIGH);
  } else {
    digitalWrite(PIN_MULTI, LOW);
  }

  if (millis() - last_time > SERIAL_PRINT_MS) {
    Serial.println(state_auto);
    last_time = millis();
    Serial.println(millis()%100);
    sv.write(20+millis()%100);
  }

  last_state = state;
}
