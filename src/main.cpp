#include <Arduino.h>

uint8_t upPin = A0;
uint8_t downPin = A1;

#define R_EN 7
#define L_EN 8
#define R_PWM 9
#define L_PWM 10

#define ENCODER_PIN_1 2
#define ENCODER_PIN_2 3

#define TRIG_PIN 6
#define ECHO_PIN 5

#define MIN_SPEED 100
#define MAX_SPEED 255
#define SPEED_STEP_DURATION 125

typedef enum {
    UP_DIR,
    DOWN_DIR,
    OFF
} MotorState;

unsigned long last_tick = millis();
uint8_t speed = 0;

MotorState state = OFF;
volatile int position = 0;

void positionChange() {
  if (state == UP_DIR) {
    position++;
  } else if (state == DOWN_DIR) {
    position--;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Hello");

  pinMode(upPin, INPUT);
  pinMode(upPin, INPUT_PULLUP);
  pinMode(downPin, INPUT);
  pinMode(downPin, INPUT_PULLUP);

  pinMode(R_EN, OUTPUT);
  digitalWrite(R_EN, LOW);
  pinMode(L_EN, OUTPUT);
  digitalWrite(L_EN, LOW);
  pinMode(R_PWM, OUTPUT);
  digitalWrite(R_PWM, LOW);
  pinMode(L_PWM, OUTPUT);
  digitalWrite(L_PWM, LOW);

//  pinMode(ENCODER_PIN_1, INPUT);
  pinMode(ENCODER_PIN_2, INPUT);

//  attachInterrupt((uint8_t) digitalPinToInterrupt(ENCODER_PIN_1), positionUp, CHANGE);
  attachInterrupt((uint8_t) digitalPinToInterrupt(ENCODER_PIN_2), positionChange, CHANGE);

  pinMode(TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO_PIN, INPUT); // Sets the echoPin as an Input
}

void updateSpeed() {
    if (state == UP_DIR) {
        analogWrite(L_PWM, 0);
        analogWrite(R_PWM, speed);
    } else if (state == DOWN_DIR) {
        analogWrite(R_PWM, 0);
        analogWrite(L_PWM, speed);
    }
}

void stop() {
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
    state = OFF;
}

void start() {
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);
}

void loop() {
  if (digitalRead(upPin) == LOW) {
    if (state != UP_DIR) {
      start();
      state = UP_DIR;
      speed = MIN_SPEED;
    }
    updateSpeed();
  } else if (digitalRead(downPin) == LOW) {
    if (state != DOWN_DIR) {
      start();
      state = DOWN_DIR;
      speed = MIN_SPEED;
    }
    updateSpeed();
  } else {
    stop();
  }

  unsigned long now = millis(), diff = now - last_tick;

//  if (diff >= SPEED_STEP_DURATION) {
//    Serial.println("POSITION:");
//    Serial.println(position);
//  }

  if (diff >= SPEED_STEP_DURATION && speed < MAX_SPEED && speed >= MIN_SPEED) {
    speed = speed + 5 % (MAX_SPEED + 1);
    last_tick = now;
    Serial.println("SPEED:");
    Serial.println(speed);
  }
}