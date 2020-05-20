#include <Arduino.h>
#include "RunningAverage.h"
#include <PinButton.h>

uint8_t upPin = A1;
uint8_t downPin = A0;

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

#define MAX_DISTANCE 101
#define MIN_DISTANCE 75

typedef enum {
    UP_DIR,
    DOWN_DIR,
    UP_AUTO,
    DOWN_AUTO,
    OFF
} MotorState;

unsigned long last_tick = millis();
unsigned long last_report_tick = millis();
uint8_t speed = 0;

double distance = 0;

MotorState state = OFF;

RunningAverage distanceRA(10);

PinButton upButton(upPin);
PinButton downButton(downPin);

void setup() {
  Serial.begin(9600);
  Serial.println("Hello");

  pinMode(R_EN, OUTPUT);
  digitalWrite(R_EN, LOW);
  pinMode(L_EN, OUTPUT);
  digitalWrite(L_EN, LOW);
  pinMode(R_PWM, OUTPUT);
  digitalWrite(R_PWM, LOW);
  pinMode(L_PWM, OUTPUT);
  digitalWrite(L_PWM, LOW);

  pinMode(TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO_PIN, INPUT); // Sets the echoPin as an Input

  distanceRA.clear();
}

void updateSpeed() {
    if (state == UP_DIR || state == UP_AUTO) {
        analogWrite(R_PWM, 0);
        analogWrite(L_PWM, speed);
    } else if (state == DOWN_DIR || state == DOWN_AUTO) {
        analogWrite(L_PWM, 0);
        analogWrite(R_PWM, speed);
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

void updateDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(20);
  digitalWrite(TRIG_PIN, LOW);
  int duration = pulseIn(ECHO_PIN, HIGH);
  double localDistance = duration*0.034/2;
  if (localDistance < 150) {
    distanceRA.addValue(localDistance);
  }

  distance = distanceRA.getAverage();
}

void loop() {
  upButton.update();
  downButton.update();

  if (upButton.isDoubleClick()) {
    state = UP_AUTO;
    Serial.println("Auto up");
    updateDistance();
    if (distance < MAX_DISTANCE) {
      start();
      speed = MIN_SPEED;
    }
  } else if (downButton.isDoubleClick()) {
    state = DOWN_AUTO;
    Serial.println("Auto Down");
    updateDistance();
    if (distance > MIN_DISTANCE) {
      start();
      speed = MIN_SPEED;
    }
  } else if (upButton.isLongClick()) {
    if (state != UP_DIR) {
      start();
      state = UP_DIR;
      speed = MIN_SPEED;
    }
  } else if (downButton.isLongClick()) {
    if (state != DOWN_DIR) {
      start();
      state = DOWN_DIR;
      speed = MIN_SPEED;
    }
  } else if ((state == DOWN_DIR || state == UP_DIR) && (downButton.isReleased() || upButton.isReleased())) {
    Serial.println("Stopping");
    stop();
  }

  unsigned long now = millis(), diff = now - last_tick, report_diff = now - last_report_tick;

  if (report_diff >= 100 && (state == UP_AUTO || state == DOWN_AUTO)) {
    updateDistance();

    if (state == UP_AUTO) {
      Serial.print("GOIND UP, DISTANCE:");
      Serial.println(distance);
      if (distance > MAX_DISTANCE) {
        stop();
      }

    } else if (state == DOWN_AUTO) {
      Serial.print("GOIND DOWN, DISTANCE:");
      Serial.println(distance);
      if (distance < MIN_DISTANCE) {
        stop();
      }
    }

    last_report_tick = now;
  }

  if (diff >= SPEED_STEP_DURATION && speed < MAX_SPEED && speed >= MIN_SPEED && state != OFF) {
    speed = speed + 5 % (MAX_SPEED + 1);
    last_tick = now;
    Serial.println("SPEED:");
    Serial.println(speed);
  }

  if (state != OFF) {
    updateSpeed();
  }
}