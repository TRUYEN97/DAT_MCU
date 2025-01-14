#include "MyEncoder.h"

MyEncoder::MyEncoder(int8_t pinA, int8_t pinB)
  : pinA(pinA), pinB(pinB), scale(10.5), distance(0), speed(0), status(0), time(0), count(0), oldTimeMs(0), hasGetDistance(true) {}



void attachPhaseA(MyEncoder *encoder) {
  encoder->attachPhaseACallback();
}

void attachPhaseB(MyEncoder *encoder) {
  encoder->attachPhaseBCallback();
}

void MyEncoder::init(unsigned int time) {
  pinMode(this->pinA, INPUT_PULLUP);
  pinMode(this->pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(this->pinA), attachPhaseA, RISING, this);
  attachInterrupt(digitalPinToInterrupt(this->pinB), attachPhaseB, RISING, this);
  pinMode(this->pinA, INPUT_PULLUP);
  pinMode(this->pinB, INPUT_PULLUP);
  oldTimeMs = millis();
  this->time = time;
}

void MyEncoder::reset() {
  this->distance = 0;
  this->count = 0;
  this->oldTimeMs = millis();
}

void MyEncoder::attachPhaseACallback() {
  if (digitalRead(this->pinB)) {
    this->count += 1;
  }
}

void MyEncoder::attachPhaseBCallback() {
  if (digitalRead(this->pinA)) {
    this->count -= 1;
  }
}

void MyEncoder::setScale(float scale) {
  this->scale = scale < 1 ? 1 : scale;
}

float MyEncoder::getScale() {
  return this->scale;
}

bool MyEncoder::isTime() {
  unsigned long delta = millis() - this->oldTimeMs;

  if (delta >= this->time) {
    unsigned long countTemp;
    noInterrupts(); 
    countTemp = this->count;
    this->count = 0;
    interrupts();
    this->oldTimeMs = millis();
    this->distance = countTemp / this->scale;
    if (countTemp == 0) {
      this->distance = 0;
      this->status = STOP;
      this->speed = 0;
    } else {
      if (delta == 0) delta = 1; 
      if (countTemp > 0) {
        this->status = FORWARD;
        this->speed = countTemp * 3.6 / this->scale / delta / 1000.0;
      } else {
        this->status = BACKWARD;
        this->speed = countTemp * -3.6 / this->scale / delta / 1000.0;
      }
    }
    this->hasGetDistance = false;
    return true;
  } else if (this->hasGetDistance) {
    this->distance = 0;
  }

  return false;
}

float MyEncoder::getDistance() {
  this->hasGetDistance = true;
  return this->distance;
}

float MyEncoder::getSpeed() {
  return this->speed;
}

int8_t MyEncoder::getStatus() {
  return this->status;
}