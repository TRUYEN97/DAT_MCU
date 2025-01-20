#include "SerialUSB.h"
#include "MyEncoder.h"

MyEncoder::MyEncoder(int8_t pinA, int8_t pinB)
  : pinA(pinA), pinB(pinB), scale(10.5), time(0), count(0), oldTimeMs(0){}

const String MyEncoder::ENCODE_KEY = "encoder";

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

bool MyEncoder::getData(double &distance, float &speed, uint8_t &status) {
  unsigned long delta = millis() - this->oldTimeMs;
  if (delta >= this->time) {
    ///////////////////////////
    noInterrupts();
    unsigned long countTemp = this->count;
    this->count = 0;
    interrupts();
    this->oldTimeMs = millis();
    ////////////////////////////////////
    ////////////////////////////////////
    if (countTemp == 0) {
      distance = 0;
      status = STOP;
      speed = 0;
    } else {
      if (delta == 0) {
        delta = 1;
      }
      distance = countTemp / this->scale;
      double m_s = distance * 1000.0 / delta;
      if (countTemp > 0) {
        status = FORWARD;
        speed = m_s * 3.6;
      } else {
        status = BACKWARD;
        speed = m_s * -3.6;
      }
    }
    return true;
  }
  return false;
}

bool MyEncoder::update(JsonDocument &data, const char *key, bool value) {
  if (data[key].as<bool>() != value) {
    data[key] = value;
    return true;
  }
  return false;
}

bool MyEncoder::getData(JsonDocument &data) {
  bool changed = false;
  double distance;
  float speed;
  uint8_t status;
  if (getData(distance, speed, status)) {
    if (update(data, "distance", distance)) {
      changed = true;
    }
    if (update(data, "speed", speed)) {
      changed = true;
    }
    if (update(data, "status", status)) {
      changed = true;
    }
  }
  return changed;
}

void MyEncoder::getConfig(JsonDocument &config) {
  config[ENCODE_KEY] = this->scale;
}

void MyEncoder::setConfig(const JsonDocument &config) {
  if (config[ENCODE_KEY].is<float>()) {
    setScale(config[ENCODE_KEY].as<float>());
  }
}