#include "MyEncoder.h"

MyEncoder::MyEncoder(int8_t pinA, int8_t pinB)
  : pinA(pinA), pinB(pinB), scale(50), time(200), count(0), oldTimeMs(0) {}

const String MyEncoder::ENCODE_KEY = "encoder";

void MyEncoder::attachPhaseA(MyEncoder *encoder) {
  encoder->attachPhaseACallback();
}

void MyEncoder::attachPhaseB(MyEncoder *encoder) {
  encoder->attachPhaseBCallback();
}

void MyEncoder::init(unsigned int time) {
  pinMode(this->pinA, INPUT_PULLUP);
  pinMode(this->pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(this->pinA), MyEncoder::attachPhaseA, RISING, this);
  attachInterrupt(digitalPinToInterrupt(this->pinB), MyEncoder::attachPhaseB, RISING, this);
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

bool MyEncoder::getData(double &distance, float &speed, int8_t &status) {
  unsigned long delta = millis() - this->oldTimeMs;
  if (delta >= this->time) {
    ///////////////////////////
    long countTemp = this->count;
    this->count = 0;
    this->oldTimeMs = millis();
    // distance = countTemp;
    // speed = 0;
    // status = STOP;
    // Serial.println(countTemp);
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
      double m_s = distance * (1000.0 / delta);
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
template<typename T>
bool MyEncoder::update(JsonDocument &data, const char *key, T value) {
  if (data[key].as<T>() != value) {
    data[key] = value;
    return true;
  }
  return false;
}

bool MyEncoder::getData(JsonDocument &data) {
  bool changed = false;
  double distance = 0;
  float speed = 0;
  int8_t status = STOP;
  if (getData(distance, speed, status)) {
    // distance += data["distance"].as<double>();
    if (update<double>(data, "distance", distance)) {
      changed = true;
    }
    if (update<float>(data, "speed", speed)) {
      changed = true;
    }
    if (update<int8_t>(data, "status", status)) {
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