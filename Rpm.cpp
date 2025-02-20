#include "Rpm.h"

const String Rpm::RPM_KEY = "rpm";
void Rpm::attachRPM(Rpm *rpm) {
  rpm->attachCallback();
}
Rpm::Rpm(int8_t pin)
  : pin(pin), scale(60), time(200), count(0), oldTimeMs(0) {
}
void Rpm::attachCallback() {
  count += 1;
}
void Rpm::init(unsigned int time) {
  pinMode(this->pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(this->pin), Rpm::attachRPM, RISING, this);
  oldTimeMs = millis();
  this->time = time;
}
void Rpm::reset() {
  this->count = 0;
  this->oldTimeMs = millis();
}
void Rpm::setScale(float scale) {
  this->scale = scale < 1 ? 1 : scale;
}
float Rpm::getScale() {
  return this->scale;
}

bool Rpm::getData(float &rpmVal) {
  unsigned long delta = millis() - this->oldTimeMs;
  if (delta >= this->time) {
    ///////////////////////////
    long countTemp = this->count;
    this->count = 0;
    rpmVal = countTemp / this->scale * (60000.0 / delta);
    return true;
  }
  return false;
}
bool Rpm::getData(JsonDocument &data) {
  float value;
  if (getData(value)) {
    if (data[RPM_KEY].as<float>() != value) {
      data[RPM_KEY] = value;
      return true;
    }
  }
  return false;
}

void Rpm::getConfig(JsonDocument &config) {
  config[RPM_KEY] = this->scale;
}

void Rpm::setConfig(const JsonDocument &config) {
  if (config[RPM_KEY].is<float>()) {
    setScale(config[RPM_KEY].as<float>());
  }
}