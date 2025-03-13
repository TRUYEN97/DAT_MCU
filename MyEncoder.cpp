#include "lwipopts.h"
#include "MyEncoder.h"

MyEncoder::MyEncoder(int8_t pinA, int8_t pinB)
  : pinA(pinA), pinB(pinB), scale(50), time(200), count(0), oldTimeMs(0), distance(0), speed(0), status(STOP) {}

const String MyEncoder::ENCODE_KEY = "encoder";
const String MyEncoder::DISTANCE_KEY = "distance";

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
  pinMode(this->pinA, INPUT_PULLUP);
  pinMode(this->pinB, INPUT_PULLUP);
  oldTimeMs = millis();
  this->time = time;
}

void MyEncoder::reset() {
  this->count = 0;
  this->distance = 0;
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
bool MyEncoder::check() {
  unsigned long delta = millis() - this->oldTimeMs;
  if (delta >= this->time) {
    ///////////////////////////
    // noInterrupts();
    long countTemp = this->count;
    this->count = 0;
    this->oldTimeMs = millis();
    // interrupts();
    // this->distance += countTemp;
    // this->speed = 0;
    // this->status = STOP;
    // Serial.println(countTemp);
    ////////////////////////////////////
    ////////////////////////////////////
    if (countTemp == 0) {
      this->status = STOP;
      this->speed = 0;
    } else {
      if (delta == 0) {
        delta = 1;
      }
      double currentDistance =  countTemp / this->scale;
      this->distance += currentDistance;
      double m_s = currentDistance * (1000.0 / delta);
      if (countTemp > 0) {
        this->status = FORWARD;
        this->speed = m_s * 3.6;
      } else {
        this->status = BACKWARD;
        this->speed = m_s * -3.6;
      }
    }
    return true;
  }
  return false;
}

void MyEncoder::getData(double &distance, float &speed, int8_t &status) {
  distance = this->distance;
  speed = this->speed;
  status = this->status;
}

template<typename T>
bool MyEncoder::update(JsonDocument &data, String key, T value) {
  if (data[key].as<T>() != value) {
    data[key] = value;
    return true;
  }
  return false;
}

bool MyEncoder::getData(JsonDocument &data) {
  bool changed = false;
  if (update<double>(data, DISTANCE_KEY, this->distance)) {
    changed = true;
  }
  if (update<float>(data, "speed", this->speed)) {
    changed = true;
  }
  if (update<int8_t>(data, "status", this->status)) {
    changed = true;
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
  if(config[DISTANCE_KEY].is<double>()){
    this->distance = config[DISTANCE_KEY].as<double>();
  }
}