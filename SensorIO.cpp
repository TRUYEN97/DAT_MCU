#include "SensorIO.h"

const String Sensor::NT_DELAY_TIME = "nt_time";
const String Sensor::NP_DELAY_TIME = "np_time";
Sensor::Sensor()
  : pt(PT_PIN), at(AT_PIN), cm(CM_PIN),
    nt(NT_PIN), np(NP_PIN), t1(T1_PIN),
    t2(T2_PIN), t3(T3_PIN), s1(S1_PIN),
    s2(S2_PIN), s3(S3_PIN), s4(S4_PIN) {
}

void Sensor::init() {
  this->cm.setInputMode(INPUT_PULLUP);
  this->nt.setHoldTime(666);
  this->np.setHoldTime(666);
  this->t1.setHoldTime(200);
  this->t2.setHoldTime(200);
  this->t3.setHoldTime(200);
}

bool Sensor::update(JsonDocument &data, const char *key, boolean value) {
  if (data[key].as<boolean>() != value) {
    data[key] = value;
    return true;
  }
  return false;
}

void Sensor::setNtHoldTime(unsigned long time) {
  this->nt.setHoldTime(time);
}

void Sensor::setNpHoldTime(unsigned long time) {
  this->np.setHoldTime(time);
}

unsigned long Sensor::getNtHoldTime() {
  return this->nt.getHoldTime();
}

unsigned long Sensor::getNpHoldTime() {
  return this->np.getHoldTime();
}

bool Sensor::hasUpdate(JsonDocument &data) {
  boolean changed = false;
  boolean target = false;
  if (update(data, "cm", this->cm.getValue(0, target))) {
    changed = true;
  }
  if (update(data, "nt", this->nt.getValue(0, target))) {
    changed = true;
  }
  if (update(data, "np", this->np.getValue(0, target))) {
    changed = true;
  }
  if (update(data, "at", this->at.getValue(0, target))) {
    changed = true;
  }
  if (update(data, "pt", this->pt.getValue(0, target))) {
    changed = true;
  }
  if (update(data, "t1", this->t1.getValue(0, target))) {
    changed = true;
  }
  if (update(data, "t2", this->t2.getValue(0, target))) {
    changed = true;
  }
  if (update(data, "t3", this->t3.getValue(0, target))) {
    changed = true;
  }
  if (update(data, "s1", this->s1.getValue(0, target))) {
    changed = true;
  }
  if (update(data, "s2", this->s2.getValue(0, target))) {
    changed = true;
  }
  if (update(data, "s3", this->s3.getValue(0, target))) {
    changed = true;
  }
  if (update(data, "s4", this->s4.getValue(0, target))) {
    changed = true;
  }
  return changed;
}

void Sensor::getConfig(JsonDocument &config) {
  config[NT_DELAY_TIME] = this->nt.getHoldTime();
  config[NP_DELAY_TIME] = this->np.getHoldTime();
}

void Sensor::setConfig(const JsonDocument &config) {
  if (config[NT_DELAY_TIME].is<unsigned long>()) {
    this->nt.setHoldTime(config[NT_DELAY_TIME].as<unsigned long>());
  }
  if (config[NP_DELAY_TIME].is<unsigned long>()) {
    this->np.setHoldTime(config[NP_DELAY_TIME].as<unsigned long>());
  }
}
