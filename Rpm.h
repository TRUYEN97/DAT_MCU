#ifndef _MY_RPM_H_
#define _MY_RPM_H_

#include <Arduino.h>
#include <ArduinoJson.h>
class Rpm {
  float scale;
  int8_t pin;
  unsigned int time;
  long count;
  unsigned long oldTimeMs;
public:
  static const String RPM_KEY;
  static void attachRPM(Rpm *rpm);
  Rpm(int8_t pin);
  void attachCallback();
  void init(unsigned int time);
  void reset();
  void setScale(float scale);
  float getScale();
  bool getData(float &rpmVal);
  bool getData(JsonDocument &data);
  void getConfig(JsonDocument &config);
  void setConfig(const JsonDocument &config);
};


#endif