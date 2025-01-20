#ifndef _MY_ENCODER_H_
#define _MY_ENCODER_H_

#include <Arduino.h>
#include <ArduinoJson.h>

class MyEncoder {
  int8_t pinA;
  int8_t pinB;
  float scale;
  unsigned int time;
  unsigned long count;
  unsigned long oldTimeMs;
  bool update(JsonDocument &data, const char *key, boolean value);
public:
  static const String ENCODE_KEY;
  static int8_t const FORWARD = 1;
  static int8_t const BACKWARD = -1;
  static int8_t const STOP = 0;
  MyEncoder(int8_t pinA, int8_t pinB);
  void attachPhaseACallback();
  void attachPhaseBCallback();
  void init(unsigned int time);
  void reset();
  void setScale(float scale);
  float getScale();
  bool getData(double &distance, float &speed, uint8_t &status);
  bool getData(JsonDocument &data);
  void getConfig(JsonDocument &config);
  void setConfig(const JsonDocument &config);
};

#endif