#ifndef _MY_ENCODER_H_
#define _MY_ENCODER_H_

#include <Arduino.h>
#include <ArduinoJson.h>

class MyEncoder {
  int8_t pinA;
  int8_t pinB;
  float scale;
  unsigned int time;
  long count;
  unsigned long oldTimeMs;
  template <typename T = bool>
  bool update(JsonDocument &data, const char *key, T value);
public:
  static const String ENCODE_KEY;
  static int8_t const FORWARD = 1;
  static int8_t const BACKWARD = -1;
  static int8_t const STOP = 0;
  static void attachPhaseA(MyEncoder *encoder);
  static void attachPhaseB(MyEncoder *encoder);
  MyEncoder(int8_t pinA, int8_t pinB);
  void attachPhaseACallback();
  void attachPhaseBCallback();
  void init(unsigned int time);
  void reset();
  void setScale(float scale);
  float getScale();
  bool getData(double &distance, float &speed, int8_t &status);
  bool getData(JsonDocument &data);
  void getConfig(JsonDocument &config);
  void setConfig(const JsonDocument &config);
};

#endif