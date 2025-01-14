#ifndef _MY_ENCODER_H_
#define _MY_ENCODER_H_

#include <Arduino.h>

class MyEncoder {
  int8_t pinA;
  int8_t pinB;
  float scale;
  float distance;
  float speed;
  int8_t status;
  unsigned int time;
  unsigned long count;
  unsigned long oldTimeMs;
  bool hasGetDistance;

public:
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
  bool isTime();
  float getDistance();
  float getSpeed();
  int8_t getStatus();
};

#endif