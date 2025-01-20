#ifndef _DAT_SENSOR_IO_H_
#define _DAT_SENSOR_IO_H_
#include <ArduinoJson.h>
#include "DigitalIO.h"

#define T3_PIN 2
#define T2_PIN 3
#define T1_PIN 6
#define PT_PIN 8
#define AT_PIN 9
#define CM_PIN 10
#define S1_PIN 12
#define S2_PIN 13
#define S3_PIN 14
#define S4_PIN 15
#define NP_PIN 21
#define NT_PIN 22
class Sensor {
  DIO pt;
  DIO at;
  DIO cm;
  DIO nt;
  DIO np;
  DIO t1;
  DIO t2;
  DIO t3;
  DIO s1;
  DIO s2;
  DIO s3;
  DIO s4;
  bool update(JsonDocument &data, const char *key, boolean value);
public:
  static const String NT_DELAY_TIME;
  static const String NP_DELAY_TIME;
  Sensor();
  void init();
  void setNtHoldTime(unsigned long time);
  void setNpHoldTime(unsigned long time);
  unsigned long getNtHoldTime();
  unsigned long getNpHoldTime();
  bool hasUpdate(JsonDocument &data);
  void getConfig(JsonDocument &config);
  void setConfig(const JsonDocument &config);
};
#endif