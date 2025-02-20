#ifndef _MY_EEPROM_H_
#define _MY_EEPROM_H_
#include <EEPROM.h>
#include <ArduinoJson.h>

class MyEeprom {
  uint32_t maxSize;
  template<typename T = int>
  void writeValue(int address, T value);
  void writeStringToEEPROM(int address, const char *data) ;
  uint8_t readStringFromEEPROM(int address, char *buffer, int bufferSize);
public:
  MyEeprom();
  void init();
  void saveUser(String user);
  String getUser();
  void saveConfig(float encode, float rpm, unsigned int nt_time, unsigned int np_time);
  void readConfig(float &encode, float &rpm, unsigned int &nt_time, unsigned int &np_time);
};

#endif