#ifndef _MY_EEPROM_H_
#define _MY_EEPROM_H_
#include <EEPROM.h>


class MyEeprom {
  template<typename T = int>
  void writeValue(int address, T value);
public:
  void init();
  void saveConfig(float encode, float rpm, unsigned int nt_time, unsigned int np_time);
  void readConfig(float &encode, float &rpm, unsigned int &nt_time, unsigned int &np_time);
};

#endif