#include "MyEeprom.h"

template<typename T>
void MyEeprom::writeValue(int address, T value){
  EEPROM.put(address, value);
  EEPROM.commit(); 
}

void MyEeprom::init(){
  EEPROM.begin(16);
}

void MyEeprom::saveConfig(float encode, float rpm, unsigned int nt_time, unsigned int np_time){
  writeValue<float>(0, encode);
  writeValue<float>(4, rpm);
  writeValue<unsigned int>(8, nt_time);
  writeValue<unsigned int>(12, np_time);
}

void MyEeprom::readConfig(float &encode, float &rpm, unsigned int &nt_time, unsigned int &np_time){
  EEPROM.get(0, encode);
  EEPROM.get(4, rpm);
  EEPROM.get(8, nt_time);
  EEPROM.get(12, np_time);
}