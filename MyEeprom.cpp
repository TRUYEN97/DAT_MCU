#include <sys/_stdint.h>
#include "MyEeprom.h"


MyEeprom::MyEeprom() {
  maxSize = 50;
}

template<typename T>
void MyEeprom::writeValue(int address, T value) {
  EEPROM.put(address, value);
  EEPROM.commit();
}

void MyEeprom::init() {
  EEPROM.begin(maxSize);
}

void MyEeprom::saveUser(String user) {
  writeStringToEEPROM(16, user.c_str());
}

String MyEeprom::getUser() {
  char buffer[30];
  if(readStringFromEEPROM(16, buffer, 30) > 0){
    return String(buffer);
  }
  return "";
}

void MyEeprom::writeStringToEEPROM(int address, const char *data) {
  int len = strlen(data);
  for (int i = 0; i < len; i++) {
    EEPROM.write(address + i, data[i]);
  }
  EEPROM.write(address + len, '\0');
  EEPROM.commit();
}

uint8_t MyEeprom::readStringFromEEPROM(int address, char *buffer, int bufferSize) {
  int i = 0;
  for (; i < bufferSize; i++) {
    buffer[i] = EEPROM.read(address + i);
    if (buffer[i] == '\0') break;  // Kết thúc khi gặp ký tự null
  }
  return i;
}

void MyEeprom::saveConfig(float encode, float rpm, unsigned int nt_time, unsigned int np_time) {
  writeValue<float>(0, encode);
  writeValue<float>(4, rpm);
  writeValue<unsigned int>(8, nt_time);
  writeValue<unsigned int>(12, np_time);
}

void MyEeprom::readConfig(float &encode, float &rpm, unsigned int &nt_time, unsigned int &np_time) {
  EEPROM.get(0, encode);
  EEPROM.get(4, rpm);
  EEPROM.get(8, nt_time);
  EEPROM.get(12, np_time);
}