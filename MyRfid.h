#ifndef _MY_RFID_H_
#define _MY_RFID_H_

#include <Arduino.h>
#include <PN532.h>
#include <PN532_HSU.h>



class MyRFID {
  uint8_t uidLength;
  uint8_t uid[7] = { 0 };
  PN532_HSU *pn532hsu = nullptr;
  PN532 *nfc = nullptr;
public:
  MyRFID(HardwareSerial &serialPort);
  ~MyRFID();
  void init();
  bool findNewTag();
  String getUid();
};


#endif