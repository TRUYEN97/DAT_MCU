#include "MyRfid.h"

MyRFID::MyRFID(HardwareSerial &serialPort)
  : pn532hsu(new PN532_HSU(serialPort)), nfc(new PN532(*pn532hsu)) {
}

MyRFID::~MyRFID() {
  delete this->nfc;
  this->nfc = nullptr;
  delete this->pn532hsu;
  this->pn532hsu = nullptr;
}

void MyRFID::init() {
  this->nfc->begin();
  this->nfc->SAMConfig();
}

bool MyRFID::findNewTag() {
  uint8_t temp[7] = { 0 };
  if (this->nfc->readPassiveTargetID(PN532_MIFARE_ISO14443A, &temp[0], &uidLength, 5000)) {
    bool isNewTag = false;
    for (uint8_t i = 0; i < uidLength; i++) {
      if (this->uid[i] != temp[i]) {
        this->uid[i] = temp[i];
        isNewTag = true;
      }
    }
    return isNewTag;
  } else {
    for (uint8_t i = 0; i < uidLength; i++) {
      this->uid[i] = 0;
    }
    uidLength = 0;
    return false;
  }
}

String MyRFID::getUid() {
  String uid;
  uint8_t id = 0;
  for (uint8_t i = 0; i < uidLength; i++) {
    id = this->uid[i];
    if (id < 100) {
      uid.concat("0");
    }
    if (id < 10) {
      uid.concat("0");
    }
    uid.concat(id);
  }
  return uid;
}