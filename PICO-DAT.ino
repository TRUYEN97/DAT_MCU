#include <ArduinoJson.h>
#include "MyEeprom.h"
#include "MyEncoder.h"
#include "SensorIO.h"
#include "MyRfid.h"
#include "Rpm.h"

uint8_t const RELAY_3 = 18;
uint8_t const RELAY_2 = 19;
uint8_t const RELAY_1 = 20;

const char* FIRST_TIME_KEY = "firstTime";

JsonDocument doc;
MyEncoder encoder(17, 16);
Rpm rpm(28);
MyEeprom myEeprom;
Sensor sensor;
MyRFID rfid(Serial1);


void sendJson(Stream &serialPort, JsonDocument &json);
void readConfigFromEEPROM();
void readSerial(Stream &serialPort);



void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  Serial.begin(115200);
  encoder.init(200);
  rpm.init(500);
  sensor.init();
  myEeprom.init();
  rfid.init();
  if (rfid.findNewTag()) {
    doc["yardUser"] = rfid.getUid();
    myEeprom.saveUser(doc["yardUser"]);
  } else {
    doc["yardUser"] = myEeprom.getUser();
  }
  readConfigFromEEPROM();
  doc[FIRST_TIME_KEY] = true;
}

unsigned long checkSendTime = millis();
void loop() {
  boolean changed = false;
  if (encoder.check() && encoder.getData(doc)) {
    changed = true;
  }
  if (sensor.hasUpdate(doc)) {
    changed = true;
  }
  if (changed || millis() - checkSendTime >= 1000) {
    sendJson(Serial, doc);
  }
  readSerial(Serial);
}


void sendJson(Stream &serialPort, JsonDocument &json) {
  digitalWrite(LED_BUILTIN, LOW);
  String jsonString;
  serializeJson(json, jsonString);
  serialPort.println(jsonString);
  digitalWrite(LED_BUILTIN, HIGH);
  checkSendTime = millis();
}

void sendConfig(Stream &serialPort) {
  JsonDocument cf;
  encoder.getConfig(cf);
  sensor.getConfig(cf);
  rpm.getConfig(cf);
  sendJson(serialPort, cf);
}

void readSerial(Stream &serialPort) {
  if (serialPort.available()) {
    String line = serialPort.readStringUntil('\n');
    line.trim();
    if (line.equalsIgnoreCase("isConnect")) {
      serialPort.println("isConnect");
    } else if (line.equalsIgnoreCase("roff")) {
      digitalWrite(RELAY_2, 0);
      digitalWrite(RELAY_1, 0);
      digitalWrite(RELAY_3, 0);
    } else if (line.equalsIgnoreCase("r1off")) {
      digitalWrite(RELAY_1, 0);
    } else if (line.equalsIgnoreCase("r1")) {
      digitalWrite(RELAY_1, 1);
    } else if (line.equalsIgnoreCase("r2off")) {
      digitalWrite(RELAY_2, 0);
    } else if (line.equalsIgnoreCase("r2")) {
      digitalWrite(RELAY_2, 1);
    } else if (line.equalsIgnoreCase("r3off")) {
      digitalWrite(RELAY_3, 0);
    } else if (line.equalsIgnoreCase("r3")) {
      digitalWrite(RELAY_3, 1);
    } else if (line.equalsIgnoreCase("reset")) {
      encoder.reset();
    } else if (line.equalsIgnoreCase("get")) {
      sendJson(serialPort, doc);
    }
    if (line.equalsIgnoreCase("getConfig")) {
      sendConfig(serialPort);
    } else if (line.charAt(0) == '{' && line.charAt(line.length() - 1) == '}') {
      doc[FIRST_TIME_KEY] = false; // reset cap nhat khoang cach tu app
      JsonDocument filter;
      filter[MyEncoder::ENCODE_KEY] = true;
      filter[MyEncoder::DISTANCE_KEY] = true; // {"distance":200}
      filter[Sensor::NT_DELAY_TIME] = true;
      filter[Sensor::NP_DELAY_TIME] = true;
      filter[Rpm::RPM_KEY] = true;
      JsonDocument config;
      deserializeJson(config, line, DeserializationOption::Filter(filter));
      encoder.setConfig(config);
      sensor.setConfig(config);
      rpm.setConfig(config);
      myEeprom.saveConfig(encoder.getScale(), rpm.getScale(), sensor.getNtHoldTime(), sensor.getNpHoldTime());
      sendConfig(serialPort);
    }
  }
}

void readConfigFromEEPROM() {
  float encodeScale;
  float rpmScale;
  unsigned int nt_time;
  unsigned int np_time;
  myEeprom.readConfig(encodeScale, rpmScale, nt_time, np_time);
  encoder.setScale(encodeScale);
  sensor.setNtHoldTime(nt_time);
  sensor.setNpHoldTime(np_time);
  rpm.setScale(rpmScale);
}