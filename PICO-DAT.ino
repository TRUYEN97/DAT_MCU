#include <ArduinoJson.h>
#include "MyEeprom.h"
#include "MyEncoder.h"
#include "SensorIO.h"

float RPM_SCALE = 60;
String yardUser = "user";

uint8_t const RELAY_3 = 18;
uint8_t const RELAY_2 = 19;
uint8_t const RELAY_1 = 20;
uint8_t const PIN_RPM = 28;

double rpm = 0;

JsonDocument doc;
MyEncoder encoder(16, 17);
MyEeprom myEeprom;
Sensor sensor;


void sendJson(Stream &serialPort, JsonDocument &json);

template<typename T = uint>
void updateConfig(const JsonDocument &config, T &field, const char *key, T spec);

void readSerial(Stream &serialPort);

template<typename T = boolean>
boolean hasUpdate(const char *key, T value);

boolean isValuesChanged(boolean changed = false);

boolean isTimeOut(unsigned long &time, const unsigned int &timeOut, boolean reset);

void sendJson(Stream &serialPort, JsonDocument &json) {
  digitalWrite(LED_BUILTIN, LOW);
  String jsonString;
  serializeJson(json, jsonString);
  serialPort.println(jsonString);
  digitalWrite(LED_BUILTIN, HIGH);
}

template<typename T>
void updateConfig(const JsonDocument &config, T &field, const char *key, T spec) {
  if (config[key].isNull()) {
    return;
  }
  T value = config[key];
  if (value >= spec) {
    field = value;
  }
}

void sendConfig(Stream &serialPort) {
  JsonDocument cf;
  encoder.getConfig(cf);
  sensor.getConfig(cf);
  cf["rpm"] = RPM_SCALE;
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
      JsonDocument filter;
      filter[MyEncoder::ENCODE_KEY] = true;
      filter[Sensor::NT_DELAY_TIME] = true;
      filter[Sensor::NP_DELAY_TIME] = true;
      filter["rpm"] = true;
      JsonDocument config;
      deserializeJson(config, line, DeserializationOption::Filter(filter));
      encoder.setConfig(config);
      sensor.setConfig(config);
      updateConfig<float>(config, RPM_SCALE, "rpm", 1);
      myEeprom.saveConfig(encoder.getScale(), RPM_SCALE, sensor.getNtHoldTime(), sensor.getNpHoldTime());
      sendConfig(serialPort);
    }
  }
}

void readConfigFromEEPROM(){
    float scale;
    float rpm;
    unsigned int nt_time;
    unsigned int np_time;
    myEeprom.readConfig(scale, rpm, nt_time, np_time);
    encoder.setScale(scale);
    sensor.setNtHoldTime(nt_time);
    sensor.setNpHoldTime(np_time);
    RPM_SCALE = rpm;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  pinMode(PIN_RPM, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), attachRPM, RISING);
  pinMode(PIN_RPM, INPUT_PULLUP);
  Serial.begin(115200);
  Serial1.begin(115200);

  encoder.init(200);
  sensor.init();
  myEeprom.init();
  readConfigFromEEPROM();
}

unsigned int rpmCount = 0;
void attachRPM() {
  rpmCount += 1;
}

template<typename T>
boolean hasUpdate(const char *key, T value) {
  if (doc[key].as<T>() != value) {
    doc[key] = value;
    return true;
  }
  return false;
}

boolean isValuesChanged(boolean changed) {
  if (hasUpdate("yardUser", yardUser)) {
    changed = true;
  }
  if (hasUpdate("rpm", rpm)) {
    changed = true;
  }
  return changed;
}

boolean isTimeOut(unsigned long &time, const unsigned int &timeOut, boolean reset = true) {
  unsigned long currentTime = millis();
  boolean rs = false;
  if (currentTime >= time) {
    rs = currentTime - time >= timeOut;
  } else {
    rs = time - currentTime >= timeOut;
  }
  if (rs && reset) {
    time = millis();
  }
  return rs;
}

unsigned long checkRPMTime = millis();
unsigned long checkSendTime = millis();
void loop() {
  boolean changed = false;
  if (encoder.getData(doc)) {
    changed = true;
  }
  if (sensor.hasUpdate(doc)) {
    changed = true;
  }
  if (isTimeOut(checkRPMTime, 500)) {
    rpm = rpmCount / RPM_SCALE * 120;
    rpmCount = 0;
  }
  if (isValuesChanged(changed) || isTimeOut(checkSendTime, 500)) {
    sendJson(Serial, doc);
    sendJson(Serial1, doc);
    checkSendTime = millis();
  }
  readSerial(Serial);
  readSerial(Serial1);
}