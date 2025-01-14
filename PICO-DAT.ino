#include <TinyGPS.h>
#include <ArduinoJson.h>
#include "MyEeprom.h"
#include "MyEncoder.h"


float RPM_SCALE = 60;
unsigned int NT_DELAY_TIME = 666;
unsigned int NP_DELAY_TIME = 666;


uint8_t const T3_PIN = 2;
uint8_t const T2_PIN = 3;
uint8_t const GPS_RX = 4;
uint8_t const GPS_TX = 5;
uint8_t const T1_PIN = 6;
uint8_t const PT_PIN = 8;
uint8_t const NT_PIN = 9;
uint8_t const CM_PIN = 10;
uint8_t const ADC_VIN_PIN = 11;
uint8_t const S1_PIN = 12;
uint8_t const S2_PIN = 13;
uint8_t const S3_PIN = 14;
uint8_t const S4_PIN = 15;
uint8_t const RELAY_3 = 18;
uint8_t const RELAY_2 = 19;
uint8_t const RELAY_1 = 20;
uint8_t const NP_PIN = 21;
uint8_t const AT_PIN = 22;
uint8_t const ADC_TEMP_PIN = 26;
uint8_t const ADC_RPM_PIN = 27;
uint8_t const PIN_RPM = 28;

double temp = 0;
double latitude = 0;
double longitude = 0;
double rpm = 0;
unsigned int rpmV = 0;

JsonDocument doc;
TinyGPS gps;
MyEncoder encoder(16, 17);
MyEeprom myEeprom;


#define ADC_RPM analogRead(ADC_RPM_PIN)
#define ADC_TEMP analogRead(ADC_TEMP_PIN)
#define ADC_VIN analogRead(ADC_VIN_PIN)

void sendJson(Stream &serialPort, JsonDocument &json);

template<typename T = uint>
void updateConfig(const JsonDocument &config, T &field, const char *key, T spec);

void readSerial(Stream &serialPort);

boolean valueOf(uint8_t const &pin, boolean status);

boolean valOfNPT(boolean value, unsigned long &time, const unsigned int &timeOut);

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

bool newData = false;
void readGPS() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (gps.encode(c)) {
      newData = true;
    }
  }
  if (newData) {
    newData = false;
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    latitude = flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat;
    longitude = flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon;
  }
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
      JsonDocument cf;
      cf["encoder"] = encoder.getScale();
      cf["nt_time"] = NT_DELAY_TIME;
      cf["np_time"] = NP_DELAY_TIME;
      cf["rpm"] = RPM_SCALE;
      sendJson(serialPort, cf);
    } else if (line.charAt(0) == '{' && line.charAt(line.length() - 1) == '}') {
      JsonDocument filter;
      filter["encoder"] = true;
      filter["nt_time"] = true;
      filter["np_time"] = true;
      filter["rpm"] = true;
      JsonDocument config;
      deserializeJson(config, line, DeserializationOption::Filter(filter));
      float encoderScale = encoder.getScale();
      updateConfig<float>(config, encoderScale, "encoder", 1);
      encoder.setScale(encoderScale);
      updateConfig<float>(config, RPM_SCALE, "rpm", 1);
      updateConfig<unsigned int>(config, NT_DELAY_TIME, "nt_time", 0);
      updateConfig<unsigned int>(config, NP_DELAY_TIME, "np_time", 0);
      myEeprom.saveConfig(encoder.getScale(), RPM_SCALE, NT_DELAY_TIME, NP_DELAY_TIME);
      JsonDocument cf;
      cf["encoder"] = encoder.getScale();
      cf["nt_time"] = NT_DELAY_TIME;
      cf["np_time"] = NP_DELAY_TIME;
      cf["rpm"] = RPM_SCALE;
      sendJson(serialPort, cf);
      sendJson(serialPort, doc);
    }
  }
}

unsigned long checkNtTime = millis();
unsigned long checkNpTime = millis();
unsigned long checkT1Time = millis();
unsigned long checkT2Time = millis();
unsigned long checkT3Time = millis();
#define CM valueOf(CM_PIN)
#define NT valOfNPT(valueOf(NT_PIN), checkNtTime, NT_DELAY_TIME)
#define NP valOfNPT(valueOf(NP_PIN), checkNpTime, NP_DELAY_TIME)
#define AT valueOf(AT_PIN)
#define PT valueOf(PT_PIN)
#define T1 valOfNPT(valueOf(T1_PIN), checkT1Time, 200)
#define T2 valOfNPT(valueOf(T2_PIN), checkT2Time, 200)
#define T3 valOfNPT(valueOf(T3_PIN), checkT3Time, 200)
#define S1 valueOf(S1_PIN)
#define S2 valueOf(S2_PIN)
#define S3 valueOf(S3_PIN)
#define S4 valueOf(S4_PIN)

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  pinMode(CM_PIN, INPUT_PULLUP);
  pinMode(NT_PIN, INPUT_PULLUP);
  pinMode(NP_PIN, INPUT_PULLUP);
  pinMode(AT_PIN, INPUT_PULLUP);
  pinMode(PT_PIN, INPUT_PULLUP);
  pinMode(T1_PIN, INPUT_PULLUP);
  pinMode(T2_PIN, INPUT_PULLUP);
  pinMode(T3_PIN, INPUT_PULLUP);
  pinMode(S1_PIN, INPUT_PULLUP);
  pinMode(S2_PIN, INPUT_PULLUP);
  pinMode(S3_PIN, INPUT_PULLUP);
  pinMode(S4_PIN, INPUT_PULLUP);
  pinMode(PIN_RPM, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), attachRPM, RISING);
  pinMode(PIN_RPM, INPUT_PULLUP);
  Serial1.begin(115200);
  Serial2.setTX(GPS_RX);
  Serial2.setRX(GPS_TX);
  Serial2.begin(9600);
  Serial.begin(115200);

  encoder.init(500);
  myEeprom.init();
  float encoderScale = encoder.getScale();
  myEeprom.readConfig(encoderScale, RPM_SCALE, NT_DELAY_TIME, NP_DELAY_TIME);
  encoder.setScale(encoderScale);
  isValuesChanged();
}

unsigned int rpmCount = 0;
void attachRPM() {
  rpmCount += 1;
}

boolean valueOf(uint8_t const &pin, boolean status = false) {
  if (digitalRead(pin) == status) {
    // delay(20);
    // if (digitalRead(pin) == status) {
    return true;
    // }
  }
  return false;
}

boolean valOfNPT(boolean value, unsigned long &time, const unsigned int &timeOut) {
  if (value == false) {
    if (isTimeOut(time, timeOut, false)) {
      return false;
    } else {
      return true;
    }
  } else {
    time = millis();
    return true;
  }
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
  if (hasUpdate("cm", CM)) {
    changed = true;
  }
  if (hasUpdate("nt", NT)) {
    changed = true;
  }
  if (hasUpdate("np", NP)) {
    changed = true;
  }
  if (hasUpdate("at", AT)) {
    changed = true;
  }
  if (hasUpdate("pt", PT)) {
    changed = true;
  }
  if (hasUpdate("t1", T1)) {
    changed = true;
  }
  if (hasUpdate("t2", T2)) {
    changed = true;
  }
  if (hasUpdate("t3", T3)) {
    changed = true;
  }
  if (hasUpdate("s1", S1)) {
    changed = true;
  }
  if (hasUpdate("s2", S2)) {
    changed = true;
  }
  if (hasUpdate("s3", S3)) {
    changed = true;
  }
  if (hasUpdate("s4", S4)) {
    changed = true;
  }
  if (hasUpdate("rpm", rpm)) {
    changed = true;
  }
  if (hasUpdate("rpmV", rpmV)) {
    changed = true;
  }
  if (hasUpdate("temp", temp)) {
    changed = true;
  }
  if (hasUpdate("latitude", latitude)) {
    changed = true;
  }
  if (hasUpdate("longitude", longitude)) {
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
  if (encoder.isTime()) {
    if (hasUpdate("status", encoder.getStatus())) {
      changed = true;
    }
    if (hasUpdate("distance", encoder.getDistance())) {
      changed = true;
    }
    if (hasUpdate("speed", encoder.getSpeed())) {
      changed = true;
    }
  }
  if (isTimeOut(checkRPMTime, 500)) {
    rpm = rpmCount / RPM_SCALE * 120;
    // Serial.println(rpmCount);
    rpmCount = 0;
    // temp = ADC_TEMP * 0.08;
    // rpmV =
  }
  if (isValuesChanged(changed) || isTimeOut(checkSendTime, 500)) {
    sendJson(Serial, doc);
    sendJson(Serial1, doc);
    checkSendTime = millis();
  }
  readSerial(Serial);
  readSerial(Serial1);
  readGPS();
}