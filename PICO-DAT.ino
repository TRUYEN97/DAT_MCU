#include <TinyGPS.h>
#include <ArduinoJson.h>

double ENCODER_SCALE = 10.5;
double RPM_SCALE = 60;
unsigned int NT_DELAY_TIME = 868;
unsigned int NP_DELAY_TIME = 868;

int8_t const FORWARD = 1;
int8_t const BACKWARD = -1;
int8_t const STOP = 0;
uint8_t const PIN_RPM = 2;
uint8_t const PIN_PHASE_B = 3;
uint8_t const PIN_PHASE_A = 22;
uint8_t const GPS_TX = 4;
uint8_t const GPS_RX = 5;
uint8_t const S1_PIN = 6;
uint8_t const S2_PIN = 7;
uint8_t const S3_PIN = 8;
uint8_t const S4_PIN = 9;
uint8_t const T1_PIN = 10;
uint8_t const T2_PIN = 11;
uint8_t const T3_PIN = 12;
uint8_t const RELAY_1 = 13;
uint8_t const RELAY_2 = 14;
uint8_t const RELAY_3 = 15;
uint8_t const CM_PIN = 16;
uint8_t const PT_PIN = 17;
uint8_t const NT_PIN = 18;
uint8_t const NP_PIN = 19;
uint8_t const AT_PIN = 20;
uint8_t const ADC_RPM_PIN = 26;
uint8_t const ADC_TEMP_PIN = 27;
uint8_t const ADC_VIN_PIN = 28;

unsigned long count = 0;
double distance = 0;
double speed = 0;
double temp = 0;
double latitude = 0;
double longitude = 0;
double rpm = 0;
unsigned int rpmV = 0;
int8_t status = 0;
boolean forward = true;

JsonDocument doc;
TinyGPS gps;

#define PHASE_A digitalRead(PIN_PHASE_A)
#define PHASE_B digitalRead(PIN_PHASE_B)
#define ADC_RPM analogRead(ADC_RPM_PIN)
#define ADC_TEMP analogRead(ADC_TEMP_PIN)
#define ADC_VIN analogRead(ADC_VIN_PIN)

void sendJson(Stream &serialPort, JsonDocument &json);

template<typename T = uint>
void updateConfig(const JsonDocument &config, T &field, const char *key, T spec);

void readSerial();

boolean valueOf(uint8_t const &pin, boolean status);

boolean valOfNPT(boolean value, unsigned long &time, const unsigned int &timeOut, boolean status);

template<typename T = boolean>
boolean hasUpdate(const char *key, T value);

boolean isValuesChanged();

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
      distance = 0;
      count = 0;
    } else if (line.equalsIgnoreCase("get")) {
      sendJson(serialPort, doc);
    }if (line.equalsIgnoreCase("getConfig")) {
      JsonDocument cf;
      cf["encoder"] = ENCODER_SCALE;
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
      updateConfig<double>(config, ENCODER_SCALE, "encoder", 1);
      updateConfig<double>(config, RPM_SCALE, "rpm", 1);
      updateConfig<uint>(config, NT_DELAY_TIME, "nt_time", 0);
      updateConfig<uint>(config, NP_DELAY_TIME, "np_time", 0);
      JsonDocument cf;
      cf["encoder"] = ENCODER_SCALE;
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
  pinMode(PIN_PHASE_A, INPUT_PULLUP);
  pinMode(PIN_PHASE_B, INPUT_PULLUP);
  pinMode(PIN_RPM, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_PHASE_A), attachPhaseA, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_PHASE_B), attachPhaseB, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), attachRPM, RISING);
  pinMode(PIN_PHASE_A, INPUT_PULLUP);
  pinMode(PIN_PHASE_B, INPUT_PULLUP);
  pinMode(PIN_RPM, INPUT_PULLUP);
  Serial1.begin(115200);
  Serial2.setTX(GPS_TX);
  Serial2.setRX(GPS_RX);
  Serial2.begin(9600);
  Serial.begin(115200);
  isValuesChanged();
}

void attachPhaseA() {
  if (PHASE_B) {
    count += 1;
    forward = true;
  }
}

void attachPhaseB() {
  if (PHASE_A) {
    count += 1;
    forward = false;
  }
}

unsigned int rpmCount = 0;
void attachRPM() {
  rpmCount += 1;
}

boolean valueOf(uint8_t const &pin, boolean status = false) {
  if (digitalRead(pin) == status) {
    delay(20);
    if (digitalRead(pin) == status) {
      return true;
    }
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

boolean isValuesChanged() {
  boolean changed = false;
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
  if (hasUpdate("status", status)) {
    changed = true;
  }
  if (hasUpdate("distance", distance)) {
    changed = true;
  }
  if (hasUpdate("speed", speed)) {
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

unsigned long checkDistanceTime = millis();
unsigned long checkRPMTime = millis();
unsigned long checkSendTime = millis();
void loop() {
  if (isTimeOut(checkDistanceTime, 500)) {
    int x = count;
    count = 0;
    double currDistance = x / ENCODER_SCALE;
    speed = x * 7.2 / ENCODER_SCALE;  //
    if (currDistance == 0) {
      distance = 0;
      status = STOP;
    } else if (forward) {
      distance = currDistance;
      status = FORWARD;
    } else {
      distance = currDistance * -1;
      status = BACKWARD;
    }
  }
  if (isTimeOut(checkRPMTime, 500)) {
    rpm = rpmCount/RPM_SCALE * 120;
    // Serial.println(rpmCount);
    rpmCount = 0;
    // temp = ADC_TEMP * 0.08;
    // rpmV =
  }
  if (isValuesChanged() || isTimeOut(checkSendTime, 500)) {
    sendJson(Serial, doc);
    sendJson(Serial1, doc);
    distance = 0;
    checkSendTime = millis();
  }
  readSerial(Serial);
  readSerial(Serial1);
  readGPS();
}