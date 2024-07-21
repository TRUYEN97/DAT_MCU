
#include <ArduinoJson.h>

unsigned int ENCODER_SCALE = 1000;
unsigned int DETA_DISTANCE_TIME = 200;
unsigned int DETA_RPM_TIME = 500;
unsigned int DETA_SEND_TIME = 200;

int8_t const FORWARD = 1;
int8_t const BACKWARD = -1;
int8_t const STOP = 0;
uint8_t const PIN_PHASE_A = 2;
uint8_t const PIN_PHASE_B = 3;
uint8_t const PIN_PHASE_Z = 4;
uint8_t const PIN_RPM = 5;
uint8_t const T1_PIN = 6;
uint8_t const T2_PIN = 7;
uint8_t const T3_PIN = 8;
uint8_t const CM_PIN = 9;
uint8_t const PT_PIN = 10;
uint8_t const S5_PIN = 11;
uint8_t const S4_PIN = 12;
uint8_t const S3_PIN = 13;
uint8_t const S2_PIN = 14;
uint8_t const S1_PIN = 15;
uint8_t const NT_PIN = 16;
uint8_t const NP_PIN = 17;
uint8_t const AT_PIN = 18;
uint8_t const RELAY_1 = 21;
uint8_t const RELAY_2 = 22;
uint8_t const RELAY_3 = 26;
uint8_t const RELAY_4 = 27;

unsigned long count = 0;
double distance = 0;
double speed = 0;
unsigned int rpm = 0;
int8_t status = 0;
boolean forward = true;
JsonDocument doc;

#define PHASE_A digitalRead(PIN_PHASE_A)
#define PHASE_B digitalRead(PIN_PHASE_B)

void sendJson() {
  String jsonString;
  serializeJson(doc, jsonString);
  Serial.println(jsonString);
}

template<typename T = uint>
void updateConfig(const JsonDocument &config, T &field, const char *key, T spec) {
  if (config[key].isNull()) {
    return;
  }
  T value = config[key];
  if (value >= spec) {
    field = value;
  }
}

void readSerial() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.equalsIgnoreCase("roff")) {
      digitalWrite(RELAY_2, 0);
      digitalWrite(RELAY_1, 0);
      digitalWrite(RELAY_3, 0);
      digitalWrite(RELAY_4, 0);
    } else if (line.equalsIgnoreCase("r1")) {
      digitalWrite(RELAY_1, 1);
    } else if (line.equalsIgnoreCase("r2")) {
      digitalWrite(RELAY_2, 1);
    } else if (line.equalsIgnoreCase("r3")) {
      digitalWrite(RELAY_3, 1);
    } else if (line.equalsIgnoreCase("r4")) {
      digitalWrite(RELAY_4, 1);
    } else if (line.equalsIgnoreCase("reset")) {
      distance = 0;
      count = 0;
    } else if (line.equalsIgnoreCase("get")) {
      sendJson();
    } else if (line.charAt(0) == '{' && line.charAt(line.length() - 1) == '}') {
      JsonDocument filter;
      filter["encoder"] = true;
      filter["distance_udtime"] = true;
      filter["rpm_udtime"] = true;
      filter["senddt_udtime"] = true;
      JsonDocument config;
      deserializeJson(config, line, DeserializationOption::Filter(filter));
      updateConfig<uint>(config, ENCODER_SCALE, "encoder", 1);
      updateConfig<uint>(config, DETA_DISTANCE_TIME, "distance_udtime", 100);
      updateConfig<uint>(config, DETA_RPM_TIME, "rpm_udtime", 200);
      updateConfig<uint>(config, DETA_SEND_TIME, "senddt_udtime", 200);
      sendJson();
    }
  }
}

void setup() {
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  pinMode(RELAY_4, OUTPUT);
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
  pinMode(S5_PIN, INPUT_PULLUP);
  pinMode(PIN_PHASE_A, INPUT_PULLUP);
  pinMode(PIN_PHASE_B, INPUT_PULLUP);
  pinMode(PIN_PHASE_Z, INPUT_PULLUP);
  pinMode(PIN_RPM, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_PHASE_A), attachPhaseA, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_PHASE_B), attachPhaseB, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), attachRPM, RISING);
  pinMode(PIN_PHASE_A, INPUT_PULLUP);
  pinMode(PIN_PHASE_B, INPUT_PULLUP);
  pinMode(PIN_PHASE_Z, INPUT_PULLUP);
  pinMode(PIN_RPM, INPUT_PULLUP);
  Serial.begin(115200);
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

boolean valueOf(uint8_t const &pin, boolean status = true) {
  if (digitalRead(pin)) {
    delay(30);
    if (digitalRead(pin)) {
      return status;
    }
  }
  return !status;
}

#define CM valueOf(CM_PIN, false)
#define NT valueOf(NT_PIN, false)
#define NP valueOf(NP_PIN, false)
#define AT valueOf(AT_PIN, false)
#define PT valueOf(PT_PIN, false)
#define T1 valueOf(T1_PIN, false)
#define T2 valueOf(T2_PIN, false)
#define T3 valueOf(T3_PIN, false)
#define S1 valueOf(S1_PIN, false)
#define S2 valueOf(S2_PIN, false)
#define S3 valueOf(S3_PIN, false)
#define S4 valueOf(S4_PIN, false)
#define S5 valueOf(S5_PIN, false)

template<typename T = boolean>
boolean hasUpdate(const char *key, T value) {
  if (doc[key] != value) {
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
  if (hasUpdate("s5", S5)) {
    changed = true;
  }
  if (hasUpdate("status", status)) {
    changed = true;
  }
  if (hasUpdate("distance", distance)) {
    changed = true;
  }
  if (hasUpdate("speed", speed)) {
    doc["speed1"] = speed * 3.6;
    changed = true;
  }
  if (hasUpdate("rpm", rpm)) {
    changed = true;
  }
  return changed;
}

boolean isTimeOut(unsigned long &time, const unsigned int &timeOut) {
  unsigned long currentTime = millis();
  boolean rs = false;
  if (currentTime > time) {
    rs = currentTime - time >= timeOut;
  } else {
    rs = time - currentTime >= timeOut;
  }
  if (rs) {
    time = currentTime;
  }
  return rs;
}

unsigned long checkDistanceTime = millis();
unsigned long checkRPMTime = millis();
unsigned long sendJsonTime = millis();
void loop() {
  if (isTimeOut(checkDistanceTime, DETA_DISTANCE_TIME)) {
    double currDistance = count / (double)ENCODER_SCALE;
    count = 0;
    speed = currDistance * (1000 / DETA_DISTANCE_TIME);
    if (currDistance == 0) {
      status = STOP;
    } else if (forward) {
      distance += currDistance;
      status = FORWARD;
    } else {
      distance -= currDistance;
      status = BACKWARD;
      if (distance < 0) {
        distance = 0;
      }
    }
  }
  if (isTimeOut(checkRPMTime, DETA_RPM_TIME)) {
    rpm = rpmCount * (1000 / DETA_RPM_TIME) * 60;
    rpmCount = 0;
  }
  if (isValuesChanged() && isTimeOut(sendJsonTime, DETA_SEND_TIME)) {
    sendJson();
  }
  readSerial();
}