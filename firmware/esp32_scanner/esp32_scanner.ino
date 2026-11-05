// ESP32 스캐너 펌웨어 (U5). 규약: docs/planning/ultrasonic-spec.md 7절 전부.
// hello, sync, sweep(서보 MG90S, GPIO 13), stream(팔 센서), stop, servo, once, gap.
// 센서 전원은 ESP32 5V, ECHO는 1k/2k 분압. 외부 라이브러리 없음(서보는 LEDC PWM 50Hz).
// 측정 줄의 "us"는 그 측정의 트리거 시각(µs)이다. 팔 위치와 맞출 때 메아리 왕복의 절반(최대 약 3.5ms)이 더해진다.
#include <Arduino.h>

const char *FW = "0.2";
const int TRIG_R = 25, ECHO_R = 26;  // 레이더
const int TRIG_A = 32, ECHO_A = 33;  // 팔
const int SERVO_PIN = 13;
const int SERVO_MIN_US = 500, SERVO_MAX_US = 2400;  // MG90S 펄스 폭
const uint32_t SERVO_START_MS = 100;  // 회전 시작 때 서보가 시작 각도까지 가는 시간

static inline bool reached(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }

uint32_t gapMs = 33;  // 어느 센서든 앞 측정과의 최소 간격 (gap 명령으로 변경)
uint32_t lastPingMs = 0;
uint32_t notBeforeMs = 0;  // 서보 이동 등으로 다음 측정을 미루는 시각

enum Mode { IDLE, SWEEP, STREAM };
Mode mode = IDLE;

// once
int onceLeft = 0, onceIdx = 0;
String onceK;
uint32_t onceMax = 600;
// sweep
int swFrom = 10, swTo = 170, swStep = 4, swAngle = 10, swDir = 1, swLast = 170;
bool swBounce = true;
long swLeft = 0;  // 한 방향 회전 횟수, 0이면 stop까지
uint32_t swMax = 600;
// stream
uint32_t strPeriodMs = 33, strMax = 300, strLast = 0;

void servoWrite(int deg) {
  deg = constrain(deg, 0, 180);
  uint32_t us = SERVO_MIN_US + (uint32_t)deg * (SERVO_MAX_US - SERVO_MIN_US) / 180;
  ledcWrite(SERVO_PIN, us * ((1u << 14) - 1) / 20000);
}

// 한 번 재서 mm를 돌려준다. 응답 없음(또는 ECHO가 LOW로 안 돌아옴) -1. trigUs = 트리거 시각.
// 메아리가 없으면 ECHO가 약 38ms HIGH로 남을 수 있어 LOW를 먼저 기다린다.
int ping(char k, uint32_t maxMm, uint32_t &trigUs) {
  int trig = k == 'r' ? TRIG_R : TRIG_A, echo = k == 'r' ? ECHO_R : ECHO_A;
  uint32_t w = millis();
  while (digitalRead(echo) == HIGH && millis() - w < 60) delayMicroseconds(100);
  trigUs = micros();
  if (digitalRead(echo) == HIGH) { lastPingMs = millis(); return -1; }
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  trigUs = micros();
  lastPingMs = millis();  // gap은 트리거 시각부터 센다
  uint32_t timeoutUs = maxMm * 2000UL / 343 + 2500;  // 왕복 시간 + 메아리 시작 대기 여유 (ESP32 pulseIn은 이 대기도 제한에 포함)
  uint32_t us = pulseIn(echo, HIGH, timeoutUs);
  if (us == 0) return -1;
  int mm = (int)(us * 343UL / 2000UL);
  return mm > (int)maxMm ? -1 : mm;
}

void emitMeasure(char k, int mm, uint32_t us, int angle, bool hasAngle) {
  Serial.print("{\"t\":\""); Serial.print(k); Serial.print("\"");
  if (k == 'r') { Serial.print(",\"a\":"); if (hasAngle) Serial.print(angle); else Serial.print("null"); }
  Serial.print(",\"d\":");
  if (mm < 0) Serial.print("null"); else Serial.print(mm);
  Serial.print(",\"us\":"); Serial.print(us); Serial.println("}");
}

// 한 줄 JSON에서 정수 값 꺼내기 (없으면 def)
long num(const String &s, const char *key, long def) {
  int i = s.indexOf(String("\"") + key + "\"");
  if (i < 0) return def;
  i = s.indexOf(':', i);
  return i < 0 ? def : s.substring(i + 1).toInt();
}

// 값을 숫자 문자열 그대로 꺼낸다(sync의 pc는 64비트 ns라 정수로 바꾸지 않고 그대로 돌려준다)
String rawNum(const String &s, const char *key) {
  int i = s.indexOf(String("\"") + key + "\"");
  if (i < 0) return "0";
  i = s.indexOf(':', i);
  if (i < 0) return "0";
  int j = i + 1;
  while (j < (int)s.length() && s[j] == ' ') j++;
  int e = j;
  while (e < (int)s.length() && (isdigit(s[e]) || s[e] == '-')) e++;
  return e > j ? s.substring(j, e) : String("0");
}

String str(const String &s, const char *key) {
  int i = s.indexOf(String("\"") + key + "\"");
  if (i < 0) return "";
  i = s.indexOf('"', s.indexOf(':', i));
  int j = s.indexOf('"', i + 1);
  return (i < 0 || j < 0) ? "" : s.substring(i + 1, j);
}

void sweepEnd() {
  Serial.print("{\"t\":\"end\",\"dir\":"); Serial.print(swDir); Serial.print(",\"us\":"); Serial.print(micros()); Serial.println("}");
  if (swBounce) { swDir = -swDir; swAngle = swDir < 0 ? swLast : swFrom; }
  else {  // 시작 각도로 크게 되돌아간다: 도착할 때까지 측정을 미룬다(MG90S 60°당 약 0.1초에 여유)
    swAngle = swFrom;
    notBeforeMs = millis() + (uint32_t)(swLast - swFrom) * 100UL / 60 * 3 / 2 + 30;
  }
  if (swLeft > 0 && --swLeft == 0) mode = IDLE;
  if (mode == SWEEP) servoWrite(swAngle);
}

void handle(const String &line) {
  String cmd = str(line, "cmd");
  uint32_t now = millis();
  if (cmd == "hello") {
    Serial.print("{\"t\":\"hello\",\"fw\":\""); Serial.print(FW);
    Serial.println("\",\"servo\":\"pwm\",\"k\":[\"r\",\"a\"]}");
  } else if (cmd == "sync") {
    Serial.print("{\"t\":\"sync\",\"pc\":"); Serial.print(rawNum(line, "pc"));
    Serial.print(",\"us\":"); Serial.print(micros()); Serial.println("}");
  } else if (cmd == "gap") {
    gapMs = constrain(num(line, "ms", 33), 5, 1000);
    Serial.print("{\"t\":\"gap\",\"ms\":"); Serial.print(gapMs); Serial.println("}");
  } else if (cmd == "once") {
    onceK = str(line, "k");
    onceLeft = (onceK.length() > 0) ? constrain(num(line, "n", 5), 1, 1000) : 0;
    onceIdx = 0;
    onceMax = num(line, "max_mm", 600);
    if (onceLeft == 0) Serial.println("{\"t\":\"err\",\"msg\":\"once needs k\"}");
  } else if (cmd == "sweep") {
    swFrom = constrain(num(line, "from", 10), 0, 180);
    swTo = constrain(num(line, "to", 170), 0, 180);
    swStep = max(1L, num(line, "step", 4));
    swBounce = line.indexOf("\"bounce\":false") < 0;
    swLeft = max(0L, num(line, "sweeps", 0));
    swMax = num(line, "max_mm", 600);
    if (swTo < swFrom) { int t = swFrom; swFrom = swTo; swTo = t; }
    swLast = swFrom + ((swTo - swFrom) / swStep) * swStep;
    swAngle = swFrom; swDir = 1;
    servoWrite(swAngle);
    notBeforeMs = now + SERVO_START_MS;
    mode = SWEEP;
  } else if (cmd == "stream") {
    long hz = constrain(num(line, "hz", 30), 1, 100);
    strPeriodMs = max((uint32_t)(1000 / hz), gapMs);
    strMax = num(line, "max_mm", 300);
    strLast = now - strPeriodMs;
    mode = STREAM;  // 팔 센서 우선: 레이더 회전은 멈춘다
  } else if (cmd == "stop") {
    mode = IDLE;
    onceLeft = 0;
  } else if (cmd == "servo") {
    servoWrite(num(line, "deg", 90));
  } else {
    Serial.println("{\"t\":\"err\",\"msg\":\"unknown cmd\"}");
  }
}

void setup() {
  Serial.begin(921600);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(TRIG_A, OUTPUT); pinMode(ECHO_A, INPUT);
  ledcAttach(SERVO_PIN, 50, 14);
  servoWrite(90);  // 대기 자세(정면)
}

String buf;
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { handle(buf); buf = ""; }
    else if (c != '\r' && buf.length() < 200) buf += c;
  }
  uint32_t now = millis();
  if (!reached(now, lastPingMs + gapMs) || !reached(now, notBeforeMs)) return;
  uint32_t us;
  if (onceLeft > 0) {  // 실측용: 센서 번갈아 n회
    char k = onceK[onceIdx];
    int mm = ping(k, onceMax, us);
    emitMeasure(k, mm, us, 0, false);
    if (++onceIdx >= (int)onceK.length()) { onceIdx = 0; onceLeft--; }
    if (onceLeft == 0) Serial.println("{\"t\":\"done\"}");
  } else if (mode == STREAM) {
    if (!reached(now, strLast + strPeriodMs)) return;
    strLast = now;
    int mm = ping('a', strMax, us);
    emitMeasure('a', mm, us, 0, false);
  } else if (mode == SWEEP) {
    int a = swAngle;
    int mm = ping('r', swMax, us);
    emitMeasure('r', mm, us, a, true);
    swAngle += swDir * swStep;  // 다음 각도를 바로 명령해 두면 서보가 간격 동안 도착해 멈춰 있다
    if (swAngle < swFrom || swAngle > swLast) sweepEnd();
    else servoWrite(swAngle);
  }
}
