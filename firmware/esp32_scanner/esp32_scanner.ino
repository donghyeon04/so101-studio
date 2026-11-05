// 실측용 최소 펌웨어 (U0). 규약: docs/planning/ultrasonic-spec.md 7절 중 hello/once/stream/stop + gap(sync는 U5에서).
// 서보 없음. 센서 전원은 ESP32 5V, ECHO는 1k/2k 분압. 외부 라이브러리 없음.
#include <Arduino.h>

const char *FW = "0.1-bench";
const int TRIG_R = 25, ECHO_R = 26;  // 레이더
const int TRIG_A = 32, ECHO_A = 33;  // 팔

uint32_t gapMs = 33;           // 어느 센서든 앞 측정과의 최소 간격 (명령 gap으로 변경)
uint32_t lastPingMs = 0;
bool streaming = false;
uint32_t streamPeriodMs = 33;
uint32_t streamMaxMm = 300;
uint32_t lastStreamMs = 0;

// 한 번 재서 mm를 돌려준다. 응답 없음(또는 ECHO가 LOW로 안 돌아옴) -1.
// 앞 트리거와 gapMs 이상 띄운다. 메아리가 없으면 ECHO가 약 38ms HIGH로 남을 수 있어 LOW를 먼저 기다린다.
int ping(char k, uint32_t maxMm) {
  int trig = k == 'r' ? TRIG_R : TRIG_A, echo = k == 'r' ? ECHO_R : ECHO_A;
  while (millis() - lastPingMs < gapMs) delay(1);
  uint32_t w = millis();
  while (digitalRead(echo) == HIGH && millis() - w < 60) delayMicroseconds(100);
  if (digitalRead(echo) == HIGH) { lastPingMs = millis(); return -1; }
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  lastPingMs = millis();  // gap은 트리거 시각부터 센다
  uint32_t timeoutUs = maxMm * 2000UL / 343 + 2500;  // 왕복 시간 + 메아리 시작 대기 여유 (ESP32 pulseIn은 이 대기도 제한에 포함)
  uint32_t us = pulseIn(echo, HIGH, timeoutUs);
  if (us == 0) return -1;
  int mm = (int)(us * 343UL / 2000UL);
  return mm > (int)maxMm ? -1 : mm;
}

void emit(char k, int mm) {
  Serial.print("{\"t\":\""); Serial.print(k); Serial.print("\",\"d\":");
  if (mm < 0) Serial.print("null"); else Serial.print(mm);
  Serial.print(",\"us\":"); Serial.print((uint32_t)micros()); Serial.println("}");
}

// 한 줄 JSON에서 숫자 값 꺼내기 (없으면 def)
long num(const String &s, const char *key, long def) {
  int i = s.indexOf(String("\"") + key + "\"");
  if (i < 0) return def;
  i = s.indexOf(':', i);
  return i < 0 ? def : s.substring(i + 1).toInt();
}

String str(const String &s, const char *key) {
  int i = s.indexOf(String("\"") + key + "\"");
  if (i < 0) return "";
  i = s.indexOf('"', s.indexOf(':', i));
  int j = s.indexOf('"', i + 1);
  return (i < 0 || j < 0) ? "" : s.substring(i + 1, j);
}

void handle(const String &line) {
  String cmd = str(line, "cmd");
  if (cmd == "hello") {
    Serial.print("{\"t\":\"hello\",\"fw\":\""); Serial.print(FW);
    Serial.println("\",\"servo\":\"none\",\"k\":[\"r\",\"a\"]}");
  } else if (cmd == "gap") {
    gapMs = constrain(num(line, "ms", 33), 5, 1000);
    Serial.print("{\"t\":\"gap\",\"ms\":"); Serial.print(gapMs); Serial.println("}");
  } else if (cmd == "once") {
    String k = str(line, "k");           // "r", "a", "ra"(번갈아)
    long n = constrain(num(line, "n", 5), 1, 1000);
    long maxMm = num(line, "max_mm", 600);
    for (long i = 0; i < n; i++)
      for (unsigned j = 0; j < k.length(); j++) emit(k[j], ping(k[j], maxMm));
    Serial.println("{\"t\":\"done\"}");
  } else if (cmd == "stream") {
    streamPeriodMs = 1000 / constrain(num(line, "hz", 30), 1, 100);
    streamMaxMm = num(line, "max_mm", 300);
    streaming = true;
  } else if (cmd == "stop") {
    streaming = false;
    Serial.println("{\"t\":\"done\"}");
  } else {
    Serial.println("{\"t\":\"err\",\"msg\":\"unknown cmd\"}");
  }
}

void setup() {
  Serial.begin(921600);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(TRIG_A, OUTPUT); pinMode(ECHO_A, INPUT);
}

String buf;
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { handle(buf); buf = ""; }
    else if (c != '\r' && buf.length() < 200) buf += c;
  }
  if (streaming && millis() - lastStreamMs >= streamPeriodMs) {
    lastStreamMs = millis();
    emit('a', ping('a', streamMaxMm));
  }
}
