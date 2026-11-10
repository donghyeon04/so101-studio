// OpenRB-150 레이더 서보 펌웨어 (결정 0043, ultrasonic-spec 3.2·3.4·7절).
// ESP32가 Serial2(UART, 3.3V, TX/RX 교차, GND 공통)로 보낸 목표 각도를 XL330-M288-T에 쓰고, 실제 각도를 되돌려 준다.
// 줄 규약(ASCII, 115200): ESP32 -> "g <deg*10>\n" (서보 각도 0~1800 = 0.0~180.0°, 90° = 정면)
//                         OpenRB -> "p <deg*10>\n" (읽은 실제 각도, 약 5ms마다), 오류 "e <코드>\n"
// 전원: XL330은 5V(3.7~6.0V). OpenRB VIN에 12V를 넣지 않는다. DXL 포트 전원은 FET라 시작 때 켠다(핀 31).
// 라이브러리: Dynamixel2Arduino (Library Manager), 보드: OpenRB-150 (package_openrb_index.json).
#include <Dynamixel2Arduino.h>

#define DXL_BUS Serial1   // OpenRB DXL 포트 (방향 핀 자동)
#define LINK Serial2      // ESP32와 UART

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL = 2.0;
const uint32_t DXL_BAUD = 57600;     // XL330 출하 값. 바꿨다면 여기도 같이
const uint32_t LINK_BAUD = 115200;   // esp32_scanner.ino의 DXL_BAUD와 같게
const int32_t ZERO_TICKS = 2048;     // 서보 90°(정면)에 해당하는 XL330 눈금 (4096/회전). 조립 뒤 B4에서 맞춘다
const int DIR = 1;                   // +1 / -1: 눈금 증가 방향이 서보 각도 증가 방향인지
const uint32_t PROFILE_VEL_UNITS = 400;      // 0.229rpm 단위 -> 약 92rpm(550°/s). 4° 이동이 33ms 간격 안에 끝나는 값, B4로 고친다
const uint32_t PROFILE_ACC_UNITS = 0;    // 0 = 제한 없음
const uint32_t REPORT_MS = 5;

#ifndef BDPIN_DXL_PWR_EN
#define BDPIN_DXL_PWR_EN 31
#endif

Dynamixel2Arduino dxl(DXL_BUS);
using namespace ControlTableItem;

String buf;
uint32_t lastReportMs = 0;
bool ready = false;

int32_t degX10ToTicks(int deg10) {
  return ZERO_TICKS + (int32_t)DIR * (int32_t)((deg10 - 900) * 4096L / 3600L);
}

int ticksToDegX10(int32_t ticks) {
  return 900 + (int)(DIR * (ticks - ZERO_TICKS) * 3600L / 4096L);
}

bool setupServo() {
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);
  if (!dxl.ping(DXL_ID)) return false;
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, PROFILE_ACC_UNITS);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, PROFILE_VEL_UNITS);
  dxl.torqueOn(DXL_ID);
  dxl.setGoalPosition(DXL_ID, ZERO_TICKS);  // 정면
  return true;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);  // DXL 포트 전원 켜기
  delay(300);                            // 서보 부팅
  LINK.begin(LINK_BAUD);
  ready = setupServo();
  if (!ready) LINK.print("e 1\n");  // XL330 ping 실패
}

void handleLine(const String &line) {
  if (!ready || !line.startsWith("g ")) return;
  int deg10 = constrain(line.substring(2).toInt(), 0, 1800);
  dxl.setGoalPosition(DXL_ID, degX10ToTicks(deg10));
}

void loop() {
  while (LINK.available()) {
    char c = LINK.read();
    if (c == '\n') { handleLine(buf); buf = ""; }
    else if (c != '\r' && buf.length() < 20) buf += c;
  }
  uint32_t now = millis();
  if (now - lastReportMs < REPORT_MS) return;
  lastReportMs = now;
  if (!ready) { ready = setupServo(); if (!ready) { LINK.print("e 1\n"); delay(500); } return; }
  float pos = dxl.getPresentPosition(DXL_ID);  // 눈금(raw). 읽기 실패면 0을 돌려주므로 오류 코드를 같이 본다
  if (dxl.getLastLibErrCode()) { LINK.print("e 2\n"); return; }  // 응답 없음
  LINK.print("p "); LINK.print(ticksToDegX10((int32_t)pos)); LINK.print('\n');
  digitalWrite(LED_BUILTIN, (now / 250) & 1);
}
