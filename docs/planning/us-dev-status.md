# 초음파 세션 현황

상태: 초안 · 2026-11-05

## 작업 진행
| # | 작업 | 상태 |
|---|---|---|
| U0 | 실측용 펌웨어 `firmware/esp32_scanner`, `perception/ultrasonic/bench.py` | 코드 작성, 펌웨어 컴파일·실물 미확인(Arduino CLI 없음). 사용자와 B1~B3 대기 |
| U2 | model, fake_esp32, radar, check, source, link, config | 대기 |
| U5 | 펌웨어 7절 전부 | 대기(서보 11/12 예정) |
| U6·U7·U9 | 총괄이 보냄 | - |

## 규약 추가(실측용)
- `{"cmd":"gap","ms":N}`: 측정 간격 변경 → `{"t":"gap","ms":N}`. 설계서 7절에 없어 정식 규약에 넣을지 기획 확인 필요.
- `once`의 `k`는 `"r"`, `"a"`, `"ra"`(번갈아 n회씩), `max_mm` 선택. 끝나면 `{"t":"done"}`.

## 실측 절차 (U0, 서보 없음)
실물 시험 전체 순서는 `measurements.md`의 "실물 시험 순서". 배선: 레이더 TRIG 25/ECHO 26, 팔 TRIG 32/ECHO 33, ECHO는 1k/2k 분압(5V→3.3V), 센서 전원 ESP32 5V. 펌웨어는 Arduino IDE 2에서 ESP32 보드로 올린다(업로드 중 시리얼 모니터는 닫는다).
- B1: 센서를 110mm 높이에 아래로 고정, 40mm 상자를 옆에서 5mm씩 밀어 넣는다. 자리마다 `bench --k a --n 20 --gap 60 --label B1-x<mm>`, 거리가 바뀌는 수평 거리를 기록(앞뒤·좌우).
- B2: 센서 20mm 높이 수평(`--k r`), 200mm 앞 상자를 5°씩 돌리며 자리마다 `--gap 60 --n 10 --label B2-<deg>`, 응답 없음이 늘어나는 각도를 기록.
- B3: 고정 물체 앞에 두 센서를 서로 비스듬히 두고 `bench --k ra --n 200 --gaps 20,25,29,33,40 --label B3`. 출력의 간격별 엉뚱한 값 비율이 낮아지는 최소 간격이 `ping_gap_ms`.
- 결과 CSV는 `runs/bench/`(git 제외). 값만 총괄에게 넘겨 `measurements.md`에 적는다.

## 측정
(없음)

## 보낸 요청
(없음)
