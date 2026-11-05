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

## 측정
(없음)

## 보낸 요청
(없음)
