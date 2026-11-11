# 작업 대기열 (총괄이 유지)

상태: 초안 · 2026-11-07 밤 다시 씀 (GUI 세션 = 총괄이 단계마다 갱신)

> 대화가 압축되거나 세션이 바뀌어도 이 문서부터 읽고 이어 간다. 사용자 결정이 필요한 것은 "사용자 결정 대기"에 쌓고, 그 밖의 일은 기다리지 않고 진행한다. 2026-11-07 새벽 큰 지시(D1~D6)는 결정 0043·0044·0045·0046으로 모두 정해졌고 원문은 git 기록에 있다.

## 지금 순서 (2026-11-07 밤 사용자 지시)
1. 실물 시험 마무리 — 완료(`measurements.md` 2026-11-07 두 절).
2. 전체 총정리(코드·문서·PR·브랜치·폴더) — 완료: 합쳐진 브랜치 정리, 임시·팔 worktree 정리, 바탕화면은 세 폴더만.
3. 새 세션용 문서화 — 이 문서·`project-status.md`·`gui-dev-status.md`·절차서 갱신.
4. 기획 세션에 총정리 보고(사용자가 붙여 넣을 프롬프트, 총괄이 만든다).
5. 기획 세션 레퍼런스 — 끝: EGM 플러그인 사용 규칙(결정 0048, #206). 전체 시험 한 번 뒤 압축.
6. 압축 뒤 첫 작업: 결정 0047 GUI 구현(`gui --share`, 원격 브라우저 조작권 거부, 운영자 화면 E-STOP 항상). 설계서 새 절을 구현하기 전에는 결정 0048대로 새로 연 짧은 세션에서 `/egm:egm-review` Stage 9.

## 2026-11-11 인계(리뷰 준비 세션 → 총괄): MuJoCo부터 총괄이 맡는다
1. 진행 중: 팔 세션 #241 후속(`so101-studio-arm41`, `dev/arm-req-056-followup`). A 수동 데모 기본 물체 높이(먼저), B 병렬 그리퍼 height_required 제외, C 상승 관절 이동 휘두름(안전), _scan_opening 상승 검사를 need_lift로, 다시 CHECK가 empty면 재시도 안 함, 수동 height_required는 재시도·실패 셈 안 함, 작은 것 2건. PR이 오면 code-review 뒤 합친다.
2. 정리(MuJoCo 전, 사용자 요청 "문서·폴더·git 다 정리"): 후속 PR 뒤 `so101-studio-arm41` worktree 제거, main에 합쳐진 원격·로컬 브랜치 삭제(`claude/hopeful-hamilton-amqaac`·`planning-inbox` 유지), `so101-studio-us`를 최신 main으로, `so101-studio`의 `.playwright-mcp/`·`.claude/skills/playwright-cli/` 처리, `runs/handoff-20261110.md`와 메모리 handoff 삭제, 상태 문서 갱신. 지우는 것은 목록을 보이고 사용자 확인.
3-1. MuJoCo 진행 순서(사용자 지시 11-11): 기획이 EGM K1~K9를 반영(85845e5) → 사용자가 Stage 9 한 번 더 → 꼭 고칠 것이 없거나 기획이 반영하면 `claude/hopeful-hamilton-amqaac`를 합친다 → REQ-056 후속(#244) 뒤 팔 세션에 M1부터 REQ(REQ-058~). GUI REQ 두 개(총괄 몫): ① M2와 같이 `gui/__main__`·`app`·`backend`에 `--sim-engine`·`--sim-scenario`·`--sim-viewer` 전달, `make_sim_driver` 사용 ② M4에서 `sim_state` 물체 표시와 `physical_objects`일 때 `no_object`를 실패로(`backend.py`·`programs.py`). `Studio`의 같은 규칙은 팔 M3. 정리 목록과 남은 질문(#244 배출 자리 규칙 등)은 기획 세션에 묻는다.
3. MuJoCo: 기획이 결정 0053·arm-software-spec 13절·M1~M4를 `claude/hopeful-hamilton-amqaac`(9636ce2)에 올렸다. 사용자가 새 세션에서 EGM Stage 9(13절, M1~M4). 꼭 고칠 것이 있으면 기획 반영 뒤, 없으면 PR로 합치고 M1부터 팔 REQ(REQ-058~)를 낸다. 요지: `--sim-engine {kinematic,mujoco}`, `make_sim_driver` 한 곳, SimDriver 메서드 13개 + `sim_state()`, 2ms·write_joints당 10스텝, menagerie `robotstudio_so101`, 완료 기준 13절 (a)~(e).
4. 실물: runbook 2-2(놓기 재시험)는 사용자가 팔 옆에 있을 때. 넓힌 관절 한계로 처음 움직이니 30% 이하.

## 사용자 결정 대기
| # | 내용 | 메모 |
|---|---|---|
| U1 | ~~결정 0047 승인~~ 끝(#202) | GUI 구현 남음 |
| U2 | 놓기 모양: 11-10 재시험 영상에서 한쪽만 벌어짐 → 11.3 대칭 V자 놓기로 개정(#223, EGM Stage 9 반영) | REQ-053 합쳐짐(#229, 11-11) → 실물 2-2 다시(리뷰 준비 세션이 사용자와) |
| U3 | ~~서보 6개 동시 풀림 대책~~ 11-10 확인 시험 통과(풀림 0, 최저 11.6V, #225) | 다시 풀리면 `hardware.md` 판정별 단계. 보강 3(어댑터) 구매는 사용자 |
| U4 | OpenRB VIN 12V 여부, STEP 파일, 작업대 조립 일정 | 사용자·하드웨어 담당 |

## 세션별 다음 일
| 세션 | 다음 일 | 기다리는 것 |
|---|---|---|
| 팔 | REQ-053(대칭 V자 놓기) `--sim`까지 | 실물 2-2는 합친 뒤 사용자와 |
| GUI(총괄) | 실물 없이 할 일은 끝. 남은 것: 마법사 2차 끝(#228), 장면 부품 편집기(STEP 뒤), 초음파 화면 실제 기록(B1 때), MuJoCo(마지막). 끝: 0047(#212), 마법사 1차(#215), known_objects(#217), 3D 다시 그리기(#218) | `gui-dev-status.md` "남은 GUI 일" |
| 초음파 | 부품 도착 뒤 B1~B4 → R1 → A0·A1 → I1 | 부품, OpenRB VIN 확인 |

## 실물로만 할 일
순서와 조건은 `measurements.md` "실물 시험 순서": 1 놓기 재시험(REQ-053 뒤) → ~~2 전원~~ ~~3 AC4·#41~~(11-10 끝) → 4 작업대 조립 뒤 runbook 2-A → 5 병렬 그리퍼 → 6~8 초음파. 9 남은 G10 실물은 생략(사용자).

## 열린 PR·브랜치
- 11-11 사용자 결정: `physical_limits_deg` 제안값 사용(REQ-054), 배출 구역은 반경 90~250mm에서 바꿀 수 있게, 약 200mm보다 먼 넓은 물체의 ③fixed_tilt는 구조 한계로 받아들임.
- 11-11 합침: #233(0° 확인 되돌아가기·0° 잠금에서 home 허용, 결정 0029 보완 승인), #228(마법사 2차), #236(REQ-055). 기획 요청 2건(결정 0029 보완 기록, 높이 모를 때 수직 집기)은 사용자가 기획 세션에 붙여 넣음.
- 아직 물을 것: 호버 높이를 approach_height_mm보다 낮게 쓰는 것(REQ-053). 실물: runbook 2-2(놓기, 새 관절 한계·table_z 확인 포함), 마법사 저장·0° 확인 첫 실물.
- `planning-inbox`: 기획 요청함 기록용, 합치지 않음.
