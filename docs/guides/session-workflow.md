# 세션 역할과 협업 방법

상태: 확정 · 갱신 2026-11-17 기획 브랜치 이름(5·7절) · 2026-11-14 자율 진행·사후 보고(7절, 사용자 지시) · 2026-11-14 새 PC 절차 점검(2절 폴더·8절 캐시 복사·시험 순서·시작 프롬프트) · 2026-11-14 밤(세션별 모델) · 2026-11-14(합친 뒤 브랜치 지우기 순서) · 2026-11-06

근거: `decisions/0024`, `0036`(로컬 세션 셋, GUI 세션 총괄). 요청 기록은 `docs/planning/session-requests.md`에 남긴다.

## 1. 세션 다섯 개 (기획 1 + 로컬 4, MuJoCo 세션은 2026-11-12 추가)
| 세션 | 위치 | 하는 일 | 고칠 수 있는 곳 | 기준 문서 |
|---|---|---|---|---|
| 기획 (Fable) | 원격 (claude.ai) | 기획, 아키텍처, 레퍼런스 조사, 결정 기록, 설계서 | `docs/` | 전체 |
| 초음파 (Sonnet) | 로컬, 폴더 `so101-studio-us` | 초음파 인식 코드, 가상 ESP32, ESP32 펌웨어 | `src/so101_studio/perception/ultrasonic/`, `firmware/`, `tests/perception/`, `config/ultrasonic.yaml` | `planning/ultrasonic-spec.md` |
| GUI (Opus, 로컬 총괄) | 로컬, 폴더 `so101-studio-gui` | GUI 앱과 시뮬레이션 화면. 총괄: 일 나누기, 요청 대장, Sonnet 세션 PR 검토, 기획 요청 모으기 | `src/so101_studio/gui/`, `tests/gui/`, `pyproject.toml`의 `[project.optional-dependencies] gui` 줄 | `planning/gui-spec.md` |
| 팔 (Sonnet) | 로컬, 폴더 `so101-studio` | 팔, 서비스, 작업 실행, 인식, CLI | 그 밖의 `src/`, `config/`, `tests/`, `so101.cmd` | `planning/arm-software-spec.md`, `planning/system-architecture.md` |
| MuJoCo (Opus, 2026-11-12 추가, 11-16 Opus로) | 로컬, 폴더 `so101-studio-sim` | 물리 시뮬레이션과 시험장(`mujoco-spec.md`, M1~M4 끝, M5~ 결정 0062). 하드웨어를 기다리는 동안의 정식 작업이고, 실물 일이 생기면 그쪽이 먼저다 | `src/so101_studio/arm/mujoco_*.py`, `assets/mujoco/`, `config/sim_scenarios/`, `tests/test_mujoco_*.py`. 13.1 표의 공유 파일(`arm/sim_driver.py`, `cli/__init__.py`, `service/diagnostics.py`, `service/arm_service.py`)은 그 표에 적힌 줄만 고치고, PR 본문에 적어 팔 세션과 겹치지 않게 총괄이 순서를 정한다 | `planning/mujoco-spec.md` |

- **모델 지금 배정**(2026-11-16, 사용자): 기획 Fable(한도가 닳으면 사용자가 정함), 총괄(GUI) Opus, **MuJoCo Opus**(하드웨어 파일 변환·좌표 맞추기와 M5 구현이 몰림), 팔 Sonnet(범위가 정해진 REQ와 runbook 실물 일. 하드웨어 반영으로 URDF 바닥을 바꿀 때는 Opus로 올림), 초음파 Sonnet.
- **기획 세션 모델**(2026-11-16): 기획도 Opus로 내린다. Fable로 올려 다시 볼 일은 `planning/project-status.md` 8절 "Fable로 다시 볼 것"에 모아 두고, 그 목록이 찰 때 한 번에 올린다.
- **모델**(2026-11-14 밤, 위 배정 전 기록): 기본은 기획 Fable, 총괄(GUI) Opus, 팔·MuJoCo·초음파 Sonnet. **Fable 사용량 초기화가 남아 있는 동안**은 총괄과 팔도 Fable로 올린다(총괄은 판단·검토, 팔은 로보나인 다시 맞추기와 실물 안전). 한도가 닳으면 팔 → Sonnet, 총괄 → Opus 순으로 내리고 기획은 끝까지 Fable이다. 새 PC 세팅처럼 기계적인 일은 Fable을 쓰지 않는다. Fable은 판단·점검·새 기능 방향 잡기에 쓴다: 전체 점검, 새 기능의 초기 설계, 총괄이 올리는 판단 요청(`[기획 요청]`). 구현은 그 영역 세션이 하고 총괄이 검토한다. 총괄이 Opus일 때도 무거운 검토(code-review medium 이상, EGM)는 하위 에이전트 모델을 Fable로 지정해 돌릴 수 있다. 총괄을 GUI와 따로 떼지 않는다(전달 단계만 늘어난다). 12/4 뒤에 다시 본다.
- 모든 세션은 `docs/planning/`의 확정 문서와 `docs/decisions/`를 근거로 일한다. 설계서는 기획 세션만 고친다.
- 로컬 세션은 자기 인수인계 문서만 고친다. GUI 세션은 `planning/gui-dev-status.md`, 팔 세션은 `planning/arm-dev-status.md`, 초음파 세션은 `planning/us-dev-status.md`(초음파 세션이 처음 만든다)이다. 실물 시험 순서와 측정값은 `planning/measurements.md`에 모은다.
- 요청 대장 `planning/session-requests.md`는 GUI 세션(총괄)이 관리한다. 어느 세션 사이의 요청이든 총괄이 번호를 매기고 상태를 갱신한다. 압축 뒤에는 대장과 현황 문서로 진행 상황을 되살린다.
- 사용자는 주로 총괄과 이야기한다. 총괄은 지시를 쪼개 각 세션에 보내고(3절 방법), 각 세션의 PR은 검토한 뒤 묻지 않고 합치고 사용자에게 사후 보고한다(7절).

## 2. 폴더와 브랜치 (같은 PC에서 로컬 세션 넷)
여러 세션이 한 폴더에서 브랜치를 바꾸면 서로의 작업이 엉킨다. git worktree로 폴더를 나눈다(2026-11-14 새 PC 기준. 예전의 `dev/gui-shell`·`dev/us-sim` 시작 브랜치와 `PYTHONPATH`를 손으로 주는 방법은 쓰지 않는다).

```bash
cd <폴더 넷을 둘 곳>/so101-studio        # 클론 원본 = 팔 폴더, 여기만 main을 체크아웃한다
git fetch origin
git worktree add ../so101-studio-gui --detach origin/main
git worktree add ../so101-studio-sim --detach origin/main
git worktree add ../so101-studio-us  --detach origin/main
```

- **`main`은 팔 폴더 하나만 체크아웃한다.** 나머지 세 폴더에서 `git switch main`은 "already checked out"으로 실패한다. 세 폴더를 최신 main으로 돌릴 때는 `git fetch origin && git switch --detach origin/main`을 쓴다(팔 폴더는 `git switch main && git pull --ff-only`). 일을 시작할 때 `git switch -c dev/<작업> origin/main`으로 브랜치를 딴다.
- **실행은 `so101.cmd`로 한다.** 런처가 그 폴더의 `src`를 `PYTHONPATH` 앞에 넣는다(#349). 그래서 editable 설치(`pip install -e .`)가 팔 폴더를 가리켜도 각 폴더의 코드가 돈다. `python -m so101_studio ...`를 직접 부를 때만 그 폴더에서 `PYTHONPATH=src`를 준다. 테스트(`python -m pytest`)는 `pyproject.toml`의 `pythonpath = ["src"]` 덕분에 그 폴더 코드를 쓴다.
- "바탕화면"은 폴더 넷을 둔 곳을 뜻한다. 바탕화면이 OneDrive로 동기화되는 PC(경로에 `OneDrive`가 들어감)면 `C:\so101\` 같은 동기화 밖 영문 경로에 둔다(`runs/`가 수백 MB이고 동기화가 파일을 잠근다).
- 작업 하나를 위해 worktree를 더 만들면(예: `so101-arm2`) PR이 합쳐진 뒤 지운다(`git worktree remove`).
- conda 환경 `so101`은 같이 쓴다. GUI 의존성(viser, nicegui)은 GUI 세션이 설치하고, 팔 세션은 쓰지 않는다.
- **합친 뒤 브랜치 지우기**(2026-11-14 사고): `gh pr merge`는 `--delete-branch` 없이 합친다. 이 플래그는 원격 브랜치뿐 아니라 그 브랜치를 체크아웃한 worktree 폴더까지 지워, 추적 안 한 파일(`runs/`, `.claude` 로컬 설정 등)이 되살릴 수 없게 사라진다(`so101-studio-sim`). 순서는 다음과 같다.
  1. 플래그 없이 합친다.
  2. 그 브랜치를 쓰는 폴더에서 `git status`로 추적 안 한 파일을 확인하고 최신 main으로 돌린다: 팔 폴더는 `git switch main && git pull --ff-only`, 나머지 세 폴더는 `git fetch origin && git switch --detach origin/main`(임시 worktree는 확인 뒤 `git worktree remove`).
  3. 원격 브랜치는 `git push origin --delete <브랜치>`로 따로 지운다.
- 각자 작업 단위마다 PR을 만들고, 총괄이 검토해 `main`에 합친다(7절). 상대 세션의 결과가 필요하면 `main`에 합쳐진 뒤 `git fetch origin && git merge origin/main`으로 받는다.

## 3. 요청 흐름 (GUI 세션 → 팔 세션)
1. GUI 세션이 팔 쪽에 필요한 것을 찾는다. 예: `Studio.step()`이 없다.
2. GUI 세션은 `gui/backend.py`에서 임시로 우회한다. 버튼을 끄거나 시뮬레이션에서만 동작하게 하고, 그 자리에 `# REQ-NNN 임시` 주석을 단다. 팔 코드는 고치지 않는다.
3. GUI 세션이 4절 양식으로 요청서를 써서 팔 세션에 보낸다.
   - 1순위: Claude Code의 세션 간 메시지. `ListAgents`로 팔 세션 이름을 찾아 `SendMessage`로 보낸다.
   - 안 되면: 사용자에게 요청서를 보여 주고, 사용자가 팔 세션에 붙여 넣는다.
4. 팔 세션은 요청을 `session-requests.md`에 한 줄로 기록하고 판단한다.
   - 기존 동작을 바꾸지 않고 함수만 더하는 요청: 바로 구현하고, 테스트한 뒤 PR을 올린다.
   - 기존 API를 바꾸거나 설계가 달라지는 요청: 멈추고 5절의 기획 요청으로 올린다. GUI 세션에는 "기획 확인 대기"라고 답한다.
5. 팔 세션이 완료 답장을 보낸다(4절 양식). PR이 `main`에 합쳐지면 GUI 세션은 `main`을 받아 임시 우회를 지우고, 대장 상태를 "반영됨"으로 바꿔 달라고 팔 세션에 알린다.

팔 세션이 GUI 쪽에 요청할 때도 같은 흐름을 쓴다. 예: 상태 필드가 바뀌어 화면 갱신이 필요한 경우.

## 4. 양식
요청서:
```
[REQ 요청] GUI 세션 → 팔 세션
제목:
필요한 곳: (페이지, 버튼, 동작)
원하는 인터페이스: 함수 이름, 인자, 반환값, 블로킹 여부, 어느 스레드에서 불리는지
기존 동작 변경: 없음(추가만) / 있음 → 기획 확인 필요
완료 기준: (어떤 테스트가 통과하면 되는지, --sim에서 확인하는 방법)
급한 정도: 막힘 / 임시 우회 중 (backend.py REQ-NNN)
```
완료 답장:
```
[REQ 완료] REQ-NNN
브랜치·PR:
사용법: (호출 예 한두 줄)
테스트: (추가한 테스트 이름, python -m pytest -q 결과)
남은 것·주의:
```

## 5. 기획 세션으로 올리는 경우 (2026-11-06 개정)
기존 API 변경, 새 모듈, 안전 정책 변경, 설계서와 다른 구현이 필요할 때는 로컬 세션이 멈추고 총괄(GUI 세션)에게 보낸다(7절). 기획 세션은 사용 한도 때문에 PR을 자동으로 보지 않는다. 흐름은 이렇다.
1. 총괄이 아래 양식으로 모아 **기획 세션에 붙여 넣을 프롬프트**로 사용자에게 준다. 한 번에 여러 건을 묶어도 된다.
2. 사용자가 그 프롬프트를 기획 세션에 붙여 넣는다.
3. 기획 세션은 설계서·결정 기록을 고쳐 **승인 대기 브랜치**(기획 세션의 `claude/*` 브랜치, 2026-11-17 Opus 기획 세션부터 `claude/determined-meitner-44rxlf`(그 전 `claude/lucid-mendel-3truea`는 main에 다 합쳐졌다). 계정이나 기획 세션이 바뀌면 이름이 바뀌므로 새 기획 세션은 첫 답에 자기 브랜치 이름을 알린다)에 커밋·푸시하고, 무엇을 바꿨는지와 커밋을 사용자에게 답한다. 합치지 않는다. 총괄에게 넘길 답("[기획 → 총괄] … 합쳐 줘")은 사용자가 그대로 복사해 붙여 넣도록 **코드 블록 하나**에 담는다(2026-11-19 사용자 지시). 문서만 바꾼 답은 `claude/*` 브랜치에 커밋 첫 줄을 "[기획 → 총괄] …"로 해서 푸시해도 된다. 총괄이 매시 확인해 합친다(2026-11-20 사용자 승인).
4. 사용자가 승인하면 총괄이 그 브랜치로 PR을 열어 `main`에 합치고, 해당 세션에 전한다. 사용자가 기획 세션에서 바로 합치라고 하면 기획 세션이 합친다.

요청함 PR #68은 2026-11-06까지의 기록으로 남긴다(합치지 않고 지우지 않음). 새 요청은 받지 않는다.
```
[기획 요청] 보낸 세션: GUI / 팔 / 초음파
무엇이 필요한가:
왜 지금 설계로는 안 되는가:
선택지와 각각의 영향:
관련 REQ, 파일, 커밋:
```

## 6. 하드웨어를 같이 쓸 때
- 실제 팔 COM 포트(PC마다 다름: 데스크톱 COM3, 노트북 COM6)와 ESP32 포트는 한 번에 한 프로그램만 열 수 있다. 실제 팔을 쓰기 전에 다른 세션에 "팔 포트 사용 시작"을 알리고, 끝나면 "팔 포트 반납"을 알린다.
- ESP32 포트는 초음파 세션이 주로 쓴다. 다른 세션이 쓰기 전과 뒤에 총괄에게 "ESP32 사용 시작/반납"을 알린다. 팔 확인(손목 센서) 실물 시험은 팔 포트와 ESP32를 같이 쓰므로 총괄이 순서를 정한다.
- GUI 세션은 `--sim`으로 개발하는 것이 기본이다. 실제 팔은 화면이 시뮬레이션에서 다 동작한 뒤에만, 속도 30% 이하로, 사람이 전원 스위치 옆에 있을 때 연결한다.

## 7. 자율 진행과 사후 보고, 질문은 총괄로 (결정 0036 보완, 2026-11-05 · 2026-11-14 사용자 지시)
**원칙(2026-11-14 사용자 지시: 묻는 횟수를 줄인다)**: 설계서·결정 기록 안에서 스스로 판단할 수 있는 일은 각 세션이 묻지 않고 끝까지 하고, 한 뒤에 총괄에 보고한다. 총괄도 같은 일은 사용자에게 묻지 않고 하고 사후에 묶어 보고한다. 미리 묻는 것은 아래 "사용자에게 묻는 것"뿐이다.

- **각 세션이 묻지 않고 하는 것**: 자기 영역의 구현 선택, `dev/` 브랜치 만들기·커밋·푸시·PR 열기·PR 고치기, 시험·`--sim` 실행, 자기 `*-dev-status.md`와 `measurements.md` 갱신, 받은 REQ 가운데 추가만 하는 것의 구현, 대장 상태 갱신.
- **사후 보고**(세션 → 총괄, `SendMessage` 한 번 + PR 본문): PR 번호, 한 것, 돌린 시험 명령과 결과(통과 수·실패 이름), 스스로 정한 것과 이유, 남은 것. 총괄의 답을 기다리지 않고 다음 일을 한다.
- **총괄 → 사용자 사후 보고**: 합친 PR, 스스로 정한 것, 다음 일을 일이 한 묶음 끝날 때 한 번에 짧게 보고하고 `work-queue.md`에 남긴다.
- **사용자에게 묻는 것(미리)**: 실제 하드웨어를 움직이거나 전원·펌웨어를 건드리는 일, 지우기(브랜치·기록·파일)와 강제 푸시, 구매·비용, 설계서와 다른 구현·새 API나 모듈·안전 정책 변경(기획 요청으로), 기획 승인 대기 브랜치 합치기(설계·결정이 바뀐다), 프로그램 설치와 계정 설정, 설계서·결정으로 답이 안 나오는 방향 선택.

팔·초음파·MuJoCo 세션은 사용자에게 직접 묻지 않는다. 위 "묻는 것"에 걸리면 총괄에 `SendMessage`로 보내고, 답을 기다리는 동안 할 수 있는 다른 일을 한다. 총괄만 사용자와 이야기한다.

| 일 | 누가 정하나 |
|---|---|
| 자기 `dev/` 브랜치 커밋·푸시, PR 열기 | 각 세션이 묻지 않고 한다. 연 뒤 총괄에게 PR 번호와 시험 결과를 알린다 |
| 설계서·결정 기록으로 답이 나오는 질문, 작업 순서, 설계서 범위 안의 구현 선택, 세션 사이 조율 | 총괄이 정해 답한다 |
| `main`에 합치기 | 총괄이 사용자에게 묻지 않고 합친다. 조건: 그 브랜치를 받아 관련 시험(결정 0064)과 `python -m ruff check src tests`(화면·동작이면 `--sim`도)를 직접 다시 돌려 통과, `code-review`(medium 이상)에 꼭 고칠 것이 없음, 아래 "사용자에게 묻는 것"에 걸리지 않음. 고칠 것이 있으면 작성 세션에 돌려보낸다. 합친 뒤 사용자에게 사후 보고한다 |
| 실제 하드웨어(팔 움직이기, ESP32 펌웨어 올리기, 서보·센서 전원 켜기) | 사용자가 옆에 있어야 한다. 총괄이 사용자에게 묻고 순서를 정한다 |
| 지우기(브랜치, 기록), 구매, 비용 | 총괄이 사용자에게 묻는다 |
| 설계서와 다른 구현, 새 API·모듈, 안전 정책 변경 | 총괄이 기획 세션에 붙여 넣을 프롬프트(5절)로 정리해 사용자에게 준다. 기획 세션의 승인 대기 브랜치는 사용자 승인 뒤 총괄이 PR로 합친다 |

- 총괄은 사용자에게 보고하거나 승인을 묻기 전에 `git fetch origin && git merge origin/main`을 하고, 기획 승인 대기 브랜치(`git log origin/main..origin/claude/determined-meitner-44rxlf`, 이름은 5절)에 합치지 않은 기획 커밋이 있는지 본다. 이미 합쳐졌거나 답이 난 항목은 질문에서 뺀다.
- 총괄은 사용자에게 물을 것을 모아 한 번에 묻는다. 번호를 매기고 항목마다 추천 답과 이유를 붙인다.
- 사용자의 답은 총괄이 해당 세션에 전하고, 요청 대장이나 현황 문서에 남긴다.
- Claude Code 앱이 띄우는 도구 권한 확인 창은 이 규칙과 별개다(앱 설정의 권한 모드로 정한다).

## 7a. 보고 체계 — 남아야 할 것은 git에 (2026-11-14)
세션끼리 주고받는 메시지와 사용자가 붙여 넣는 프롬프트는 편의일 뿐, 계정이나 세션이 바뀌면 사라진다. 다음 넷만 남는다고 본다.

| 무엇 | 남는 곳 | 누가 |
|---|---|---|
| 요청(REQ) | `planning/session-requests.md` 행 + 그 일을 한 PR 본문에 REQ 번호 | 보내는 쪽이 행을 더하고, 받는 쪽이 상태를 고친다 |
| 세션 → 총괄 보고 | PR 본문(한 것·시험 명령과 결과) + 자기 `*-dev-status.md` | 각 세션 |
| 총괄 → 기획 보고·요청 | `planning-inbox` 브랜치 `inbox/YYYY-MM-DD-제목.md`. 사용자는 기획에 "[기획 요청] … 원문: inbox/…"만 붙여 넣는다 | 총괄 |
| 결정 | 사용자 승인 → 기획이 `decisions/NNNN`을 쓰고 `work-queue.md` 결정 대기에서 지운다 | 기획 |
| 지금 상태·할 일 | `project-status.md`(기획, 큰 단계마다), `work-queue.md`(총괄, 매일) | 기획·총괄 |
- 메시지로 받은 내용 가운데 이어 가야 할 것은 받은 세션이 위 자리 중 하나에 적는다.
- 측정은 `measurements.md`, 실물 기록 원본은 `runs/`(git 밖)다.

## 8. 새 계정·새 PC로 옮길 때 (2026-11-06, 2026-11-14 같은 PC 계정 이전 보완)
대화 기억(자동 메모리 포함)은 계정과 PC를 따라오지 않는다. 이어 갈 사람과 세션은 **레포만** 보고 시작한다. 그래서 옮기기 전에 모든 것을 레포에 남긴다.

**옮기기 전 (각 세션)**
1. 하던 일을 안전한 지점에서 멈춘다. 끝나지 않은 일은 `dev/` 브랜치에 커밋·푸시하고 PR을 연다(합치지 않아도 된다). 실제 팔·ESP32는 토크를 끄고 포트를 닫는다.
2. 자기 현황 문서(`gui-dev-status.md`, `arm-dev-status.md`, `us-dev-status.md`, MuJoCo는 `mujoco-spec.md` 13.9) 맨 위에 "인수인계" 절을 둔다: 지금 브랜치와 PR, 하던 일과 다음 한 걸음, 막힌 것, 알려진 함정.
3. 총괄은 요청 대장과 `measurements.md` 실물 시험 순서를 최신으로 맞추고, `gui-dev-status.md` 인수인계 절에 로컬 세션 넷의 상태(폴더, 브랜치, 열린 PR, 다음 한 걸음, 막힌 것)를 요약한다. MuJoCo 세션은 `mujoco-spec.md` 13.9에 남긴다.
4. 기획은 `planning/project-status.md`를 갱신한다.
5. git에 없는 파일을 새 PC로 복사한다: `config/local.yaml`(PC별 포트·팔), `config/ultrasonic_baseline.json`(레이더 기준값, 없으면 새로 잰다), 필요한 `runs/` 기록. 서보 캘리브레이션은 `config/calibration/`(git)에 있고, 새 PC에서 처음 연결하는 순서는 `arm-hardware-setup.md`를 따른다.

**새 PC에서**
1. 개발 도구: Git for Windows, Miniforge, USB 시리얼 드라이버, Arduino IDE 2(또는 arduino-cli) + ESP32 보드 패키지 3.x (`tooling-setup.md` C-1).
2. 레포: `git clone https://github.com/donghyeon04/so101-studio.git` → `conda env create -f environment.yml`(lerobot·GUI·시험 도구까지 한 번에 설치한다. `pip install -e .[gui]`를 따로 하지 않는다. numpy를 pip로 다시 설치하지 않는다) → `conda activate so101` → `python -m playwright install chromium`(화면 시험·스크린샷용, 한 번). Miniforge는 `C:\miniforge3`처럼 한글이 없는 경로에 설치한다(사용자 폴더 이름이 한글이면 기본 위치를 쓰지 않는다. `so101.cmd`는 `C:\so101-tools\miniforge3`, `C:\miniforge3`, 사용자 폴더 순으로 찾는다).
3. worktree: 2절 명령대로 `../so101-studio-gui`, `../so101-studio-us`, `../so101-studio-sim`을 `--detach origin/main`으로(팔은 클론 원본 `so101-studio`). 이어 갈 브랜치가 있으면 `git worktree add ../so101-studio-gui <그 브랜치>`.
4. 확인(5분 안): `so101.cmd --sim status`, `so101.cmd doctor --no-probe`, `python -m ruff check src tests`, 빠른 묶음 `python -m pytest -q -m "not slow and not measure" --ignore=tests/gui --ignore=tests/gui_visual`. 전체 `python -m pytest -q`는 1시간 반이 넘으므로 여기서 돌리지 않고 총괄이 한 번 돌린다(아래 "새 PC로" 7).
5. CLAUDE.md의 Git Bash PATH 줄은 `<miniforge>` 자리에 그 PC의 Miniforge 경로를 넣어 쓴다(`so101.cmd`는 스스로 찾는다).
6. Claude Code 도구: `tooling-setup.md` A·C-2·C-4를 설치한다. 프로젝트 스킬(`.claude/skills/`)은 레포에 있어 따로 설치하지 않는다. 새 계정은 claude.ai 커넥터에서 GitHub를 연결하고 이 레포 접근 권한이 있어야 한다.
7. 세션을 만들고 각 세션에 아래 "시작 프롬프트"를 준다.

**새 PC로 옮길 때 (2026-11-14 밤 실제 순서)**
1. 옛 PC에서 총괄이 git 밖 파일을 바탕화면 `so101-이전-1114.zip`에 모았다(`README.txt`에 각 파일의 새 자리). 새 PC로 복사한다.
2. 새 PC: 위 "새 PC에서" 1~2(도구, 클론, 환경). MuJoCo는 `environment.yml`에 없으므로 `tooling-setup.md` C24의 고정 버전 줄로 따로 설치한다(안 하면 MuJoCo 시험은 건너뛰고 `--sim-engine mujoco`는 종료 코드 2).
3. 폴더 넷: 바탕화면(OneDrive 동기화면 `C:\so101\`, 2절)에 `so101-studio`(팔, 클론 원본)과 worktree `so101-studio-gui`·`so101-studio-sim`·`so101-studio-us`. 세 worktree는 2절 명령(`--detach origin/main`)으로 만들고, 일을 시작할 때 `dev/<작업>` 브랜치를 딴다(`main`은 팔 폴더에서만 체크아웃된다).
4. zip을 풀어 `README.txt`대로 넣는다(`runs/handoff-1114/` → GUI 폴더, 교시 기록 → 각 `config/sequences/`, `.claude/launch.json` → GUI 폴더). 팀 CAD 원본은 레포 밖 팀 드라이브에 둔다.
5. `so101.cmd`는 그 폴더의 `src`를 쓴다(#349). 각 폴더에서 `so101.cmd --sim status`와 `so101.cmd doctor --no-probe`로 확인한다.
6. 실물 팔(7의 전체 시험이 끝난 뒤에 보드를 꽂는다): 새 PC에는 LeRobot 캘리브레이션 캐시가 없다. **먼저 `config/calibration/so101_follower_01.json`을 `%USERPROFILE%\.cache\huggingface\lerobot\calibration\robots\so_follower\`로 복사한다**(폴더가 없으면 만든다). 복사하지 않으면 `doctor`의 캘리브레이션 대조는 레포 사본으로 통과하는데도 실제 연결은 "캘리브레이션 불일치"로 잠기고 GUI는 "has no calibration registered"로 켜지지 않는다. 그다음 `so101.cmd doctor`(서보 6/6, 대조 같음) → `so101.cmd status` → `so101.cmd --speed 20 home`(클램프·전원 스위치 옆). 0° 확인 기록(`calibration_checked`)은 팔 파일에 있어 git으로 따라오고 캘리브레이션 파일 해시와 맞으므로(#344), **새 PC라고 `calibrate --verify-zero`를 다시 해야 움직이는 것은 아니다.** 다시 하는 때는 캘리브레이션 파일이 바뀌었을 때(`--save` 뒤)와 로보나인으로 바꾼 뒤(U5)다.
7. 첫 일: 총괄이 합친 main 전체 시험(**보드를 꽂기 전에**, 다른 GUI·감시 프로세스를 끄고, 그동안 다른 세션은 `tests/gui`·`tests/gui_visual`을 돌리지 않는다) → EGM drift-report(`mujoco-spec.md`). 순서는 `work-queue.md` 맨 위.
8. Claude Code: 새 계정이면 `tooling-setup.md`로 플러그인·MCP를 맞추고, 세션 다섯을 아래 시작 프롬프트로 연다.

**같은 PC에서 계정만 바꿀 때**(2026-11-14): 폴더·`runs/`·추적 안 한 파일은 그대로 남는다. 잃는 것은 대화 기억과 세션 메모리뿐이다. 그래서 "옮기기 전" 1~4만 하고, 새 계정에서 `tooling-setup.md`(설치 세션)로 플러그인·MCP를 맞춘 뒤 세션 다섯을 시작 프롬프트로 연다. git 밖 파일 목록은 `project-status.md` 7절이다.

**시작 프롬프트**(새 세션의 첫 메시지. `<…>`만 채운다)
```
[기획(원격)] 너는 so101-studio의 기획 세션이다. 기획·설계서·결정 기록·레퍼런스 조사만 하고 프로젝트 코드는 쓰지 않는다. 한국어로 결론부터 짧게 쓴다. 판단 요청은 질문을 하나씩(소크라테스), 실행 요청은 바로 한다. 커밋은 승인 대기 브랜치(claude/*)에만 하고 합치기는 총괄이 한다. 답 끝에 총괄에게 붙여 넣을 문장을 준다. 먼저 git fetch origin && git merge origin/main 하고 CLAUDE.md, docs/planning/project-status.md, work-queue.md, docs/guides/session-workflow.md 7a절을 읽고 지금 상태를 세 줄로 알려 줘. 총괄 보고는 planning-inbox 브랜치 inbox/에 온다. 새 PC 첫 기획 일(Fable): ① 로컬 세팅 전에 session-workflow 8절·시작 프롬프트·work-queue 첫날 순서를 실제로 따라간다고 치고 빠진 단계·충돌·옛 경로를 찾아 고친다. ② 결정 0001~0064와 설계서 다섯을 함께 읽고 대체된 결정을 전제로 쓴 곳·숫자 불일치를 찾는다(꼭 고칠 것만 고치고 나머지는 목록으로). ①의 보정 중 세팅하는 사용자가 바로 알아야 할 것은 브랜치 커밋과 별도로 채팅에 붙여 넣을 문장으로 준다(합치기 전이라 새 PC의 main에는 아직 없다). 그다음 EGM drift-report 결과를 받아 mujoco-spec을 고친 뒤 M5·M6 세부 설계(로보나인 손 위)와 로보나인 레퍼런스 조사(손목 기본 방향 가로/세로, 남들 초기값)다.
```
```
[총괄(GUI)] 너는 so101-studio의 총괄(GUI) 세션이다. 폴더는 바탕화면 so101-studio-gui. gui/·tests/gui/만 고치고 gui-spec.md를 따르며, 일 나누기·REQ 대장·PR 검토(code-review)·합치기(--delete-branch 쓰지 않음)·EGM Stage 9를 맡는다. 먼저 git status -sb로 지금 위치를 보고(이 폴더는 worktree라 main을 체크아웃하지 못한다) 하던 브랜치가 없으면 git fetch origin && git switch --detach origin/main. 그다음 CLAUDE.md, docs/planning/project-status.md, work-queue.md, docs/guides/session-workflow.md(2·7a·8절), gui-dev-status.md 인수인계 절, session-requests.md를 읽고 work-queue "새 PC 첫날 순서"부터 이어 간다(기획 승인 대기 브랜치에 안 합친 커밋이 있으면 사용자 승인 뒤 PR로 먼저 합치기 → 합친 main 전체 시험(팔 보드를 꽂기 전에, 다른 세션의 GUI 시험과 겹치지 않게) → EGM drift-report mujoco-spec → CHECK 기준점 REQ). 기획 승인 대기 브랜치는 session-workflow 5절에 적힌 이름이다. 기획에 올릴 것은 planning-inbox inbox/에 파일로 쓴다.
```
```
[팔] 너는 so101-studio의 팔 세션이다. 폴더는 바탕화면 so101-studio. gui/·mujoco·초음파 영역 밖의 코드를 고치고 arm-software-spec.md를 따른다. 먼저 git status -sb(지워진 브랜치면 git switch main && git pull --ff-only, 이 폴더만 main을 체크아웃한다), CLAUDE.md, docs/planning/project-status.md, work-queue.md, arm-dev-status.md 인수인계 절, session-requests.md에서 팔 앞 REQ를 읽고 이어 간다. 첫 일은 새 PC에서 실물 팔 첫 연결(총괄의 전체 시험이 끝난 뒤, session-workflow 8절 "새 PC로" 6: 캘리브레이션 파일을 LeRobot 캐시에 복사 → doctor → 20% home. 0° 확인을 다시 할 필요는 없다)과 CHECK 경로 기준점 REQ(병렬은 두 턱 끝 가운데)다. 작업은 main에서 dev/<작업> 브랜치, 끝나면 PR. 실물은 30% 이하로 시작하고 사용자가 전원 옆에 있을 때만.
```
```
[MuJoCo] 너는 so101-studio의 MuJoCo 세션이다. 폴더는 바탕화면 so101-studio-sim. arm/mujoco_*, assets/mujoco/, config/sim_scenarios/, tests/test_mujoco_*를 고치고 docs/planning/mujoco-spec.md를 따른다(시험장, 결정 0062·0063). 먼저 git status -sb(하던 브랜치가 없으면 git fetch origin && git switch --detach origin/main. 이 폴더는 main을 체크아웃하지 못한다), python -c "import mujoco"가 되는지(안 되면 tooling-setup.md C24), CLAUDE.md, project-status.md, work-queue.md, mujoco-spec.md 13.9·13.10을 읽고 이어 간다. M4b(로보나인 손)는 끝났고, 첫 일은 병렬 손으로 13.7 (a)(b)·sim_blocked·부하 문턱을 다시 재는 것이다. 설계서와 다르게 해야 하면 멈추고 총괄에게 알린다.
```
```
[초음파] 너는 so101-studio의 초음파 세션이다. 폴더는 바탕화면 so101-studio-us. perception/ultrasonic/, firmware/, tests/perception/, config/ultrasonic.yaml만 고치고 ultrasonic-spec.md를 따른다. 먼저 git status -sb 하고 하던 브랜치가 없으면 git fetch origin && git switch --detach origin/main(이 폴더는 main을 체크아웃하지 못한다). CLAUDE.md, project-status.md, work-queue.md, us-dev-status.md 인수인계 절, session-requests.md에서 초음파 앞 REQ를 읽고 이어 간다. 센서 오프셋 {0.6, 66.5, 99.5}·ring 60은 #359로 들어갔다. 첫 일은 센서 높이 154.5mm에서 CHECK 높이·경로를 벤치마크로 다시 고르는 것(ultrasonic-spec 1절). OpenRB VIN 12V 확인 전에는 배선하지 않는다. 첫 일 전에 us-dev-status.md에 #359(오프셋·ring 60·센서 높이 약 154.5mm, B1 높이 110 → 약 155mm)를 반영해 dev/ 브랜치 PR로 낸다.
```

