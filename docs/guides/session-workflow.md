# 세션 역할과 협업 방법

상태: 확정 · 갱신 2026-11-05

근거: `decisions/0024`, `0036`(로컬 세션 셋, GUI 세션 총괄). 요청 기록은 `docs/planning/session-requests.md`에 남긴다.

## 1. 세션 네 개 (기획 1 + 로컬 3)
| 세션 | 위치 | 하는 일 | 고칠 수 있는 곳 | 기준 문서 |
|---|---|---|---|---|
| 기획 | 원격 (claude.ai) | 기획, 아키텍처, 레퍼런스 조사, 결정 기록, 설계서 | `docs/` | 전체 |
| 초음파 (Sonnet) | 로컬, 폴더 `so101-studio-us` | 초음파 인식 코드, 가상 ESP32, ESP32 펌웨어 | `src/so101_studio/perception/ultrasonic/`, `firmware/`, `tests/perception/`, `config/ultrasonic.yaml` | `planning/ultrasonic-spec.md` |
| GUI (Opus, 로컬 총괄) | 로컬, 폴더 `so101-studio-gui` | GUI 앱과 시뮬레이션 화면. 총괄: 일 나누기, 요청 대장, Sonnet 세션 PR 검토, 기획 요청 모으기 | `src/so101_studio/gui/`, `tests/gui/`, `pyproject.toml`의 `[project.optional-dependencies] gui` 줄 | `planning/gui-spec.md` |
| 팔 (Sonnet) | 로컬, 폴더 `so101-studio` | 팔, 서비스, 작업 실행, 인식, CLI | 그 밖의 `src/`, `config/`, `tests/`, `so101.cmd` | `planning/arm-software-spec.md`, `planning/system-architecture.md` |

- 모든 세션은 `docs/planning/`의 확정 문서와 `docs/decisions/`를 근거로 일한다. 설계서는 기획 세션만 고친다.
- 로컬 세션은 자기 인수인계 문서만 고친다. GUI 세션은 `planning/gui-dev-status.md`, 팔 세션은 `planning/arm-dev-status.md`, 초음파 세션은 `planning/us-dev-status.md`(초음파 세션이 처음 만든다)이다. 실물 시험 순서와 측정값은 `planning/measurements.md`에 모은다.
- 요청 대장 `planning/session-requests.md`는 GUI 세션(총괄)이 관리한다. 어느 세션 사이의 요청이든 총괄이 번호를 매기고 상태를 갱신한다. 압축 뒤에는 대장과 현황 문서로 진행 상황을 되살린다.
- 사용자는 주로 총괄과 이야기한다. 총괄은 지시를 쪼개 각 세션에 보내고(3절 방법), Sonnet 세션의 PR은 합치기 전에 검토한 뒤 사용자에게 합칠지 묻는다.

## 2. 폴더와 브랜치 (같은 PC에서 두 세션)
두 세션이 한 폴더에서 브랜치를 바꾸면 서로의 작업이 엉킨다. git worktree로 폴더를 나눈다.

```bash
cd C:/Users/c/Desktop/so101-studio
git fetch origin
git worktree add ../so101-studio-gui -b dev/gui-shell origin/main
git worktree add ../so101-studio-us -b dev/us-sim origin/main
```

- `pip install -e .`은 원래 폴더를 가리킨다. GUI 폴더에서 실행할 때는 그 폴더의 코드가 쓰이도록 `PYTHONPATH`를 GUI 폴더의 `src`로 지정한다. PowerShell: `$env:PYTHONPATH="C:/Users/c/Desktop/so101-studio-gui/src"`. 실행 전에 `python -c "import so101_studio; print(so101_studio.__file__)"`가 GUI 폴더를 가리키는지 확인한다. 테스트(`python -m pytest -q`)는 `pyproject.toml`의 `pythonpath = ["src"]` 덕분에 그대로 GUI 폴더 코드를 쓴다.

- GUI 세션은 `../so101-studio-gui`에서, 초음파 세션은 `../so101-studio-us`에서, 팔 세션은 원래 폴더에서 실행한다. 초음파 폴더에서 실행할 때도 `PYTHONPATH`를 그 폴더의 `src`로 지정한다.
- conda 환경 `so101`은 같이 쓴다. GUI 의존성(viser, nicegui)은 GUI 세션이 설치하고, 팔 세션은 쓰지 않는다.
- 각자 작업 단위마다 PR을 만들고, 사용자가 승인하면 `main`에 합친다. 상대 세션의 결과가 필요하면 `main`에 합쳐진 뒤 `git fetch origin && git merge origin/main`으로 받는다.

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

## 5. 기획 세션으로 올리는 경우
기존 API 변경, 새 모듈, 안전 정책 변경, 설계서와 다른 구현이 필요할 때는 로컬 세션이 멈추고 사용자에게 아래 양식을 보여 준다. 사용자가 기획 세션에 가져오면, 기획 세션이 설계서나 결정 기록을 고친 뒤 다시 내려보낸다.
```
[기획 요청] 보낸 세션: GUI / 팔
무엇이 필요한가:
왜 지금 설계로는 안 되는가:
선택지와 각각의 영향:
관련 REQ, 파일, 커밋:
```

## 6. 하드웨어를 같이 쓸 때
- 실제 팔(COM3)과 ESP32 포트는 한 번에 한 프로그램만 열 수 있다. 실제 팔을 쓰기 전에 다른 세션에 "COM3 사용 시작"을 알리고, 끝나면 "COM3 반납"을 알린다.
- ESP32 포트는 초음파 세션이 주로 쓴다. 다른 세션이 쓰기 전과 뒤에 총괄에게 "ESP32 사용 시작/반납"을 알린다. 팔 확인(손목 센서) 실물 시험은 COM3과 ESP32를 같이 쓰므로 총괄이 순서를 정한다.
- GUI 세션은 `--sim`으로 개발하는 것이 기본이다. 실제 팔은 화면이 시뮬레이션에서 다 동작한 뒤에만, 속도 30% 이하로, 사람이 전원 스위치 옆에 있을 때 연결한다.

## 7. 질문과 승인은 총괄로 (결정 0036 보완, 2026-11-05 사용자 요청)
팔·초음파 세션(Sonnet)은 사용자에게 직접 묻거나 승인을 받지 않는다. 묻거나 승인받을 일은 모두 총괄(GUI 세션)에 `SendMessage`로 보내고, 답을 기다리는 동안 할 수 있는 다른 일을 한다. 총괄만 사용자와 이야기한다.

| 일 | 누가 정하나 |
|---|---|
| 자기 `dev/` 브랜치 커밋·푸시, PR 열기 | 각 세션이 묻지 않고 한다. 연 뒤 총괄에게 PR 번호와 시험 결과를 알린다 |
| 설계서·결정 기록으로 답이 나오는 질문, 작업 순서, 설계서 범위 안의 구현 선택, 세션 사이 조율 | 총괄이 정해 답한다 |
| `main`에 합치기 | 총괄이 검토한 뒤 사용자 승인을 받아 합친다 |
| 실제 하드웨어(팔 움직이기, ESP32 펌웨어 올리기, 서보·센서 전원 켜기) | 사용자가 옆에 있어야 한다. 총괄이 사용자에게 묻고 순서를 정한다 |
| 지우기(브랜치, 기록), 구매, 비용 | 총괄이 사용자에게 묻는다 |
| 설계서와 다른 구현, 새 API·모듈, 안전 정책 변경 | 총괄이 [기획 요청](5절)으로 모아 사용자에게 준다. 사용자가 기획 세션에 가져간다 |

- 총괄은 사용자에게 물을 것을 모아 한 번에 묻는다. 번호를 매기고 항목마다 추천 답과 이유를 붙인다.
- 사용자의 답은 총괄이 해당 세션에 전하고, 요청 대장이나 현황 문서에 남긴다.
- Claude Code 앱이 띄우는 도구 권한 확인 창은 이 규칙과 별개다(앱 설정의 권한 모드로 정한다).
