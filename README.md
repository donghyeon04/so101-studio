# so101-studio

카메라 없이 저가 초음파 센서 2개로 물체의 위치·크기·방향을 인식해 SO-101 로봇팔이 집어 옮기는 시스템. 같은 작업을 카메라 방식과 비교해 각 센서가 유리한 환경을 밝히고, 전체를 GUI로 제어·모니터링한다.

지금은 시뮬레이션으로 전체가 돌고 실물 시험을 앞둔 단계다. 현황 한 장은 [`docs/planning/project-status.md`](docs/planning/project-status.md), 기획서는 [`docs/planning/overview.md`](docs/planning/overview.md), 문서 색인은 [`docs/README.md`](docs/README.md)에 있다.

## 팀원 설치 (Windows)
1. 레포를 받는다. `git clone https://github.com/donghyeon04/so101-studio` 가 좋다(켤 때마다 자동으로 최신이 된다). git이 없으면 GitHub의 Code → Download ZIP을 풀어도 되지만, 그러면 자동 업데이트가 없어 새 버전은 ZIP을 다시 받아야 한다.
2. `설치.cmd`를 더블클릭한다. conda가 없으면 Miniforge를 `C:\so101-tools\miniforge3`에 설치하고(관리자 권한 필요 없음, 한글 사용자 폴더 문제를 피하려고 영문 경로), git이 없으면 받고, `so101` 환경을 만든 뒤 점검(`doctor`)까지 보여 준다. 처음 설치에 약 3GB를 받는다(이 노트북 `so101` 환경은 2.7GB). 다시 실행하면 환경을 갱신한다.
3. `시뮬레이션 실행.cmd`를 더블클릭한다. 팔이 없어도 GUI가 시뮬레이션으로 켜진다(브라우저에서 `http://127.0.0.1:8080`).
4. 실물 팔이 있으면 `실물 팔 실행.cmd`를 쓴다.
   - USB 시리얼 드라이버(CH340)를 먼저 설치하고, 캘리브레이션이 다르면 팔이 잠겨 읽기만 된다([`docs/guides/arm-hardware-setup.md`](docs/guides/arm-hardware-setup.md)).
   - 안전: 팔은 클램프로 고정하고, 처음에는 속도 30% 이하로, 사람이 전원 스위치 옆에서 지켜본다. 주변 반경 50cm를 비운다.
   - 초음파 센서의 실물 연결과 화면 연결은 아직이다(가상 실행만 된다).

두 실행 파일은 켤 때마다 `git pull --ff-only`를 하고(실패해도 지금 버전으로 계속), `so101.cmd [--sim] gui`를 부른다. `so101.cmd`는 `C:\so101-tools\miniforge3`, `C:\miniforge3`, 사용자 폴더의 miniforge·miniconda에서 `so101` 환경을 찾는다. 근거는 [`decisions/0040`](docs/decisions/0040-team-local-install.md)이다.
