# 0040. 팀원 배포는 시뮬레이션·실물 겸용 로컬 설치로 하고, 서버(웹 링크) 배포는 하지 않는다

상태: 확정 · 2026-11-06

## 맥락
기초 구현 뒤 팀원에게 개발 초안을 나눠 준다. 서버 방식은 한 시뮬레이션을 모두가 나눠 쓰고(조작권 한 명), Hugging Face Docker Space는 PRO(월 9달러)가 필요해 사용자가 원하지 않았다. 실물 팔은 USB와 안전(전원 스위치 옆, E-STOP) 때문에 원격으로 다루지 않는다. 총괄이 후보 12개를 조사해 점수를 매겼고 사용자가 1위를 승인했다.

## 결정
- Miniforge 무인 설치 + 배치 스크립트로 한다. 기존 `environment.yml`과 `so101.cmd`를 그대로 쓴다.
- `설치.cmd`: conda가 없으면 Miniforge를 영문 경로 `C:\so101-tools\miniforge3`에 `/InstallationType=JustMe /S`로 설치하고(한글 사용자 폴더 문제 회피), git은 conda-forge로 받고, `conda env create/update -f environment.yml`, 끝에 `doctor --no-probe`.
- `시뮬레이션 실행.cmd` / `실물 팔 실행.cmd`: 켤 때마다 `git pull --ff-only`(실패해도 계속) 뒤 `so101.cmd [--sim] gui`.
- 버린 후보: uv(placo Windows 휠 없음), PyInstaller exe(NiceGUI 실행 실패·백신 오탐 보고), Docker(Windows USB 시리얼), conda-pack·constructor·포터블 zip(업데이트마다 수 GB), pixi(근소 2위, 설치 기준 파일이 둘이 됨).
- 구현은 REQ-029(팔 세션, 레포 루트 스크립트와 README).
