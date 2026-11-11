# 0053. MuJoCo 물리 시뮬레이션은 드라이버 자리에 끼우는 `MuJoCoDriver`로 한다

상태: 확정 · 2026-11-11 (사용자 승인, `researching-references`로 후보 13개 비교, 로컬 총괄. 합치기 전 EGM Stage 9 K1~K9 반영). 0044의 마지막 순서(MuJoCo)와 0050 R3를 구체화한다.

## 맥락
- 지금 `SimDriver`는 기구학만 본다. 그리퍼는 정해진 폭에서 멈추는 것으로 접촉을 흉내 낸다.
- 그래서 턱이 물체 윗모서리를 치거나, 물체를 밀거나 쓰러뜨려도 결과가 `ok`로 나온다. 11-10 실물 실패(배출 구역 다시 집기 `no_object`)를 시뮬레이션이 잡지 못했다.

## 결정
1. `MuJoCoDriver`를 `SimDriver`와 같은 드라이버 메서드로 만들어 하드웨어 자리에 끼운다(mujoco_ros2_control과 같은 구조). `--sim`과 함께 `--sim-engine`으로 엔진을 고르고, 기본은 지금 `SimDriver`다. `ArmService`·`Skills`·기존 시험은 고치지 않는다. GUI는 드라이버를 만드는 곳과 인자 전달만 고친다(화면은 그대로).
   - 물리는 드라이버 안 백그라운드 스레드가 벽시계에 맞춰 계속 돌린다(제어 주기와 무관).
   - 시뮬레이션 팔의 `no_object`를 성공으로 치던 규칙은 MuJoCo(`physical_objects`)에서는 쓰지 않는다.
   - 세부는 `arm-software-spec.md` 13절.
2. 모델은 google-deepmind/mujoco_menagerie의 `robotstudio_so101`에서 시작한다.
   - 그리퍼 값 0~100과 관절의 대응은 우리 `VALUE_TO_Q`(단동식, 턱 각도)에 맞춘다.
   - 받침 충돌 메쉬가 빠져 있다는 보고가 있어 확인한다.
   - 모델 라이선스를 확인하고 함께 둔다.
3. 접촉은 MuJoCo 공식 문서의 미끄럼 방지 조합에서 시작한다: 손끝 `condim` 4 이상, 타원 마찰 원뿔(`cone="elliptic"`), `impratio` 1 초과, 적분기 `implicitfast`. 값은 시험값이다.
4. 서보 응답(kp·kv·forcerange·armature)은 menagerie 기본값으로 시작한다. 공개된 kp 값이 17.8~998로 10배 넘게 갈리므로, 나중에 실물 기록(`status.jsonl`)으로 맞춘다.
5. 물체는 수동 물체 설정과 장면 부품(`config/scene.yaml`)을 높이를 포함한 상자로 놓는다.
6. 쓰지 않는 것:
   - ManiSkill3·Isaac Lab·lerobot-sim2real: GPU·Linux 중심의 학습용이라 우리 드라이버 구조와 맞지 않는다.
   - gym-soarm·gym-so100: 학습용 gym 환경이다.
7. 0037(검증 시뮬레이터는 주제마다 하나)과의 관계:
   - `MuJoCoDriver`는 별도 계산기가 아니라 드라이버다. 같은 `ArmService` 위 시험을 엔진만 바꿔 돌린다.
   - 초음파 벤치마크(`ultrasonic-spec.md` 8.5)는 지금 가상 팔을 그대로 쓴다. MuJoCo는 나중에 그 도구의 옵션으로만 붙인다.

## 영향
- 팔 세션: `arm-software-spec.md` 13절(REQ-056 뒤 REQ로).
- GUI: 드라이버 생성·인자 전달(M2와 같이), `no_object` 규칙과 물체 표시(M4)는 GUI REQ.
