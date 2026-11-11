# 출처
google-deepmind/mujoco_menagerie `robotstudio_so101` (Apache-2.0, LICENSE 참조), 기준 커밋 3abca2e302625cec9429c49806121ea2855cef81.
`so101.xml`과 그것이 읽는 `assets/*.stl`, `LICENSE`, `README.md`, `CHANGELOG.md`는 원본 그대로다. 원본의 `scene.xml`·`scene_box.xml`·`so101.png`와 쓰지 않는 STL은 크기 때문에 가져오지 않았다.
우리 쪽 추가(받침 상자, gripper_frame site, 작업대, 장면 부품, 물체, 턱 접촉값)는 이 폴더가 아니라 `src/so101_studio/arm/mujoco_model.py`의 `load_model`이 MjSpec으로 더한다.
