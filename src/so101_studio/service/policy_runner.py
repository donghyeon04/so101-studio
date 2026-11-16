"""정책 실행 1차 (arm-software-spec 6.10): fps마다 관측을 모아 정책에 넣고 나온 절대 관절 목표를 servo_joints로 흘려 넣는다.
일시 정지·이어 하기는 2차라 없다. 12/4 전에는 MuJoCo 엔진(driver.render가 있는 드라이버)에서만 시작한다(결정 0066 4항)."""
import threading
import time
from dataclasses import astuple

import numpy as np

from ..core.types import JointState

CAMERAS = ("top", "wrist")  # 관측 키 observation.images.<이름> (mujoco-spec 13.10.4 기록기와 같다)
JUDGE_DELAY_S = 1.0  # 끝나고 물체가 가라앉은 뒤 판정한다


def load_act(checkpoint_dir):
    """lerobot 0.6.1의 ACT 정책을 CPU로 열고 같은 폴더의 전후 처리기를 붙인다. select_action(obs) -> 6값, reset()."""
    import torch
    from lerobot.common.control_utils import predict_action
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors

    path = str(checkpoint_dir)
    cfg = PreTrainedConfig.from_pretrained(path)
    cfg.device = "cpu"  # 학습을 GPU에서 했어도 앱은 CPU로 돈다
    policy = ACTPolicy.from_pretrained(path, config=cfg)
    pre, post = make_pre_post_processors(cfg, pretrained_path=path, preprocessor_overrides={"device_processor": {"device": "cpu"}})
    cpu = torch.device("cpu")

    class _Act:
        def select_action(self, obs):
            return predict_action(obs, policy, cpu, pre, post, use_amp=False).squeeze(0).tolist()

        def reset(self):
            policy.reset()
            pre.reset()
            post.reset()

    return _Act()


class PolicyRunner:
    """judge: 끝나고 JUDGE_DELAY_S 뒤 부를 판정 함수 judge(driver) -> dict(outcome, …). 배치를 묶어(functools.partial) 넘긴다. 없으면 outcome None."""

    def __init__(self, svc, driver, checkpoint_dir=None, fps: int = 25, policy=None, max_s: float = 20, judge=None):
        self.svc, self.driver, self.checkpoint_dir = svc, driver, checkpoint_dir
        self.fps, self.policy, self.max_s, self.judge = fps, policy, max_s, judge
        self.result: dict | None = None  # 끝난 뒤 ARM-POLICY-STOP에 남긴 내용
        self._quit = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> str | None:
        """시작하면 None, 못 하면 사람이 읽을 이유."""
        if not callable(getattr(self.driver, "render", None)):
            return "정책 실행은 MuJoCo 엔진에서만 한다(실제 팔·기구학 엔진에는 카메라가 없다)"
        if self._thread is not None and self._thread.is_alive():
            return "이미 실행 중"
        if self.policy is None:
            try:
                self.policy = load_act(self.checkpoint_dir)
            except Exception as e:  # noqa: BLE001 torch·lerobot이 없거나 체크포인트가 깨졌다: 안내만 하고 앱은 그대로 돈다
                return f"정책을 불러오지 못했다({self.checkpoint_dir}): {e!r}"[:300]
        self.policy.reset()
        self._quit.clear()
        self.result = None
        self.svc.log_event("policy_start", {"checkpoint": None if self.checkpoint_dir is None else str(self.checkpoint_dir), "fps": self.fps},
                           code="ARM-POLICY-START")
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return None

    def stop(self, timeout: float = 3.0) -> None:
        self._quit.set()
        if self._thread is not None:
            self._thread.join(timeout)

    def _observe(self) -> dict:
        obs = {"observation.state": np.array(astuple(self.driver.read_joints()), np.float32)}
        for cam in CAMERAS:
            obs[f"observation.images.{cam}"] = self.driver.render(cam)
        return obs

    def _run(self) -> None:
        period = 1.0 / self.fps
        t0 = nxt = time.perf_counter()
        infer, skipped, reason = [], 0, "max_s"
        while time.perf_counter() - t0 < self.max_s:
            if self._quit.is_set():
                reason = "stop"
                break
            try:
                obs = self._observe()
                a0 = time.perf_counter()
                act = self.policy.select_action(obs)
            except Exception as e:  # noqa: BLE001 관측·추론 실패는 러너만 끝낸다
                reason = f"error: {e!r}"[:200]
                break
            infer.append(time.perf_counter() - a0)
            if infer[-1] > period:  # 한 주기를 넘긴 추론: 낡은 목표라 보내지 않는다
                skipped += 1
            elif not self.svc.servo_joints(JointState(*map(float, act))):  # 정지·E-STOP·fault·안전 거부
                reason = f"servo:{self.svc.ctl.servo_reason}"
                break
            nxt = max(nxt + period, time.perf_counter() - period)  # 밀렸으면 몰아서 따라잡지 않는다
            self._quit.wait(max(0.0, nxt - time.perf_counter()))
        self.svc.stop()  # 서보 목표를 비워 0.5초를 기다리지 않고 지금 자리에서 선다
        outcome = None
        if self.judge is not None:
            time.sleep(JUDGE_DELAY_S)
            try:
                outcome = self.judge(self.driver).get("outcome")
            except Exception as e:  # noqa: BLE001 판정 실패도 결과로 남긴다
                outcome = f"judge_error: {e!r}"[:200]
        ms = [t * 1000 for t in infer]
        self.result = {"reason": reason, "outcome": outcome, "steps": len(ms), "skipped": skipped,
                       "infer_ms_mean": round(sum(ms) / len(ms), 2) if ms else None, "infer_ms_max": round(max(ms), 2) if ms else None}
        self.svc.log_event("policy_stop", self.result, level="WARN" if reason.startswith(("servo", "error")) else "INFO", code="ARM-POLICY-STOP")
