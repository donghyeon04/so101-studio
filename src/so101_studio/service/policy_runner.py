"""정책 실행 1차 (arm-software-spec 6.10): fps마다 관측을 모아 정책에 넣고 나온 절대 관절 목표를 servo_joints로 흘려 넣는다.
일시 정지·이어 하기는 pause()/resume()(gui-spec 12.10, REQ-108). 12/4 전에는 MuJoCo 엔진(driver.render가 있는 드라이버)에서만 시작한다(결정 0066 4항)."""
import threading
import time
from dataclasses import astuple

import numpy as np

from ..core.types import JointState

CAMERAS = ("top", "wrist")  # 관측 키 observation.images.<이름> (mujoco-spec 13.10.4 기록기와 같다)
JUDGE_DELAY_S = 1.0  # 끝나고 물체가 가라앉은 뒤 판정한다
HARD_STOP = ("stopped", "fault", "locked", "busy", "real_arm")  # servo_joints가 이 이유로 거부하면 바로 끝낸다
REJECT_S = 0.5  # 안전 거부(관절 한계·작업 영역·장면 부품)는 이만큼 이어질 때만 끝낸다 (spec 6.10 "거부가 이어짐")
SLOW_S = 1.0  # 이만큼 아무 목표도 보내지 못하면(추론이 늘 한 주기를 넘김) too_slow로 끝낸다


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
    """judge: 끝나고 JUDGE_DELAY_S 뒤 부를 판정 함수 judge(driver) -> dict(outcome, …). 배치를 묶어(functools.partial) 넘긴다. 없으면 outcome None.
    돌리는 법 둘: start()는 벽시계 스레드(GUI·실시간). 시계로 돌릴 때(StepClock.every, mujoco-spec 13.10.2)는 begin() 뒤 걸음마다
    `why = runner.due_stop(slow=False) or runner.step()`, why가 생기면 finish(why)."""

    def __init__(self, svc, driver, checkpoint_dir=None, fps: int = 25, policy=None, max_s: float = 20, judge=None):
        self.svc, self.driver, self.checkpoint_dir = svc, driver, checkpoint_dir
        self.fps, self.policy, self.max_s, self.judge = fps, policy, max_s, judge
        self.result: dict | None = None  # 끝난 뒤 ARM-POLICY-STOP에 남긴 내용
        self._quit = threading.Event()
        self._thread: threading.Thread | None = None
        self._infer: list[float] = []  # 걸음마다 추론 걸린 초(벽시계)
        self._t0, self._skipped, self._sent, self._rejected = 0.0, 0, 0.0, None  # 시작 시각, 건너뛴 걸음, 마지막으로 보낸 시각, 안전 거부가 시작된 시각(제어 시계)
        self._paused_at: float | None = None  # 일시 정지한 시각(제어 시계). None이면 돌고 있다

    def start(self) -> str | None:
        """벽시계 스레드로 시작한다. 시작하면 None, 못 하면 사람이 읽을 이유."""
        if self._thread is not None and self._thread.is_alive():
            return "이미 실행 중"
        if (why := self.begin()) is not None:
            return why
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return None

    def begin(self, **log) -> str | None:
        """한 번 돌리기를 준비한다: 정책을 열고(없으면) reset, 시작 시각·걸음 기록을 비우고 ARM-POLICY-START(log를 더해)를 남긴다. 못 하면 이유."""
        if not callable(getattr(self.driver, "render", None)):
            return "정책 실행은 MuJoCo 엔진에서만 한다(실제 팔·기구학 엔진에는 카메라가 없다)"
        if self.policy is None:
            try:
                self.policy = load_act(self.checkpoint_dir)
            except Exception as e:  # noqa: BLE001 torch·lerobot이 없거나 체크포인트가 깨졌다: 안내만 하고 앱은 그대로 돈다
                return f"정책을 불러오지 못했다({self.checkpoint_dir}): {e!r}"[:300]
        self.policy.reset()
        self._quit.clear()
        self.result = None
        self._t0 = self._sent = self.svc.clock.now()
        self._infer, self._skipped, self._rejected, self._paused_at = [], 0, None, None
        self.svc.log_event("policy_start", {"checkpoint": None if self.checkpoint_dir is None else str(self.checkpoint_dir), "fps": self.fps, **log},
                           code="ARM-POLICY-START")
        return None

    def due_stop(self, slow: bool = True) -> str | None:
        """지금 끝내야 하면 이유: max_s(제어 시계로 max_s 지남), stop(stop() 불림), too_slow(slow이고 SLOW_S 동안 아무 목표도 못 보냄). 아니면 None.
        slow=False는 시계로 돌릴 때다(추론 지연이 시뮬레이션 시간 0이라 too_slow가 없다)."""
        if self._quit.is_set():  # 멈춘 중에도 stop()은 바로 끝낸다
            return "stop"
        if self._paused_at is not None:  # 멈춘 동안은 max_s·too_slow 시계가 가지 않는다
            return None
        now = self.svc.clock.now()
        if now - self._t0 >= self.max_s:
            return "max_s"
        if slow and now - self._sent > SLOW_S:
            return "too_slow"
        return None

    @property
    def paused(self) -> bool:
        return self._paused_at is not None

    def pause(self) -> None:
        """목표를 그만 보낸다. 팔은 마지막 목표 자리에 선다(서보 흐름이 끊기면 컨트롤러가 그 자리를 지킨다). 이미 멈췄으면 그대로."""
        if self._paused_at is None:
            self._paused_at = self.svc.clock.now()
            self.svc.log_event("policy_pause", {}, code="ARM-POLICY-PAUSE")

    def resume(self) -> None:
        """이어 한다. 멈춘 시간만큼 시작·마지막 보낸·거부 시각을 미뤄 max_s·too_slow·REJECT_S가 멈춘 시간을 빼고 센다."""
        if self._paused_at is None:
            return
        d = self.svc.clock.now() - self._paused_at
        self._t0, self._sent, self._paused_at = self._t0 + d, self._sent + d, None
        self._rejected = None if self._rejected is None else self._rejected + d
        self.svc.log_event("policy_resume", {"paused_s": round(d, 3)}, code="ARM-POLICY-RESUME")

    def stop(self, timeout: float = 3.0) -> None:
        self._quit.set()
        if self._thread is not None:
            self._thread.join(timeout)

    def _observe(self) -> dict:
        obs = {"observation.state": np.array(astuple(self.driver.read_joints()), np.float32)}
        for cam in CAMERAS:
            obs[f"observation.images.{cam}"] = self.driver.render(cam)
        return obs

    def step(self, skip_after_s: float | None = None) -> str | None:
        """한 걸음: 관측 -> 추론 -> servo_joints. 끝내야 하면 이유(servo:<이유> | error: ...), 아니면 None (mujoco-spec 13.10.2).
        skip_after_s: 추론이 이보다 오래 걸리면 낡은 목표라 보내지 않는다(벽시계 실행만. StepClock에서는 추론 지연이 0이라 None).
        안전 거부는 REJECT_S(제어 시계) 동안 이어질 때만 끝낸다. infer_ms는 벽시계로 잰다. 일시 정지 중이면 아무것도 하지 않는다."""
        if self._paused_at is not None:
            return None
        try:
            obs = self._observe()
            a0 = time.perf_counter()
            act = self.policy.select_action(obs)
        except Exception as e:  # noqa: BLE001 관측·추론 실패는 러너만 끝낸다
            return f"error: {e!r}"[:200]
        self._infer.append(time.perf_counter() - a0)
        now = self.svc.clock.now()
        if skip_after_s is not None and self._infer[-1] > skip_after_s:  # 한 주기를 넘긴 추론
            self._skipped += 1
        elif self.svc.servo_joints(JointState(*map(float, act))):
            self._sent, self._rejected = now, None
        else:
            why = self.svc.ctl.servo_reason
            self._rejected = self._rejected or now
            if why in HARD_STOP or now - self._rejected >= REJECT_S:
                return f"servo:{why}"
            self._sent = now  # 거부는 REJECT_S가 따로 센다: too_slow로 세지 않는다
        return None

    def _run(self) -> None:
        """벽시계 스레드(GUI·실시간): fps마다 step()을 부른다."""
        period, clock = 1.0 / self.fps, self.svc.clock
        nxt = clock.now()
        while (reason := self.due_stop() or self.step(skip_after_s=period)) is None:
            nxt = max(nxt + period, clock.now() - period)  # 밀렸으면 몰아서 따라잡지 않는다
            self._quit.wait(max(0.0, nxt - clock.now()))
        self.finish(reason)

    def finish(self, reason: str) -> None:
        """끝낸다: 지금 자리에 서고, judge가 있으면 JUDGE_DELAY_S(제어 시계) 뒤 판정, result를 채우고 ARM-POLICY-STOP을 남긴다."""
        self.svc.stop()  # 서보 목표를 비워 0.5초를 기다리지 않고 지금 자리에서 선다
        outcome = None
        if self.judge is not None:
            self.svc.clock.sleep(JUDGE_DELAY_S)
            try:
                outcome = self.judge(self.driver).get("outcome")
            except Exception as e:  # noqa: BLE001 판정 실패도 결과로 남긴다
                outcome = f"judge_error: {e!r}"[:200]
        ms = [t * 1000 for t in self._infer]
        self.result = {"reason": reason, "outcome": outcome, "steps": len(ms), "skipped": self._skipped,
                       "infer_ms_mean": round(sum(ms) / len(ms), 2) if ms else None, "infer_ms_max": round(max(ms), 2) if ms else None}
        self.svc.log_event("policy_stop", self.result, level="WARN" if reason.startswith(("servo", "error")) else "INFO", code="ARM-POLICY-STOP")
