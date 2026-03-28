from __future__ import annotations
import math
import logging
import gopigo3


WHEEL_DIAMETER_MM:      float = 66.5
WHEEL_BASE_WIDTH_MM:    float = 117.0
WHEEL_CIRCUMFERENCE_MM: float = math.pi * WHEEL_DIAMETER_MM



def _ms_to_dps(v_ms: float) -> float:
    v_mm_s = v_ms * 1_000.0
    return (v_mm_s / WHEEL_CIRCUMFERENCE_MM) * 360.0


def _radps_to_dps(omega: float) -> float:
    delta_v_ms = omega * (WHEEL_BASE_WIDTH_MM / 2.0) / 1_000.0
    return _ms_to_dps(delta_v_ms)


class GoPiGo3Driver:
    def __init__(
        self,
        base_speed: int          = 200,
        max_speed:  int          = 400,
        steer_gain: float | None = None,
    ) -> None:
        self._log = logging.getLogger(self.__class__.__name__)

        self.base_speed = base_speed
        self.max_speed  = max_speed
        self.steer_gain = steer_gain

        self._gpg = gopigo3.GoPiGo3()

        self._gpg.set_motor_limits(
            self._gpg.MOTOR_LEFT + self._gpg.MOTOR_RIGHT,
            dps=self.max_speed,
        )
        self._reset_encoders()

        self._log.info(
            "GoPiGo3Driver initialisé — base=%d DPS  max=%d DPS  steer_gain=%s",
            base_speed,
            max_speed,
            f"{steer_gain:.1f}" if steer_gain is not None else "cinématique",
        )

    # ── API publique ──────────────────────────────────────────────────────────

    def apply_twist(self, linear_x: float, angular_z: float) -> None:
        base_dps = _ms_to_dps(linear_x)

        if self.steer_gain is not None:
            diff_dps = angular_z * self.steer_gain
        else:
            diff_dps = _radps_to_dps(angular_z)

        left_dps  = max(-self.max_speed, min(self.max_speed, base_dps + diff_dps))
        right_dps = max(-self.max_speed, min(self.max_speed, base_dps - diff_dps))

        self._gpg.set_motor_dps(self._gpg.MOTOR_LEFT,  -int(left_dps))
        self._gpg.set_motor_dps(self._gpg.MOTOR_RIGHT, -int(right_dps))

        self._log.debug(
            "apply_twist  v=%.3f m/s  ω=%+.3f rad/s  → L=%+.0f DPS  R=%+.0f DPS",
            linear_x, angular_z, left_dps, right_dps,
        )

    def stop(self) -> None:
        self._gpg.set_motor_dps(
            self._gpg.MOTOR_LEFT + self._gpg.MOTOR_RIGHT, 0
        )
        self._log.info("GoPiGo3Driver — STOP")

    def get_encoders(self) -> tuple[int, int]:
        return (
            self._gpg.get_motor_encoder(self._gpg.MOTOR_LEFT),
            self._gpg.get_motor_encoder(self._gpg.MOTOR_RIGHT),
        )

    def get_motor_status(self) -> tuple:
        return (
            self._gpg.get_motor_status(self._gpg.MOTOR_LEFT),
            self._gpg.get_motor_status(self._gpg.MOTOR_RIGHT),
        )

    def reset(self) -> None:
        self._gpg.reset_all()
        self._log.info("GoPiGo3Driver — reset_all()")

    def _reset_encoders(self) -> None:
        self._gpg.offset_motor_encoder(
            self._gpg.MOTOR_LEFT,
            self._gpg.get_motor_encoder(self._gpg.MOTOR_LEFT),
        )
        self._gpg.offset_motor_encoder(
            self._gpg.MOTOR_RIGHT,
            self._gpg.get_motor_encoder(self._gpg.MOTOR_RIGHT),
        )

    def __del__(self) -> None:
        try:
            self.stop()
        except Exception:
            pass

    def __repr__(self) -> str:
        return (
            f"GoPiGo3Driver("
            f"base_speed={self.base_speed}, "
            f"max_speed={self.max_speed}, "
            f"steer_gain={self.steer_gain})"
        )