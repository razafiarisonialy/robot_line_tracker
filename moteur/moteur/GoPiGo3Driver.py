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
        max_speed:  int          = 400,
        steer_gain: float | None = None,
    ) -> None:
        self._log = logging.getLogger(self.__class__.__name__)

        self.max_speed  = max_speed
        self.steer_gain = steer_gain

        self._gpg = gopigo3.GoPiGo3()

        self._gpg.set_motor_limits(
            self._gpg.MOTOR_LEFT + self._gpg.MOTOR_RIGHT,
            dps=self.max_speed,
        )

        self._log.info(
            "GoPiGo3Driver initialisé — max=%d DPS  steer_gain=%s",
            max_speed,
            f"{steer_gain:.1f}" if steer_gain is not None else "cinématique",
        )

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

    def stop(self) -> None:
        self._gpg.set_motor_dps(
            self._gpg.MOTOR_LEFT + self._gpg.MOTOR_RIGHT, 0
        )

    def get_encoders(self) -> tuple[int, int]:
        return (
            self._gpg.get_motor_encoder(self._gpg.MOTOR_LEFT),
            self._gpg.get_motor_encoder(self._gpg.MOTOR_RIGHT),
        )

    def __del__(self) -> None:
        try:
            self.stop()
        except Exception:
            pass
