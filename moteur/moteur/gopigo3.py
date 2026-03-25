
import numpy as np

try:
    from easygopigo3 import EasyGoPiGo3
    GOPIGO_AVAILABLE = True
except ImportError:
    # Sur un PC de développement sans GoPiGo3 → mode simulation
    GOPIGO_AVAILABLE = False


class GoPiGo3Driver:

    DPS_MIN  =   0
    DPS_MAX  = 600

    def __init__(self,
                 base_speed:    int   = 200,
                 max_speed:     int   = 400,
                 steer_gain:    float = 50.0,
                 min_speed_pct: float = 0.3,
                 simulation:    bool  = False):

        self.base_speed    = int(np.clip(base_speed, self.DPS_MIN, self.DPS_MAX))
        self.max_speed     = int(np.clip(max_speed,  self.DPS_MIN, self.DPS_MAX))
        self.steer_gain    = steer_gain
        self.min_speed_pct = min_speed_pct
        self._simulation   = simulation or not GOPIGO_AVAILABLE

        if self._simulation:
            self._gpg = None
            print('[GoPiGo3Driver] Mode SIMULATION (pas de matériel détecté)')
        else:
            self._gpg = EasyGoPiGo3()
            self._gpg.set_speed(self.base_speed)
            print(f'[GoPiGo3Driver] GoPiGo3 initialisé — '
                  f'vitesse de base {self.base_speed} DPS')


    def apply_twist(self, linear_x: float, angular_z: float):
        if linear_x == 0.0 and angular_z == 0.0:
            self.stop()
            return

        speed_dps = int(abs(linear_x) * self.max_speed)
        speed_dps = int(np.clip(speed_dps, 0, self.max_speed))

        if speed_dps < 50:
            speed_dps = 150

        correction = float(np.clip(
            self.steer_gain * angular_z, -100.0, 100.0))

        left_pct  = int(np.clip(100.0 - correction, -100, 100))
        right_pct = int(np.clip(100.0 + correction, -100, 100))

        speed_factor = 1.0 - (1.0 - self.min_speed_pct) * \
                       (abs(angular_z) / max(abs(angular_z), 0.001))
        speed_factor = float(np.clip(speed_factor, self.min_speed_pct, 1.0))
        speed_dps    = int(speed_dps * speed_factor)

        self._set_speed_and_steer(speed_dps, left_pct, right_pct)

    def stop(self):
        if self._simulation:
            print('[GoPiGo3Driver] STOP')
        else:
            self._gpg.stop()

    def forward(self, speed: int = None):
        spd = speed if speed is not None else self.base_speed
        if self._simulation:
            print(f'[GoPiGo3Driver] FORWARD speed={spd} DPS')
        else:
            self._gpg.set_speed(spd)
            self._gpg.forward()


    def _set_speed_and_steer(self, speed_dps: int,
                             left_pct: int, right_pct: int):
        if self._simulation:
            print(f'[GoPiGo3Driver] set_speed={speed_dps} DPS  '
                  f'steer(L={left_pct}%, R={right_pct}%)')
        else:
            self._gpg.set_speed(speed_dps)
            self._gpg.steer(left_pct, right_pct)

    def __repr__(self):
        mode = 'SIMULATION' if self._simulation else 'HARDWARE'
        return (f"GoPiGo3Driver(mode={mode}, "
                f"base_speed={self.base_speed}, "
                f"max_speed={self.max_speed}, "
                f"steer_gain={self.steer_gain})")