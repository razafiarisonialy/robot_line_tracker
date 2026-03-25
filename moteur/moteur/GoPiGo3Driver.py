"""
gopigo3_driver.py
-----------------
Classe GoPiGo3Driver — couche d'abstraction entre le nœud ROS2 motorNode
et l'API bas niveau gopigo3.

Conversion Twist (ROS2) → DPS moteurs (GoPiGo3)
  - linear_x  [m/s]   → vitesse de base des deux roues en DPS
  - angular_z [rad/s] → différentiel gauche / droite

Architecture ROS2 :
    CameraNode  →  /line/error  →  ControllerNode  →  /cmd_vel  →  MotorNode
                                                                       │
                                                              GoPiGo3Driver  (ce fichier)
                                                                       │
                                                                  gopigo3.GoPiGo3
"""

from __future__ import annotations

import math
import logging

import gopigo3


# ── Constantes physiques du GoPiGo3 ────────────────────────────────────────
# (valeurs officielles Dexter Industries)
WHEEL_DIAMETER_MM:      float = 66.5
WHEEL_BASE_WIDTH_MM:    float = 117.0
WHEEL_CIRCUMFERENCE_MM: float = math.pi * WHEEL_DIAMETER_MM   # ≈ 208.9 mm


def _ms_to_dps(v_ms: float) -> float:
    """Convertit une vitesse linéaire [m/s] en degrés par seconde [DPS]."""
    v_mm_s = v_ms * 1_000.0
    return (v_mm_s / WHEEL_CIRCUMFERENCE_MM) * 360.0


def _radps_to_dps(omega: float) -> float:
    """
    Convertit une vitesse angulaire [rad/s] en différentiel de roue [DPS].
    Utilise la cinématique d'un robot différentiel :
        Δv = ω × (L/2)   [m/s par roue]
    """
    delta_v_ms = omega * (WHEEL_BASE_WIDTH_MM / 2.0) / 1_000.0
    return _ms_to_dps(delta_v_ms)


class GoPiGo3Driver:
    """
    Pilote GoPiGo3 compatible avec l'interface Twist de ROS2.

    Paramètres
    ----------
    base_speed : int
        Vitesse de croisière en DPS (degrés/seconde). Utilisée comme
        référence pour les limites moteur. Default : 200.
    max_speed : int
        Vitesse maximale absolue autorisée en DPS. Default : 400.
    steer_gain : float | None
        Si fourni, remplace le calcul cinématique pour le différentiel :
            diff_dps = angular_z × steer_gain
        Pratique pour un réglage empirique rapide.
        Si None (défaut), la cinématique exacte est utilisée.
    """

    def __init__(
        self,
        base_speed: int        = 200,
        max_speed:  int        = 400,
        steer_gain: float | None = None,
    ) -> None:
        self._log = logging.getLogger(self.__class__.__name__)

        self.base_speed = base_speed
        self.max_speed  = max_speed
        self.steer_gain = steer_gain

        # Instanciation du robot
        self._gpg = gopigo3.GoPiGo3()

        # Limites hardware : protège les moteurs
        self._gpg.set_motor_limits(
            self._gpg.MOTOR_LEFT + self._gpg.MOTOR_RIGHT,
            dps=self.max_speed,
        )

        # Remise à zéro des encodeurs au démarrage
        self._gpg.offset_motor_encoder(
            self._gpg.MOTOR_LEFT,
            self._gpg.get_motor_encoder(self._gpg.MOTOR_LEFT),
        )
        self._gpg.offset_motor_encoder(
            self._gpg.MOTOR_RIGHT,
            self._gpg.get_motor_encoder(self._gpg.MOTOR_RIGHT),
        )

        self._log.info(
            "GoPiGo3Driver initialisé — base=%d DPS  max=%d DPS  steer_gain=%s",
            base_speed, max_speed,
            f"{steer_gain:.1f}" if steer_gain is not None else "cinématique",
        )

    # ── Interface principale ────────────────────────────────────────────────

    def apply_twist(self, linear_x: float, angular_z: float) -> None:
        """
        Applique une commande Twist aux moteurs.

        Convention ROS2 (vue de dessus) :
            linear_x  > 0  → avance
            angular_z > 0  → tourne à gauche (CCW)

        Pour tourner à gauche :
            roue gauche  ralentit  →  left_dps  = base - diff
            roue droite  accélère  →  right_dps = base + diff

        Paramètres
        ----------
        linear_x  : vitesse linéaire souhaitée [m/s]
        angular_z : vitesse angulaire souhaitée [rad/s]
        """
        # 1. Vitesse de base commune aux deux roues
        base_dps = _ms_to_dps(linear_x)

        # 2. Différentiel gauche / droite
        if self.steer_gain is not None:
            diff_dps = angular_z * self.steer_gain      # réglage empirique
        else:
            diff_dps = _radps_to_dps(angular_z)         # cinématique exacte

        # 3. Vitesse individuelle de chaque roue
        #    angular_z > 0 → vire à gauche → roue G ralentit
        left_dps  = base_dps - diff_dps
        right_dps = base_dps + diff_dps

        # 4. Saturation
        left_dps  = max(-self.max_speed, min(self.max_speed, left_dps))
        right_dps = max(-self.max_speed, min(self.max_speed, right_dps))

        # 5. Envoi aux moteurs
        self._gpg.set_motor_dps(self._gpg.MOTOR_LEFT,  int(left_dps))
        self._gpg.set_motor_dps(self._gpg.MOTOR_RIGHT, int(right_dps))

        self._log.debug(
            "apply_twist  v=%.3f m/s  ω=%+.3f rad/s  "
            "→  L=%+.0f DPS  R=%+.0f DPS",
            linear_x, angular_z, left_dps, right_dps,
        )

    def stop(self) -> None:
        """Arrêt immédiat des deux moteurs (DPS = 0)."""
        self._gpg.set_motor_dps(
            self._gpg.MOTOR_LEFT + self._gpg.MOTOR_RIGHT, 0
        )
        self._log.info("GoPiGo3Driver — STOP")

    # ── Accès aux encodeurs (utile pour debug / odométrie) ──────────────────

    def get_encoders(self) -> tuple[int, int]:
        """Retourne (encoder_gauche, encoder_droit) en degrés."""
        return (
            self._gpg.get_motor_encoder(self._gpg.MOTOR_LEFT),
            self._gpg.get_motor_encoder(self._gpg.MOTOR_RIGHT),
        )

    def get_motor_status(self) -> tuple:
        """Retourne le statut brut des deux moteurs."""
        return (
            self._gpg.get_motor_status(self._gpg.MOTOR_LEFT),
            self._gpg.get_motor_status(self._gpg.MOTOR_RIGHT),
        )

    # ── Nettoyage ───────────────────────────────────────────────────────────

    def reset(self) -> None:
        """
        Remet le GoPiGo3 dans son état d'origine :
        désactive les capteurs, arrête les moteurs, restaure les LEDs.
        À appeler à la fermeture du nœud.
        """
        self._gpg.reset_all()
        self._log.info("GoPiGo3Driver — reset_all()")

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