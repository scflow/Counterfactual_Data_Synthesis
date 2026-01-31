from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PIDConfig:
    kp: float
    ki: float
    kd: float


class PIDController:
    def __init__(self, cfg: PIDConfig) -> None:
        self.cfg = cfg
        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0

    def step(self, error: float, dt: float) -> float:
        self._integral += error * dt
        deriv = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error
        return self.cfg.kp * error + self.cfg.ki * self._integral + self.cfg.kd * deriv
