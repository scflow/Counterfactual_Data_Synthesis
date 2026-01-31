from __future__ import annotations

from dataclasses import dataclass


@dataclass
class IDMConfig:
    desired_speed: float
    min_gap: float
    time_headway: float
    max_accel: float = 1.0
    comfortable_brake: float = 2.0
    delta: float = 4.0


class IDMController:
    def __init__(self, cfg: IDMConfig) -> None:
        self.cfg = cfg

    def step(self, v: float, distance: float, rel_speed: float) -> float:
        cfg = self.cfg
        # Desired gap s* = s0 + v*T + v*Δv/(2*sqrt(a*b))
        s_star = cfg.min_gap + v * cfg.time_headway
        if cfg.max_accel > 0 and cfg.comfortable_brake > 0:
            s_star += (v * rel_speed) / (2.0 * (cfg.max_accel * cfg.comfortable_brake) ** 0.5)
        distance = max(distance, 1e-3)
        free_road = 1.0 - (v / max(cfg.desired_speed, 1e-3)) ** cfg.delta
        interaction = (s_star / distance) ** 2
        accel = cfg.max_accel * (free_road - interaction)
        return accel
