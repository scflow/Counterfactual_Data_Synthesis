from __future__ import annotations

from dataclasses import dataclass
import hashlib
import random
from typing import Optional


@dataclass
class Perturbation:
    kind: str
    steer_delta: float
    acc_delta: float
    start_t: float
    duration: float
    lateral_offset: float = 0.0


class PerturbationFactory:
    def __init__(self, config: dict) -> None:
        self.config = config

    def sample(self, scene_token: str) -> Perturbation:
        cfg = self.config.get("perturb", self.config)
        global_seed = int(self.config.get("seed", 0))
        rng = random.Random(_seed_from(scene_token, global_seed))

        types = cfg.get("types", ["impulse"])
        kind = rng.choice(types) if types else "impulse"
        subcfg = cfg.get(kind, {})

        steer_delta = _sample_range(rng, subcfg.get("steer_delta"))
        acc_delta = _sample_range(rng, subcfg.get("acc_delta"))
        lateral_offset = _sample_range(rng, subcfg.get("lateral_offset"))

        start_t = _sample_range(rng, subcfg.get("start_t"))
        if start_t == 0.0:
            start_t = _sample_range(rng, subcfg.get("start_t_range"))

        duration = _sample_range(rng, subcfg.get("duration_sec"))
        if duration == 0.0:
            duration_steps = _sample_int_range(rng, subcfg.get("duration_steps"))
            if duration_steps > 0:
                dt = _extract_dt(self.config)
                duration = duration_steps * dt if dt is not None else float(duration_steps)

        return Perturbation(
            kind=kind,
            steer_delta=steer_delta,
            acc_delta=acc_delta,
            start_t=start_t,
            duration=duration,
            lateral_offset=lateral_offset,
        )


def _extract_dt(config: dict) -> Optional[float]:
    sample_cfg = config.get("sample", {})
    dt = sample_cfg.get("dt", None)
    if dt is None:
        dt = config.get("dt")
    return None if dt in (None, 0) else float(dt)


def _sample_range(rng: random.Random, value: Optional[object]) -> float:
    if value is None:
        return 0.0
    if isinstance(value, (list, tuple)) and len(value) == 2:
        return rng.uniform(float(value[0]), float(value[1]))
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def _sample_int_range(rng: random.Random, value: Optional[object]) -> int:
    if value is None:
        return 0
    if isinstance(value, (list, tuple)) and len(value) == 2:
        low, high = int(value[0]), int(value[1])
        return rng.randint(low, high)
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


def _seed_from(scene_token: str, global_seed: int) -> int:
    digest = hashlib.sha256(scene_token.encode("utf-8")).hexdigest()
    return (int(digest[:16], 16) ^ global_seed) & 0xFFFFFFFF
