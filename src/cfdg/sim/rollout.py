from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

from .bicycle_model import VehicleParams, VehicleState, step


@dataclass
class SimFrame:
    t: float
    state: VehicleState
    cmd_steer: float
    cmd_accel: float
    perturb_on: bool


ControlFn = Callable[[VehicleState, float], Tuple[float, float]]
PerturbFn = Callable[[float, VehicleState, float, float], Tuple[float, float, bool]]


class Simulator:
    def __init__(self, params: VehicleParams) -> None:
        self.params = params

    def rollout(
        self,
        init: VehicleState,
        steps: int,
        dt: float,
        control_fn: ControlFn,
        perturb_fn: Optional[PerturbFn] = None,
    ) -> List[SimFrame]:
        frames: List[SimFrame] = []
        state = init
        t = 0.0
        for _ in range(steps):
            steer, accel = control_fn(state, t)
            perturb_on = False
            if perturb_fn is not None:
                steer, accel, perturb_on = perturb_fn(t, state, steer, accel)
            state = step(state, steer, accel, dt, self.params)
            frames.append(SimFrame(t=t, state=state, cmd_steer=steer, cmd_accel=accel, perturb_on=perturb_on))
            t += dt
        return frames
