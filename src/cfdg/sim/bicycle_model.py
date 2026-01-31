from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin, tan


@dataclass
class VehicleState:
    x: float
    y: float
    yaw: float
    v: float


@dataclass
class VehicleParams:
    wheel_base: float
    steer_limit: float
    a_min: float
    a_max: float
    a_lat_max: float


def step(state: VehicleState, steer: float, accel: float, dt: float, params: VehicleParams) -> VehicleState:
    steer = max(-params.steer_limit, min(params.steer_limit, steer))
    accel = max(params.a_min, min(params.a_max, accel))
    x = state.x + state.v * cos(state.yaw) * dt
    y = state.y + state.v * sin(state.yaw) * dt
    yaw = state.yaw + (state.v / params.wheel_base) * tan(steer) * dt
    v = state.v + accel * dt
    return VehicleState(x=x, y=y, yaw=yaw, v=v)
