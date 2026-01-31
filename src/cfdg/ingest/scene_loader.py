from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Any, Callable, Iterable, List, Optional


@dataclass
class EgoState:
    t: float
    x: float
    y: float
    yaw: float
    v: float
    a: float
    steer: float


@dataclass
class AgentState:
    track_token: str
    t: float
    x: float
    y: float
    yaw: float
    v: float
    length: float
    width: float
    obj_type: str


@dataclass
class Frame:
    t: float
    ego: EgoState
    agents: List[AgentState]


@dataclass
class Scenario:
    scene_token: str
    map_name: str
    frames: List[Frame]


ScenarioProvider = Callable[[str], Any]


def _get_attr(obj: Any, names: Iterable[str], default: Any = None) -> Any:
    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    return default


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _extract_pose(obj: Any) -> Optional[Any]:
    if obj is None:
        return None
    # Common nuPlan ego/object pose access patterns.
    for name in ("rear_axle", "center", "pose"):
        if hasattr(obj, name):
            return getattr(obj, name)
    if hasattr(obj, "car_footprint") and hasattr(obj.car_footprint, "center"):
        return obj.car_footprint.center
    if hasattr(obj, "box") and hasattr(obj.box, "center"):
        return obj.box.center
    return None


def _extract_xy_yaw(pose: Any) -> tuple[float, float, float]:
    if pose is None:
        return 0.0, 0.0, 0.0
    x = _as_float(_get_attr(pose, ("x", "pos_x")))
    y = _as_float(_get_attr(pose, ("y", "pos_y")))
    yaw = _as_float(_get_attr(pose, ("heading", "yaw")))
    return x, y, yaw


def _extract_time_s(ego_state: Any, fallback: float) -> float:
    time_point = _get_attr(ego_state, ("time_point", "timestamp"))
    if time_point is None:
        return fallback
    for name, scale in (("time_s", 1.0), ("time_ms", 1e-3), ("time_us", 1e-6)):
        if hasattr(time_point, name):
            return _as_float(getattr(time_point, name)) * scale
    return fallback


def _extract_speed_accel(ego_state: Any) -> tuple[float, float]:
    dyn = _get_attr(ego_state, ("dynamic_car_state", "dynamic_state"))
    if dyn is None:
        return 0.0, 0.0
    speed = _get_attr(dyn, ("speed", "velocity", "velocity_2d", "rear_axle_velocity_2d"))
    accel = _get_attr(dyn, ("acceleration", "acceleration_2d", "rear_axle_acceleration_2d"))

    if hasattr(speed, "x") and hasattr(speed, "y"):
        speed = hypot(_as_float(speed.x), _as_float(speed.y))
    speed = _as_float(speed)

    if hasattr(accel, "x") and hasattr(accel, "y"):
        accel = hypot(_as_float(accel.x), _as_float(accel.y))
    accel = _as_float(accel)
    return speed, accel


def _extract_steer(ego_state: Any) -> float:
    steer = _get_attr(ego_state, ("tire_steering_angle", "steering_angle", "front_wheel_angle"))
    return _as_float(steer)


def _to_str(value: Any, default: str = "unknown") -> str:
    if value is None:
        return default
    if isinstance(value, str):
        return value
    for attr in ("name", "value"):
        if hasattr(value, attr):
            return str(getattr(value, attr))
    return str(value)


def _iter_tracked_objects(tracked: Any) -> List[Any]:
    if tracked is None:
        return []
    if isinstance(tracked, (list, tuple)):
        return list(tracked)
    tracked_objects = _get_attr(tracked, ("tracked_objects",))
    if tracked_objects is not None:
        return list(tracked_objects)
    if hasattr(tracked, "get_tracked_objects"):
        return list(tracked.get_tracked_objects())
    return []


def _agent_from_obj(obj: Any, t: float) -> AgentState:
    token = _to_str(_get_attr(obj, ("track_token", "token", "track_id")), default="")
    obj_type = _to_str(_get_attr(obj, ("tracked_object_type", "object_type", "category_name")))

    pose = _extract_pose(obj)
    x, y, yaw = _extract_xy_yaw(pose)

    length = _as_float(_get_attr(obj, ("length",)))
    width = _as_float(_get_attr(obj, ("width",)))

    if hasattr(obj, "box"):
        box = obj.box
        length = _as_float(_get_attr(box, ("length", "size_x")), default=length)
        width = _as_float(_get_attr(box, ("width", "size_y")), default=width)
        pose = _extract_pose(box)
        if pose is not None:
            x, y, yaw = _extract_xy_yaw(pose)

    v = _as_float(_get_attr(obj, ("velocity", "speed")))
    return AgentState(track_token=token, t=t, x=x, y=y, yaw=yaw, v=v, length=length, width=width, obj_type=obj_type)


class NuPlanScenarioAdapter:
    """Adapter to extract data from a nuPlan AbstractScenario-like object."""

    def __init__(self, scenario_provider: ScenarioProvider) -> None:
        self._scenario_provider = scenario_provider

    def get_scenario(self, scene_token: str) -> Any:
        return self._scenario_provider(scene_token)

    def get_map_name(self, scenario: Any) -> str:
        map_api = _get_attr(scenario, ("map_api",))
        if map_api is not None and hasattr(map_api, "map_name"):
            return str(map_api.map_name)
        return _to_str(_get_attr(scenario, ("map_name", "map")), default="")

    def get_num_iterations(self, scenario: Any) -> int:
        if hasattr(scenario, "get_number_of_iterations"):
            return int(scenario.get_number_of_iterations())
        return int(_get_attr(scenario, ("num_iterations", "number_of_iterations"), default=0))

    def get_ego_state(self, scenario: Any, iteration: int) -> EgoState:
        if hasattr(scenario, "get_ego_state_at_iteration"):
            ego_state = scenario.get_ego_state_at_iteration(iteration)
        elif hasattr(scenario, "ego_states"):
            ego_state = scenario.ego_states[iteration]
        else:
            raise AttributeError("Scenario does not provide ego state accessors.")

        pose = _extract_pose(ego_state)
        x, y, yaw = _extract_xy_yaw(pose)
        v, a = _extract_speed_accel(ego_state)
        steer = _extract_steer(ego_state)
        t = _extract_time_s(ego_state, float(iteration))
        return EgoState(t=t, x=x, y=y, yaw=yaw, v=v, a=a, steer=steer)

    def get_agents(self, scenario: Any, iteration: int, t: float) -> List[AgentState]:
        tracked = None
        if hasattr(scenario, "get_tracked_objects_at_iteration"):
            tracked = scenario.get_tracked_objects_at_iteration(iteration)
        elif hasattr(scenario, "get_detections_at_iteration"):
            tracked = scenario.get_detections_at_iteration(iteration)
        agents = [_agent_from_obj(obj, t) for obj in _iter_tracked_objects(tracked)]
        return agents


class SceneLoader:
    """Load nuPlan scenes into internal structures."""

    def __init__(
        self,
        nuplan_db_path: str,
        map_root: str,
        scenario_provider: Optional[ScenarioProvider] = None,
    ) -> None:
        self.nuplan_db_path = nuplan_db_path
        self.map_root = map_root
        if scenario_provider is None:
            raise RuntimeError(
                "scenario_provider is required. Pass a callable that returns a nuPlan scenario object "
                "for the given scene_token."
            )
        self._adapter = NuPlanScenarioAdapter(scenario_provider)

    def load(self, scene_token: str) -> Scenario:
        scenario = self._adapter.get_scenario(scene_token)
        map_name = self._adapter.get_map_name(scenario)
        num_iters = self._adapter.get_num_iterations(scenario)
        if num_iters <= 0:
            raise ValueError("Scenario has no iterations to load.")

        frames: List[Frame] = []
        for idx in range(num_iters):
            ego = self._adapter.get_ego_state(scenario, idx)
            agents = self._adapter.get_agents(scenario, idx, ego.t)
            frames.append(Frame(t=ego.t, ego=ego, agents=agents))

        return Scenario(scene_token=scene_token, map_name=map_name, frames=frames)
