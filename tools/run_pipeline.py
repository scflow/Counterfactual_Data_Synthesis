from __future__ import annotations

import argparse
from math import atan2
from typing import Dict, List, Optional, Tuple

from cfdg.ingest.scene_loader import SceneLoader
from cfdg.control.idm import IDMConfig, IDMController
from cfdg.control.pid import PIDConfig, PIDController
from cfdg.io.writer import OutputWriter
from cfdg.label.labeler import Labeler
from cfdg.map.map_api import MapAPI
from cfdg.perturb.perturbation import PerturbationFactory
from cfdg.sim.bicycle_model import VehicleParams, VehicleState
from cfdg.sim.rollout import Simulator
from cfdg.utils.config import AppConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run counterfactual data synthesis pipeline")
    parser.add_argument("--config", required=True)
    parser.add_argument("--scenario_filter", required=False)
    parser.add_argument("--perturb", required=False)
    parser.add_argument("--controller", required=False)
    parser.add_argument("--nuplan_db", required=True)
    parser.add_argument("--map_root", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--scene_token", required=False)
    return parser.parse_args()


def _import_nuplan():
    try:
        from nuplan.planning.scenario_builder.nuplan_db.nuplan_scenario_builder import NuPlanScenarioBuilder
        from nuplan.planning.scenario_builder.scenario_filter import ScenarioFilter
    except Exception as exc:  # pragma: no cover - runtime dependency
        raise RuntimeError(
            "nuPlan devkit is not available. Install it and retry."
        ) from exc
    return NuPlanScenarioBuilder, ScenarioFilter


def _scenario_token(scenario) -> str:
    for attr in ("token", "scenario_token"):
        if hasattr(scenario, attr):
            return str(getattr(scenario, attr))
    raise AttributeError("Scenario does not expose a token attribute.")


def _build_scenarios(args: argparse.Namespace, cfg: AppConfig):
    NuPlanScenarioBuilder, ScenarioFilter = _import_nuplan()
    try:
        from nuplan.planning.utils.multithreading.worker_sequential import Sequential
    except Exception:
        Sequential = None

    nuplan_cfg = cfg.raw.get("nuplan", {})
    filter_cfg = cfg.raw.get("scenario_filter", {})

    scenario_builder = NuPlanScenarioBuilder(
        data_root=args.nuplan_db,
        map_root=args.map_root,
        sensor_root=None,
        db_files=None,
        map_version=nuplan_cfg.get("map_version", "nuplan-maps-v1.0"),
    )

    scenario_filter = ScenarioFilter(
        scenario_types=filter_cfg.get("scenario_types"),
        scenario_tokens=[args.scene_token] if args.scene_token else filter_cfg.get("scenario_tokens"),
        log_names=filter_cfg.get("log_names"),
        map_names=filter_cfg.get("map_names"),
        num_scenarios_per_type=filter_cfg.get("num_scenarios_per_type", 1),
        limit_total_scenarios=filter_cfg.get("limit_total_scenarios", 1 if args.scene_token is None else None),
        timestamp_threshold_s=filter_cfg.get("timestamp_threshold_s"),
        ego_displacement_minimum_m=filter_cfg.get("ego_displacement_minimum_m"),
        expand_scenarios=bool(filter_cfg.get("expand_scenarios", False)),
        remove_invalid_goals=bool(filter_cfg.get("remove_invalid_goals", True)),
        shuffle=filter_cfg.get("shuffle", True),
        ego_start_speed_threshold=filter_cfg.get("ego_start_speed_threshold"),
        ego_stop_speed_threshold=filter_cfg.get("ego_stop_speed_threshold"),
        speed_noise_tolerance=filter_cfg.get("speed_noise_tolerance"),
        token_set_path=filter_cfg.get("token_set_path"),
        fraction_in_token_set_threshold=filter_cfg.get("fraction_in_token_set_threshold"),
        ego_route_radius=filter_cfg.get("ego_route_radius"),
    )

    worker = Sequential() if Sequential is not None else None
    return list(scenario_builder.get_scenarios(scenario_filter, worker=worker))


def _map_provider_from_scenario(scenario):
    def _provider(_root: str, _name: str):
        return scenario.map_api

    return _provider


def _compute_errors(map_api: MapAPI, x: float, y: float, yaw: float) -> Tuple[float, float]:
    centerline = map_api.lane_centerline(x, y)
    if len(centerline) < 2:
        return 0.0, 0.0

    min_dist = float("inf")
    best_seg = None
    for i in range(len(centerline) - 1):
        p0 = centerline[i]
        p1 = centerline[i + 1]
        proj, dist = _project_point_to_segment((x, y), p0, p1)
        if dist < min_dist:
            min_dist = dist
            best_seg = (p0, p1)

    if best_seg is None:
        return 0.0, 0.0

    p0, p1 = best_seg
    seg_heading = atan2(p1[1] - p0[1], p1[0] - p0[0])
    heading_err = _wrap_angle(seg_heading - yaw)

    vx = x - p0[0]
    vy = y - p0[1]
    sx = p1[0] - p0[0]
    sy = p1[1] - p0[1]
    cross = sx * vy - sy * vx
    cte = min_dist if cross >= 0 else -min_dist
    return cte, heading_err


def _project_point_to_segment(
    point: Tuple[float, float], p0: Tuple[float, float], p1: Tuple[float, float]
) -> Tuple[Tuple[float, float], float]:
    px, py = point
    x1, y1 = p0
    x2, y2 = p1
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return p0, ((px - x1) ** 2 + (py - y1) ** 2) ** 0.5
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    proj = (x1 + t * dx, y1 + t * dy)
    dist = ((px - proj[0]) ** 2 + (py - proj[1]) ** 2) ** 0.5
    return proj, dist


def _wrap_angle(angle: float) -> float:
    while angle > 3.141592653589793:
        angle -= 6.283185307179586
    while angle < -3.141592653589793:
        angle += 6.283185307179586
    return angle


def main() -> None:
    args = parse_args()
    configs = [args.config]
    if args.scenario_filter:
        configs.append(args.scenario_filter)
    if args.perturb:
        configs.append(args.perturb)
    if args.controller:
        configs.append(args.controller)
    cfg = AppConfig.from_files(*configs)

    scenarios = _build_scenarios(args, cfg)
    if not scenarios:
        raise SystemExit("No scenarios matched the filter.")

    scenario_by_token: Dict[str, object] = {_scenario_token(s): s for s in scenarios}
    scene_token = args.scene_token or _scenario_token(scenarios[0])

    loader = SceneLoader(args.nuplan_db, args.map_root, scenario_provider=lambda token: scenario_by_token[token])
    internal = loader.load(scene_token)
    scenario = scenario_by_token[scene_token]
    map_api = MapAPI(args.map_root, internal.map_name, map_provider=_map_provider_from_scenario(scenario))

    print(f"Loaded scenario token={internal.scene_token}, map={internal.map_name}, frames={len(internal.frames)}")

    sample_cfg = cfg.raw.get("sample", {})
    dt = sample_cfg.get("dt")
    if dt is None and len(internal.frames) > 1:
        dt = internal.frames[1].t - internal.frames[0].t
    if dt is None or dt <= 0:
        dt = 0.1

    init = internal.frames[0].ego
    init_state = VehicleState(x=init.x, y=init.y, yaw=init.yaw, v=init.v)
    steps = len(internal.frames)

    vehicle_cfg = cfg.raw.get("vehicle", {})
    params = VehicleParams(
        wheel_base=float(vehicle_cfg.get("wheel_base", 2.8)),
        steer_limit=float(vehicle_cfg.get("steer_limit", 0.6)),
        a_min=float(vehicle_cfg.get("a_min", -6.0)),
        a_max=float(vehicle_cfg.get("a_max", 3.0)),
        a_lat_max=float(vehicle_cfg.get("a_lat_max", 4.0)),
    )

    pid_cfg = {"kp": 1.2, "ki": 0.0, "kd": 0.2}
    pid_cfg.update(cfg.raw.get("controller", {}).get("pid", {}))
    idm_cfg = {
        "desired_speed": 13.9,
        "min_gap": 2.0,
        "time_headway": 1.2,
        "max_accel": 1.5,
        "comfortable_brake": 2.0,
        "delta": 4.0,
    }
    idm_cfg.update(cfg.raw.get("controller", {}).get("idm", {}))
    pid = PIDController(PIDConfig(**pid_cfg))
    idm = IDMController(IDMConfig(**idm_cfg))
    pid.reset()

    perturb_factory = PerturbationFactory(cfg.raw)
    perturb = perturb_factory.sample(scene_token)

    recover_cfg = cfg.raw.get("recover", {})
    recover_cte = float(recover_cfg.get("cte_threshold", 0.8))
    recover_yaw = float(recover_cfg.get("yaw_threshold", 0.2))
    recover_gain = float(recover_cfg.get("pid_gain_scale", 2.0))
    recover_min_frames = int(recover_cfg.get("min_frames", 5))
    recover_window = float(recover_cfg.get("window_sec", 2.0))
    recover_speed = float(recover_cfg.get("desired_speed_recover", 6.0))
    recover_active = False
    recover_ok_frames = 0

    def control_fn(state: VehicleState, t: float):
        cte, heading_err = _compute_errors(map_api, state.x, state.y, state.yaw)
        steer = pid.step(cte + heading_err, dt)
        accel = idm.step(state.v, distance=1e6, rel_speed=0.0)
        return steer, accel, cte, heading_err

    def perturb_fn(t: float, _state: VehicleState, steer: float, accel: float):
        on = perturb.start_t <= t <= perturb.start_t + perturb.duration
        if on:
            steer += perturb.steer_delta
            accel += perturb.acc_delta
        return steer, accel, on

    def control_with_recovery(state: VehicleState, t: float):
        nonlocal recover_active, recover_ok_frames
        steer, accel, cte, heading_err = control_fn(state, t)

        if abs(cte) > recover_cte or abs(heading_err) > recover_yaw:
            recover_active = True
        if perturb.start_t + perturb.duration <= t <= perturb.start_t + perturb.duration + recover_window:
            recover_active = True

        if recover_active:
            steer *= recover_gain
            if state.v > recover_speed:
                accel = min(accel, -1.0)

            eps_cte = cfg.raw.get("label", {}).get("eps_cte", 0.3)
            eps_yaw = cfg.raw.get("label", {}).get("eps_yaw", 0.1)
            if abs(cte) <= eps_cte and abs(heading_err) <= eps_yaw:
                recover_ok_frames += 1
            else:
                recover_ok_frames = 0
            if recover_ok_frames >= recover_min_frames:
                recover_active = False

        return steer, accel

    sim = Simulator(params)
    sim_frames = sim.rollout(init_state, steps=steps, dt=dt, control_fn=control_with_recovery, perturb_fn=perturb_fn)

    labeler = Labeler(cfg.raw.get("label", {}))
    labels = labeler.compute(sim_frames, internal.frames, map_api)

    trajectory = [
        {
            "t": f.t,
            "x": f.state.x,
            "y": f.state.y,
            "yaw": f.state.yaw,
            "v": f.state.v,
            "cmd_steer": f.cmd_steer,
            "cmd_accel": f.cmd_accel,
            "perturb_on": f.perturb_on,
        }
        for f in sim_frames
    ]
    meta = {
        "scene_token": internal.scene_token,
        "map_name": internal.map_name,
        "perturbation": {
            "kind": perturb.kind,
            "steer_delta": perturb.steer_delta,
            "acc_delta": perturb.acc_delta,
            "start_t": perturb.start_t,
            "duration": perturb.duration,
        },
        "dt": dt,
    }

    writer = OutputWriter(args.output)
    writer.write_scene(internal.scene_token, trajectory, labels.__dict__, meta)

    print(
        f"Wrote scene output to {args.output}/scenes/{internal.scene_token} "
        f"(collision={labels.collision}, off_road={labels.off_road}, min_ttc={labels.min_ttc:.2f})"
    )


if __name__ == "__main__":
    main()
