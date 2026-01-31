from __future__ import annotations

import argparse
from io import BytesIO
from typing import Iterable, List, Tuple

from PIL import Image
import matplotlib.pyplot as plt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render nuPlan scenario to GIF")
    parser.add_argument("--nuplan_db", required=True)
    parser.add_argument("--map_root", required=True)
    parser.add_argument("--scene_token", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--map_radius", type=float, default=60.0)
    parser.add_argument("--step", type=int, default=2)
    parser.add_argument("--dpi", type=int, default=120)
    parser.add_argument("--trail", type=int, default=50)
    return parser.parse_args()


def _import_nuplan():
    from nuplan.planning.scenario_builder.nuplan_db.nuplan_scenario_builder import NuPlanScenarioBuilder
    from nuplan.planning.scenario_builder.scenario_filter import ScenarioFilter
    from nuplan.planning.utils.multithreading.worker_sequential import Sequential

    return NuPlanScenarioBuilder, ScenarioFilter, Sequential


def _to_xy_list(path) -> List[Tuple[float, float]]:
    if path is None:
        return []
    if isinstance(path, list):
        if len(path) == 0:
            return []
        if isinstance(path[0], (tuple, list)) and len(path[0]) >= 2:
            return [(float(p[0]), float(p[1])) for p in path]
        if hasattr(path[0], "x") and hasattr(path[0], "y"):
            return [(float(p.x), float(p.y)) for p in path]
    if hasattr(path, "discrete_path"):
        return _to_xy_list(path.discrete_path)
    if hasattr(path, "path"):
        return _to_xy_list(path.path)
    if hasattr(path, "xyz"):
        return [(float(p[0]), float(p[1])) for p in path.xyz]
    return []


def _extract_centerline(lane_obj) -> List[Tuple[float, float]]:
    centerline = getattr(lane_obj, "centerline", None)
    if centerline is None:
        centerline = getattr(lane_obj, "baseline_path", None)
    return _to_xy_list(centerline)


def _box_polygon(x: float, y: float, yaw: float, length: float, width: float) -> List[Tuple[float, float]]:
    import math

    dx = length / 2.0
    dy = width / 2.0
    corners = [(-dx, -dy), (-dx, dy), (dx, dy), (dx, -dy)]
    c = math.cos(yaw)
    s = math.sin(yaw)
    points: List[Tuple[float, float]] = []
    for cx, cy in corners:
        rx = cx * c - cy * s + x
        ry = cx * s + cy * c + y
        points.append((rx, ry))
    return points


def _get_pose(obj):
    if obj is None:
        return None
    for name in ("rear_axle", "center", "pose"):
        if hasattr(obj, name):
            return getattr(obj, name)
    if hasattr(obj, "car_footprint") and hasattr(obj.car_footprint, "center"):
        return obj.car_footprint.center
    if hasattr(obj, "box") and hasattr(obj.box, "center"):
        return obj.box.center
    return None


def _xy_yaw(pose) -> Tuple[float, float, float]:
    if pose is None:
        return 0.0, 0.0, 0.0
    x = float(getattr(pose, "x", 0.0))
    y = float(getattr(pose, "y", 0.0))
    yaw = float(getattr(pose, "heading", getattr(pose, "yaw", 0.0)))
    return x, y, yaw


def _iter_tracked(tracked) -> Iterable:
    if tracked is None:
        return []
    if isinstance(tracked, (list, tuple)):
        return tracked
    tracked_objects = getattr(tracked, "tracked_objects", None)
    if tracked_objects is not None:
        return tracked_objects
    if hasattr(tracked, "get_tracked_objects"):
        return tracked.get_tracked_objects()
    return []


def main() -> None:
    args = parse_args()
    NuPlanScenarioBuilder, ScenarioFilter, Sequential = _import_nuplan()

    scenario_builder = NuPlanScenarioBuilder(
        data_root=args.nuplan_db,
        map_root=args.map_root,
        sensor_root=None,
        db_files=None,
        map_version="nuplan-maps-v1.0",
    )
    scenario_filter = ScenarioFilter(
        scenario_types=None,
        scenario_tokens=[args.scene_token],
        log_names=None,
        map_names=None,
        num_scenarios_per_type=None,
        limit_total_scenarios=1,
        timestamp_threshold_s=None,
        ego_displacement_minimum_m=None,
        expand_scenarios=False,
        remove_invalid_goals=True,
        shuffle=False,
    )

    scenarios = list(scenario_builder.get_scenarios(scenario_filter, worker=Sequential()))
    if not scenarios:
        raise SystemExit("No scenario found for token")
    scenario = scenarios[0]
    map_api = scenario.map_api

    num_iters = scenario.get_number_of_iterations()
    step = max(1, int(args.step))

    frames: List[Image.Image] = []
    ego_trail: List[Tuple[float, float]] = []

    for it in range(0, num_iters, step):
        ego = scenario.get_ego_state_at_iteration(it)
        pose = _get_pose(ego)
        ex, ey, eyaw = _xy_yaw(pose)
        ego_trail.append((ex, ey))
        if len(ego_trail) > args.trail:
            ego_trail = ego_trail[-args.trail :]

        fig, ax = plt.subplots(figsize=(6, 6), dpi=args.dpi)
        ax.set_aspect("equal")
        ax.set_facecolor("#f6f1ea")

        # Draw lanes
        try:
            from nuplan.common.actor_state.state_representation import Point2D

            query_point = Point2D(x=ex, y=ey)
        except Exception:
            query_point = (ex, ey)

        try:
            layers = map_api.get_proximal_map_objects(query_point, radius=args.map_radius, layers=["lane"])
            lanes = layers.get("lane", []) if isinstance(layers, dict) else []
        except Exception:
            lanes = []

        for lane in lanes:
            centerline = _extract_centerline(lane)
            if len(centerline) >= 2:
                xs = [p[0] for p in centerline]
                ys = [p[1] for p in centerline]
                ax.plot(xs, ys, color="#7aa5d2", linewidth=1.0, alpha=0.9)

        # Draw agents
        tracked = scenario.get_tracked_objects_at_iteration(it)
        for obj in _iter_tracked(tracked):
            pose = _get_pose(obj)
            x, y, yaw = _xy_yaw(pose)
            length = float(getattr(obj, "length", getattr(getattr(obj, "box", None), "length", 4.0)))
            width = float(getattr(obj, "width", getattr(getattr(obj, "box", None), "width", 2.0)))
            poly = _box_polygon(x, y, yaw, length, width)
            xs = [p[0] for p in poly] + [poly[0][0]]
            ys = [p[1] for p in poly] + [poly[0][1]]
            ax.fill(xs, ys, color="#8fd3a9", alpha=0.8, linewidth=0)

        # Draw ego
        ego_poly = _box_polygon(ex, ey, eyaw, 4.8, 2.0)
        xs = [p[0] for p in ego_poly] + [ego_poly[0][0]]
        ys = [p[1] for p in ego_poly] + [ego_poly[0][1]]
        ax.fill(xs, ys, color="#2f6fed", alpha=0.95, linewidth=0)

        # Draw trail
        if len(ego_trail) > 1:
            tx = [p[0] for p in ego_trail]
            ty = [p[1] for p in ego_trail]
            ax.plot(tx, ty, color="#f59f00", linewidth=1.2, alpha=0.9)

        ax.set_xlim(ex - args.map_radius, ex + args.map_radius)
        ax.set_ylim(ey - args.map_radius, ey + args.map_radius)
        ax.axis("off")

        buf = BytesIO()
        fig.savefig(buf, format="png", bbox_inches="tight", pad_inches=0)
        plt.close(fig)
        buf.seek(0)
        frames.append(Image.open(buf).convert("RGB"))

    if not frames:
        raise SystemExit("No frames rendered")

    frames[0].save(
        args.output,
        save_all=True,
        append_images=frames[1:],
        duration=100,
        loop=0,
    )

    print(f"Wrote GIF: {args.output} ({len(frames)} frames)")


if __name__ == "__main__":
    main()
