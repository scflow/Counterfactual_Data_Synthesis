from __future__ import annotations

import argparse
import os
from io import BytesIO
from typing import Dict, Iterable, List, Optional, Tuple

import geopandas as gpd
from PIL import Image
import matplotlib.pyplot as plt
from shapely.affinity import translate
from shapely.geometry import box


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render nuPlan scenario with full HD map layers to GIF")
    parser.add_argument("--nuplan_db", required=True)
    parser.add_argument("--map_root", required=True)
    parser.add_argument("--scene_token", required=True)
    parser.add_argument("--output_base", required=True, help="output prefix, e.g. data/output/scene")
    parser.add_argument("--map_radius", type=float, default=80.0)
    parser.add_argument("--step", type=int, default=3)
    parser.add_argument("--dpi", type=int, default=180)
    parser.add_argument("--figsize", type=float, nargs=2, default=(6.0, 6.0), metavar=("W", "H"))
    parser.add_argument("--trail", type=int, default=60)
    parser.add_argument("--utm_epsg", type=int, default=None, help="override map UTM EPSG")
    parser.add_argument("--cf_traj", required=False, help="counterfactual trajectory parquet/json path")
    return parser.parse_args()


def _import_nuplan():
    from nuplan.planning.scenario_builder.nuplan_db.nuplan_scenario_builder import NuPlanScenarioBuilder
    from nuplan.planning.scenario_builder.scenario_filter import ScenarioFilter
    from nuplan.planning.utils.multithreading.worker_sequential import Sequential

    return NuPlanScenarioBuilder, ScenarioFilter, Sequential


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


def _find_gpkg(map_root: str, map_name: str) -> str:
    base = os.path.join(map_root, map_name)
    if not os.path.isdir(base):
        raise FileNotFoundError(f"Map folder not found: {base}")
    candidates = []
    for root, _dirs, files in os.walk(base):
        for f in files:
            if f.lower().endswith(".gpkg"):
                candidates.append(os.path.join(root, f))
    if not candidates:
        raise FileNotFoundError(f"No .gpkg found under {base}")
    candidates.sort()
    return candidates[0]


def _load_layers(gpkg_path: str, bbox_bounds, utm_epsg: Optional[int]) -> Dict[str, gpd.GeoDataFrame]:
    layers = gpd.list_layers(gpkg_path)
    layer_names = list(layers["name"]) if hasattr(layers, "__getitem__") else []
    result: Dict[str, gpd.GeoDataFrame] = {}
    for name in layer_names:
        gdf = None
        try:
            gdf = gpd.read_file(gpkg_path, layer=name, bbox=bbox_bounds)
        except Exception:
            gdf = None
        if gdf is None or gdf.empty or "geometry" not in gdf:
            try:
                gdf = gpd.read_file(gpkg_path, layer=name)
            except Exception:
                continue
        if gdf is None or gdf.empty or "geometry" not in gdf:
            continue
        if utm_epsg is not None and gdf.crs is not None:
            try:
                gdf = gdf.to_crs(epsg=utm_epsg)
            except Exception:
                pass
        try:
            minx, miny, maxx, maxy = bbox_bounds
            gdf = gdf.cx[minx:maxx, miny:maxy]
        except Exception:
            pass
        if gdf.empty:
            continue
        result[name] = gdf
    return result


def _compute_bbox(scenario, map_radius: float) -> Tuple[float, float, float, float]:
    num_iters = scenario.get_number_of_iterations()
    xs: List[float] = []
    ys: List[float] = []
    for it in range(num_iters):
        ego = scenario.get_ego_state_at_iteration(it)
        pose = _get_pose(ego)
        x, y, _ = _xy_yaw(pose)
        xs.append(x)
        ys.append(y)
        tracked = scenario.get_tracked_objects_at_iteration(it)
        for obj in _iter_tracked(tracked):
            pose = _get_pose(obj)
            ox, oy, _ = _xy_yaw(pose)
            xs.append(ox)
            ys.append(oy)
    if not xs or not ys:
        return -map_radius, -map_radius, map_radius, map_radius
    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)
    return minx - map_radius, miny - map_radius, maxx + map_radius, maxy + map_radius


def _draw_layers(ax, layers: Dict[str, gpd.GeoDataFrame], offset: Tuple[float, float]):
    ox, oy = offset
    # Approximate nuPlan styling for readability.
    layer_style = {
        "lanes_polygons": {"fill": "#4D6680", "alpha": 0.5, "edge": "#2d3ea7", "lw": 0.6},
        "lane_groups_polygons": {"fill": "#4D6680", "alpha": 0.3, "edge": "#2d3ea7", "lw": 0.5},
        "lane_group_connectors": {"fill": "#4D6680", "alpha": 0.2, "edge": "#2d3ea7", "lw": 0.5},
        "intersections": {"fill": "#7C8691", "alpha": 0.5, "edge": "#2d3ea7", "lw": 0.5},
        "stop_polygons": {"fill": "#FF0101", "alpha": 0.5, "edge": "#FF0101", "lw": 0.6},
        "crosswalks": {"fill": "#B5B5B5", "alpha": 0.3, "edge": "#B5B5B5", "lw": 0.5},
        "walkways": {"fill": "#7e772e", "alpha": 0.5, "edge": "#7e772e", "lw": 0.5},
        "carpark_areas": {"fill": "#ff7f00", "alpha": 0.4, "edge": "#ff7f00", "lw": 0.5},
        "road_segments": {"fill": "#4D6680", "alpha": 0.2, "edge": "#2d3ea7", "lw": 0.4},
        "generic_drivable_areas": {"fill": "#4D6680", "alpha": 0.15, "edge": "#2d3ea7", "lw": 0.4},
        "gen_lane_connectors_scaled_width_polygons": {
            "fill": "#4D6680",
            "alpha": 0.2,
            "edge": "#2d3ea7",
            "lw": 0.4,
        },
        "baseline_paths": {"line": "#CBCBCB", "alpha": 1.0, "lw": 0.6, "dash": (2, 2)},
        "lane_connectors": {"line": "#CBCBCB", "alpha": 1.0, "lw": 0.6, "dash": (2, 2)},
        "boundaries": {"line": "#8f9bb3", "alpha": 0.8, "lw": 0.5},
        "traffic_lights": {"point": "#ff4d4d", "alpha": 0.9, "size": 8},
        "dubins_nodes": {"point": "#00f6ff", "alpha": 0.8, "size": 6},
    }

    draw_order = [
        "generic_drivable_areas",
        "road_segments",
        "lanes_polygons",
        "lane_groups_polygons",
        "lane_group_connectors",
        "gen_lane_connectors_scaled_width_polygons",
        "intersections",
        "crosswalks",
        "stop_polygons",
        "walkways",
        "carpark_areas",
        "baseline_paths",
        "lane_connectors",
        "boundaries",
        "traffic_lights",
        "dubins_nodes",
    ]
    remaining = [name for name in layers.keys() if name not in draw_order]
    draw_order.extend(remaining)

    for name in draw_order:
        if name not in layers:
            continue
        gdf = layers[name]
        geom = gdf.geometry
        if ox != 0.0 or oy != 0.0:
            geom = geom.apply(lambda g: translate(g, xoff=-ox, yoff=-oy))
        style = layer_style.get(name, {})
        geom_type = geom.geom_type
        if geom_type.isin(["Polygon", "MultiPolygon"]).any():
            geom.plot(
                ax=ax,
                color=style.get("fill", "#e6e1d8"),
                edgecolor=style.get("edge", "#c9c2b8"),
                linewidth=style.get("lw", 0.4),
                alpha=style.get("alpha", 0.6),
            )
        elif geom_type.isin(["LineString", "MultiLineString"]).any():
            geom.plot(
                ax=ax,
                color=style.get("line", "#7aa5d2"),
                linewidth=style.get("lw", 0.6),
                alpha=style.get("alpha", 0.9),
            )
        elif geom_type.isin(["Point", "MultiPoint"]).any():
            geom.plot(
                ax=ax,
                color=style.get("point", "#ff4d4d"),
                markersize=style.get("size", 6),
                alpha=style.get("alpha", 0.9),
            )


def _render_gif(
    scenario,
    layers: Dict[str, gpd.GeoDataFrame],
    output: str,
    step: int,
    dpi: int,
    figsize: Tuple[float, float],
    trail: int,
    offset: Tuple[float, float],
    cf_traj: Optional[List[Tuple[float, float]]] = None,
):
    num_iters = scenario.get_number_of_iterations()
    frames: List[Image.Image] = []
    ego_trail: List[Tuple[float, float]] = []
    cf_trail: List[Tuple[float, float]] = []

    for it in range(0, num_iters, step):
        ego = scenario.get_ego_state_at_iteration(it)
        pose = _get_pose(ego)
        ex, ey, eyaw = _xy_yaw(pose)
        ex -= offset[0]
        ey -= offset[1]

        ego_trail.append((ex, ey))
        if len(ego_trail) > trail:
            ego_trail = ego_trail[-trail:]
        if cf_traj is not None and it < len(cf_traj):
            cx, cy = cf_traj[it]
            cx -= offset[0]
            cy -= offset[1]
            cf_trail.append((cx, cy))
            if len(cf_trail) > trail:
                cf_trail = cf_trail[-trail:]

        fig, ax = plt.subplots(figsize=figsize, dpi=dpi)
        ax.set_aspect("equal")
        ax.set_facecolor("#000000")

        _draw_layers(ax, layers, offset)

        tracked = scenario.get_tracked_objects_at_iteration(it)
        for obj in _iter_tracked(tracked):
            pose = _get_pose(obj)
            x, y, yaw = _xy_yaw(pose)
            x -= offset[0]
            y -= offset[1]
            length = float(getattr(obj, "length", getattr(getattr(obj, "box", None), "length", 4.0)))
            width = float(getattr(obj, "width", getattr(getattr(obj, "box", None), "width", 2.0)))
            poly = _box_polygon(x, y, yaw, length, width)
            xs = [p[0] for p in poly] + [poly[0][0]]
            ys = [p[1] for p in poly] + [poly[0][1]]
            ax.fill(xs, ys, color="#8fd3a9", alpha=0.85, linewidth=0)

        ego_poly = _box_polygon(ex, ey, eyaw, 4.8, 2.0)
        xs = [p[0] for p in ego_poly] + [ego_poly[0][0]]
        ys = [p[1] for p in ego_poly] + [ego_poly[0][1]]
        ax.fill(xs, ys, color="#2f6fed", alpha=0.95, linewidth=0)

        if len(ego_trail) > 1:
            tx = [p[0] for p in ego_trail]
            ty = [p[1] for p in ego_trail]
            ax.plot(tx, ty, color="#f59f00", linewidth=1.2, alpha=0.9)
        if len(cf_trail) > 1:
            tx = [p[0] for p in cf_trail]
            ty = [p[1] for p in cf_trail]
            ax.plot(tx, ty, color="#d9480f", linewidth=1.4, alpha=0.9)

        ax.axis("off")
        buf = BytesIO()
        fig.savefig(buf, format="png", bbox_inches="tight", pad_inches=0)
        plt.close(fig)
        buf.seek(0)
        frames.append(Image.open(buf).convert("RGB"))

    if not frames:
        raise SystemExit("No frames rendered")

    frames[0].save(output, save_all=True, append_images=frames[1:], duration=100, loop=0)


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

    map_name = scenario.map_api.map_name
    gpkg = _find_gpkg(args.map_root, map_name)

    minx, miny, maxx, maxy = _compute_bbox(scenario, args.map_radius)
    bbox_bounds = (minx, miny, maxx, maxy)
    map_to_utm = {
        "us-nv-las-vegas-strip": 32611,
        "us-ma-boston": 32619,
        "us-pa-pittsburgh-hazelwood": 32617,
        "sg-one-north": 32648,
    }
    utm_epsg = args.utm_epsg or map_to_utm.get(map_name)
    layers = _load_layers(gpkg, bbox_bounds, utm_epsg)
    if not layers:
        raise SystemExit("No layers loaded from gpkg")

    cf_points: Optional[List[Tuple[float, float]]] = None
    if args.cf_traj:
        cf_path = args.cf_traj
    else:
        cf_path = os.path.join("data", "output", "scenes", args.scene_token, "trajectory.parquet")
    if os.path.exists(cf_path):
        try:
            import pandas as pd

            if cf_path.endswith(".parquet"):
                df = pd.read_parquet(cf_path)
            else:
                df = pd.read_json(cf_path)
            cf_points = list(zip(df["x"].tolist(), df["y"].tolist()))
        except Exception:
            cf_points = None

    # ENU
    enu_out = f"{args.output_base}_enu.gif"
    _render_gif(
        scenario,
        layers,
        enu_out,
        args.step,
        args.dpi,
        tuple(args.figsize),
        args.trail,
        offset=(0.0, 0.0),
        cf_traj=cf_points,
    )

    # Relative to initial ego pose
    ego0 = scenario.get_ego_state_at_iteration(0)
    ex0, ey0, _ = _xy_yaw(_get_pose(ego0))
    rel_out = f"{args.output_base}_rel.gif"
    _render_gif(
        scenario,
        layers,
        rel_out,
        args.step,
        args.dpi,
        tuple(args.figsize),
        args.trail,
        offset=(ex0, ey0),
        cf_traj=cf_points,
    )

    print(f"Wrote GIFs: {enu_out}, {rel_out}")


if __name__ == "__main__":
    main()
