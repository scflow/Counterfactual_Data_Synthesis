from __future__ import annotations

from typing import Any, Callable, Iterable, List, Optional, Tuple


MapProvider = Callable[[str, str], Any]


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


def _to_xy_list(path: Any) -> List[Tuple[float, float]]:
    if path is None:
        return []
    if isinstance(path, list):
        if len(path) == 0:
            return []
        if isinstance(path[0], (tuple, list)) and len(path[0]) >= 2:
            return [(float(p[0]), float(p[1])) for p in path]
        # list of objects with x/y
        if hasattr(path[0], "x") and hasattr(path[0], "y"):
            return [(float(p.x), float(p.y)) for p in path]
    # handle nuPlan baseline path / discrete path objects
    if hasattr(path, "discrete_path"):
        return _to_xy_list(path.discrete_path)
    if hasattr(path, "path"):
        return _to_xy_list(path.path)
    if hasattr(path, "xyz"):
        return [(float(p[0]), float(p[1])) for p in path.xyz]
    return []


def _extract_centerline(lane_obj: Any) -> List[Tuple[float, float]]:
    centerline = _get_attr(lane_obj, ("centerline", "baseline_path"))
    if centerline is None and hasattr(lane_obj, "baseline_path"):
        centerline = lane_obj.baseline_path
    return _to_xy_list(centerline)


def _safe_import_shapely():
    try:
        from shapely.geometry import Polygon
        from shapely.ops import unary_union

        return Polygon, unary_union
    except Exception:
        return None, None


class MapAPI:
    """Thin wrapper around nuPlan map queries.

    This wrapper avoids hard dependency on nuPlan at import time.
    Provide a map_provider callable that returns a nuPlan-like map object.
    """

    def __init__(self, map_root: str, map_name: str, map_provider: Optional[MapProvider] = None) -> None:
        self.map_root = map_root
        self.map_name = map_name
        if map_provider is None:
            raise RuntimeError(
                "map_provider is required. Pass a callable that returns a map API instance for the given map name."
            )
        self._map_api = map_provider(map_root, map_name)

    def lane_centerline(self, x: float, y: float) -> List[Tuple[float, float]]:
        # nuPlan map API (recommended path)
        if hasattr(self._map_api, "get_proximal_map_objects"):
            try:
                from nuplan.common.actor_state.state_representation import Point2D

                point = Point2D(x=x, y=y)
            except Exception:
                point = (x, y)
            try:
                layers = self._map_api.get_proximal_map_objects(point, radius=5.0, layers=["lane"])
                lanes = layers.get("lane", []) if isinstance(layers, dict) else []
                if lanes:
                    return _extract_centerline(lanes[0])
            except Exception:
                pass

        # Direct map API passthroughs.
        if hasattr(self._map_api, "lane_centerline"):
            return _to_xy_list(self._map_api.lane_centerline(x, y))
        if hasattr(self._map_api, "get_lane_centerline"):
            return _to_xy_list(self._map_api.get_lane_centerline(x, y))

        # Try radius-based lane queries.
        lane_ids = None
        if hasattr(self._map_api, "get_lane_ids_in_radius"):
            lane_ids = self._map_api.get_lane_ids_in_radius(x, y, radius=5.0)
        elif hasattr(self._map_api, "get_lane_ids_in_xy_radius"):
            lane_ids = self._map_api.get_lane_ids_in_xy_radius(x, y, 5.0)
        elif hasattr(self._map_api, "get_nearest_lane_id"):
            lane_ids = [self._map_api.get_nearest_lane_id(x, y)]
        elif hasattr(self._map_api, "get_lane_id_at_position"):
            lane_ids = [self._map_api.get_lane_id_at_position(x, y)]

        if not lane_ids:
            return []

        lane_id = lane_ids[0]
        lane_obj = None
        if hasattr(self._map_api, "get_lane"):
            lane_obj = self._map_api.get_lane(lane_id)
        elif hasattr(self._map_api, "get_map_object"):
            lane_obj = self._map_api.get_map_object(lane_id, "LANE")

        if lane_obj is None and hasattr(self._map_api, "get_lane_centerline"):
            return _to_xy_list(self._map_api.get_lane_centerline(lane_id))
        if lane_obj is None:
            return []
        return _extract_centerline(lane_obj)

    def is_off_road(self, polygon_xy: List[Tuple[float, float]]) -> bool:
        if not polygon_xy:
            return True

        # Map API with direct polygon check.
        for method_name in ("is_on_road", "is_in_road", "is_on_drivable_area", "is_in_drivable_area"):
            if hasattr(self._map_api, method_name):
                checker = getattr(self._map_api, method_name)
                # Use centroid for efficiency; if centroid is off-road, treat as off-road.
                cx = sum(p[0] for p in polygon_xy) / len(polygon_xy)
                cy = sum(p[1] for p in polygon_xy) / len(polygon_xy)
                return not bool(checker(cx, cy))

        # Try to obtain drivable area polygons.
        drivable = None
        if hasattr(self._map_api, "get_drivable_area_polygon"):
            drivable = self._map_api.get_drivable_area_polygon()
        elif hasattr(self._map_api, "drivable_area"):
            drivable = self._map_api.drivable_area

        Polygon, unary_union = _safe_import_shapely()
        if Polygon is None:
            # Fallback: if no geometry engine, rely on centroid query only.
            cx = sum(p[0] for p in polygon_xy) / len(polygon_xy)
            cy = sum(p[1] for p in polygon_xy) / len(polygon_xy)
            return not (cx == cx and cy == cy)  # NaN guard; otherwise assume on-road

        ego_poly = Polygon(polygon_xy)
        if drivable is None:
            # Unknown drivable area -> conservatively keep as on-road.
            return False

        if isinstance(drivable, list):
            drivable_poly = unary_union(drivable)
        else:
            drivable_poly = drivable

        return not drivable_poly.contains(ego_poly)
