from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, sin, sqrt
from typing import List, Optional, Tuple

from shapely.geometry import Polygon

from cfdg.ingest.scene_loader import Frame
from cfdg.map.map_api import MapAPI
from cfdg.sim.rollout import SimFrame


@dataclass
class Labels:
    collision: bool
    off_road: bool
    is_recovered: bool
    min_ttc: float


class Labeler:
    def __init__(self, config: dict) -> None:
        self.config = config

    def compute(
        self,
        sim_frames: List[SimFrame],
        scenario_frames: List[Frame],
        map_api: MapAPI,
    ) -> Labels:
        if not sim_frames:
            return Labels(collision=False, off_road=False, is_recovered=False, min_ttc=float("inf"))

        ego_length = float(self.config.get("ego_length", 4.8))
        ego_width = float(self.config.get("ego_width", 2.0))
        agent_length_default = float(self.config.get("agent_length", 4.8))
        agent_width_default = float(self.config.get("agent_width", 2.0))

        recover_time = float(self.config.get("recover_time", 3.0))
        eps_cte = float(self.config.get("eps_cte", 0.3))
        eps_yaw = float(self.config.get("eps_yaw", 0.1))

        collision = False
        off_road = False
        min_ttc = float("inf")
        recovered = False

        for idx, sim in enumerate(sim_frames):
            ego_poly = _box_polygon(sim.state.x, sim.state.y, sim.state.yaw, ego_length, ego_width)
            off_road = off_road or map_api.is_off_road(list(ego_poly.exterior.coords)[:-1])

            if idx < len(scenario_frames):
                agents = scenario_frames[idx].agents
            else:
                agents = []

            for agent in agents:
                length = agent.length if agent.length > 0 else agent_length_default
                width = agent.width if agent.width > 0 else agent_width_default
                agent_poly = _box_polygon(agent.x, agent.y, agent.yaw, length, width)
                if ego_poly.intersects(agent_poly):
                    collision = True
                # simple TTC estimate
                dist = sqrt((agent.x - sim.state.x) ** 2 + (agent.y - sim.state.y) ** 2)
                rel_speed = sim.state.v - agent.v
                if rel_speed > 0.1:
                    min_ttc = min(min_ttc, dist / rel_speed)

            if sim.t >= recover_time:
                cte, heading_err = _compute_errors(map_api, sim.state.x, sim.state.y, sim.state.yaw)
                if abs(cte) <= eps_cte and abs(heading_err) <= eps_yaw:
                    recovered = True

        return Labels(collision=collision, off_road=off_road, is_recovered=recovered, min_ttc=min_ttc)


def _box_polygon(x: float, y: float, yaw: float, length: float, width: float) -> Polygon:
    dx = length / 2.0
    dy = width / 2.0
    corners = [(-dx, -dy), (-dx, dy), (dx, dy), (dx, -dy)]
    c = cos(yaw)
    s = sin(yaw)
    points: List[Tuple[float, float]] = []
    for cx, cy in corners:
        rx = cx * c - cy * s + x
        ry = cx * s + cy * c + y
        points.append((rx, ry))
    return Polygon(points)


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
            best_seg = (p0, p1, proj)

    if best_seg is None:
        return 0.0, 0.0

    p0, p1, proj = best_seg
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
        return p0, sqrt((px - x1) ** 2 + (py - y1) ** 2)
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    proj = (x1 + t * dx, y1 + t * dy)
    dist = sqrt((px - proj[0]) ** 2 + (py - proj[1]) ** 2)
    return proj, dist


def _wrap_angle(angle: float) -> float:
    while angle > 3.141592653589793:
        angle -= 2.0 * 3.141592653589793
    while angle < -3.141592653589793:
        angle += 2.0 * 3.141592653589793
    return angle
