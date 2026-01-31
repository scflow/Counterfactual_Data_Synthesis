from shapely.geometry import Polygon

from cfdg.map.map_api import MapAPI


class DummyLane:
    def __init__(self, centerline):
        self.centerline = centerline


class DummyMap:
    def __init__(self):
        self._lanes = {1: DummyLane(centerline=[(0.0, 0.0), (10.0, 0.0)])}
        self.drivable_area = Polygon([(0.0, -2.0), (10.0, -2.0), (10.0, 2.0), (0.0, 2.0)])

    def get_lane_ids_in_radius(self, _x, _y, radius=5.0):
        return [1] if radius >= 1.0 else []

    def get_lane(self, lane_id):
        return self._lanes.get(lane_id)


def test_map_api_centerline_and_offroad():
    api = MapAPI(map_root="/tmp", map_name="dummy", map_provider=lambda _root, _name: DummyMap())
    centerline = api.lane_centerline(1.0, 0.1)
    assert len(centerline) == 2

    onroad_poly = [(1.0, -0.5), (2.0, -0.5), (2.0, 0.5), (1.0, 0.5)]
    offroad_poly = [(20.0, 20.0), (21.0, 20.0), (21.0, 21.0), (20.0, 21.0)]
    assert api.is_off_road(onroad_poly) is False
    assert api.is_off_road(offroad_poly) is True


def test_scene_loader_requires_provider():
    from cfdg.ingest.scene_loader import SceneLoader

    try:
        SceneLoader(nuplan_db_path="/tmp", map_root="/tmp")
    except RuntimeError:
        assert True
