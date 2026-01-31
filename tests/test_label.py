from cfdg.ingest.scene_loader import AgentState, EgoState, Frame
from cfdg.label.labeler import Labeler
from cfdg.map.map_api import MapAPI
from cfdg.sim.rollout import SimFrame
from cfdg.sim.bicycle_model import VehicleState


class DummyMap:
    def lane_centerline(self, _x, _y):
        return [(0.0, 0.0), (10.0, 0.0)]

    def is_off_road(self, _poly):
        return False


def test_labeler_runs():
    labeler = Labeler(config={})
    sim_frames = [
        SimFrame(t=0.0, state=VehicleState(0.0, 0.0, 0.0, 1.0), cmd_steer=0.0, cmd_accel=0.0, perturb_on=False)
    ]
    agents = [AgentState(track_token="a", t=0.0, x=5.0, y=0.0, yaw=0.0, v=0.0, length=4.0, width=2.0, obj_type="car")]
    scenario_frames = [Frame(t=0.0, ego=EgoState(0, 0, 0, 0, 0, 0, 0), agents=agents)]

    api = MapAPI(map_root="/tmp", map_name="dummy", map_provider=lambda _r, _n: DummyMap())
    labels = labeler.compute(sim_frames, scenario_frames, api)
    assert labels.min_ttc >= 0.0
