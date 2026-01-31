from cfdg.sim.bicycle_model import VehicleParams, VehicleState, step


def test_bicycle_step_runs():
    params = VehicleParams(wheel_base=2.8, steer_limit=0.6, a_min=-6.0, a_max=3.0, a_lat_max=4.0)
    state = VehicleState(x=0.0, y=0.0, yaw=0.0, v=1.0)
    next_state = step(state, steer=0.0, accel=0.0, dt=0.1, params=params)
    assert next_state.x > state.x


def test_rollout_callback_runs():
    from cfdg.sim.rollout import Simulator

    params = VehicleParams(wheel_base=2.8, steer_limit=0.6, a_min=-6.0, a_max=3.0, a_lat_max=4.0)
    sim = Simulator(params)

    def control_fn(_state, _t):
        return 0.0, 0.0

    frames = sim.rollout(VehicleState(0.0, 0.0, 0.0, 1.0), steps=5, dt=0.1, control_fn=control_fn)
    assert len(frames) == 5
