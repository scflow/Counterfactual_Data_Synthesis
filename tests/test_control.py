from cfdg.control.idm import IDMConfig, IDMController
from cfdg.control.pid import PIDConfig, PIDController


def test_pid_step_runs():
    pid = PIDController(PIDConfig(kp=1.0, ki=0.0, kd=0.0))
    out = pid.step(1.0, 0.1)
    assert out == 1.0


def test_idm_step_runs():
    idm = IDMController(IDMConfig(desired_speed=10.0, min_gap=2.0, time_headway=1.0))
    accel = idm.step(v=5.0, distance=10.0, rel_speed=-1.0)
    assert isinstance(accel, float)
