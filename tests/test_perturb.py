from cfdg.perturb.perturbation import PerturbationFactory


def test_perturbation_deterministic():
    cfg = {
        "seed": 42,
        "sample": {"dt": 0.1},
        "perturb": {
            "types": ["impulse"],
            "impulse": {
                "steer_delta": [-0.2, 0.2],
                "acc_delta": [-1.0, 1.0],
                "duration_steps": [1, 3],
                "start_t_range": [0.5, 1.0],
            },
        },
    }
    factory = PerturbationFactory(cfg)
    p1 = factory.sample("scene-abc")
    p2 = factory.sample("scene-abc")
    assert p1 == p2
