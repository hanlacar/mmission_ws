"""Basic package import test."""


def test_bicycle_model_import():
    from mission_sim.bicycle_model import BicycleModel
    assert BicycleModel is not None
