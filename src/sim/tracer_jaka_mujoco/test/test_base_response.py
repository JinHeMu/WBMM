import math
import pytest
from tracer_jaka_mujoco.base_response import BaseResponse


def test_ideal_mode_preserves_existing_commands():
    plant = BaseResponse()
    assert plant.step(0.0, 0.002, .1, -.4) == (.1, -.4)


def test_step_matches_analytic_first_order():
    plant = BaseResponse(True, linear_tau=.2, angular_tau=.3,
                         linear_gain=.8, angular_gain=1.1)
    for i in range(100):
        v, w = plant.step(i * .002, .002, .1, -.4)
    assert v == pytest.approx(.08 * (1 - math.exp(-1)))
    assert w == pytest.approx(-.44 * (1 - math.exp(-.2 / .3)))


def test_delays_are_independent_and_reset_flushes_pending_commands():
    plant = BaseResponse(True, linear_delay=.12, angular_delay=.14)
    for i in range(60):
        assert plant.step(i * .002, .002, .1, .4) == (0, 0)
    v, w = plant.step(.12, .002, .1, .4)
    assert v > 0 and w == 0
    plant.reset()
    assert plant.step(.122, .002, 0, 0) == (0, 0)
    for i in range(200):
        assert plant.step(.124 + i * .002, .002, 0, 0) == (0, 0)


def test_reversal_retains_momentum_and_release_decays():
    plant = BaseResponse(True)
    for i in range(500):
        plant.step(i * .002, .002, .1, .4)
    v, w = plant.step(1.0, .002, -.1, -.4)
    assert v > 0 and w > 0
    for i in range(1, 1001):
        v, w = plant.step(1 + i * .002, .002, 0, 0)
    assert 0 <= v < .001
    assert 0 <= w < .02


@pytest.mark.parametrize('kwargs', [dict(linear_tau=0), dict(angular_gain=float('nan')),
                                    dict(linear_delay=-.1)])
def test_invalid_settings(kwargs):
    with pytest.raises(ValueError):
        BaseResponse(**kwargs)
