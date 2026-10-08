"""Check servo feedforward against actual MuJoCo dynamics and actuator bounds."""
import mujoco
import numpy as np
import pytest
from tracer_jaka_mujoco.arm_servo import ArmBiasCompensator, configure_position_servos


def model():
    return mujoco.MjModel.from_xml_string('''
    <mujoco><option timestep="0.002"/>
      <worldbody><body pos="0 0 1">
        <joint name="arm" type="hinge" axis="0 1 0" damping="1"/>
        <geom type="capsule" fromto="0 0 0 0.4 0 0" size="0.04" mass="2"/>
      </body></worldbody>
      <actuator><position joint="arm" kp="100" kv="10" ctrlrange="-2 2"
         forcerange="-20 20"/></actuator>
    </mujoco>''')


def test_feedforward_supplies_bias_without_overwriting_state_or_force():
    m = model(); d = mujoco.MjData(m)
    d.qpos[0] = 0.2
    mujoco.mj_forward(m, d)
    before = d.qpos.copy()
    c = ArmBiasCompensator(m, [0], [0])
    d.ctrl[:] = c.controls(d, before)
    mujoco.mj_forward(m, d)
    np.testing.assert_allclose(d.qfrc_actuator, d.qfrc_bias, atol=1e-10)
    np.testing.assert_array_equal(d.qpos, before)
    np.testing.assert_array_equal(d.qfrc_applied, [0])


def test_static_gravity_sag_is_removed_in_physics():
    def settle(compensate):
        m = model(); d = mujoco.MjData(m); d.qpos[0] = 0.2
        c = ArmBiasCompensator(m, [0], [0])
        mujoco.mj_forward(m, d)
        for _ in range(1500):
            d.ctrl[:] = c.controls(d, [0.2]) if compensate else [0.2]
            mujoco.mj_step(m, d)
        return abs(d.qpos[0] - 0.2)
    assert settle(False) > 0.02
    assert settle(True) < 1e-6


def test_control_and_force_limits_remain_in_effect():
    m = model(); d = mujoco.MjData(m)
    c = ArmBiasCompensator(m, [0], [0])
    d.qfrc_bias[0] = 1e6
    d.ctrl[:] = c.controls(d, [1.9])
    assert d.ctrl[0] == 2.0
    mujoco.mj_forward(m, d)
    assert abs(d.actuator_force[0]) <= 20.0


def test_rejects_incompatible_transmission():
    m = model(); m.actuator_gear[0, 0] = 2
    with pytest.raises(ValueError, match='unit-gear'):
        ArmBiasCompensator(m, [0], [0])


def test_tuned_servo_settles_after_step_with_gravity_and_force_limits():
    m = model(); d = mujoco.MjData(m)
    force_range = m.actuator_forcerange.copy()
    configure_position_servos(m, [0], [0], [600], [25], 'implicitfast')
    c = ArmBiasCompensator(m, [0], [0])
    mujoco.mj_forward(m, d)
    samples = []
    for _ in range(1000):
        d.ctrl[:] = c.controls(d, [0.15])
        mujoco.mj_step(m, d)
        samples.append(d.qpos[0])
        assert abs(d.actuator_force[0]) <= 20
    assert abs(samples[-1] - 0.15) < 1e-6
    assert max(samples) < 0.151
    np.testing.assert_array_equal(m.actuator_forcerange, force_range)


def test_physical_servo_still_stops_at_contact():
    xml = '''<mujoco><option timestep="0.002" integrator="implicitfast"/>
      <worldbody><geom type="box" pos="0.33 0 0.86" size="0.05 0.1 0.025"/>
        <body pos="0 0 1"><joint name="arm" type="hinge" axis="0 1 0" damping="1"/>
          <geom type="capsule" fromto="0 0 0 0.4 0 0" size="0.04" mass="2"/>
        </body></worldbody>
      <actuator><position joint="arm" kp="600" kv="25" forcerange="-20 20"/></actuator>
    </mujoco>'''
    m = mujoco.MjModel.from_xml_string(xml); d = mujoco.MjData(m)
    c = ArmBiasCompensator(m, [0], [0])
    mujoco.mj_forward(m, d)
    contact_seen = False
    for _ in range(1000):
        d.ctrl[:] = c.controls(d, [0.7])
        mujoco.mj_step(m, d)
        contact_seen |= d.ncon > 0
        assert abs(d.actuator_force[0]) <= 20
    assert contact_seen
    assert 0.05 < d.qpos[0] < 0.5  # Collision prevents reaching the requested 0.7.


@pytest.mark.parametrize('kp,kv,integrator', [
    ([100], [float('nan')], 'implicitfast'),
    ([-1], [10], 'implicitfast'),
    ([100], [-1], 'implicitfast'),
    ([100, 200], [10], 'implicitfast'),
    ([100], [10], 'model'),  # fixture uses Euler
    ([100], [10], 'typo'),
])
def test_invalid_servo_configuration_leaves_model_unchanged(kp, kv, integrator):
    m = model()
    gains = m.actuator_gainprm.copy(); biases = m.actuator_biasprm.copy()
    original_integrator = m.opt.integrator
    with pytest.raises(ValueError):
        configure_position_servos(m, [0], [0], kp, kv, integrator)
    np.testing.assert_array_equal(m.actuator_gainprm, gains)
    np.testing.assert_array_equal(m.actuator_biasprm, biases)
    assert m.opt.integrator == original_integrator
