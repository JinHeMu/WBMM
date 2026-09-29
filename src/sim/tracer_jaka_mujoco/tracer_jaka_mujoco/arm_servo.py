"""Bias feedforward for direct-drive MuJoCo position servos (no ROS dependency)."""
import mujoco
import numpy as np


class ArmBiasCompensator:
    """Add gravity/Coriolis feedforward through the existing bounded actuators.

    For a unit-gear position servo, force = kp*(ctrl-q) - kv*qdot.
    Adding qfrc_bias/kp to ctrl preserves the requested position while supplying
    the model's gravity/Coriolis load. Contact forces are deliberately excluded.
    Neither qpos nor external applied forces are overwritten.
    """

    def __init__(self, model, actuator_ids, dof_ids):
        self.actuator_ids = np.asarray(actuator_ids, dtype=int)
        self.dof_ids = np.asarray(dof_ids, dtype=int)
        if len(self.actuator_ids) != len(self.dof_ids):
            raise ValueError('Arm actuator and DOF counts differ')
        for aid, did in zip(self.actuator_ids, self.dof_ids):
            if aid < 0 or aid >= model.nu or did < 0 or did >= model.nv:
                raise ValueError('Missing arm actuator or DOF')
            joint = model.actuator_trnid[aid, 0]
            kp = model.actuator_gainprm[aid, 0]
            if (model.actuator_trntype[aid] != mujoco.mjtTrn.mjTRN_JOINT or
                    model.jnt_dofadr[joint] != did or
                    model.jnt_type[joint] not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE) or
                    not np.array_equal(model.actuator_gear[aid], [1, 0, 0, 0, 0, 0]) or
                    model.actuator_gaintype[aid] != mujoco.mjtGain.mjGAIN_FIXED or
                    model.actuator_biastype[aid] != mujoco.mjtBias.mjBIAS_AFFINE or
                    model.actuator_dyntype[aid] != mujoco.mjtDyn.mjDYN_NONE or
                    not np.isfinite(kp) or kp <= 0 or
                    not np.isclose(model.actuator_biasprm[aid, 1], -kp) or
                    model.actuator_biasprm[aid, 0] != 0):
                raise ValueError('Arm bias compensation requires unit-gear position servos')
        self.kp = model.actuator_gainprm[self.actuator_ids, 0].copy()
        self.limited = model.actuator_ctrllimited[self.actuator_ids].astype(bool)
        self.ranges = model.actuator_ctrlrange[self.actuator_ids].copy()

    def controls(self, data, positions):
        controls = np.asarray(positions, dtype=float) + data.qfrc_bias[self.dof_ids] / self.kp
        controls[self.limited] = np.clip(controls[self.limited],
                                          self.ranges[self.limited, 0],
                                          self.ranges[self.limited, 1])
        return controls
