import numpy as np

from g1_teleop.bridge.arm_tracking_hold import ArmTrackingHoldState, apply_arm_tracking_hold


def _base_targets():
    pos = np.array([0.4, 0.12, 0.1, 0.4, -0.12, 0.1, 0.0, 0.0, 0.75], dtype=np.float64)
    orn = np.array([1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0], dtype=np.float64)
    return pos, orn


def test_hold_left_arm_when_tracking_lost():
    state = ArmTrackingHoldState()
    pos, orn = _base_targets()
    left_joints = np.array([0.1, 0.2, 0.3], dtype=np.float64)

    apply_arm_tracking_hold(
        pos, orn, left_joints, None,
        left_tracked=True, right_tracked=True,
        left_active=True, right_active=True,
        state=state,
    )

    lost_pos = pos.copy()
    lost_pos[0:3] = 0.0
    lost_orn = orn.copy()
    lost_orn[0:4] = np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float64)
    lost_joints = np.zeros(3, dtype=np.float64)

    out_pos, out_orn, out_left, _ = apply_arm_tracking_hold(
        lost_pos, lost_orn, lost_joints, None,
        left_tracked=False, right_tracked=True,
        left_active=True, right_active=True,
        state=state,
    )

    assert np.allclose(out_pos[0:3], pos[0:3])
    assert np.allclose(out_orn[0:4], orn[0:4])
    assert np.allclose(out_left, left_joints)
    assert state.debug["left_hold"] is True


def test_bootstrap_hold_from_last_sent():
    state = ArmTrackingHoldState()
    pos, orn = _base_targets()
    last_pos = pos.copy()
    last_orn = orn.copy()
    snap_pos = pos.copy()
    snap_pos[0:3] = 0.22

    out_pos, out_orn, _, _ = apply_arm_tracking_hold(
        snap_pos, orn,
        None, None,
        left_tracked=False, right_tracked=True,
        left_active=True, right_active=True,
        state=state,
        last_sent_position=last_pos,
        last_sent_orientation=last_orn,
    )

    assert np.allclose(out_pos[0:3], last_pos[0:3])
    assert state.debug["left_hold"] is True


def test_tracking_return_updates_hold():
    state = ArmTrackingHoldState()
    pos, orn = _base_targets()

    apply_arm_tracking_hold(
        pos, orn, None, None,
        left_tracked=True, right_tracked=True,
        left_active=True, right_active=True,
        state=state,
    )

    new_pos = pos.copy()
    new_pos[0:3] = np.array([0.5, 0.15, 0.12], dtype=np.float64)
    apply_arm_tracking_hold(
        new_pos, orn, None, None,
        left_tracked=True, right_tracked=True,
        left_active=True, right_active=True,
        state=state,
    )

    assert np.allclose(state.left_position, new_pos[0:3])
