"""Wrist rotation sign/remap tests for left vs right arm."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from g1_teleop.bridge.vr_targets import (
    apply_side_wrist_rotation_signs,
    remap_wrist_rotation_delta,
    rotmat_to_rotvec,
    rotvec_to_rotmat,
    wrist_axis_remap_matrix,
)


def _args(**overrides):
    defaults = dict(
        wrist_axis_remap="avp-palm",
        left_wrist_axis_remap=None,
        right_wrist_axis_remap=None,
        left_wrist_rot_sign_x=-1.0,
        left_wrist_rot_sign_y=-1.0,
        left_wrist_rot_sign_z=-1.0,
        right_wrist_rot_sign_x=-1.0,
        right_wrist_rot_sign_y=1.0,
        right_wrist_rot_sign_z=-1.0,
    )
    defaults.update(overrides)
    return SimpleNamespace(**defaults)


def test_left_default_y_sign_mirrors_right():
    args = _args()
    delta = rotvec_to_rotmat(np.array([0.0, 0.4, 0.0], dtype=np.float64))
    left = apply_side_wrist_rotation_signs(delta, "left", args)
    right = apply_side_wrist_rotation_signs(delta, "right", args)
    assert rotmat_to_rotvec(left)[1] * rotmat_to_rotvec(right)[1] < 0.0


def test_avp_palm_left_flips_y_basis_only():
    args = _args(left_wrist_axis_remap="avp-palm-left")
    left = wrist_axis_remap_matrix(args, "left")
    right = wrist_axis_remap_matrix(args, "right")
    assert np.allclose(left[0], right[0])
    assert np.allclose(left[2], right[2])
    assert left[1, 1] == -right[1, 1]
