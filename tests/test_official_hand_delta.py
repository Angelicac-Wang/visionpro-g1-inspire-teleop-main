"""Regression tests for per-arm official-calib wrist delta mapping."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from g1_teleop.bridge.vr_targets import official_hand_delta


def _args(**overrides):
    defaults = dict(
        official_delta_sign_x=-1.0,
        official_delta_sign_y=-1.0,
        official_delta_sign_z=1.0,
        left_official_delta_sign_x=None,
        left_official_delta_sign_y=None,
        left_official_delta_sign_z=None,
        right_official_delta_sign_x=None,
        right_official_delta_sign_y=None,
        right_official_delta_sign_z=None,
        hand_forward_scale=1.55,
        hand_backward_scale=0.35,
        left_hand_forward_scale=None,
        left_hand_backward_scale=None,
        right_hand_forward_scale=None,
        right_hand_backward_scale=None,
        left_hand_delta_remap="unitree-left-arm",
        right_hand_delta_remap="identity",
        body_scale=1.0,
        left_hand_delta_scale=1.0,
        left_hand_delta_z_scale=0.45,
        left_hand_delta_z_up_scale=1.0,
        right_hand_delta_scale=1.0,
    )
    defaults.update(overrides)
    return SimpleNamespace(**defaults)


def test_right_upper_reach_stays_forward():
    """Logged upper-right motion should keep a strong positive robot-X delta."""
    calib = np.array([-0.244, 0.011, -0.385], dtype=np.float64)
    rel = np.array([-0.483, 0.130, -0.027], dtype=np.float64)
    delta = official_hand_delta(rel, calib, _args(), side="right")
    assert delta[0] > 0.30
    assert delta[1] < 0.0
    assert delta[2] > 0.30


def test_left_upper_reach_no_longer_pulls_back():
    """Logged upper-left motion previously produced negative robot-X."""
    calib = np.array([-0.155, -0.222, -0.392], dtype=np.float64)
    rel = np.array([0.062, -0.480, 0.002], dtype=np.float64)
    delta = official_hand_delta(rel, calib, _args(), side="left")
    assert delta[0] > 0.0
    assert delta[1] > 0.25
    assert delta[2] > 0.10


def test_left_forward_reach_stays_forward():
    calib = np.array([-0.155, -0.222, -0.392], dtype=np.float64)
    rel = np.array([-0.199, -0.276, -0.260], dtype=np.float64)
    delta = official_hand_delta(rel, calib, _args(), side="left")
    assert delta[0] > 0.0


def test_legacy_shared_mapping_preserved_for_right_with_identity():
    calib = np.zeros(3, dtype=np.float64)
    rel = np.array([-0.10, 0.05, 0.08], dtype=np.float64)
    delta = official_hand_delta(rel, calib, _args(), side="right")
    expected_x = 0.10 * 1.55
    expected_y = -0.05
    expected_z = 0.08
    np.testing.assert_allclose(delta, [expected_x, expected_y, expected_z], rtol=1e-6)


def test_left_identity_remap_reproduces_old_pullback():
    calib = np.array([-0.155, -0.222, -0.392], dtype=np.float64)
    rel = np.array([0.062, -0.480, 0.002], dtype=np.float64)
    delta = official_hand_delta(rel, calib, _args(left_hand_delta_remap="identity"), side="left")
    assert delta[0] < 0.0


def test_left_reach_up_uses_full_z_not_forward_damp():
    """Regression: reach-up with positive rel_x should not get forward Z damp."""
    calib = np.array([-0.035, -0.033, -0.370], dtype=np.float64)
    rel = np.array([0.233, 0.033, -0.288], dtype=np.float64)
    delta = official_hand_delta(rel, calib, _args(), side="left")
    assert delta[2] > 0.07


def test_left_forward_extension_still_damps_spurious_z():
    calib = np.array([-0.155, -0.222, -0.392], dtype=np.float64)
    rel = np.array([-0.255, -0.230, -0.312], dtype=np.float64)
    delta = official_hand_delta(rel, calib, _args(), side="left")
    damped = official_hand_delta(rel, calib, _args(left_hand_delta_z_scale=1.0), side="left")
    assert delta[2] < damped[2]
