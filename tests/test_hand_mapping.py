"""Replay captured AVP metrics through the real hand command mapping."""

import sys

import numpy as np
import pytest

from g1_teleop.bridge.cli import parse_args
from g1_teleop.hand.mapping import HandCalibration, InspireHandMapper


def arguments(monkeypatch, *options):
    monkeypatch.setattr(sys, "argv", ["bridge", *options])
    return parse_args()


def replay(mapper, values):
    # Geometry was already measured in the live bridge capture. Exercise range
    # mapping, expansion, smoothing and command conversion without an AVP.
    raw = dict(zip(("little", "ring", "middle", "index"), values))
    raw.update(thumb_bend=23.8, thumb_rot=10.279)
    mapper.measure_raw = lambda hand: (raw, {})
    for _ in range(30):
        command, _ = mapper.build_command(None)
    return command


def test_recorded_fist_closes_and_open_pose_reopens(monkeypatch, tmp_path):
    args = arguments(monkeypatch, "--enable-inspire-hand-sim")
    mapper = InspireHandMapper.from_args(args, tmp_path / "missing.json")
    assert np.all(replay(mapper, [5, 5, 5, 5])[:4] >= 980)
    # Actual left-hand sample: legacy mapping stayed around 590-665.
    assert np.all(replay(mapper, [65.4, 73.8, 75.0, 69.8])[:4] <= 50)
    assert np.all(replay(mapper, [5, 5, 5, 5])[:4] >= 980)


@pytest.mark.parametrize("options", [[], ["--enable-inspire-hand-dds"],
    ["--enable-inspire-hand-sim", "--enable-inspire-hand-dds"]])
def test_physical_or_mixed_output_keeps_legacy_ranges(monkeypatch, tmp_path, options):
    args = arguments(monkeypatch, *options)
    mapper = InspireHandMapper.from_args(args, tmp_path / "missing.json")
    assert mapper.calibration.range_for("index") == (10, 165)
    assert np.all(replay(mapper, [65.4, 73.8, 75.0, 69.8])[:4] > 500)


def test_personal_calibration_overrides_sim_fallback(monkeypatch, tmp_path):
    args = arguments(monkeypatch, "--enable-inspire-hand-sim")
    path = tmp_path / "personal.json"
    calibration = HandCalibration()
    calibration.channels["index"] = {"open": 4, "close": 85}
    calibration.save(path)
    mapper = InspireHandMapper.from_args(args, path)
    assert mapper.calibration.channels == calibration.channels


def test_distance_thumb_configuration_preserved(monkeypatch, tmp_path):
    args = arguments(monkeypatch, "--enable-inspire-hand-sim",
                     "--thumb-rotation-metric", "distance")
    mapper = InspireHandMapper.from_args(args, tmp_path / "missing.json")
    assert mapper.calibration.range_for("thumb_rot") == (0.12, 0.035)
