"""Protocol and real MuJoCo dynamics checks; never connect to robot DDS."""

from pathlib import Path
import time

import mujoco
import numpy as np
import pytest

from g1_teleop.bridge.constants import HEADER_SIZE
from g1_teleop.bridge.zmq_pub import PackedPublisher
from g1_teleop.sim.inspire_hand import InspireHandController, decode_command

SCENE = Path(__file__).resolve().parents[1] / "assets/mujoco/g1_runtime/scene_inspire_hand.xml"


def packet(left, right):
    """Use the actual bridge encoder, without binding its production port."""
    pub = object.__new__(PackedPublisher)
    messages = []

    class Sink:
        def send(self, message):
            messages.append(message)

    pub.socket = Sink()
    pub.send_inspire_hand(np.asarray(left), np.asarray(right))
    return messages[0]


@pytest.fixture
def hand():
    if not SCENE.exists():
        pytest.skip("Run tools/setup_inspire_sim.py to fetch official model assets")
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    model.opt.gravity[:] = 0
    model.opt.timestep = 0.005
    data = mujoco.MjData(model)
    controller = InspireHandController(model, data, connect=False)
    yield model, data, controller
    controller.close()


def advance(model, data, controller, command, steps=300):
    for frame in range(steps):
        now = frame * model.opt.timestep
        assert controller.receive(command, now)
        torque = controller.torques(model.opt.timestep, now)
        data.ctrl[:] = 0
        data.ctrl[controller.indices["left"] - 1] = torque[:12]
        data.ctrl[controller.indices["right"] - 1] = torque[12:]
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all()
    assert not any(warning.number for warning in data.warning)


def test_bridge_packet_roundtrip():
    left = [1000, 800, 600, 400, 200, 0]
    right = left[::-1]
    decoded = decode_command(packet(left, right))
    np.testing.assert_allclose(decoded["left"], left)
    np.testing.assert_allclose(decoded["right"], right)


@pytest.mark.parametrize("message", [
    b"invalid", b"inspire_hand" + b"[]".ljust(HEADER_SIZE, b"\0"),
    packet([np.nan] * 6, [1000] * 6), packet([0] * 6, [0] * 6)[:-2],
])
def test_invalid_command_preserves_last_target(hand, message):
    _, _, controller = hand
    assert controller.receive(packet([400] * 6, [600] * 6), 1.0)
    assert not controller.receive(message, 2.0)
    assert controller.last_received == 1.0
    np.testing.assert_array_equal(controller.commands["left"], [400] * 6)


def test_independent_channels_and_mimic_dynamics(hand):
    model, data, controller = hand
    # Left index only, right little only; these must not move the opposite digit.
    left = [1000, 1000, 1000, 0, 1000, 1000]
    right = [0, 1000, 1000, 1000, 1000, 1000]
    advance(model, data, controller, packet(left, right))
    for side, channel in (("left", 3), ("right", 0)):
        q = data.qpos[model.jnt_qposadr[controller.active[side]]]
        assert q[channel] > 1.3
        assert np.max(np.abs(np.delete(q, channel))) < 0.06
        finger = "index" if side == "left" else "little"
        distal = model.joint(f"{side}_hand_{finger}_2_joint").qposadr[0]
        assert abs(data.qpos[distal] - 1.0843 * q[channel]) < 0.03


def test_both_hands_close_and_reopen(hand):
    model, data, controller = hand
    advance(model, data, controller, packet([0] * 6, [0] * 6))
    for side in ("left", "right"):
        active = controller.active[side]
        np.testing.assert_allclose(data.qpos[model.jnt_qposadr[active]], model.jnt_range[active, 1], atol=0.06)
    advance(model, data, controller, packet([1000] * 6, [1000] * 6))
    for active in controller.active.values():
        np.testing.assert_allclose(data.qpos[model.jnt_qposadr[active]], 0, atol=0.06)


def test_captured_fist_mapping_reaches_sim_finger_limits(hand, monkeypatch, tmp_path):
    import sys
    from g1_teleop.bridge.cli import parse_args
    from g1_teleop.hand.mapping import InspireHandMapper

    monkeypatch.setattr(sys, "argv", ["bridge", "--enable-inspire-hand-sim"])
    mapper = InspireHandMapper.from_args(parse_args(), tmp_path / "missing.json")
    raw = dict(little=65.4, ring=73.8, middle=75.0, index=69.8,
               thumb_bend=23.8, thumb_rot=10.279)
    monkeypatch.setattr(mapper, "measure_raw", lambda hand: (raw, {}))
    command, _ = mapper.build_command(None)
    model, data, controller = hand
    advance(model, data, controller, packet(command, command))
    for active in controller.active.values():
        fingers = active[:4]
        np.testing.assert_allclose(data.qpos[model.jnt_qposadr[fingers]],
                                   model.jnt_range[fingers, 1], atol=0.08)


def test_timeout_opens_fingers(hand):
    _, _, controller = hand
    controller.receive(packet([0] * 6, [0] * 6), 10.0)
    controller.torques(0.005, now=10.6)
    assert controller.timed_out
    for command in controller.commands.values():
        np.testing.assert_array_equal(command, [1000] * 6)


def test_model_preserves_body_actuator_order(hand):
    model, _, _ = hand
    body_names = ["hip", "knee", "ankle", "waist", "shoulder", "elbow", "wrist"]
    body_joints = [i for i in range(model.njnt) if any(n in model.joint(i).name for n in body_names)]
    assert len(body_joints) == 29
    assert model.nu == 53
    for joint in body_joints:
        assert model.actuator_trnid[joint - 1, 0] == joint
    assert model.camera("head_camera").id >= 0


def test_real_zmq_subscription(hand):
    model, data, _ = hand
    # ZMQ selects a private random port; production port 5556 is untouched.
    import zmq
    pub = PackedPublisher("127.0.0.1", 0, connect_delay=0)
    endpoint = pub.socket.getsockopt(zmq.LAST_ENDPOINT).decode()
    controller = InspireHandController(model, data, endpoint)
    try:
        deadline = time.monotonic() + 3
        while not controller.received and time.monotonic() < deadline:
            pub.send_inspire_hand(np.full(6, 300), np.full(6, 700))
            time.sleep(0.02)
            controller.torques(0.005)
        assert controller.received
        np.testing.assert_array_equal(controller.commands["left"], [300] * 6)
        np.testing.assert_array_equal(controller.commands["right"], [700] * 6)
    finally:
        controller.close()
        pub.close()
