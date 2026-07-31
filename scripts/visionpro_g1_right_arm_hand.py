import argparse
import os
import sys
import time
from contextlib import contextmanager

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
UNITREE_SIM_ROOT = os.environ.get("UNITREE_SIM_ROOT", "/mnt/newssd/unitree_sim_isaaclab")
UNITREE_SDK2_ROOT = os.environ.get("UNITREE_SDK2_ROOT", os.path.join(UNITREE_SIM_ROOT, "unitree_sdk2_python"))
XR_TELEOP_ROOT = os.environ.get("XR_TELEOP_ROOT", os.path.join(UNITREE_SIM_ROOT, "xr_teleoperate"))
XR_TELEOP_TELEOP = os.path.join(XR_TELEOP_ROOT, "teleop")
INSPIRE_HAND_SDK_ROOT = os.environ.get(
    "INSPIRE_HAND_SDK_ROOT",
    os.path.join(UNITREE_SIM_ROOT, "inspire_hand_ws", "inspire_hand_sdk"),
)
VISIONPRO_TELEOP_ROOT = os.environ.get(
    "VISIONPRO_TELEOP_ROOT",
    os.path.join(UNITREE_SIM_ROOT, "inspire_hand_ws", "VisionProTeleop"),
)

if UNITREE_SDK2_ROOT not in sys.path:
    sys.path.insert(0, UNITREE_SDK2_ROOT)
if INSPIRE_HAND_SDK_ROOT not in sys.path:
    sys.path.insert(0, INSPIRE_HAND_SDK_ROOT)
if VISIONPRO_TELEOP_ROOT not in sys.path:
    sys.path.insert(0, VISIONPRO_TELEOP_ROOT)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import numpy as np
import pinocchio as pin
from avp_stream import VisionProStreamer
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher

if XR_TELEOP_ROOT not in sys.path:
    sys.path.append(XR_TELEOP_ROOT)
if XR_TELEOP_TELEOP not in sys.path:
    sys.path.append(XR_TELEOP_TELEOP)

from teleop.robot_control.robot_arm import G1_29_ArmController
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK

from inspire_sdkpy import inspire_dds, inspire_hand_defaut

from g1_teleop.hand.mapping import (
    HandCalibration,
    InspireHandMapper,
    format_debug as format_hand_debug,
    sample_raw_metrics,
)


YUP2ZUP = np.array(
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
T_ROBOT_OPENXR = np.array(
    [
        [0.0, 0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
T_OPENXR_ROBOT = np.array(
    [
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
T_TO_UNITREE_HUMANOID_RIGHT_ARM = np.array(
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
INV_YUP2ZUP = np.linalg.inv(YUP2ZUP)

FINGER_CHAINS = {
    "little": ("littleKnuckle", "littleIntermediateBase", "littleIntermediateTip", "littleTip"),
    "ring": ("ringKnuckle", "ringIntermediateBase", "ringIntermediateTip", "ringTip"),
    "middle": ("middleKnuckle", "middleIntermediateBase", "middleIntermediateTip", "middleTip"),
    "index": ("indexKnuckle", "indexIntermediateBase", "indexIntermediateTip", "indexTip"),
    "thumb": ("thumbKnuckle", "thumbIntermediateBase", "thumbIntermediateTip", "thumbTip"),
}


@contextmanager
def pushd(path):
    previous = os.getcwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(previous)


def pose_to_vec(pose):
    return np.asarray(pose[:3, 3], dtype=np.float64)


def clip_vector(vec, limits):
    clipped = np.asarray(vec, dtype=np.float64).copy()
    clipped[0] = np.clip(clipped[0], -limits[0], limits[0])
    clipped[1] = np.clip(clipped[1], -limits[1], limits[1])
    clipped[2] = np.clip(clipped[2], -limits[2], limits[2])
    return clipped


def rotation_matrix_xyz_deg(rx_deg, ry_deg, rz_deg):
    rx = np.deg2rad(rx_deg)
    ry = np.deg2rad(ry_deg)
    rz = np.deg2rad(rz_deg)

    rot_x = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(rx), -np.sin(rx)],
            [0.0, np.sin(rx), np.cos(rx)],
        ],
        dtype=np.float64,
    )
    rot_y = np.array(
        [
            [np.cos(ry), 0.0, np.sin(ry)],
            [0.0, 1.0, 0.0],
            [-np.sin(ry), 0.0, np.cos(ry)],
        ],
        dtype=np.float64,
    )
    rot_z = np.array(
        [
            [np.cos(rz), -np.sin(rz), 0.0],
            [np.sin(rz), np.cos(rz), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return rot_x @ rot_y @ rot_z


def tracking_to_unitree_right_wrist_pose(tracking, head_to_waist_offset):
    right_wrist_avp = np.asarray(tracking.right.wrist, dtype=np.float64)
    head_avp = np.asarray(tracking.head, dtype=np.float64)

    right_wrist_openxr = INV_YUP2ZUP @ right_wrist_avp
    head_openxr = INV_YUP2ZUP @ head_avp

    right_wrist_robot = T_ROBOT_OPENXR @ right_wrist_openxr @ T_OPENXR_ROBOT
    head_robot = T_ROBOT_OPENXR @ head_openxr @ T_OPENXR_ROBOT

    right_wrist_unitree = right_wrist_robot @ T_TO_UNITREE_HUMANOID_RIGHT_ARM
    right_wrist_unitree[:3, 3] -= head_robot[:3, 3]
    right_wrist_unitree[:3, 3] += head_to_waist_offset
    return right_wrist_unitree


def direct_target_pose(
    current_human_pose,
    position_scale,
    workspace_limits,
    reference_robot_pose,
    enable_rotation,
    wrist_rotation_offset,
):
    target_pose = current_human_pose.copy()
    target_pose[:3, 3] = clip_vector(position_scale * current_human_pose[:3, 3], workspace_limits)

    if enable_rotation:
        target_pose[:3, :3] = target_pose[:3, :3] @ wrist_rotation_offset
    else:
        target_pose[:3, :3] = reference_robot_pose[:3, :3]

    return target_pose


def current_dual_ee_poses(arm_ik, dual_arm_q):
    pin.framesForwardKinematics(arm_ik.reduced_robot.model, arm_ik.reduced_robot.data, dual_arm_q)
    left_pose = arm_ik.reduced_robot.data.oMf[arm_ik.L_hand_id].homogeneous.copy()
    right_pose = arm_ik.reduced_robot.data.oMf[arm_ik.R_hand_id].homogeneous.copy()
    return left_pose, right_pose


def build_arm_ik():
    with pushd(XR_TELEOP_TELEOP):
        return G1_29_ArmIK()


def _point(transform):
    return np.asarray(transform, dtype=np.float64)[:3, 3]


def _safe_normalize(value, lower, upper):
    if upper <= lower:
        return 0.0
    return float(np.clip((value - lower) / (upper - lower), 0.0, 1.0))


def _bend_score(a, b, c):
    ba = a - b
    bc = c - b
    denom = np.linalg.norm(ba) * np.linalg.norm(bc)
    if denom < 1e-8:
        return 0.0

    cos_angle = np.clip(np.dot(ba, bc) / denom, -1.0, 1.0)
    angle = float(np.arccos(cos_angle))
    return float(np.clip((np.pi - angle) / np.pi, 0.0, 1.0))


def _chain_straightness(points):
    chain_length = sum(np.linalg.norm(points[i + 1] - points[i]) for i in range(len(points) - 1))
    if chain_length < 1e-8:
        return 1.0

    span = np.linalg.norm(points[-1] - points[0])
    return float(np.clip(span / chain_length, 0.0, 1.0))


def finger_curl(hand, chain_name):
    points = [_point(getattr(hand, joint_name)) for joint_name in FINGER_CHAINS[chain_name]]
    bends = [_bend_score(points[i - 1], points[i], points[i + 1]) for i in range(1, len(points) - 1)]
    angle_curl = float(np.clip(np.mean(bends), 0.0, 1.0))
    straightness_curl = 1.0 - _chain_straightness(points)

    # Blend joint-angle curl with fingertip travel so full flex reaches the robot limits.
    return float(np.clip(0.35 * angle_curl + 0.65 * straightness_curl, 0.0, 1.0))


def thumb_straightness(hand):
    points = [_point(getattr(hand, joint_name)) for joint_name in FINGER_CHAINS["thumb"]]
    return _chain_straightness(points)


def thumb_bend_score(hand, open_straightness, close_straightness):
    straightness = thumb_straightness(hand)
    return 1.0 - _safe_normalize(straightness, close_straightness, open_straightness)


def thumb_opposition(hand, open_distance, close_distance):
    thumb_tip = _point(hand.thumbTip)
    index_knuckle = _point(hand.indexKnuckle)
    distance = np.linalg.norm(thumb_tip - index_knuckle)
    return 1.0 - _safe_normalize(distance, close_distance, open_distance)


def normalize_curl(value, open_threshold, close_threshold):
    return _safe_normalize(value, open_threshold, close_threshold)


def expand_range(value, scale):
    value = float(np.clip(value, 0.0, 1.0))
    scale = max(float(scale), 0.0)
    return float(np.clip(0.5 + scale * (value - 0.5), 0.0, 1.0))


def unit_to_inspire(value, open_angle, close_angle):
    value = float(np.clip(value, 0.0, 1.0))
    return int(round(open_angle + value * (close_angle - open_angle)))


def build_hand_command(
    hand,
    finger_open_angle,
    finger_close_angle,
    curl_open_threshold,
    curl_close_threshold,
    finger_range_scale,
    thumb_bend_open_angle,
    thumb_bend_close_angle,
    thumb_rotation_open_angle,
    thumb_rotation_close_angle,
    thumb_bend_open_straightness,
    thumb_bend_close_straightness,
    thumb_bend_range_scale,
    thumb_open_distance,
    thumb_close_distance,
    thumb_rotation_range_scale,
):
    little_curl = expand_range(
        normalize_curl(finger_curl(hand, "little"), curl_open_threshold, curl_close_threshold),
        finger_range_scale,
    )
    ring_curl = expand_range(
        normalize_curl(finger_curl(hand, "ring"), curl_open_threshold, curl_close_threshold),
        finger_range_scale,
    )
    middle_curl = expand_range(
        normalize_curl(finger_curl(hand, "middle"), curl_open_threshold, curl_close_threshold),
        finger_range_scale,
    )
    index_curl = expand_range(
        normalize_curl(finger_curl(hand, "index"), curl_open_threshold, curl_close_threshold),
        finger_range_scale,
    )
    thumb_bend = expand_range(
        thumb_bend_score(hand, thumb_bend_open_straightness, thumb_bend_close_straightness),
        thumb_bend_range_scale,
    )
    thumb_rotation = expand_range(
        thumb_opposition(hand, thumb_open_distance, thumb_close_distance),
        thumb_rotation_range_scale,
    )

    little = unit_to_inspire(little_curl, finger_open_angle, finger_close_angle)
    ring = unit_to_inspire(ring_curl, finger_open_angle, finger_close_angle)
    middle = unit_to_inspire(middle_curl, finger_open_angle, finger_close_angle)
    index = unit_to_inspire(index_curl, finger_open_angle, finger_close_angle)
    thumb_flex = unit_to_inspire(thumb_bend, thumb_bend_open_angle, thumb_bend_close_angle)
    thumb_rotate = unit_to_inspire(thumb_rotation, thumb_rotation_open_angle, thumb_rotation_close_angle)
    return np.array([little, ring, middle, index, thumb_flex, thumb_rotate], dtype=np.float64)


def publish_hand_command(publisher, values):
    cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
    cmd.angle_set = [int(v) for v in np.asarray(values, dtype=np.int16).tolist()]
    cmd.mode = 0b0001
    return publisher.Write(cmd)


def run_hand_calibration(streamer, mapper, sample_seconds, output_path):
    if not output_path:
        raise ValueError("--hand-calibration-file is required when using --calibrate-hand")

    print("\nHand calibration will sample your AVP right hand.")
    print("Pose 1: fully open the hand, keep fingers naturally straight, then press Enter.")
    input()
    open_samples = sample_raw_metrics(streamer, mapper, sample_seconds, side="right")
    print(f"Captured {len(open_samples)} open-hand samples.")

    print("Pose 2: make the most useful closed grasp/fist, including thumb opposition, then press Enter.")
    input()
    close_samples = sample_raw_metrics(streamer, mapper, sample_seconds, side="right")
    print(f"Captured {len(close_samples)} closed-hand samples.")

    if not open_samples or not close_samples:
        raise RuntimeError("No AVP hand samples were captured; check Vision Pro tracking before calibrating.")

    calibration = HandCalibration()
    calibration.set_from_samples(open_samples, close_samples)
    calibration.save(output_path)
    mapper.calibration = calibration
    mapper.reset()
    print(f"Saved hand calibration to {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Control the G1 right arm and one Inspire hand from Vision Pro right-hand tracking.")
    parser.add_argument("--avp-endpoint", required=True, help="Vision Pro IP or external-network room code.")
    parser.add_argument("--dds-network", default=None, help="Optional DDS NIC name for Unitree channel init.")
    parser.add_argument("--publish-rate", type=float, default=30.0, help="Teleop update rate in Hz.")
    parser.add_argument(
        "--tracking-timeout",
        type=float,
        default=1.0,
        help="If tracking is lost for longer than this, hold the arm target and safe-open the hand.",
    )
    parser.add_argument(
        "--print-debug",
        action="store_true",
        help="Print arm target and hand command summaries once per second.",
    )

    parser.add_argument(
        "--position-scale",
        type=float,
        default=1.0,
        help="Scale factor for the waist-frame wrist translation. Use this to expand or shrink motion range.",
    )
    parser.add_argument("--max-dx", type=float, default=0.65, help="Maximum absolute x reach in the robot waist frame in meters.")
    parser.add_argument("--max-dy", type=float, default=0.65, help="Maximum absolute y reach in the robot waist frame in meters.")
    parser.add_argument("--max-dz", type=float, default=0.65, help="Maximum absolute z reach in the robot waist frame in meters.")
    parser.add_argument(
        "--head-to-waist-x",
        type=float,
        default=0.15,
        help="Robot-frame x offset from Vision Pro head origin to the G1 IK waist origin.",
    )
    parser.add_argument(
        "--head-to-waist-y",
        type=float,
        default=0.0,
        help="Robot-frame y offset from Vision Pro head origin to the G1 IK waist origin.",
    )
    parser.add_argument(
        "--head-to-waist-z",
        type=float,
        default=0.45,
        help="Robot-frame z offset from Vision Pro head origin to the G1 IK waist origin.",
    )
    parser.add_argument(
        "--no-rotation",
        action="store_true",
        help="Only use wrist translation and keep the robot wrist orientation fixed at calibration.",
    )
    parser.add_argument(
        "--disable-arm",
        action="store_true",
        help="Do not send G1 arm commands. Useful while tuning only the Inspire hand mapping.",
    )
    parser.add_argument(
        "--wrist-offset-x-deg",
        type=float,
        default=0.0,
        help="Extra local X-axis rotation applied to the AVP wrist pose before IK.",
    )
    parser.add_argument(
        "--wrist-offset-y-deg",
        type=float,
        default=-90.0,
        help="Extra local Y-axis rotation applied to the AVP wrist pose before IK.",
    )
    parser.add_argument(
        "--wrist-offset-z-deg",
        type=float,
        default=0.0,
        help="Extra local Z-axis rotation applied to the AVP wrist pose before IK.",
    )

    parser.add_argument(
        "--hand-topic-side",
        choices=("l", "r"),
        default="l",
        help="DDS side for the physical right hand. Default is l because Headless_driver_r currently drives the left hand on your setup.",
    )
    parser.add_argument(
        "--hand-map-mode",
        choices=("angle", "legacy"),
        default="angle",
        help="angle uses AVP joint flexion angles and optional calibration; legacy uses the original curl/straightness heuristic.",
    )
    parser.add_argument(
        "--hand-calibration-file",
        default=os.path.join(SCRIPT_DIR, "visionpro_right_hand_calibration.json"),
        help="JSON file for AVP open/closed hand calibration used by --hand-map-mode angle.",
    )
    parser.add_argument(
        "--calibrate-hand",
        action="store_true",
        help="Capture open and closed AVP hand poses, save --hand-calibration-file, then continue teleop.",
    )
    parser.add_argument(
        "--hand-calibration-seconds",
        type=float,
        default=2.0,
        help="Seconds of samples to average for each hand calibration pose.",
    )
    parser.add_argument(
        "--open-angle",
        type=int,
        default=1000,
        help="Inspire command value for open non-thumb fingers. Default uses the full open end of the SDK range.",
    )
    parser.add_argument(
        "--close-angle",
        type=int,
        default=0,
        help="Inspire command value for closed non-thumb fingers. Default uses the full close end of the SDK range.",
    )
    parser.add_argument(
        "--curl-open-threshold",
        type=float,
        default=0.10,
        help="Finger curl score treated as fully open. Raise this if the robot closes too early.",
    )
    parser.add_argument(
        "--curl-close-threshold",
        type=float,
        default=0.30,
        help="Finger curl score treated as fully closed. Lower this if you cannot reach robot max closure.",
    )
    parser.add_argument(
        "--finger-range-scale",
        type=float,
        default=1.15,
        help="Expand non-thumb finger motion around the midpoint. Values above 1.0 use more robot range.",
    )
    parser.add_argument("--thumb-bend-open-angle", type=int, default=800, help="Inspire command value for open thumb bend.")
    parser.add_argument("--thumb-bend-close-angle", type=int, default=200, help="Inspire command value for closed thumb bend.")
    parser.add_argument("--thumb-rotation-open-angle", type=int, default=200, help="Inspire command value for open thumb rotation.")
    parser.add_argument("--thumb-rotation-close-angle", type=int, default=800, help="Inspire command value for closed thumb rotation.")
    parser.add_argument(
        "--thumb-bend-open-straightness",
        type=float,
        default=0.95,
        help="Thumb straightness treated as fully open.",
    )
    parser.add_argument(
        "--thumb-bend-close-straightness",
        type=float,
        default=0.68,
        help="Thumb straightness treated as fully bent.",
    )
    parser.add_argument(
        "--thumb-bend-range-scale",
        type=float,
        default=1.35,
        help="Expand thumb bend motion around the midpoint. Values above 1.0 use more robot range.",
    )
    parser.add_argument(
        "--thumb-open-distance",
        type=float,
        default=0.12,
        help="Thumb-index-knuckle distance treated as fully open thumb rotation in meters.",
    )
    parser.add_argument(
        "--thumb-close-distance",
        type=float,
        default=0.035,
        help="Thumb-index-knuckle distance treated as fully opposed thumb rotation in meters.",
    )
    parser.add_argument(
        "--thumb-rotation-range-scale",
        type=float,
        default=1.25,
        help="Expand thumb rotation motion around the midpoint. Values above 1.0 use more robot range.",
    )
    parser.add_argument(
        "--smoothing",
        type=float,
        default=0.35,
        help="EMA factor for hand command smoothing in [0, 1]. Higher is more responsive.",
    )
    parser.add_argument("--finger-mcp-weight", type=float, default=0.45)
    parser.add_argument("--finger-pip-weight", type=float, default=0.40)
    parser.add_argument("--finger-dip-weight", type=float, default=0.15)
    parser.add_argument("--thumb-mcp-weight", type=float, default=0.65)
    parser.add_argument("--thumb-ip-weight", type=float, default=0.35)
    parser.add_argument("--finger-smoothing", type=float, default=0.45)
    parser.add_argument("--thumb-smoothing", type=float, default=0.35)
    parser.add_argument("--finger-deadband", type=float, default=0.015)
    parser.add_argument("--thumb-deadband", type=float, default=0.02)
    parser.add_argument("--thumb-rotation-metric", choices=("angle", "distance"), default="angle")
    parser.add_argument("--flip-thumb-rotation", action="store_true")
    parser.add_argument(
        "--invert-thumb-rotation-command",
        action="store_true",
        help="Reverse only the Inspire thumb-root rotation command direction after AVP calibration.",
    )
    args = parser.parse_args()

    if args.dds_network:
        ChannelFactoryInitialize(0, args.dds_network)
    else:
        ChannelFactoryInitialize(0)

    streamer = VisionProStreamer(ip=args.avp_endpoint)
    mapper = InspireHandMapper.from_args(args)

    arm_ik = None
    arm_ctrl = None
    mode_machine = None
    if not args.disable_arm:
        arm_ik = build_arm_ik()
        arm_ctrl = G1_29_ArmController(motion_mode=True, simulation_mode=False)
        arm_ctrl.speed_gradual_max()
        mode_machine = arm_ctrl.get_mode_machine()

    hand_publisher = ChannelPublisher(
        f"rt/inspire_hand/ctrl/{args.hand_topic_side}",
        inspire_dds.inspire_hand_ctrl,
    )
    hand_publisher.Init()

    if args.calibrate_hand:
        run_hand_calibration(streamer, mapper, args.hand_calibration_seconds, args.hand_calibration_file)

    max_translation = np.array([args.max_dx, args.max_dy, args.max_dz], dtype=np.float64)
    head_to_waist_offset = np.array(
        [args.head_to_waist_x, args.head_to_waist_y, args.head_to_waist_z],
        dtype=np.float64,
    )
    wrist_rotation_offset = rotation_matrix_xyz_deg(
        args.wrist_offset_x_deg,
        args.wrist_offset_y_deg,
        args.wrist_offset_z_deg,
    )
    period = 1.0 / max(args.publish_rate, 1e-6)

    hand_open_command = np.array(
        [
            args.open_angle,
            args.open_angle,
            args.open_angle,
            args.open_angle,
            args.thumb_bend_open_angle,
            args.thumb_rotation_open_angle,
        ],
        dtype=np.float64,
    )
    hand_close_command = np.array(
        [
            args.close_angle,
            args.close_angle,
            args.close_angle,
            args.close_angle,
            args.thumb_bend_close_angle,
            args.thumb_rotation_close_angle,
        ],
        dtype=np.float64,
    )
    hand_safe_open = hand_open_command.copy()
    hand_command_min = np.minimum(hand_open_command, hand_close_command)
    hand_command_max = np.maximum(hand_open_command, hand_close_command)

    robot_left_hold_pose = None
    robot_right_reference_pose = None
    hand_smoothed = None
    last_arm_tracking_time = 0.0
    last_hand_tracking_time = 0.0
    last_debug_time = 0.0

    print("Vision Pro G1 right-arm + Inspire hand teleop started.")
    if args.disable_arm:
        print("Arm command publishing is disabled; tuning Inspire hand only.")
    else:
        print("Publishing arm commands to rt/arm_sdk via the Unitree G1 arm controller.")
    print(f"Publishing hand commands to rt/inspire_hand/ctrl/{args.hand_topic_side}")
    print(f"Hand mapping mode: {args.hand_map_mode}")
    if args.hand_map_mode == "angle":
        print(f"Hand calibration file: {args.hand_calibration_file}")
    if mode_machine is not None:
        print(f"Detected G1 mode_machine: {mode_machine}")
    if mode_machine is not None and mode_machine not in (2, 5):
        print("Warning: expected a 29-dof G1 mode_machine like 2 or 5. Arm SDK may be ignored in the current robot mode.")
    if not args.disable_arm:
        print("Keep the G1 in the ready/regular mode where the official arm SDK example works.")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            tracking = streamer.get_latest()
            now = time.time()

            arm_debug = None
            hand_debug = None

            if tracking and tracking.right is not None:
                if args.hand_map_mode == "angle":
                    hand_target, hand_map_debug = mapper.build_command(tracking.right)
                    hand_command = np.clip(hand_target, hand_command_min, hand_command_max)
                    publish_hand_command(hand_publisher, hand_command)
                    last_hand_tracking_time = now

                    if args.print_debug and now - last_debug_time >= 1.0:
                        hand_debug = format_hand_debug(hand_command, hand_map_debug)
                else:
                    hand_target = build_hand_command(
                        tracking.right,
                        finger_open_angle=args.open_angle,
                        finger_close_angle=args.close_angle,
                        curl_open_threshold=args.curl_open_threshold,
                        curl_close_threshold=args.curl_close_threshold,
                        finger_range_scale=args.finger_range_scale,
                        thumb_bend_open_angle=args.thumb_bend_open_angle,
                        thumb_bend_close_angle=args.thumb_bend_close_angle,
                        thumb_rotation_open_angle=args.thumb_rotation_open_angle,
                        thumb_rotation_close_angle=args.thumb_rotation_close_angle,
                        thumb_bend_open_straightness=args.thumb_bend_open_straightness,
                        thumb_bend_close_straightness=args.thumb_bend_close_straightness,
                        thumb_bend_range_scale=args.thumb_bend_range_scale,
                        thumb_open_distance=args.thumb_open_distance,
                        thumb_close_distance=args.thumb_close_distance,
                        thumb_rotation_range_scale=args.thumb_rotation_range_scale,
                    )
                    if hand_smoothed is None:
                        hand_smoothed = hand_target
                    else:
                        alpha = float(np.clip(args.smoothing, 0.0, 1.0))
                        hand_smoothed = (1.0 - alpha) * hand_smoothed + alpha * hand_target

                    hand_command = np.clip(hand_smoothed, hand_command_min, hand_command_max)
                    publish_hand_command(hand_publisher, hand_command)
                    last_hand_tracking_time = now

                    if args.print_debug and now - last_debug_time >= 1.0:
                        finger_raw = [
                            finger_curl(tracking.right, "little"),
                            finger_curl(tracking.right, "ring"),
                            finger_curl(tracking.right, "middle"),
                            finger_curl(tracking.right, "index"),
                        ]
                        finger_norm = [
                            expand_range(
                                normalize_curl(value, args.curl_open_threshold, args.curl_close_threshold),
                                args.finger_range_scale,
                            )
                            for value in finger_raw
                        ]
                        thumb_bend = thumb_bend_score(
                            tracking.right,
                            args.thumb_bend_open_straightness,
                            args.thumb_bend_close_straightness,
                        )
                        thumb_rotation = thumb_opposition(
                            tracking.right,
                            args.thumb_open_distance,
                            args.thumb_close_distance,
                        )
                        hand_debug = (
                            "hand_cmd: "
                            f"{np.round(hand_command).astype(int).tolist()} "
                            f"| finger_curl={np.round(finger_raw, 3).tolist()}->{np.round(finger_norm, 3).tolist()} "
                            f"| thumb_bend={thumb_bend:.3f} thumb_rotation={thumb_rotation:.3f}"
                        )
            elif last_hand_tracking_time and now - last_hand_tracking_time >= args.tracking_timeout:
                publish_hand_command(hand_publisher, hand_safe_open)
                hand_smoothed = hand_safe_open.copy()
                mapper.reset()
                last_hand_tracking_time = now
                if args.print_debug and now - last_debug_time >= 1.0:
                    hand_debug = "hand tracking lost -> safe open"

            if not args.disable_arm and tracking and tracking.right is not None and tracking.head is not None:
                current_dual_arm_q = arm_ctrl.get_current_dual_arm_q()
                current_dual_arm_dq = arm_ctrl.get_current_dual_arm_dq()

                current_human_pose = tracking_to_unitree_right_wrist_pose(tracking, head_to_waist_offset)

                if robot_right_reference_pose is None:
                    robot_left_hold_pose, robot_right_reference_pose = current_dual_ee_poses(arm_ik, current_dual_arm_q)
                    last_arm_tracking_time = now
                    if args.print_debug and now - last_debug_time >= 1.0:
                        print("xr_target_ref:", np.round(pose_to_vec(current_human_pose), 4).tolist())
                        print("calibrated robot_ref:", np.round(pose_to_vec(robot_right_reference_pose), 4).tolist())

                target_right_pose = direct_target_pose(
                    current_human_pose=current_human_pose,
                    position_scale=args.position_scale,
                    workspace_limits=max_translation,
                    reference_robot_pose=robot_right_reference_pose,
                    enable_rotation=not args.no_rotation,
                    wrist_rotation_offset=wrist_rotation_offset,
                )
                sol_q, sol_tauff = arm_ik.solve_ik(
                    robot_left_hold_pose,
                    target_right_pose,
                    current_dual_arm_q,
                    current_dual_arm_dq,
                )
                joint_delta_max = float(np.max(np.abs(sol_q - current_dual_arm_q)))
                arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)
                last_arm_tracking_time = now

                if args.print_debug and now - last_debug_time >= 1.0:
                    arm_debug = (
                        "right_target: "
                        f"{np.round(pose_to_vec(target_right_pose), 4).tolist()} "
                        f"| joint_delta_max: {round(joint_delta_max, 4)}"
                    )
            elif robot_right_reference_pose is not None and now - last_arm_tracking_time >= args.tracking_timeout:
                if args.print_debug and now - last_debug_time >= 1.0:
                    arm_debug = "arm tracking lost -> holding last target"

            if args.print_debug and now - last_debug_time >= 1.0:
                if arm_debug:
                    print(arm_debug)
                if hand_debug:
                    print(hand_debug)
                if arm_debug or hand_debug:
                    last_debug_time = now

            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopping Vision Pro G1 right-arm + Inspire hand teleop...")
    finally:
        try:
            publish_hand_command(hand_publisher, hand_safe_open)
        except Exception as exc:
            print(f"Failed to safe-open the hand cleanly: {exc}")
        try:
            if arm_ctrl is not None:
                arm_ctrl.ctrl_dual_arm_go_home()
        except Exception as exc:
            print(f"Failed to send arms home cleanly: {exc}")


if __name__ == "__main__":
    main()
