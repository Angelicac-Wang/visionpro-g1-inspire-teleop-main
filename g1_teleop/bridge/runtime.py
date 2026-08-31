"""AVP → SONIC bridge runtime loop."""

from __future__ import annotations

import sys
import time

import numpy as np

from g1_teleop.bridge.cli import parse_args, resolve_hand_calibration_files
from g1_teleop.bridge.constants import T_TO_UNITREE_HUMANOID_LEFT_ARM, T_TO_UNITREE_HUMANOID_RIGHT_ARM
from g1_teleop.bridge.dds_hand import DdsInspireHandsPublisher
from g1_teleop.bridge.keyboard import BridgeState, KeyboardHoldTracker, RawKeyboard, handle_key
from g1_teleop.bridge.locomotion_io import (
    loco_robot_base_quat,
    loco_robot_base_yaw,
    loco_sync_head_pose,
)
from g1_teleop.bridge.smoothing import OrientationSmoother, PositionSmoother, _normalize_quat_wxyz
from g1_teleop.bridge.vr_targets import (
    avp_to_robot,
    build_calibrated_vr_targets,
    build_head_relative_vr_targets,
    build_official_calib_vr_targets,
    build_vr_targets,
    capture_official_calibration,
    default_official_base_orientations,
    default_official_base_positions,
    finalize_calib_buffer,
    hand_is_active,
    pose_or_none,
    print_staged_calib_help,
    tracking_arm_poses,
)
from g1_teleop.bridge.zmq_pub import PackedPublisher
from g1_teleop.calibration.session import (
    CalibPhase,
    CalibSession,
    FkRobotReference,
    ZmqFeedbackClient,
    arm_hold,
    collect_hold_sample,
    merge_calibration,
    robot_base_from_feedback,
    sync_loco_zeros,
)
from g1_teleop.hand.mapping import InspireHandMapper, format_debug as format_hand_debug
from g1_teleop.locomotion.head import (
    HeadHeightSquatConfig,
    HeadLocomotionConfig,
    HeadLocomotionState,
    SonicPlannerCommand,
    apply_height_to_planner_command,
    compute_head_pelvis_height,
    compute_sonic_planner_command,
    head_pose_from_tracking,
    reset_head_locomotion_state,
    update_facing_from_head,
)
from g1_teleop.locomotion.hybrid import (
    KeyboardLocomotionController,
    keyboard_planner_command,
    merge_hybrid_planner_commands,
)
from g1_teleop.eval.task_a import RemsEvalLogger
from g1_teleop.paths import ensure_scripts_on_path, visionpro_teleop_root

ensure_scripts_on_path()
vp_root = visionpro_teleop_root()
if vp_root not in sys.path:
    sys.path.insert(0, vp_root)

from avp_stream import VisionProStreamer


def main():
    args = parse_args()
    if args.head_vertical_follow is None:
        args.head_vertical_follow = bool(args.head_height_squat)
    if not args.head_locomotion:
        args.hybrid_locomotion = False
    period = 1.0 / max(args.publish_rate, 1e-6)
    streamer = VisionProStreamer(ip=args.avp_endpoint, record=False, benchmark_quiet=True)
    fpv_streamer = None
    if args.enable_mujoco_fpv:
        try:
            from g1_teleop.sim.mujoco_fpv import MujocoFpvStreamer

            fpv_streamer = MujocoFpvStreamer(
                streamer,
                camera_host=args.mujoco_camera_host,
                camera_port=args.mujoco_camera_port,
                camera_name=args.mujoco_camera_name,
                webrtc_port=args.avp_webrtc_port,
                video_size=args.avp_video_size,
                fps=args.avp_video_fps,
            )
        except Exception as exc:
            print(f"WARNING: MuJoCo FPV disabled: {exc}")
    publisher = PackedPublisher(args.host, args.port)
    state = BridgeState()
    kb_state = BridgeState()
    kb_controller = KeyboardLocomotionController()
    kb_hold_tracker = KeyboardHoldTracker()
    eval_logger = RemsEvalLogger(args.eval_log) if args.eval_log else None
    pending_waypoint_mark: int | None = None
    smoother = PositionSmoother()
    orientation_smoother = OrientationSmoother()

    def seed_arm_smoothers_from_vr(vr_position: np.ndarray, vr_orientation: np.ndarray, now: float) -> None:
        smoother.position = np.asarray(vr_position, dtype=np.float64).reshape(9).copy()
        smoother.last_time = now
        orientation_smoother.orientation = np.asarray(vr_orientation, dtype=np.float64).reshape(12).copy()
        orientation_smoother.last_time = now
        for i in range(0, 12, 4):
            orientation_smoother.orientation[i : i + 4] = _normalize_quat_wxyz(
                orientation_smoother.orientation[i : i + 4]
            )

    def capture_robot_hold_base(label: str, *, update_mapping_base: bool) -> str | None:
        nonlocal official_base_positions, official_base_orientations
        if feedback_client is None:
            return None
        feedback = feedback_client.latest()
        base_pos, base_orn, source = robot_base_from_feedback(
            feedback,
            fk_ref,
            fallback_positions=official_base_positions,
            fallback_orientations=official_base_orientations,
            prefer_fk=args.use_fk_calib,
            last_sent_positions=last_sent_vr_position,
            last_sent_orientations=last_sent_vr_orientation,
        )
        if source == "fallback_init":
            print(
                f"\nWARNING: {label} — no FK/feedback; keeping current targets "
                f"(avoid init-pose snap). Check :{args.zmq_feedback_port} g1_debug."
            )
            return None
        if update_mapping_base:
            official_base_positions = base_pos
            official_base_orientations = base_orn
        print(f"\n{label}: robot pose from {source}")
        return source

    def resolve_stand_hold_targets() -> tuple[np.ndarray, np.ndarray]:
        """Stand-hold VR targets after ENGAGE."""
        if args.stand_hold_mode == "init-pose":
            return official_base_positions.copy(), official_base_orientations.copy()

        if feedback_client is None:
            if last_sent_vr_position is not None:
                return last_sent_vr_position.copy(), last_sent_vr_orientation.copy()
            return official_base_positions.copy(), official_base_orientations.copy()

        feedback = feedback_client.latest()
        base_pos, base_orn, source = robot_base_from_feedback(
            feedback,
            fk_ref,
            fallback_positions=official_base_positions,
            fallback_orientations=official_base_orientations,
            prefer_fk=args.use_fk_calib,
            last_sent_positions=last_sent_vr_position,
            last_sent_orientations=last_sent_vr_orientation,
        )
        if source == "fk(body_q)":
            return base_pos, base_orn
        if last_sent_vr_position is not None:
            return last_sent_vr_position.copy(), last_sent_vr_orientation.copy()
        if source in ("last_sent_command", "vr_3point_feedback"):
            return base_pos, base_orn
        return official_base_positions.copy(), official_base_orientations.copy()

    def send_idle_planner(
        vr_position: np.ndarray,
        vr_orientation: np.ndarray,
        *,
        left_wrist_joints: np.ndarray | None = None,
        right_wrist_joints: np.ndarray | None = None,
    ) -> None:
        publisher.send_planner(
            mode=0,
            movement=np.zeros(3, dtype=np.float64),
            facing=np.array([1.0, 0.0, 0.0], dtype=np.float64),
            speed=-1.0,
            height=-1.0,
            vr_position=vr_position,
            vr_orientation=vr_orientation,
            left_wrist_joints=left_wrist_joints,
            right_wrist_joints=right_wrist_joints,
        )

    def should_preenage_stream() -> bool:
        """Keep deploy VR buffer on L init before ] (avoids default arms-down snap)."""
        return (
            args.stream_init_pose
            and not policy_started
            and not live_teleop
            and initial_head_robot_pos is not None
        )

    def prime_deploy_l_hold(
        hold_pos: np.ndarray,
        hold_orn: np.ndarray,
        now: float,
        *,
        bursts: int = 3,
    ) -> None:
        """Burst idle L targets so the first policy tick never sees default wrists-down."""
        nonlocal last_sent_vr_position, last_sent_vr_orientation
        for _ in range(max(1, bursts)):
            send_idle_planner(hold_pos, hold_orn)
        last_sent_vr_position = hold_pos.copy()
        last_sent_vr_orientation = hold_orn.copy()
        seed_arm_smoothers_from_vr(hold_pos, hold_orn, now)

    def publish_init_pose_hold(now: float, *, ramp: bool = True) -> None:
        """Stream configured L init over ZMQ (buffer for deploy; ramp if policy on)."""
        nonlocal last_sent_vr_position, last_sent_vr_orientation
        vr_position = official_base_positions.copy()
        vr_orientation = official_base_orientations.copy()
        if policy_started and args.mapping_mode in ("hybrid", "official-calib"):
            if smoother.position is None and last_sent_vr_position is not None:
                seed_arm_smoothers_from_vr(last_sent_vr_position, last_sent_vr_orientation, now)
            vr_position, vr_orientation = apply_vr_output_smoothing(
                vr_position,
                vr_orientation,
                now,
                ramp=ramp,
            )
        last_sent_vr_position = vr_position.copy()
        last_sent_vr_orientation = vr_orientation.copy()
        send_idle_planner(vr_position, vr_orientation)

    def apply_vr_output_smoothing(
        vr_position: np.ndarray,
        vr_orientation: np.ndarray,
        now: float,
        *,
        ramp: bool,
    ) -> tuple[np.ndarray, np.ndarray]:
        if ramp:
            pos_tau = args.vr_ramp_smoothing_tau
            pos_speed = args.vr_ramp_max_speed
            orn_tau = args.vr_ramp_orientation_tau
            orn_speed = args.vr_ramp_max_angular_speed
        else:
            pos_tau = args.hybrid_smoothing_tau
            pos_speed = args.hybrid_max_speed
            orn_tau = args.arm_orientation_smoothing_tau
            orn_speed = args.arm_max_angular_speed
        vr_position = smoother.update(vr_position, now, tau=pos_tau, max_speed=pos_speed)
        vr_orientation = orientation_smoother.update(
            vr_orientation,
            now,
            tau=orn_tau,
            max_angular_speed=orn_speed,
        )
        return vr_position, vr_orientation
    enable_inspire_hand = args.enable_inspire_hand_sim or args.enable_inspire_hand_dds
    left_hand_calib_file, right_hand_calib_file = resolve_hand_calibration_files(args)
    left_hand_mapper = (
        InspireHandMapper.from_args(args, calibration_file=left_hand_calib_file)
        if enable_inspire_hand
        else None
    )
    right_hand_mapper = (
        InspireHandMapper.from_args(args, calibration_file=right_hand_calib_file)
        if enable_inspire_hand
        else None
    )
    hand_dds_sides = args.hand_dds_sides
    dds_hand_publisher = (
        DdsInspireHandsPublisher(hand_dds_sides, args.hand_dds_network)
        if args.enable_inspire_hand_dds
        else None
    )
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
    hand_command_min = np.minimum(hand_open_command, hand_close_command)
    hand_command_max = np.maximum(hand_open_command, hand_close_command)
    left_hand_command = hand_open_command.copy()
    right_hand_command = hand_open_command.copy()
    last_hand_tracking_time = 0.0

    print(f"AVP -> SONIC bridge bound to {publisher.endpoint}")
    print(f"Mapping mode: {args.mapping_mode}")
    print(f"Active hands: {args.active_hands}")
    if args.head_locomotion:
        print(
            "Head locomotion ON: move head to walk/turn (work-master velocity model). "
            f"max_speed={args.loco_max_speed} smooth={args.loco_smooth} "
            f"imu_yaw={'ON' if args.loco_imu_yaw_enabled else 'OFF'}"
        )
        if args.hybrid_locomotion:
            print(
                "Hybrid locomotion ON: hold W/S/,/. to move; release to coast-stop. "
                "Hold A/D or j/l to turn while stopped (slow walk-turn). "
                f"speed={args.keyboard_loco_speed}  space=stop (keeps facing)"
            )
    if args.eval_log:
        print(
            f"Eval log ON: {args.eval_log}  "
            "Mark Task A waypoints during teleop with keys 4-8 (WP1-WP5)."
        )
    if args.head_height_squat:
        print(
            "Head-height squat ON: duck to lower pelvis (mode 4/6). "
            f"walk_above={args.squat_walk_threshold}m drop_squat={args.head_drop_to_squat}m"
        )
        if args.head_vertical_follow:
            print(f"Head vertical follow ON: scale={args.head_vertical_scale}")
    if args.enable_mujoco_fpv:
        print(
            "MuJoCo FPV ON: robot head_camera -> Vision Pro WebRTC "
            f"(sim ZMQ {args.mujoco_camera_host}:{args.mujoco_camera_port}, "
            f"webrtc :{args.avp_webrtc_port})"
        )
    head_height_cfg = HeadHeightSquatConfig(
        walk_height_threshold=args.squat_walk_threshold,
        squat_height_min=args.squat_height_min,
        kneel_height_min=args.squat_kneel_height,
        head_drop_start=args.head_drop_start,
        head_drop_to_squat=args.head_drop_to_squat,
        head_drop_to_kneel=args.head_drop_to_kneel,
        smooth_alpha=args.squat_height_smooth,
    )
    if args.enable_inspire_hand_sim:
        print(f"Publishing sim Inspire hand commands on topic '{args.inspire_hand_topic}'.")
        print(f"  Left hand calib:  {left_hand_calib_file}")
        print(f"  Right hand calib: {right_hand_calib_file}")
    if dds_hand_publisher is not None:
        topics = ", ".join(f"rt/inspire_hand/ctrl/{side}" for side in dds_hand_publisher.sides)
        if hand_dds_sides == "both":
            print(f"Publishing physical Inspire hands: AVP left->l, AVP right->r ({topics}).")
        elif hand_dds_sides == "l":
            print(f"Publishing physical Inspire left hand commands to rt/inspire_hand/ctrl/l.")
        else:
            print(f"Publishing physical Inspire right hand commands to rt/inspire_hand/ctrl/r.")
    if args.mapping_mode == "official-calib" and args.staged_calib:
        print_staged_calib_help()
    elif args.mapping_mode == "official-calib":
        print(
            "Official calibration mode: startup holds the configured robot init pose; "
            "press c, hold your hands for the calibration delay, then tracking starts."
        )
    print("Waiting for AVP head tracking...")

    initial_head_robot_pos = None
    neutral_head_pose = None
    neutral_left_pose = None
    neutral_right_pose = None
    official_calibration = None
    official_base_positions = default_official_base_positions(args)
    official_base_orientations = default_official_base_orientations()
    calib_hold_until = 0.0
    calib_session = CalibSession()
    pending_calibration_deadline = 0.0
    pending_base_positions = None
    pending_base_orientations = None
    feedback_client = (
        ZmqFeedbackClient(args.zmq_feedback_host, args.zmq_feedback_port, args.zmq_feedback_topic)
        if args.mapping_mode == "official-calib" and args.staged_calib
        else None
    )
    fk_ref = FkRobotReference() if args.use_fk_calib else None
    if feedback_client is not None and fk_ref is not None and not fk_ref.available and args.print_debug:
        print(f"FK calib unavailable: {fk_ref._load_error}")
    last_sent_vr_position = None
    last_sent_vr_orientation = None
    head_loco_state = HeadLocomotionState()
    head_loco_cfg = HeadLocomotionConfig(
        velocity_gain=args.loco_velocity_gain,
        yaw_rate_gain=args.loco_yaw_gain,
        forward_scale=args.loco_forward_scale,
        lateral_scale=args.loco_lateral_scale,
        lateral_left_scale=args.loco_lateral_left_scale,
        lateral_right_scale=args.loco_lateral_right_scale,
        lateral_displacement_gain=args.loco_lateral_displacement_gain,
        lateral_velocity_deadzone=args.loco_lateral_deadzone,
        sign_x=args.loco_sign_x,
        sign_y=args.loco_sign_y,
        max_speed=args.loco_max_speed,
        max_yaw_rate=args.loco_max_yaw_rate,
        velocity_deadzone=args.loco_velocity_deadzone,
        yaw_rate_deadzone=args.loco_yaw_deadzone,
        smooth_alpha=args.loco_smooth,
        facing_smooth_alpha=args.loco_facing_smooth,
        output_deadzone=args.loco_output_deadzone,
        idle_decay=args.loco_idle_decay,
        imu_yaw_enabled=args.loco_imu_yaw_enabled,
        imu_yaw_gain=args.loco_imu_yaw_gain,
        imu_yaw_deadzone=args.loco_imu_yaw_deadzone,
        imu_yaw_max_correction=args.loco_imu_yaw_max_correction,
    )
    head_loco_calib_pos = None
    head_loco_calib_rot = None
    last_head_loco_time = None
    stand_hold = True
    policy_started = False
    live_teleop = False

    def sync_kb_body_facing_from_robot() -> None:
        feedback = feedback_client.latest() if feedback_client is not None else None
        robot_yaw = loco_robot_base_yaw(feedback)
        if robot_yaw is None:
            return
        if head_loco_state.robot_base_yaw_at_calib is not None:
            kb_controller.sync_body_facing(
                float(robot_yaw - head_loco_state.robot_base_yaw_at_calib)
            )
        else:
            kb_controller.sync_body_facing(float(robot_yaw))

    def apply_loco_sync(head_pose: np.ndarray) -> None:
        nonlocal head_loco_calib_pos, head_loco_calib_rot
        feedback = feedback_client.latest() if feedback_client is not None else None
        robot_base_yaw = loco_robot_base_yaw(feedback)
        head_loco_calib_pos, head_loco_calib_rot, calib_yaw = sync_loco_zeros(
            head_pose,
            head_loco_state,
            reset_head_locomotion_state,
            robot_base_yaw=robot_base_yaw,
        )
        if robot_base_yaw is not None:
            print(
                f"  Loco IMU zero: robot_base_yaw={robot_base_yaw:.3f} rad "
                f"({np.degrees(robot_base_yaw):.1f} deg)",
                flush=True,
            )
        elif args.loco_imu_yaw_enabled:
            print(
                "  WARNING: Loco IMU zero unavailable (no base_quat in g1_debug). "
                "Head locomotion stays open-loop until F/T/H with deploy feedback.",
                flush=True,
            )
        sync_kb_body_facing_from_robot()

    def begin_calib_hold(kind: str) -> None:
        nonlocal calib_hold_until, official_base_positions, official_base_orientations
        arm_hold(calib_session, kind, args.calib_hold_sec)
        calib_hold_until = calib_session.hold_deadline
        if kind == "full":
            official_base_positions = default_official_base_positions(args)
            official_base_orientations = default_official_base_orientations()
            print(
                f"\nCALIB_FULL: hold forearms-forward for {args.calib_hold_sec:.1f}s "
                f"(robot init L={np.round(official_base_positions[0:3], 3).tolist()} "
                f"R={np.round(official_base_positions[3:6], 3).tolist()})"
            )
        elif kind == "sync":
            if (
                args.stand_hold_mode == "init-pose"
                and last_sent_vr_position is not None
                and last_sent_vr_orientation is not None
            ):
                # Robot was commanded to configured L init during stand-hold; FK body_q
                # often lags (especially left wrist X). Use the streamed command as sync
                # base so T does not pull arms back toward a shorter FK pose.
                official_base_positions = last_sent_vr_position.copy()
                official_base_orientations = last_sent_vr_orientation.copy()
                source = "last_sent_command"
            else:
                feedback = feedback_client.latest() if feedback_client is not None else None
                base_pos, base_orn, source = robot_base_from_feedback(
                    feedback,
                    fk_ref,
                    fallback_positions=official_base_positions,
                    fallback_orientations=official_base_orientations,
                    prefer_fk=args.use_fk_calib,
                    last_sent_positions=last_sent_vr_position,
                    last_sent_orientations=last_sent_vr_orientation,
                )
                if source == "fallback_init":
                    print(
                        "\nWARNING: CALIB_SYNC has no FK/feedback — keeping mapping base. "
                        f"Ensure deploy publishes g1_debug on :{args.zmq_feedback_port}."
                    )
                else:
                    official_base_positions = base_pos
                    official_base_orientations = base_orn
            if source == "vr_3point_feedback":
                print(
                    "  NOTE: sync base from commanded vr_3point (FK unavailable). "
                    "Prefer body_q FK on real robot."
                )
            print(
                f"\nCALIB_SYNC: align YOUR arms to the robot's current pose, hold {args.calib_hold_sec:.1f}s "
                f"(mapping base from {source}, "
                f"L={np.round(official_base_positions[0:3], 3).tolist()} "
                f"R={np.round(official_base_positions[3:6], 3).tolist()})"
            )
            if args.stand_hold_mode == "init-pose":
                print(
                    "  init-pose mode: match the robot L-shape on screen, then hold still."
                )
        elif kind == "head":
            print(f"\nHEAD zero: look forward, hold {args.calib_hold_sec:.1f}s")

    def commit_calib_hold() -> bool:
        nonlocal official_calibration, calib_hold_until, live_teleop
        captured, quality = finalize_calib_buffer(
            calib_session.buffer,
            args,
            head_only=(calib_session.hold_kind == "head"),
        )
        calib_session.last_quality = quality
        print(f"\nCalibration quality: {quality.summary()} ({quality.message})")
        if captured is None:
            print("Calibration FAILED — hold still and retry.")
            calib_session.phase = (
                CalibPhase.ENGAGED if policy_started else CalibPhase.READY
            )
            calib_hold_until = 0.0
            return False

        head_pose = captured.head_pose
        if calib_session.hold_kind == "sync" and official_calibration is not None:
            official_calibration = merge_calibration(
                official_calibration,
                captured,
                preserve_head=True,
                preserve_wrists=False,
            )
        elif calib_session.hold_kind == "head" and official_calibration is not None:
            official_calibration = merge_calibration(
                official_calibration,
                captured,
                preserve_head=False,
                preserve_wrists=True,
            )
        else:
            official_calibration = captured

        loco_head = loco_sync_head_pose(
            captured,
            calib_session.hold_kind,
            official_calibration,
        )
        if calib_session.hold_kind == "sync":
            print(
                "  squat/walk head zero kept from CALIB_FULL "
                f"(Z={loco_head[2, 3]:.3f} m; S only updates wrists)"
            )
        apply_loco_sync(loco_head)
        calib_hold_until = 0.0

        if calib_session.hold_kind == "full":
            calib_session.full_done = True
            calib_session.phase = CalibPhase.READY
            print("CALIB_FULL done. Press ] to engage policy and wait for balance, then S then T.")
        elif calib_session.hold_kind == "sync":
            calib_session.sync_done = True
            calib_session.phase = CalibPhase.ENGAGED if not live_teleop else CalibPhase.TELEOP
            print("CALIB_SYNC done. Press T for teleop (or S again if arms drift).")
        elif calib_session.hold_kind == "head":
            calib_session.phase = CalibPhase.TELEOP if live_teleop else CalibPhase.ENGAGED
            print("HEAD zero updated.")
        return True
    try:
        with RawKeyboard() as keyboard:
            last_debug = 0.0
            last_start = 0.0
            while True:
                now = time.time()
                tracking = streamer.get_latest()

                if tracking is not None and tracking.head is not None and initial_head_robot_pos is None:
                    neutral_head_pose = avp_to_robot(tracking.head)
                    initial_head_robot_pos = neutral_head_pose[:3, 3].copy()
                    neutral_left_pose = pose_or_none(tracking, "left", T_TO_UNITREE_HUMANOID_LEFT_ARM)
                    neutral_right_pose = pose_or_none(tracking, "right", T_TO_UNITREE_HUMANOID_RIGHT_ARM)
                    if args.mapping_mode != "official-calib":
                        official_calibration = capture_official_calibration(tracking, args)
                    official_base_positions = default_official_base_positions(args)
                    official_base_orientations = default_official_base_orientations()
                    print("AVP tracking locked.")
                    print(
                        "Robot base target: left=",
                        np.round(official_base_positions[0:3], 3).tolist(),
                        "right=",
                        np.round(official_base_positions[3:6], 3).tolist(),
                        "head=",
                        np.round(official_base_positions[6:9], 3).tolist(),
                    )
                    if args.mapping_mode == "official-calib" and args.staged_calib:
                        calib_session.phase = CalibPhase.READY
                        print("Holding robot init pose. Press F (or c) for CALIB_FULL.")
                    elif args.mapping_mode == "official-calib":
                        print("Holding robot init pose. Press C to calibrate and start AVP tracking.")
                    if not args.no_auto_start and not args.staged_calib:
                        publisher.send_command(start=True, stop=False, planner=True)
                        last_start = now
                        policy_started = True
                        print("Sent SONIC start command in planner mode (stand-hold until T).")

                if initial_head_robot_pos is None:
                    if args.stream_init_pose:
                        publish_init_pose_hold(now, ramp=policy_started)
                    time.sleep(period if args.stream_init_pose else 0.02)
                    continue

                if feedback_client is not None:
                    feedback_client.poll()

                if fpv_streamer is not None:
                    fpv_streamer.push_latest()

                head_pose, left_pose, right_pose = tracking_arm_poses(tracking, args)

                if calib_session.phase in (
                    CalibPhase.FULL_HOLD,
                    CalibPhase.SYNC_HOLD,
                    CalibPhase.HEAD_HOLD,
                ):
                    if head_pose is not None:
                        collect_hold_sample(calib_session, head_pose, left_pose, right_pose)
                    if calib_session.hold_deadline > 0.0 and now >= calib_session.hold_deadline:
                        commit_calib_hold()

                keys = keyboard.read_keys()
                if keys and args.head_locomotion and args.hybrid_locomotion:
                    kb_hold_tracker.refresh(keys, now)

                for key in keys:
                    if args.staged_calib and args.mapping_mode == "official-calib":
                        if key in ("f", "F", "c", "C"):
                            if calib_session.phase in (
                                CalibPhase.FULL_HOLD,
                                CalibPhase.SYNC_HOLD,
                                CalibPhase.HEAD_HOLD,
                            ):
                                print("\nCalibration already in progress.")
                            else:
                                begin_calib_hold("full")
                            continue
                        if key == "]":
                            if not calib_session.full_done:
                                print("\nDo CALIB_FULL (F) before engaging policy.")
                                continue
                            if args.stand_hold_mode == "init-pose":
                                hold_pos, hold_orn = official_base_positions.copy(), official_base_orientations.copy()
                            else:
                                hold_pos, hold_orn = resolve_stand_hold_targets()
                            prime_deploy_l_hold(hold_pos, hold_orn, now)
                            publisher.send_command(start=True, stop=False, planner=True)
                            send_idle_planner(hold_pos, hold_orn)
                            last_start = now
                            policy_started = True
                            stand_hold = True
                            live_teleop = False
                            calib_session.phase = CalibPhase.ENGAGED
                            state.set_idle()
                            state.facing_angle = 0.0
                            captured_src = capture_robot_hold_base(
                                "ENGAGE stand-hold",
                                update_mapping_base=False,
                            )
                            if args.stand_hold_mode == "init-pose":
                                print(
                                    "\nENGAGE: stand-hold = configured L init-pose "
                                    f"(L={np.round(hold_pos[0:3], 3).tolist()} / "
                                    f"R={np.round(hold_pos[3:6], 3).tolist()}), ramp to target."
                                )
                            elif captured_src is None:
                                print(
                                    "  Stand-hold: track-robot (mapping base still from F until S)."
                                )
                            print(
                                "\nENGAGE: policy started. "
                                "Wait for balance, match your arms, press S for CALIB_SYNC."
                            )
                            continue
                        if key in ("s", "S") and not live_teleop:
                            if not policy_started:
                                print("\nPress ] first to engage policy and wait for balance.")
                                continue
                            if not calib_session.full_done:
                                print("\nDo CALIB_FULL (F) first.")
                                continue
                            begin_calib_hold("sync")
                            continue
                        if key in ("t", "T"):
                            if not policy_started:
                                publisher.send_command(start=True, stop=False, planner=True)
                                last_start = now
                                policy_started = True
                            if not calib_session.full_done:
                                print("\nDo CALIB_FULL (F) first.")
                                continue
                            if args.require_sync_calib and not calib_session.sync_done:
                                print("\nDo CALIB_SYNC (S) after robot balances, then press T.")
                                continue
                            if official_calibration is None:
                                print("\nCalibration missing — run F then S.")
                                continue
                            if (
                                args.staged_calib
                                and args.mapping_mode == "official-calib"
                                and tracking is not None
                            ):
                                snap = capture_official_calibration(tracking, args)
                                if snap is not None:
                                    official_calibration = merge_calibration(
                                        official_calibration,
                                        snap,
                                        preserve_head=True,
                                        preserve_wrists=False,
                                    )
                                    print(
                                        "  T wrist zero: snapped to your current AVP pose "
                                        "(hold the same shape as S to avoid a jump)."
                                    )
                            stand_hold = False
                            live_teleop = True
                            calib_session.paused = False
                            calib_session.phase = CalibPhase.TELEOP
                            if head_pose is not None:
                                apply_loco_sync(head_pose)
                                print(
                                    "  Walk zero synced to current head "
                                    "(H = re-sync facing/height only)."
                                )
                            msg = "TELEOP live: AVP hands + head walk/squat."
                            print(f"\n{msg}")
                            continue
                        if key in ("h", "H"):
                            if official_calibration is None:
                                print("\nRun CALIB_FULL (F) first.")
                                continue
                            begin_calib_hold("head")
                            continue
                        if key in ("p", "P"):
                            if not live_teleop:
                                print("\nPAUSE only applies during teleop.")
                                continue
                            calib_session.paused = not calib_session.paused
                            if calib_session.paused:
                                calib_session.phase = CalibPhase.PAUSED
                                stand_hold = True
                                print("\nPAUSED: frozen at last robot target. Align body, press S then P to resume.")
                            else:
                                calib_session.phase = CalibPhase.TELEOP
                                stand_hold = False
                                print("\nResumed teleop.")
                            continue

                    if key == "]" and (
                        not args.staged_calib or args.mapping_mode != "official-calib"
                    ):
                        publisher.send_command(start=True, stop=False, planner=True)
                        last_start = now
                        policy_started = True
                        stand_hold = True
                        state.set_idle()
                        state.facing_angle = 0.0
                        print(
                            "\nSent SONIC start command (stand-hold). "
                            "Keep head/hands still until robot balances, then press T for teleop."
                        )
                        continue
                    if key in ("t", "T") and (
                        not args.staged_calib or args.mapping_mode != "official-calib"
                    ):
                        if not policy_started:
                            publisher.send_command(start=True, stop=False, planner=True)
                            last_start = now
                            policy_started = True
                        stand_hold = False
                        live_teleop = True
                        if head_pose is not None:
                            apply_loco_sync(head_pose)
                        msg = "Teleop enabled: AVP hands drive upper body."
                        if args.head_locomotion:
                            msg += " Lean head to walk/turn."
                            if args.hybrid_locomotion:
                                msg += " Hold W/S to move; space stops (keeps facing)."
                        else:
                            msg += " Use WASD in this terminal to walk."
                        print(f"\n{msg}")
                        continue
                    if key in ("c", "C") and (
                        not args.staged_calib or args.mapping_mode != "official-calib"
                    ):
                        if args.mapping_mode == "official-calib":
                            if args.official_calib_base == "current-target" and last_sent_vr_position is not None:
                                pending_base_positions = np.asarray(last_sent_vr_position, dtype=np.float64).reshape(9).copy()
                                if last_sent_vr_orientation is not None:
                                    pending_base_orientations = np.asarray(last_sent_vr_orientation, dtype=np.float64).reshape(12).copy()
                                else:
                                    pending_base_orientations = official_base_orientations.copy()
                            else:
                                pending_base_positions = default_official_base_positions(args)
                                pending_base_orientations = default_official_base_orientations()

                            official_base_positions = pending_base_positions.copy()
                            official_base_orientations = pending_base_orientations.copy()
                            pending_calibration_deadline = time.time() + max(args.calib_hold_sec, 0.0)
                            calib_hold_until = pending_calibration_deadline
                            print(
                                f"\nCalibration armed: hold your hands at the desired zero pose for {max(args.calib_hold_sec, 0.0):.1f}s.",
                                "Robot target held at left=",
                                np.round(official_base_positions[0:3], 3).tolist(),
                                "right=",
                                np.round(official_base_positions[3:6], 3).tolist(),
                            )
                        else:
                            captured = capture_official_calibration(tracking, args)
                            if captured is None:
                                print("\nCalibration skipped: AVP head tracking is not ready.")
                            else:
                                official_calibration = captured
                                print("\nOfficial-style AVP calibration captured.")
                        continue
                    if eval_logger is not None and key in ("4", "5", "6", "7", "8") and live_teleop:
                        wp = int(key) - 3
                        eval_logger.mark_waypoint(wp)
                        pending_waypoint_mark = wp
                        print(f"\nEval: marked Task A waypoint WP{wp}")
                        continue
                    if (
                        args.head_locomotion
                        and args.hybrid_locomotion
                        and live_teleop
                        and key in (" ", "r", "R")
                    ):
                        kb_controller.request_stop()
                        kb_hold_tracker.clear()
                        continue
                    loco_key_target = kb_state if args.head_locomotion and args.hybrid_locomotion else state
                    if not handle_key(
                        loco_key_target,
                        key,
                        head_locomotion=args.head_locomotion,
                        hybrid_locomotion=args.hybrid_locomotion,
                        keyboard_walk_speed=args.keyboard_loco_speed,
                    ):
                        if eval_logger is not None:
                            eval_logger.close()
                        publisher.send_command(start=False, stop=True, planner=True)
                        print("\nSent SONIC stop command.")
                        return

                if args.staged_calib and args.mapping_mode == "official-calib":
                    head_motion_allowed = (
                        live_teleop
                        and not calib_session.paused
                        and official_calibration is not None
                    )
                else:
                    head_motion_allowed = (
                        not stand_hold
                        and (args.mapping_mode != "official-calib" or official_calibration is not None)
                    )

                if head_pose is None and tracking is not None and tracking.head is not None:
                    head_pose = head_pose_from_tracking(tracking)

                planner_cmd = None
                head_planner_cmd = None
                kb_planner_cmd = None
                if (
                    args.head_locomotion
                    and head_motion_allowed
                    and head_pose is not None
                    and head_loco_calib_pos is not None
                ):
                    loop_dt = max(now - last_head_loco_time, period) if last_head_loco_time else period
                    last_head_loco_time = now
                    loco_feedback = feedback_client.latest() if feedback_client is not None else None
                    robot_base_quat = loco_robot_base_quat(loco_feedback)
                    head_planner_cmd = compute_sonic_planner_command(
                        head_pose,
                        head_loco_state,
                        head_loco_cfg,
                        loop_dt,
                        calib_pos=head_loco_calib_pos,
                        locomotion_mode=args.loco_mode,
                        robot_base_quat=robot_base_quat,
                    )
                    planner_cmd = head_planner_cmd

                if (
                    args.head_locomotion
                    and args.hybrid_locomotion
                    and live_teleop
                    and not stand_hold
                    and head_motion_allowed
                ):
                    if kb_controller.pending_imu_sync:
                        sync_kb_body_facing_from_robot()
                        kb_controller.pending_imu_sync = False
                    kb_planner_cmd = keyboard_planner_command(
                        kb_controller,
                        kb_hold_tracker.held(now),
                        head_loco_cfg,
                        period,
                        locomotion_mode=args.loco_mode,
                        default_speed=args.keyboard_loco_speed,
                        smooth_alpha=max(args.loco_smooth, 0.18),
                    )
                    planner_cmd = merge_hybrid_planner_commands(
                        head_planner_cmd,
                        kb_planner_cmd,
                        head_loco_cfg,
                        locomotion_mode=args.loco_mode,
                        kb_has_control=kb_controller.has_control,
                        kb_facing=kb_controller.output_facing(),
                    )

                if (
                    args.head_height_squat
                    and head_motion_allowed
                    and head_pose is not None
                    and head_loco_calib_pos is not None
                ):
                    pelvis_height, height_debug = compute_head_pelvis_height(
                        head_pose,
                        head_loco_calib_pos,
                        head_loco_state,
                        head_height_cfg,
                    )
                    if planner_cmd is None:
                        loco_feedback = feedback_client.latest() if feedback_client is not None else None
                        facing = update_facing_from_head(
                            head_pose,
                            head_loco_state,
                            head_loco_cfg,
                            robot_base_quat=loco_robot_base_quat(loco_feedback),
                        )
                        planner_cmd = SonicPlannerCommand(
                            mode=0,
                            movement=np.zeros(3, dtype=np.float64),
                            facing=facing,
                            speed=-1.0,
                        )
                    planner_cmd = apply_height_to_planner_command(
                        planner_cmd,
                        pelvis_height,
                        head_height_cfg,
                    )
                    head_loco_state.debug.update(height_debug)

                if planner_cmd is not None:
                    state.mode = planner_cmd.mode
                    state.movement = planner_cmd.movement.copy()
                    state.speed = planner_cmd.speed
                    state.height = planner_cmd.height
                    state.facing_angle = float(
                        np.arctan2(planner_cmd.facing[1], planner_cmd.facing[0])
                    )

                if eval_logger is not None and live_teleop and not stand_hold:
                    loco_feedback = feedback_client.latest() if feedback_client is not None else None
                    head_active = bool(
                        head_planner_cmd is not None
                        and head_planner_cmd.mode > 0
                        and head_planner_cmd.speed > 0.0
                    )
                    kb_active = bool(
                        kb_planner_cmd is not None
                        and kb_planner_cmd.mode > 0
                        and kb_planner_cmd.speed > 0.0
                    )
                    kb_dbg = {}
                    if kb_planner_cmd is not None and kb_active:
                        kb_dbg = {
                            "vx": round(float(kb_planner_cmd.movement[0] * kb_planner_cmd.speed), 4),
                            "vy": round(float(kb_planner_cmd.movement[1] * kb_planner_cmd.speed), 4),
                        }
                    eval_logger.write_row(
                        mode=state.mode,
                        movement=state.movement,
                        facing=state.facing,
                        speed=state.speed,
                        head_active=head_active,
                        kb_active=kb_active,
                        imu_debug=head_loco_state.debug.get("imu"),
                        head_cmd_debug=head_loco_state.debug,
                        kb_cmd_debug=kb_dbg,
                        feedback=loco_feedback,
                        waypoint_mark=pending_waypoint_mark,
                    )
                    pending_waypoint_mark = None

                if stand_hold:
                    state.set_idle()
                    kb_state.set_idle()
                    kb_controller.reset_motion()
                    kb_hold_tracker.clear()

                if not args.no_auto_start and now - last_start > 2.0:
                    publisher.send_command(start=True, stop=False, planner=True)
                    last_start = now
                    policy_started = True

                if (
                    not args.staged_calib
                    and pending_calibration_deadline > 0.0
                    and now >= pending_calibration_deadline
                ):
                    captured = capture_official_calibration(tracking, args)
                    if captured is None:
                        print("\nCalibration delayed: AVP head tracking is not ready.")
                        pending_calibration_deadline = now + 0.25
                        calib_hold_until = pending_calibration_deadline
                    else:
                        official_calibration = captured
                        if head_pose is not None:
                            apply_loco_sync(head_pose)
                        if pending_base_positions is not None:
                            official_base_positions = pending_base_positions.copy()
                        if pending_base_orientations is not None:
                            official_base_orientations = pending_base_orientations.copy()
                        pending_calibration_deadline = 0.0
                        pending_base_positions = None
                        pending_base_orientations = None
                        calib_hold_until = 0.0
                        print(
                            "\nCalibration captured at delayed hand pose:",
                            "left_base=",
                            np.round(official_base_positions[0:3], 3).tolist(),
                            "right_base=",
                            np.round(official_base_positions[3:6], 3).tolist(),
                        )

                hand_debug_lines = []
                hand_teleop_active = (
                    enable_inspire_hand
                    and (
                        not (args.staged_calib and args.mapping_mode == "official-calib")
                        or (live_teleop and not calib_session.paused)
                    )
                )
                if hand_teleop_active:
                    saw_hand_tracking = False
                    for side, mapper in (
                        ("left", left_hand_mapper),
                        ("right", right_hand_mapper),
                    ):
                        if not hand_is_active(args, side):
                            continue
                        hand = getattr(tracking, side, None) if tracking is not None else None
                        if hand is None:
                            continue
                        try:
                            command, hand_map_debug = mapper.build_command(hand)
                        except AttributeError as exc:
                            if args.print_debug and now - last_debug > 1.0:
                                hand_debug_lines.append(f"{side}_hand missing joint: {exc}")
                            continue

                        command = np.clip(command, hand_command_min, hand_command_max)
                        if side == "left":
                            left_hand_command = command
                        else:
                            right_hand_command = command
                        saw_hand_tracking = True
                        if args.print_debug and now - last_debug > 1.0:
                            hand_debug_lines.append(f"{side}_{format_hand_debug(command, hand_map_debug)}")

                    if saw_hand_tracking:
                        last_hand_tracking_time = now
                        if args.enable_inspire_hand_sim:
                            publisher.send_inspire_hand(
                                left_hand_command,
                                right_hand_command,
                                topic=args.inspire_hand_topic,
                            )
                        if dds_hand_publisher is not None:
                            dds_hand_publisher.send_physical_hands(
                                left_command=left_hand_command,
                                right_command=right_hand_command,
                                mode=hand_dds_sides,
                            )
                    elif (
                        last_hand_tracking_time
                        and now - last_hand_tracking_time >= args.hand_tracking_timeout
                    ):
                        left_hand_command = hand_open_command.copy()
                        right_hand_command = hand_open_command.copy()
                        left_hand_mapper.reset()
                        right_hand_mapper.reset()
                        last_hand_tracking_time = now
                        if args.enable_inspire_hand_sim:
                            publisher.send_inspire_hand(
                                left_hand_command,
                                right_hand_command,
                                topic=args.inspire_hand_topic,
                            )
                        if dds_hand_publisher is not None:
                            dds_hand_publisher.send_physical_hands(
                                left_command=left_hand_command,
                                right_command=right_hand_command,
                                mode=hand_dds_sides,
                            )
                        if args.print_debug and now - last_debug > 1.0:
                            hand_debug_lines.append("hand tracking lost -> sim safe open")

                target_debug = {}
                left_wrist_joints = None
                right_wrist_joints = None
                if stand_hold:
                    targets = resolve_stand_hold_targets()
                elif args.legacy_head_relative:
                    targets = build_vr_targets(tracking, initial_head_robot_pos, args)
                elif args.mapping_mode == "relative-zero":
                    targets = build_calibrated_vr_targets(
                        tracking,
                        neutral_left_pose,
                        neutral_right_pose,
                        neutral_head_pose,
                        args,
                    )
                elif args.mapping_mode == "official-calib":
                    if official_calibration is None:
                        targets = (official_base_positions.copy(), official_base_orientations.copy())
                    else:
                        result = build_official_calib_vr_targets(
                            tracking,
                            official_calibration,
                            official_base_positions,
                            official_base_orientations,
                            args,
                            force_base=(
                                now < calib_hold_until
                                or calib_session.paused
                                or (
                                    args.staged_calib
                                    and args.mapping_mode == "official-calib"
                                    and not live_teleop
                                )
                            ),
                        )
                        if result is None:
                            targets = None
                        else:
                            vr_position, vr_orientation, target_debug, left_wrist_joints, right_wrist_joints = result
                            targets = (vr_position, vr_orientation)
                else:
                    result = build_head_relative_vr_targets(
                        tracking,
                        neutral_left_pose,
                        neutral_right_pose,
                        args,
                    )
                    if result is None:
                        targets = None
                    else:
                        vr_position, vr_orientation, target_debug = result
                        targets = (vr_position, vr_orientation)
                preengage_stream = should_preenage_stream()
                if targets is not None and (policy_started or preengage_stream):
                    vr_position, vr_orientation = targets
                    use_ramp = (
                        stand_hold
                        or calib_session.paused
                        or not live_teleop
                        or now < calib_hold_until
                    )
                    if policy_started and args.mapping_mode in ("hybrid", "official-calib") and not args.legacy_head_relative:
                        if smoother.position is None and last_sent_vr_position is not None:
                            seed_arm_smoothers_from_vr(
                                last_sent_vr_position, last_sent_vr_orientation, now
                            )
                        vr_position, vr_orientation = apply_vr_output_smoothing(
                            vr_position,
                            vr_orientation,
                            now,
                            ramp=use_ramp,
                        )
                    last_sent_vr_position = vr_position.copy()
                    last_sent_vr_orientation = vr_orientation.copy()
                    if preengage_stream and not policy_started:
                        send_idle_planner(vr_position, vr_orientation)
                    else:
                        publisher.send_planner(
                            mode=state.mode,
                            movement=state.movement,
                            facing=np.array([1.0, 0.0, 0.0], dtype=np.float64)
                            if stand_hold
                            else state.facing,
                            speed=state.speed,
                            height=state.height,
                            vr_position=vr_position,
                            vr_orientation=vr_orientation,
                            left_wrist_joints=left_wrist_joints,
                            right_wrist_joints=right_wrist_joints,
                        )

                    if args.print_debug and now - last_debug > 1.0:
                        print(
                            "mode=",
                            state.mode,
                            "height=",
                            round(state.height, 3) if state.height >= 0 else state.height,
                            "move=",
                            np.round(state.movement, 3).tolist(),
                            "face=",
                            np.round(state.facing, 3).tolist(),
                            "vr_pos=",
                            np.round(vr_position, 3).tolist(),
                        )
                        if target_debug:
                            print(
                                "rel_to_head left=",
                                None
                                if target_debug.get("left_rel") is None
                                else np.round(target_debug.get("left_rel"), 3).tolist(),
                                "right=",
                                None
                                if target_debug.get("right_rel") is None
                                else np.round(target_debug.get("right_rel"), 3).tolist(),
                            )
                            if "left_delta" in target_debug:
                                print(
                                    "calib_delta left=",
                                    None
                                    if target_debug.get("left_delta") is None
                                    else np.round(target_debug.get("left_delta"), 3).tolist(),
                                    "right=",
                                    None
                                    if target_debug.get("right_delta") is None
                                    else np.round(target_debug.get("right_delta"), 3).tolist(),
                                )
                            if left_wrist_joints is not None or right_wrist_joints is not None:
                                print(
                                    "wrist_joints left=",
                                    None if left_wrist_joints is None else np.round(left_wrist_joints, 3).tolist(),
                                    "right=",
                                    None if right_wrist_joints is None else np.round(right_wrist_joints, 3).tolist(),
                                )
                        if args.head_locomotion and head_loco_state.debug:
                            print(
                                "head_loco delta_body=",
                                head_loco_state.debug.get("delta_body"),
                                "cmd=",
                                head_loco_state.debug.get("cmd"),
                                "strafe=",
                                head_loco_state.debug.get("strafe_intent"),
                                "fwd=",
                                head_loco_state.debug.get("forward_intent"),
                                "imu=",
                                head_loco_state.debug.get("imu"),
                                "head_dz=",
                                head_loco_state.debug.get("head_delta_z"),
                                "head_drop=",
                                head_loco_state.debug.get("head_drop"),
                                "pelvis_h=",
                                head_loco_state.debug.get("pelvis_height"),
                            )
                        for line in hand_debug_lines:
                            print(line)
                        last_debug = now

                time.sleep(period)
    finally:
        if fpv_streamer is not None:
            fpv_streamer.close()
        if feedback_client is not None:
            feedback_client.close()
        if dds_hand_publisher is not None:
            try:
                dds_hand_publisher.safe_open(hand_open_command)
            except Exception as exc:
                print(f"Failed to safe-open Inspire hand cleanly: {exc}")
        publisher.close()


if __name__ == "__main__":
    main()
