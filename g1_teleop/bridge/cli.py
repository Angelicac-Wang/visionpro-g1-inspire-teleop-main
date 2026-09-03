"""CLI argument definitions for the AVP → SONIC bridge."""

from __future__ import annotations

import argparse
import os

from g1_teleop.paths import SCRIPTS_DIR, inspire_hand_sdk_root


def default_hand_calibration_file(side: str = "right") -> str:
    filename = (
        "visionpro_left_hand_calibration.json"
        if side == "left"
        else "visionpro_right_hand_calibration.json"
    )
    repo_path = os.path.join(SCRIPTS_DIR, filename)
    if side == "right":
        sdk_root = inspire_hand_sdk_root()
        sdk_example_path = os.path.join(sdk_root, "example", filename)
        if os.path.exists(repo_path):
            return repo_path
        if os.path.exists(sdk_example_path):
            return sdk_example_path
    return repo_path


def resolve_hand_calibration_files(args) -> tuple[str, str]:
    if args.hand_calibration_file:
        return args.hand_calibration_file, args.hand_calibration_file
    return args.hand_calibration_file_left, args.hand_calibration_file_right


def parse_args():
    parser = argparse.ArgumentParser(description="Send AVP tracking to SONIC ZMQManager.")
    parser.add_argument("--avp-endpoint", default="192.168.2.45", help="Vision Pro IP or room code.")
    parser.add_argument("--host", default="*", help="ZMQ PUB bind host. Use '*' when SONIC uses --zmq-host localhost.")
    parser.add_argument("--port", type=int, default=5556, help="ZMQ PUB port.")
    parser.add_argument("--publish-rate", type=float, default=50.0)
    start_group = parser.add_mutually_exclusive_group()
    start_group.add_argument(
        "--no-auto-start",
        dest="no_auto_start",
        action="store_true",
        default=True,
        help="Do not send SONIC start command automatically.",
    )
    start_group.add_argument(
        "--auto-start",
        dest="no_auto_start",
        action="store_false",
        help="Send SONIC start commands automatically after AVP tracking is ready.",
    )
    parser.add_argument(
        "--active-hands",
        choices=("both", "left", "right"),
        default="both",
        help="Only track the selected hand(s); inactive hands are held at the configured robot init target.",
    )
    parser.add_argument("--head-to-waist-x", type=float, default=0.15)
    parser.add_argument("--head-to-waist-y", type=float, default=0.0)
    parser.add_argument("--head-to-waist-z", type=float, default=0.45)
    parser.add_argument("--head-position-scale", type=float, default=0.5)
    parser.add_argument("--hand-position-scale", type=float, default=0.8)
    parser.add_argument(
        "--mapping-mode",
        choices=("relative-zero", "head-relative", "hybrid", "official-calib"),
        default="official-calib",
        help=(
            "relative-zero maps startup hand pose to neutral targets; head-relative uses "
            "live wrist-head offsets; hybrid uses wrist-head offsets with smoothing; "
            "official-calib uses Pico-style headset-yaw-compensated calibration deltas."
        ),
    )
    parser.add_argument("--body-scale", type=float, default=1.0, help="Scale applied to wrist-head relative positions.")
    parser.add_argument("--head-relative-x-offset", type=float, default=0.0)
    parser.add_argument("--head-relative-y-offset", type=float, default=0.0)
    parser.add_argument("--head-relative-z-offset", type=float, default=0.0)
    parser.add_argument(
        "--hybrid-add-head-base-to-hands",
        action="store_true",
        help="Old hybrid behavior: add robot head base height to hand targets.",
    )
    parser.add_argument(
        "--hybrid-smoothing-tau",
        type=float,
        default=0.04,
        help="Low-pass time constant (s) for arm/head vr_position in official-calib teleop.",
    )
    parser.add_argument(
        "--hybrid-max-speed",
        type=float,
        default=0.45,
        help="Max target speed (m/s) for each VR point during teleop smoothing.",
    )
    parser.add_argument(
        "--arm-orientation-smoothing-tau",
        type=float,
        default=0.12,
        help="Low-pass time constant (s) for wrist/head quaternions during teleop.",
    )
    parser.add_argument(
        "--arm-max-angular-speed",
        type=float,
        default=2.0,
        help="Max wrist/head angular speed (rad/s) during teleop orientation smoothing.",
    )
    parser.add_argument(
        "--vr-ramp-max-speed",
        type=float,
        default=0.08,
        help="Max VR point speed (m/s) during stand-hold / CALIB_SYNC / pre-teleop (safety ramp).",
    )
    parser.add_argument(
        "--vr-ramp-smoothing-tau",
        type=float,
        default=0.14,
        help="Position smoothing tau (s) during stand-hold / pre-teleop ramp.",
    )
    parser.add_argument(
        "--vr-ramp-max-angular-speed",
        type=float,
        default=0.9,
        help="Max wrist/head angular speed (rad/s) during stand-hold / pre-teleop ramp.",
    )
    parser.add_argument(
        "--vr-ramp-orientation-tau",
        type=float,
        default=0.20,
        help="Orientation smoothing tau (s) during stand-hold / pre-teleop ramp.",
    )
    parser.add_argument(
        "--stand-hold-mode",
        choices=("init-pose", "track-robot"),
        default="init-pose",
        help=(
            "After ] ENGAGE: init-pose keeps configured L-shape VR targets (tuning workflow). "
            "track-robot holds current FK arm pose (safer if robot arms were down)."
        ),
    )
    parser.add_argument(
        "--stream-init-pose",
        action="store_true",
        default=True,
        help="Publish configured init-pose VR targets over ZMQ even before AVP tracking locks.",
    )
    parser.add_argument(
        "--no-stream-init-pose",
        dest="stream_init_pose",
        action="store_false",
        help="Wait for AVP tracking before sending any VR targets.",
    )
    parser.add_argument(
        "--official-calib-base",
        choices=("current-target", "init-pose", "neutral"),
        default="init-pose",
        help=(
            "Robot-side base used when pressing c in official-calib mode. "
            "current-target avoids a jump by calibrating against the current published target; "
            "init-pose returns to the configured robot init pose. neutral is kept as an alias."
        ),
    )
    parser.add_argument(
        "--calib-hold-sec",
        type=float,
        default=2.0,
        help="Seconds to hold still during F/S/H calibration captures.",
    )
    parser.add_argument(
        "--calib-min-frames",
        type=int,
        default=30,
        help="Minimum AVP frames averaged per calibration hold.",
    )
    parser.add_argument(
        "--calib-max-head-std",
        type=float,
        default=0.025,
        help="Max head position std (m) during calibration hold.",
    )
    parser.add_argument(
        "--calib-max-wrist-std",
        type=float,
        default=0.030,
        help="Max wrist position std (m) during calibration hold.",
    )
    parser.add_argument(
        "--staged-calib",
        action="store_true",
        default=True,
        help="Use F/]/S/T staged calibration (PICO VR_3PT aligned).",
    )
    parser.add_argument(
        "--legacy-calib-flow",
        dest="staged_calib",
        action="store_false",
        help="Use legacy c/]/T calibration flow.",
    )
    parser.add_argument(
        "--require-sync-calib",
        action="store_true",
        default=True,
        help="Require S (CALIB_SYNC) before T when staged-calib is enabled.",
    )
    parser.add_argument(
        "--no-require-sync-calib",
        dest="require_sync_calib",
        action="store_false",
        help="Allow T after F and ] without S.",
    )
    parser.add_argument(
        "--zmq-feedback-host",
        default="127.0.0.1",
        help="SONIC deploy ZMQ feedback host (g1_debug PUB, default port 5557).",
    )
    parser.add_argument(
        "--zmq-feedback-port",
        type=int,
        default=5557,
        help="SONIC deploy ZMQ feedback port.",
    )
    parser.add_argument(
        "--zmq-feedback-topic",
        default="g1_debug",
        help="SONIC deploy ZMQ feedback topic prefix.",
    )
    parser.add_argument(
        "--use-fk-calib",
        action="store_true",
        default=True,
        help="Prefer G1 FK from body_q for CALIB_SYNC robot base when available.",
    )
    parser.add_argument(
        "--no-use-fk-calib",
        dest="use_fk_calib",
        action="store_false",
        help="Use vr_3point feedback only for CALIB_SYNC robot base.",
    )
    head_translation_group = parser.add_mutually_exclusive_group()
    head_translation_group.add_argument(
        "--lock-head-translation",
        dest="lock_head_translation",
        action="store_true",
        default=True,
        help="Keep the robot head target xyz fixed after calibration; headset walking will not move the upper-body target.",
    )
    head_translation_group.add_argument(
        "--allow-head-translation",
        dest="lock_head_translation",
        action="store_false",
        help="Allow headset translation to move the robot head target.",
    )
    head_rotation_group = parser.add_mutually_exclusive_group()
    head_rotation_group.add_argument(
        "--head-yaw-only",
        dest="head_yaw_only",
        action="store_true",
        default=True,
        help="Track only headset yaw for the robot head target; ignore headset pitch/roll.",
    )
    head_rotation_group.add_argument(
        "--full-head-rotation",
        dest="head_yaw_only",
        action="store_false",
        help="Track headset yaw, pitch, and roll for the robot head target.",
    )
    parser.add_argument(
        "--official-delta-sign-x",
        type=float,
        default=-1.0,
        help="Axis sign for official-calib hand forward/back delta.",
    )
    parser.add_argument(
        "--official-delta-sign-y",
        type=float,
        default=-1.0,
        help="Axis sign for official-calib hand left/right delta.",
    )
    parser.add_argument(
        "--official-delta-sign-z",
        type=float,
        default=1.0,
        help="Axis sign for official-calib hand up/down delta.",
    )
    for side in ("left", "right"):
        parser.add_argument(
            f"--{side}-official-delta-sign-x",
            type=float,
            default=None,
            help=f"Optional override for {side}-arm official-calib delta sign on robot X.",
        )
        parser.add_argument(
            f"--{side}-official-delta-sign-y",
            type=float,
            default=None,
            help=f"Optional override for {side}-arm official-calib delta sign on robot Y.",
        )
        parser.add_argument(
            f"--{side}-official-delta-sign-z",
            type=float,
            default=None,
            help=f"Optional override for {side}-arm official-calib delta sign on robot Z.",
        )
    parser.add_argument(
        "--hand-forward-scale",
        type=float,
        default=1.55,
        help="Extra scale for positive robot-X hand motion in official-calib mode.",
    )
    parser.add_argument(
        "--hand-backward-scale",
        type=float,
        default=0.35,
        help="Extra scale for negative robot-X hand motion in official-calib mode.",
    )
    for side in ("left", "right"):
        parser.add_argument(
            f"--{side}-hand-forward-scale",
            type=float,
            default=None,
            help=f"Optional override for {side}-arm positive robot-X reach scale.",
        )
        parser.add_argument(
            f"--{side}-hand-backward-scale",
            type=float,
            default=None,
            help=f"Optional override for {side}-arm negative robot-X reach scale.",
        )
    _hand_delta_remap_choices = ("identity", "unitree-left-arm")
    parser.add_argument(
        "--left-hand-delta-remap",
        choices=_hand_delta_remap_choices,
        default="identity",
        help=(
            "Linear 3x3 basis applied to left-arm AVP rel delta before axis signs. "
            "'unitree-left-arm' decouples robot-X from vertical/lateral AVP coupling; "
            "identity is the current sim default for freer left reach-up."
        ),
    )
    parser.add_argument(
        "--right-hand-delta-remap",
        choices=_hand_delta_remap_choices,
        default="identity",
        help="Linear 3x3 basis applied to right-arm AVP rel delta before axis signs.",
    )
    parser.add_argument(
        "--left-hand-delta-scale",
        type=float,
        default=1.0,
        help="Multiply official-calib AVP-minus-calibration wrist delta for the left arm.",
    )
    parser.add_argument(
        "--left-hand-delta-z-scale",
        type=float,
        default=0.45,
        help=(
            "Scale left-arm positive-Z delta during forward extension only "
            "(AVP couples spurious lift when rel_x decreases). Reach-up uses "
            "--left-hand-delta-z-up-scale instead."
        ),
    )
    parser.add_argument(
        "--left-hand-delta-z-up-scale",
        type=float,
        default=1.0,
        help="Scale left-arm positive-Z delta when motion is reach-up (not forward-dominant).",
    )
    parser.add_argument(
        "--right-hand-delta-scale",
        type=float,
        default=1.0,
        help="Multiply official-calib AVP-minus-calibration wrist delta for the right arm.",
    )
    parser.add_argument(
        "--min-hand-x",
        type=float,
        default=0.20,
        help="Lower robot-X limit for hand targets after official-calib mapping.",
    )
    parser.add_argument(
        "--max-hand-x",
        type=float,
        default=0.88,
        help="Upper robot-X limit for hand targets after official-calib mapping.",
    )
    parser.add_argument(
        "--robot-init-pose",
        choices=("forearms-forward", "debug-ready", "arms-forward", "low-ready", "custom"),
        default="forearms-forward",
        help=(
            "Fixed robot-side pose used by official-calib startup and by c when "
            "--official-calib-base init-pose is selected."
        ),
    )
    parser.add_argument(
        "--wrist-orientation-mode",
        choices=("neutral", "identity", "live", "calibrated", "wrist-joints"),
        default="calibrated",
        help=(
            "How to fill wrist quaternions in vr_orientation. 'calibrated' tracks "
            "wrist rotation through the policy; 'wrist-joints' keeps policy orientation neutral "
            "and sends direct wrist roll/pitch/yaw overrides."
        ),
    )
    _wrist_orient_choices = ("neutral", "identity", "live", "calibrated", "wrist-joints")
    parser.add_argument(
        "--left-wrist-orientation-mode",
        choices=_wrist_orient_choices,
        default="calibrated",
        help=(
            "Left-arm wrist quaternion mode. Default calibrated + avp-palm-left for sim teleop; "
            "use neutral if the torso hunches when the left hand moves."
        ),
    )
    parser.add_argument(
        "--right-wrist-orientation-mode",
        choices=_wrist_orient_choices,
        default=None,
        help="Optional override for right-arm wrist orientation (default: global mode).",
    )
    parser.add_argument(
        "--wrist-rotation-scale",
        type=float,
        default=0.90,
        help="Scale applied to calibrated wrist rotation. Use 0.5-0.7 if wrist tracking feels too aggressive.",
    )
    parser.add_argument(
        "--wrist-axis-remap",
        choices=("identity", "avp-palm", "avp-palm-left", "x-to-y", "x-to-z", "y-to-x", "z-to-x"),
        default="avp-palm",
        help="Extra local-basis remap for calibrated wrist rotation.",
    )
    _wrist_remap_choices = ("identity", "avp-palm", "avp-palm-left", "x-to-y", "x-to-z", "y-to-x", "z-to-x")
    parser.add_argument(
        "--left-wrist-axis-remap",
        choices=_wrist_remap_choices,
        default="avp-palm-left",
        help=(
            "Left-arm wrist rotation basis. Default avp-palm-left; set identity to use "
            "--wrist-axis-remap only."
        ),
    )
    parser.add_argument(
        "--right-wrist-axis-remap",
        choices=_wrist_remap_choices,
        default=None,
        help="Optional override for right-arm wrist rotation basis (default: --wrist-axis-remap).",
    )
    parser.add_argument("--left-wrist-rot-sign-x", type=float, default=-1.0)
    parser.add_argument("--left-wrist-rot-sign-y", type=float, default=1.0)
    parser.add_argument("--left-wrist-rot-sign-z", type=float, default=-1.0)
    parser.add_argument("--right-wrist-rot-sign-x", type=float, default=-1.0)
    parser.add_argument("--right-wrist-rot-sign-y", type=float, default=1.0)
    parser.add_argument("--right-wrist-rot-sign-z", type=float, default=-1.0)
    parser.add_argument("--wrist-joint-sign-roll", type=float, default=1.0)
    parser.add_argument("--wrist-joint-sign-pitch", type=float, default=1.0)
    parser.add_argument("--wrist-joint-sign-yaw", type=float, default=1.0)
    parser.add_argument("--max-wrist-roll", type=float, default=1.65)
    parser.add_argument("--max-wrist-pitch", type=float, default=1.65)
    parser.add_argument("--max-wrist-yaw", type=float, default=1.2)
    parser.add_argument(
        "--legacy-head-relative",
        action="store_true",
        help="Use the old head-relative mapping instead of startup neutral calibration.",
    )
    parser.add_argument("--neutral-x", type=float, default=0.22, help="Neutral wrist target x in SONIC local frame.")
    parser.add_argument("--neutral-left-y", type=float, default=0.24, help="Neutral left wrist y.")
    parser.add_argument("--neutral-right-y", type=float, default=-0.24, help="Neutral right wrist y.")
    parser.add_argument("--neutral-hand-z", type=float, default=0.18, help="Neutral wrist height. Lower this if arms start high.")
    parser.add_argument("--neutral-head-x", type=float, default=0.0)
    parser.add_argument("--neutral-head-y", type=float, default=0.0)
    parser.add_argument("--neutral-head-z", type=float, default=0.75)
    parser.add_argument("--max-x", type=float, default=1.0)
    parser.add_argument("--max-y", type=float, default=1.0)
    parser.add_argument("--max-z", type=float, default=1.0)
    parser.add_argument("--left-fallback-y", type=float, default=0.25)
    parser.add_argument("--right-fallback-y", type=float, default=-0.25)
    parser.add_argument("--print-debug", action="store_true")
    parser.add_argument(
        "--enable-inspire-hand-sim",
        action="store_true",
        help="Also publish AVP finger tracking as sim-only Inspire hand ZMQ commands.",
    )
    parser.add_argument(
        "--enable-inspire-hand-dds",
        action="store_true",
        help="Publish AVP finger tracking to physical Inspire hand DDS topic(s).",
    )
    parser.add_argument("--inspire-hand-topic", default="inspire_hand")
    parser.add_argument(
        "--hand-dds-sides",
        choices=("l", "r", "both"),
        default="both",
        help=(
            "Physical Inspire hand DDS topics. both publishes AVP left->l and AVP right->r; "
            "l or r publishes only that side."
        ),
    )
    parser.add_argument(
        "--hand-topic-side",
        choices=("l", "r"),
        default="l",
        help="Deprecated alias for single-hand setups. Prefer --hand-dds-sides.",
    )
    parser.add_argument(
        "--hand-dds-network",
        default=None,
        help="Optional DDS NIC for physical Inspire hand commands, e.g. enp3s0.",
    )
    parser.add_argument("--hand-tracking-timeout", type=float, default=1.0)
    parser.add_argument(
        "--arm-tracking-hold",
        action="store_true",
        default=None,
        help="When wrist tracking is lost, hold the last valid arm target until it returns.",
    )
    parser.add_argument(
        "--no-arm-tracking-hold",
        dest="arm_tracking_hold",
        action="store_false",
        help="Snap lost arms back to init/base target (legacy behavior).",
    )
    parser.set_defaults(arm_tracking_hold=True)
    parser.add_argument(
        "--hand-calibration-file-left",
        default=default_hand_calibration_file("left"),
        help="JSON open/close calibration for AVP left hand -> Inspire left.",
    )
    parser.add_argument(
        "--hand-calibration-file-right",
        default=default_hand_calibration_file("right"),
        help="JSON open/close calibration for AVP right hand -> Inspire right.",
    )
    parser.add_argument(
        "--hand-calibration-file",
        default=None,
        help="Optional: use one calibration file for both hands (overrides left/right paths).",
    )
    parser.add_argument("--open-angle", type=int, default=1000)
    parser.add_argument("--close-angle", type=int, default=0)
    parser.add_argument("--thumb-bend-open-angle", type=int, default=800)
    parser.add_argument("--thumb-bend-close-angle", type=int, default=200)
    parser.add_argument("--thumb-rotation-open-angle", type=int, default=200)
    parser.add_argument("--thumb-rotation-close-angle", type=int, default=800)
    parser.add_argument("--finger-range-scale", type=float, default=1.15)
    parser.add_argument("--thumb-bend-range-scale", type=float, default=1.35)
    parser.add_argument("--thumb-rotation-range-scale", type=float, default=1.25)
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
    parser.add_argument("--thumb-open-distance", type=float, default=0.12)
    parser.add_argument("--thumb-close-distance", type=float, default=0.035)
    parser.add_argument("--flip-thumb-rotation", action="store_true")
    parser.add_argument(
        "--head-locomotion",
        action="store_true",
        help="Drive SONIC walking from head motion (work-master velocity derivative).",
    )
    parser.add_argument(
        "--hybrid-locomotion",
        action="store_true",
        default=None,
        help="Allow WASD keyboard walk while head locomotion is active (vector sum).",
    )
    parser.add_argument(
        "--no-hybrid-locomotion",
        dest="hybrid_locomotion",
        action="store_false",
        help="Disable keyboard walk overlay when --head-locomotion is enabled.",
    )
    parser.add_argument(
        "--keyboard-loco-speed",
        type=float,
        default=0.42,
        help="Walk speed (m/s) for keyboard overlay in hybrid locomotion.",
    )
    parser.add_argument(
        "--eval-log",
        default=None,
        metavar="PATH",
        help="Write Task A / REMS eval CSV (locomotion + IMU fields) to PATH.",
    )
    parser.add_argument("--loco-velocity-gain", type=float, default=1.0)
    parser.add_argument("--loco-yaw-gain", type=float, default=0.9)
    parser.add_argument("--loco-forward-scale", type=float, default=1.0)
    parser.add_argument("--loco-lateral-scale", type=float, default=1.25)
    parser.add_argument("--loco-lateral-left-scale", type=float, default=1.0)
    parser.add_argument("--loco-lateral-right-scale", type=float, default=1.4)
    parser.add_argument("--loco-lateral-displacement-gain", type=float, default=0.0)
    parser.add_argument("--loco-lateral-deadzone", type=float, default=0.035)
    parser.add_argument("--loco-sign-x", type=float, default=1.0)
    parser.add_argument("--loco-sign-y", type=float, default=1.0)
    parser.add_argument("--loco-max-speed", type=float, default=0.50)
    parser.add_argument("--loco-max-yaw-rate", type=float, default=0.30)
    parser.add_argument("--loco-velocity-deadzone", type=float, default=0.06)
    parser.add_argument("--loco-yaw-deadzone", type=float, default=0.12)
    parser.add_argument("--loco-smooth", type=float, default=0.18)
    parser.add_argument("--loco-facing-smooth", type=float, default=0.28)
    parser.add_argument("--loco-output-deadzone", type=float, default=0.04)
    parser.add_argument("--loco-idle-decay", type=float, default=0.85)
    parser.add_argument(
        "--no-loco-imu-correction",
        dest="loco_imu_yaw_enabled",
        action="store_false",
        help="Disable base IMU closed-loop yaw correction for head locomotion.",
    )
    parser.add_argument("--loco-imu-yaw-gain", type=float, default=1.0)
    parser.add_argument(
        "--loco-imu-yaw-deadzone",
        type=float,
        default=0.05,
        help="Ignore IMU yaw error inside this deadband (rad, default ~3 deg).",
    )
    parser.add_argument("--loco-imu-yaw-max-correction", type=float, default=0.45)
    parser.set_defaults(loco_imu_yaw_enabled=True, hybrid_locomotion=False)
    parser.add_argument("--loco-mode", type=int, default=1, help="SONIC planner mode when walking.")
    parser.add_argument(
        "--head-height-squat",
        action="store_true",
        default=False,
        help="Map AVP head vertical drop to SONIC squat/kneel planner height (mode 4/6).",
    )
    parser.add_argument(
        "--no-head-height-squat",
        dest="head_height_squat",
        action="store_false",
        help="Disable head-height squat mapping.",
    )
    parser.add_argument(
        "--head-vertical-follow",
        action="store_true",
        default=None,
        help="Move robot head target Z with operator squat while keeping XY locked.",
    )
    parser.add_argument(
        "--no-head-vertical-follow",
        dest="head_vertical_follow",
        action="store_false",
        help="Keep robot head target height fixed even when squat mapping is enabled.",
    )
    parser.add_argument(
        "--head-vertical-scale",
        type=float,
        default=0.85,
        help="Scale applied to AVP head Z delta when --head-vertical-follow is enabled.",
    )
    parser.add_argument("--squat-walk-threshold", type=float, default=0.72, help="Pelvis height above this allows walking.")
    parser.add_argument("--squat-height-min", type=float, default=0.50, help="Pelvis height at full squat band.")
    parser.add_argument("--squat-kneel-height", type=float, default=0.35, help="Pelvis height at deep kneel.")
    parser.add_argument("--head-drop-start", type=float, default=0.06, help="Ignore head drop below this (meters).")
    parser.add_argument("--head-drop-to-squat", type=float, default=0.24, help="Head drop (m) mapped to full squat band.")
    parser.add_argument("--head-drop-to-kneel", type=float, default=0.42, help="Head drop (m) mapped to kneel band.")
    parser.add_argument("--squat-height-smooth", type=float, default=0.18, help="Low-pass alpha for pelvis height.")
    parser.add_argument(
        "--enable-mujoco-fpv",
        action="store_true",
        default=False,
        help="Stream MuJoCo head_camera to Vision Pro via Tracking Streamer WebRTC.",
    )
    parser.add_argument(
        "--no-mujoco-fpv",
        dest="enable_mujoco_fpv",
        action="store_false",
        help="Disable MuJoCo first-person video on Vision Pro.",
    )
    parser.add_argument("--mujoco-camera-host", default="127.0.0.1", help="MuJoCo camera ZMQ host.")
    parser.add_argument("--mujoco-camera-port", type=int, default=5555, help="MuJoCo camera ZMQ port.")
    parser.add_argument(
        "--mujoco-camera-name",
        default="ego_view",
        help="Camera name from MuJoCo sim image publish (ego_view = head_camera).",
    )
    parser.add_argument(
        "--avp-webrtc-port",
        type=int,
        default=9999,
        help="WebRTC port for Vision Pro Tracking Streamer video.",
    )
    parser.add_argument("--avp-video-size", default="1280x720", help="Video resolution sent to Vision Pro.")
    parser.add_argument("--avp-video-fps", type=int, default=30, help="Target FPV frame rate.")
    thumb_rotation_command_group = parser.add_mutually_exclusive_group()
    thumb_rotation_command_group.add_argument(
        "--invert-thumb-rotation-command",
        dest="invert_thumb_rotation_command",
        action="store_true",
        default=True,
        help="Reverse the physical Inspire thumb-root command direction after AVP calibration.",
    )
    thumb_rotation_command_group.add_argument(
        "--no-invert-thumb-rotation-command",
        dest="invert_thumb_rotation_command",
        action="store_false",
        help="Use the non-inverted physical Inspire thumb-root command direction.",
    )
    return parser.parse_args()
