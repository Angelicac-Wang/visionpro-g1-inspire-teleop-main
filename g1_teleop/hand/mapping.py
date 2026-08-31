import json
import os
import time

import numpy as np


FINGER_ORDER = ("little", "ring", "middle", "index")
COMMAND_ORDER = ("little", "ring", "middle", "index", "thumb_bend", "thumb_rot")

FINGER_CHAINS = {
    "little": {
        "metacarpal": "littleMetacarpal",
        "knuckle": "littleKnuckle",
        "prox": "littleIntermediateBase",
        "mid": "littleIntermediateTip",
        "tip": "littleTip",
    },
    "ring": {
        "metacarpal": "ringMetacarpal",
        "knuckle": "ringKnuckle",
        "prox": "ringIntermediateBase",
        "mid": "ringIntermediateTip",
        "tip": "ringTip",
    },
    "middle": {
        "metacarpal": "middleMetacarpal",
        "knuckle": "middleKnuckle",
        "prox": "middleIntermediateBase",
        "mid": "middleIntermediateTip",
        "tip": "middleTip",
    },
    "index": {
        "metacarpal": "indexMetacarpal",
        "knuckle": "indexKnuckle",
        "prox": "indexIntermediateBase",
        "mid": "indexIntermediateTip",
        "tip": "indexTip",
    },
    "thumb": {
        "knuckle": "thumbKnuckle",
        "prox": "thumbIntermediateBase",
        "mid": "thumbIntermediateTip",
        "tip": "thumbTip",
    },
}

DEFAULT_CALIBRATION = {
    "little": {"open": 10.0, "close": 165.0},
    "ring": {"open": 10.0, "close": 165.0},
    "middle": {"open": 10.0, "close": 165.0},
    "index": {"open": 10.0, "close": 165.0},
    "thumb_bend": {"open": 5.0, "close": 75.0},
    "thumb_rot": {"open": 90.0, "close": 165.0},
}


def _point(transform):
    return np.asarray(transform, dtype=np.float64)[:3, 3]


def _get_point(hand, name, fallback=None):
    if hasattr(hand, name):
        return _point(getattr(hand, name))
    if fallback and hasattr(hand, fallback):
        return _point(getattr(hand, fallback))
    raise AttributeError(f"AVP hand tracking is missing joint '{name}'")


def _norm(v):
    return float(np.linalg.norm(v))


def _normalize(v, eps=1e-8):
    n = np.linalg.norm(v)
    if n < eps:
        return np.zeros_like(v, dtype=np.float64)
    return v / n


def _angle_deg(v1, v2, eps=1e-8):
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < eps or n2 < eps:
        return 0.0
    c = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
    return float(np.degrees(np.arccos(c)))


def _project_to_plane(v, normal):
    normal_u = _normalize(normal)
    return v - np.dot(v, normal_u) * normal_u


def _signed_angle_deg(v1, v2, normal, eps=1e-8):
    a = _normalize(v1, eps)
    b = _normalize(v2, eps)
    n = _normalize(normal, eps)
    if _norm(a) < eps or _norm(b) < eps or _norm(n) < eps:
        return 0.0
    dot = np.clip(np.dot(a, b), -1.0, 1.0)
    angle = np.degrees(np.arccos(dot))
    sign = np.sign(np.dot(np.cross(a, b), n))
    return float(angle * sign)


def _clamp01(value):
    return float(np.clip(value, 0.0, 1.0))


def _map_range(value, open_value, close_value):
    span = close_value - open_value
    if abs(span) < 1e-8:
        return 0.0
    return _clamp01((value - open_value) / span)


def _expand_mid(value, scale):
    return _clamp01(0.5 + max(float(scale), 0.0) * (_clamp01(value) - 0.5))


def _ema(previous, current, alpha):
    if previous is None:
        return float(current)
    alpha = _clamp01(alpha)
    return float((1.0 - alpha) * previous + alpha * current)


def _median(samples, channel, pose):
    values = [sample[channel] for sample in samples if channel in sample]
    if not values:
        return float(DEFAULT_CALIBRATION[channel][pose])
    return float(np.median(values))


class HandCalibration:
    def __init__(self, channels=None):
        self.channels = {
            key: {"open": float(value["open"]), "close": float(value["close"])}
            for key, value in (channels or DEFAULT_CALIBRATION).items()
        }

    @classmethod
    def load(cls, path):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        channels = data.get("channels", data)
        merged = {key: dict(value) for key, value in DEFAULT_CALIBRATION.items()}
        for key, value in channels.items():
            if key in merged and "open" in value and "close" in value:
                merged[key] = {"open": float(value["open"]), "close": float(value["close"])}
        return cls(merged)

    def save(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        payload = {
            "version": 1,
            "channels": self.channels,
            "command_order": list(COMMAND_ORDER),
            "notes": "open/close are raw AVP geometric metrics, not Inspire command values.",
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, sort_keys=True)
            f.write("\n")

    def set_from_samples(self, open_samples, close_samples):
        for channel in COMMAND_ORDER:
            self.channels[channel] = {
                "open": _median(open_samples, channel, "open"),
                "close": _median(close_samples, channel, "close"),
            }

    def normalize(self, channel, raw_value):
        cfg = self.channels[channel]
        return _map_range(float(raw_value), cfg["open"], cfg["close"])

    def range_for(self, channel):
        cfg = self.channels[channel]
        return (float(cfg["open"]), float(cfg["close"]))


class InspireHandMapper:
    def __init__(self, args, calibration=None):
        self.args = args
        self.calibration = calibration or HandCalibration()
        self.smoothed = {channel: None for channel in COMMAND_ORDER}

    @classmethod
    def from_args(cls, args, calibration_file=None):
        calibration = HandCalibration()
        path = calibration_file
        if path is None:
            path = getattr(args, "hand_calibration_file", None)
        if path and os.path.exists(path):
            calibration = HandCalibration.load(path)
        elif args.thumb_rotation_metric == "distance":
            calibration.channels["thumb_rot"] = {
                "open": float(args.thumb_open_distance),
                "close": float(args.thumb_close_distance),
            }
        return cls(args, calibration)

    def reset(self):
        self.smoothed = {channel: None for channel in COMMAND_ORDER}

    def _palm_frame(self, hand):
        wrist = _get_point(hand, "wrist")
        index_base = _get_point(hand, "indexMetacarpal", fallback="indexKnuckle")
        little_base = _get_point(hand, "littleMetacarpal", fallback="littleKnuckle")
        middle_base = _get_point(hand, "middleMetacarpal", fallback="middleKnuckle")

        radial = _normalize(index_base - little_base)
        palm_forward = _normalize(middle_base - wrist)
        palm_normal = _normalize(np.cross(radial, palm_forward))
        if _norm(palm_normal) < 1e-8:
            palm_normal = _normalize(np.cross(palm_forward, radial))
        return {"normal": palm_normal, "forward": palm_forward}

    def _finger_points(self, hand, finger):
        spec = FINGER_CHAINS[finger]
        return (
            _get_point(hand, spec["metacarpal"], fallback=spec["knuckle"]),
            _get_point(hand, spec["knuckle"]),
            _get_point(hand, spec["prox"]),
            _get_point(hand, spec["mid"]),
            _get_point(hand, spec["tip"]),
        )

    def _thumb_points(self, hand):
        spec = FINGER_CHAINS["thumb"]
        return (
            _get_point(hand, spec["knuckle"]),
            _get_point(hand, spec["prox"]),
            _get_point(hand, spec["mid"]),
            _get_point(hand, spec["tip"]),
        )

    def _finger_flexion_deg(self, hand, finger):
        met, knuckle, prox, mid, tip = self._finger_points(hand, finger)
        mcp = _angle_deg(knuckle - met, prox - knuckle)
        pip = _angle_deg(prox - knuckle, mid - prox)
        dip = _angle_deg(mid - prox, tip - mid)
        total = (
            self.args.finger_mcp_weight * mcp
            + self.args.finger_pip_weight * pip
            + self.args.finger_dip_weight * dip
        )
        return total, {"mcp": mcp, "pip": pip, "dip": dip}

    def _thumb_bend_deg(self, hand):
        knuckle, prox, mid, tip = self._thumb_points(hand)
        mcp_like = _angle_deg(prox - knuckle, mid - prox)
        ip_like = _angle_deg(mid - prox, tip - mid)
        total = self.args.thumb_mcp_weight * mcp_like + self.args.thumb_ip_weight * ip_like
        return total, {"mcp_like": mcp_like, "ip_like": ip_like}

    def _thumb_rotation_metric(self, hand, palm):
        if self.args.thumb_rotation_metric == "distance":
            thumb_tip = _get_point(hand, "thumbTip")
            index_knuckle = _get_point(hand, "indexKnuckle")
            return float(np.linalg.norm(thumb_tip - index_knuckle)), {"metric": "distance"}

        knuckle, prox, _, _ = self._thumb_points(hand)
        thumb_axis = _normalize(prox - knuckle)
        thumb_proj = _normalize(_project_to_plane(thumb_axis, palm["normal"]))
        ref_dir = _normalize(_project_to_plane(palm["forward"], palm["normal"]))
        signed = _signed_angle_deg(ref_dir, thumb_proj, palm["normal"])
        angle = abs(signed)
        if self.args.flip_thumb_rotation:
            angle = 180.0 - angle
        return angle, {"metric": "angle", "signed": signed}

    def measure_raw(self, hand):
        palm = self._palm_frame(hand)
        raw = {}
        parts = {}
        for finger in FINGER_ORDER:
            raw[finger], parts[finger] = self._finger_flexion_deg(hand, finger)
        raw["thumb_bend"], parts["thumb_bend"] = self._thumb_bend_deg(hand)
        raw["thumb_rot"], parts["thumb_rot"] = self._thumb_rotation_metric(hand, palm)
        return raw, parts

    def _smooth_unit(self, channel, unit):
        previous = self.smoothed[channel]
        deadband = self.args.thumb_deadband if channel.startswith("thumb") else self.args.finger_deadband
        smoothing = self.args.thumb_smoothing if channel.startswith("thumb") else self.args.finger_smoothing
        if previous is not None and abs(unit - previous) < deadband:
            unit = previous
        unit = _ema(previous, unit, smoothing)
        self.smoothed[channel] = unit
        return unit

    def _to_inspire(self, unit_value, open_angle, close_angle):
        return int(round(open_angle + _clamp01(unit_value) * (close_angle - open_angle)))

    def build_command(self, hand):
        raw, parts = self.measure_raw(hand)
        units = {}
        for channel in COMMAND_ORDER:
            unit = self.calibration.normalize(channel, raw[channel])
            scale = self.args.thumb_rotation_range_scale if channel == "thumb_rot" else (
                self.args.thumb_bend_range_scale if channel == "thumb_bend" else self.args.finger_range_scale
            )
            if channel == "thumb_rot" and self.args.invert_thumb_rotation_command:
                unit = 1.0 - unit
            units[channel] = self._smooth_unit(channel, _expand_mid(unit, scale))

        command = np.array(
            [
                self._to_inspire(units["little"], self.args.open_angle, self.args.close_angle),
                self._to_inspire(units["ring"], self.args.open_angle, self.args.close_angle),
                self._to_inspire(units["middle"], self.args.open_angle, self.args.close_angle),
                self._to_inspire(units["index"], self.args.open_angle, self.args.close_angle),
                self._to_inspire(units["thumb_bend"], self.args.thumb_bend_open_angle, self.args.thumb_bend_close_angle),
                self._to_inspire(
                    units["thumb_rot"],
                    self.args.thumb_rotation_open_angle,
                    self.args.thumb_rotation_close_angle,
                ),
            ],
            dtype=np.float64,
        )
        return command, {"raw": raw, "units": units, "parts": parts, "calibration": self.calibration.channels}


def sample_raw_metrics(streamer, mapper, seconds, side="right"):
    samples = []
    deadline = time.time() + max(float(seconds), 0.1)
    while time.time() < deadline:
        tracking = streamer.get_latest()
        hand = getattr(tracking, side, None) if tracking is not None else None
        if hand is not None:
            raw, _ = mapper.measure_raw(hand)
            samples.append(raw)
        time.sleep(1.0 / 60.0)
    return samples


def run_hand_calibration(streamer, mapper, sample_seconds, output_path, side="right"):
    if not output_path:
        raise ValueError("output_path is required for hand calibration")

    print(f"\nHand calibration will sample your AVP {side} hand.")
    print("Pose 1: fully open the hand, keep fingers naturally straight, then press Enter.")
    input()
    open_samples = sample_raw_metrics(streamer, mapper, sample_seconds, side=side)
    print(f"Captured {len(open_samples)} open-hand samples.")

    print("Pose 2: make the most useful closed grasp/fist, including thumb opposition, then press Enter.")
    input()
    close_samples = sample_raw_metrics(streamer, mapper, sample_seconds, side=side)
    print(f"Captured {len(close_samples)} closed-hand samples.")

    if not open_samples or not close_samples:
        raise RuntimeError("No AVP hand samples were captured; check Vision Pro tracking before calibrating.")

    calibration = HandCalibration()
    calibration.set_from_samples(open_samples, close_samples)
    calibration.save(output_path)
    mapper.calibration = calibration
    mapper.reset()
    print(f"Saved {side} hand calibration to {output_path}")
    return calibration


def format_debug(command, debug):
    raw = debug["raw"]
    units = debug["units"]
    return (
        f"hand_cmd={np.round(command).astype(int).tolist()} | "
        f"raw little/ring/middle/index="
        f"{[round(raw[name], 1) for name in FINGER_ORDER]} -> "
        f"{[round(units[name], 3) for name in FINGER_ORDER]} | "
        f"thumb_bend={raw['thumb_bend']:.1f}->{units['thumb_bend']:.3f} "
        f"thumb_rot={raw['thumb_rot']:.3f}->{units['thumb_rot']:.3f}"
    )
