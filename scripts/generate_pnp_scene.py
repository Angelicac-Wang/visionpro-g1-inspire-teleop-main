#!/usr/bin/env python3
"""Generate MuJoCo pick-up scene beside symlinked G1 model assets (mesh paths)."""

from __future__ import annotations

import os
from pathlib import Path


def _ensure_g1_asset_links(target_dir: Path, source_dir: Path) -> None:
    target_dir.mkdir(parents=True, exist_ok=True)
    for item in source_dir.iterdir():
        link = target_dir / item.name
        if link.exists() or link.is_symlink():
            continue
        link.symlink_to(item)


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    groot_root = Path(os.environ.get("GR00T_ROOT", "/mnt/newssd/GR00T-WholeBodyControl"))
    g1_src = groot_root / "gear_sonic/data/robot_model/model_data/g1"
    out_dir = repo_root / "assets/mujoco/g1_runtime"
    template = repo_root / "assets/mujoco/scene_43dof_inspire_hand_pnp_cube.xml.in"
    out_path = out_dir / "scene_43dof_inspire_hand_pnp_cube.xml"

    if not g1_src.is_dir():
        raise SystemExit(f"G1 model directory not found: {g1_src}")
    if not template.is_file():
        raise SystemExit(f"Scene template not found: {template}")

    _ensure_g1_asset_links(out_dir, g1_src)

    content = template.read_text(encoding="utf-8").replace(
        "@G1_ROBOT_INCLUDE@",
        "g1_29dof_with_inspire_hand.xml",
    )
    out_path.write_text(content, encoding="utf-8")
    print(out_path)


if __name__ == "__main__":
    main()
