# Legacy / experimental stacks

This folder is **not part of the supported public workflow**.

## `isaac/` — Isaac Sim + xr_teleoperate

An older integration path using Unitree Isaac Lab and `xr_teleoperate` WebXR (ports 8012 / 60001). It predates the SONIC MuJoCo stack and is kept for reference only.

The main project now recommends:

- **Whole-body teleop:** `bin/sonic-*.sh` (see [docs/OPERATIONS.md](../docs/OPERATIONS.md))
- **Arm + hand only:** `bin/hand-teleop.sh`

If you still need Isaac scripts, run them from the repository root and update paths inside `legacy/isaac/bin/` — they assume an older layout.

## `command.txt`

Internal operator notes (mixed Chinese/English). Superseded by [docs/OPERATIONS.md](../docs/OPERATIONS.md).
