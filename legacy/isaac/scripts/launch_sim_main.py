#!/usr/bin/env python3
"""Launch unitree sim_main.py with a user-writable teleimager config."""

import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
UNITREE_SIM_ROOT = os.environ.get(
    "UNITREE_SIM_ROOT",
    "/mnt/newssd/unitree_sim_isaaclab",
)
CAM_CONFIG = os.environ.get(
    "TELEIMAGER_CONFIG",
    os.path.join(REPO_ROOT, "config", "cam_config_server.yaml"),
)
USER_CERT_DIR = os.path.join(os.path.expanduser("~"), ".config", "xr_teleoperate")

if not os.path.isfile(CAM_CONFIG):
    raise SystemExit(f"Missing teleimager config: {CAM_CONFIG}")

os.environ.setdefault("XR_TELEOP_CERT", os.path.join(USER_CERT_DIR, "cert.pem"))
os.environ.setdefault("XR_TELEOP_KEY", os.path.join(USER_CERT_DIR, "key.pem"))

sys.path.insert(0, UNITREE_SIM_ROOT)
sys.path.insert(0, os.path.join(UNITREE_SIM_ROOT, "teleimager", "src"))


def _patch_shared_memory_manager() -> None:
    """Fix named shared-memory creation so external tools can attach to isaac_* segments."""
    from multiprocessing import shared_memory as _shm

    import dds.sharedmemorymanager as smm

    def _fixed_init(self, name=None, size=512):
        import threading

        self.size = size
        self.lock = threading.RLock()
        if name:
            try:
                self.shm = _shm.SharedMemory(name=name)
                self.shm_name = name
                self.created = False
            except FileNotFoundError:
                self.shm = _shm.SharedMemory(name=name, create=True, size=size)
                self.shm_name = name
                self.created = True
        else:
            self.shm = _shm.SharedMemory(create=True, size=size)
            self.shm_name = self.shm.name
            self.created = True

    smm.SharedMemoryManager.__init__ = _fixed_init


_patch_shared_memory_manager()


def _run_sim_main() -> None:
    """Run sim_main.py, patching whole-body reset so category 2 restores the robot."""
    path = os.path.join(UNITREE_SIM_ROOT, "sim_main.py")
    with open(path, encoding="utf-8") as fh:
        source = fh.read()

    old_block = (
        "                            if (args_cli.enable_wholebody_dds and (reset_category == '1' or reset_category == '2')) "
        "or (not args_cli.enable_wholebody_dds and reset_category == '1'):\n"
        "                                print(\"reset object\")\n"
        "                                env_cfg.event_manager.trigger(\"reset_object_self\", env)\n"
        "                                reset_pose_dds.write_reset_pose_command(-1)\n"
        "                            elif reset_category == '2' and not args_cli.enable_wholebody_dds:"
    )
    new_block = (
        "                            if reset_category == '1':\n"
        "                                print(\"reset object\")\n"
        "                                env_cfg.event_manager.trigger(\"reset_object_self\", env)\n"
        "                                reset_pose_dds.write_reset_pose_command(-1)\n"
        "                            elif reset_category == '2':"
    )
    if old_block in source:
        source = source.replace(old_block, new_block)
    else:
        print(
            "[launch_sim] WARNING: wholebody reset patch not applied "
            "(sim_main.py layout changed). Fall recovery may need sim restart.",
            file=sys.stderr,
        )

    globs = {"__name__": "__main__", "__file__": path}
    exec(compile(source, path, "exec"), globs)


import teleimager.image_server as image_server_module

image_server_module.CONFIG_PATH = CAM_CONFIG

os.chdir(UNITREE_SIM_ROOT)
sys.argv = ["sim_main.py", *sys.argv[1:]]
_run_sim_main()
