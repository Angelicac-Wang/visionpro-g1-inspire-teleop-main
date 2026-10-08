import sys, time, threading
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import gear_sonic.utils.mujoco_sim.base_sim as base_sim
from gear_sonic.utils.mujoco_sim.unitree_sdk2py_bridge import UnitreeSdk2Bridge
st = {"steps": 0, "cmds": 0, "gap": 0.0, "last": None}
lock = threading.Lock()
_step = base_sim.DefaultEnv.sim_step
def sim_step(self):
    st["steps"] += 1
    return _step(self)
base_sim.DefaultEnv.sim_step = sim_step
_h = UnitreeSdk2Bridge.LowCmdHandler
def LowCmdHandler(self, msg):
    now = time.monotonic()
    with lock:
        if st["last"] is not None:
            st["gap"] = max(st["gap"], now - st["last"])
        st["last"] = now
        st["cmds"] += 1
    return _h(self, msg)
UnitreeSdk2Bridge.LowCmdHandler = LowCmdHandler
def report():
    while True:
        time.sleep(2)
        with lock:
            s, c, g = st["steps"], st["cmds"], st["gap"]
            st["steps"] = st["cmds"] = 0; st["gap"] = 0.0
        print(f"[probe] sim {s/2:.0f} steps/s (target 200) | LowCmd {c/2:.0f}/s | max cmd gap {g*1000:.0f} ms", flush=True)
threading.Thread(target=report, daemon=True).start()
import sonic_run_sim_loop
sonic_run_sim_loop.main()
