import argparse
import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
UNITREE_SIM_ROOT = os.environ.get("UNITREE_SIM_ROOT", "/mnt/newssd/unitree_sim_isaaclab")
INSPIRE_HAND_SDK_ROOT = os.environ.get(
    "INSPIRE_HAND_SDK_ROOT",
    os.path.join(UNITREE_SIM_ROOT, "inspire_hand_ws", "inspire_hand_sdk"),
)

if INSPIRE_HAND_SDK_ROOT not in sys.path:
    sys.path.insert(0, INSPIRE_HAND_SDK_ROOT)

from inspire_sdkpy import inspire_hand_defaut, inspire_sdk


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Inspire hand Modbus driver for DDS teleop commands.")
    parser.add_argument("--ip", default="192.168.123.211", help="Inspire hand Modbus IP address.")
    parser.add_argument("--lr", choices=("l", "r"), default="l", help="DDS side to subscribe to.")
    parser.add_argument("--device-id", type=int, default=1, help="Inspire hand Modbus device ID.")
    args = parser.parse_args()

    handler = inspire_sdk.ModbusDataHandler(ip=args.ip, LR=args.lr, device_id=args.device_id)
    time.sleep(0.5)

    call_count = 0  # 记录调用次数
    start_time = time.perf_counter()  # 记录开始时间

    try:
        while True:
            data_dict = handler.read()  # 读取数据

            call_count += 1  # 增加调用计数
            time.sleep(0.001)  # 暂停 5 毫秒

            # 每秒计算并打印一次调用频率
            if call_count % 10 == 0:  # 每 200 次调用计算一次频率
                elapsed_time = time.perf_counter() - start_time  # 计算总耗时
                frequency = call_count / elapsed_time  # 计算频率 (Hz)
                print(f"当前频率: {frequency:.2f} Hz, 调用次数: {call_count}, 耗时: {elapsed_time:.6f} 秒")
    except KeyboardInterrupt:
        elapsed_time = time.perf_counter() - start_time  # 计算总耗时
        frequency = call_count / elapsed_time if elapsed_time > 0 else 0  # 计算最终频率
        print(f"程序结束. 总调用次数: {call_count}, 总耗时: {elapsed_time:.6f} 秒, 最终频率: {frequency:.2f} Hz")
