#!/usr/bin/env python3
import rospy
import subprocess
import time

def call(name, args):
    """通过 subprocess 调用 rosservice call"""
    cmd = ["rosservice", "call", name] + args
    try:
        res = subprocess.run(cmd, check=True, capture_output=True, text=True)
        rospy.loginfo(f"{name} {' '.join(args)} → OK\n{res.stdout.strip()}")
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"{name} {' '.join(args)} → FAILED\n{e.stderr.strip()}")

if __name__ == "__main__":
    rospy.init_node("set_camera_params", anonymous=True)
    # 等待 camera node 完全上线
    rospy.loginfo("等待相机服务启动...")
    time.sleep(2.0)

    # —— 1. 曝光 设置 —— 
    call("/camera/set_color_auto_exposure",  ["{data: false}"])
    call("/camera/set_left_ir_auto_exposure", ["{data: false}"])
    call("/camera/set_color_exposure",       ["{data: 4000}"])
    call("/camera/set_left_ir_exposure",     ["{data: 4000}"])

    # —— 2. 增益 设置 —— 
    call("/camera/set_auto_color_gain",      ["{data: false}"])
    call("/camera/set_auto_left_ir_gain",    ["{data: false}"])
    call("/camera/set_color_gain",           ["{data: 200}"])
    call("/camera/set_left_ir_gain",         ["{data: 200}"])

    # —— 3. 白平衡 设置 —— 
    call("/camera/set_color_auto_white_balance", ["{data: false}"])
    call("/camera/set_color_white_balance",      ["{data: 4500}"])

    rospy.loginfo("✅ Camera 参数配置完毕，节点退出。")
    # 配置完成后自动退出
