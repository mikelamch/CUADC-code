#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import torch
import os
import numpy as np
from datetime import datetime

class YoloPublisher:
    def __init__(self):
        self.bridge = CvBridge()
        self.use_ros_topic = rospy.get_param("~use_ros_topic", True)
        self.model_path = rospy.get_param("~model_path", os.path.expanduser(
            "~/cuadc_ws/src/yolo_ros/scripts/models/best_20250613_151329.pt"))

        # 设置设备
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        rospy.loginfo(f"✅ 当前使用设备: {self.device}")

        # 加载模型
        self.model = YOLO(self.model_path)

        # Warm-up（用于加速后续推理）
        dummy_input = np.zeros((320, 320, 3), dtype=np.uint8)
        self.model.predict(dummy_input, device=self.device, verbose=False)

        self.image_pub = rospy.Publisher("/yolo/image", Image, queue_size=1)
        self.label_pub = rospy.Publisher("/yolo/labels", String, queue_size=1)

        if self.use_ros_topic:
            rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
            rospy.loginfo("✅ 使用 ROS 图像话题 /camera/color/image_raw 进行检测")
        else:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                rospy.logerr("❌ 无法打开物理摄像头 /dev/video0")
                exit(1)
            rospy.loginfo("✅ 使用物理摄像头进行检测")
            rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def timer_callback(self, event):
        ret, frame = self.cap.read()
        if not ret:
            rospy.logwarn("⚠️ 获取摄像头帧失败")
            return
        self.run_detection(frame)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"图像转换失败: {e}")
            return
        self.run_detection(frame)

    def run_detection(self, frame):
        results = self.model.predict(frame, stream=True, imgsz=320, device=self.device)
        label_info = []

        for result in results:
            boxes = result.boxes
            annotated_frame = result.plot()

            for box in boxes:
                cls = int(box.cls)
                name = self.model.names[cls]
                conf = float(box.conf)
                label_info.append(f"{name}({conf:.2f})")

            # 发布图像（RViz）
            img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.image_pub.publish(img_msg)

            # 发布标签
            self.label_pub.publish(", ".join(label_info))
            rospy.loginfo(f"检测到: {label_info}")

def main():
    rospy.init_node("yolo_publisher", anonymous=True)
    node = YoloPublisher()
    rospy.spin()

if __name__ == "__main__":
    main()
