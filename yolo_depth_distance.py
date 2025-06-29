#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YoloDepthVisualizer:
    def __init__(self):
        rospy.init_node("yolo_depth_visualizer", anonymous=True)
        self.bridge = CvBridge()
        self.model = YOLO(rospy.get_param("~model_path", "yolo11n.pt"))

        # 订阅相机数据
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=1)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback, queue_size=1)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback, queue_size=1)

        # 发布图像、标签、3D 点、可视化箭头
        self.image_pub = rospy.Publisher("/yolo/image", Image, queue_size=1)
        self.label_pub = rospy.Publisher("/yolo/labels", String, queue_size=1)
        self.point_pub = rospy.Publisher("/yolo/target_position", PointStamped, queue_size=10)
        self.marker_pub = rospy.Publisher("/yolo/visualization_marker", Marker, queue_size=10)

        # 相机图像缓存
        self.color_frame = None
        self.depth_frame = None
        self.fx = self.fy = self.cx = self.cy = None
        self.marker_id = 0

    def camera_info_callback(self, msg):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def depth_callback(self, msg):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(f"[深度图] 转换失败: {e}")

    def image_callback(self, msg):
        try:
            self.color_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"[彩色图] 转换失败: {e}")
            return

        if self.depth_frame is None or None in [self.fx, self.fy, self.cx, self.cy]:
            rospy.logwarn("⚠️ 等待深度图或相机内参初始化")
            return

        results = self.model(self.color_frame)
        boxes = results[0].boxes
        annotated_img = results[0].plot()
        label_texts = []

        for box in boxes:
            cls = int(box.cls)
            name = self.model.names[cls]
            conf = float(box.conf)
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            if 0 <= cy < self.depth_frame.shape[0] and 0 <= cx < self.depth_frame.shape[1]:
                depth = self.depth_frame[cy, cx] / 1000.0
                if depth > 0 and not np.isnan(depth):
                    X = (cx - self.cx) * depth / self.fx
                    Y = (cy - self.cy) * depth / self.fy
                    Z = depth

                    label = f"{name} ({conf:.2f}) at ({X:.2f}, {Y:.2f}, {Z:.2f}) m"
                    label_texts.append(label)

                    # 图像标注
                    cv2.circle(annotated_img, (cx, cy), 5, (0, 255, 255), -1)
                    cv2.putText(annotated_img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

                    # 发布坐标
                    point_msg = PointStamped()
                    point_msg.header.stamp = rospy.Time.now()
                    point_msg.header.frame_id = "camera_color_optical_frame"
                    point_msg.point.x = X
                    point_msg.point.y = Y
                    point_msg.point.z = Z
                    self.point_pub.publish(point_msg)

                    # 发布箭头 Marker
                    self.publish_marker(X, Y, Z)
                else:
                    label_texts.append(f"{name} depth invalid")
            else:
                label_texts.append(f"{name} out of bounds")

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_img, "bgr8"))
        self.label_pub.publish("\n".join(label_texts))
        rospy.loginfo_throttle(1.0, "\n".join(label_texts))

    def publish_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "camera_color_optical_frame"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "yolo_arrows"
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # 起点：相机光心
        start = Point()
        start.x = 0.0
        start.y = 0.0
        start.z = 0.0
        # 终点：目标位置
        end = Point()
        end.x = x
        end.y = y
        end.z = z
        marker.points = [start, end]

        marker.scale.x = 0.02  # shaft diameter
        marker.scale.y = 0.05  # head diameter
        marker.scale.z = 0.05  # head length

        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.1
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration(1.0)
        self.marker_pub.publish(marker)

    def run(self):
        rospy.loginfo("✅ YOLO + 深度图 + Arrow Marker 节点已启动")
        rospy.spin()

if __name__ == "__main__":
    try:
        YoloDepthVisualizer().run()
    except rospy.ROSInterruptException:
        pass
