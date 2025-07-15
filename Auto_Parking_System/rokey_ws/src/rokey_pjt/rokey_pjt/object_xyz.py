# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo
# from geometry_msgs.msg import PointStamped
# from cv_bridge import CvBridge
# import tf2_ros
# import tf2_geometry_msgs
# import cv2
# import numpy as np
# import math
# import os
# import sys
# from ultralytics import YOLO

# # ========================
# # 상수 정의
# # ========================
# MODEL_PATH = '/home/rokey/best.pt'
# RGB_TOPIC = '/robot2/oakd/rgb/preview/image_raw'
# DEPTH_TOPIC = '/robot2/oakd/stereo/image_raw'
# CAMERA_INFO_TOPIC = '/robot2/oakd/stereo/camera_info'
# TARGET_CLASS_ID = 0
# NORMALIZE_DEPTH_RANGE = 3.0
# CONF = 0.7

# class CombinedViewerNode(Node):
#     def __init__(self):
#         super().__init__('combined_viewer_node')
#         self.bridge = CvBridge()
#         self.K = None
#         self.should_shutdown = False

#         # Load YOLO
#         if not os.path.exists(MODEL_PATH):
#             self.get_logger().error(f"Model not found: {MODEL_PATH}")
#             sys.exit(1)
#         self.model = YOLO(MODEL_PATH)
#         self.class_names = getattr(self.model, 'names', [])

#         # Subscribers
#         self.rgb_sub = self.create_subscription(Image, RGB_TOPIC, self.rgb_callback, 10)
#         self.depth_sub = self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, 10)
#         self.camera_info_sub = self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 10)

#         # Depth storage
#         self.latest_depth_mm = None

#         # TF listener setup
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
#         self.get_logger().info("Waiting 5s for TF to initialize...")
#         self.start_timer = self.create_timer(5.0, self.start_tf_timer)

#     def start_tf_timer(self):
#         self.get_logger().info("TF initialized. Starting periodic TF transform.")
#         self.tf_timer = self.create_timer(2.0, self.transform_point)
#         self.start_timer.cancel()

#     def camera_info_callback(self, msg):
#         if self.K is None:
#             self.K = np.array(msg.k).reshape(3, 3)
#             self.get_logger().info(f"CameraInfo received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")

#     def depth_callback(self, msg):
#         self.latest_depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         '''depth_vis = np.nan_to_num(self.latest_depth_mm, nan=0.0)
#         depth_vis = np.clip(depth_vis, 0, NORMALIZE_DEPTH_RANGE * 1000)
#         depth_vis = (depth_vis / (NORMALIZE_DEPTH_RANGE * 1000) * 255).astype(np.uint8)
#         depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

#         if self.K is not None:
#             u = int(self.K[0, 2])
#             v = int(self.K[1, 2])
#             cv2.circle(depth_colored, (u, v), 5, (0, 0, 0), -1)
#             cv2.line(depth_colored, (0, v), (depth_colored.shape[1], v), (0, 0, 0), 1)
#             cv2.line(depth_colored, (u, 0), (u, depth_colored.shape[0]), (0, 0, 0), 1)

#         cv2.imshow("Depth Image", depth_colored)'''

#     def rgb_callback(self, msg):
#         img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         display_img = img.copy()
#         object_count = 0
#         results = self.model(img, stream=True)

#         for r in results:
#             for box in r.boxes:
#                 cls = int(box.cls[0])
#                 if cls != TARGET_CLASS_ID:
#                     continue
#                 x1, y1, x2, y2 = map(int, box.xyxy[0])
#                 conf = math.ceil(box.conf[0] * 100) / 100
#                 label = self.class_names[cls] if cls < len(self.class_names) else f"class_{cls}"

#                 u = (x1 + x2) // 2
#                 v = (y1 + y2) // 2

#                 if self.K is not None and self.latest_depth_mm is not None:
#                     h, w = self.latest_depth_mm.shape
#                     if 0 <= v < h and 0 <= u < w:
#                         distance_m = self.latest_depth_mm[v, u] / 1000.0  # mm → m

#                         # 픽셀 좌표 (u, v) + depth → 카메라 좌표 (x, y, z)
#                         fx = self.K[0, 0]
#                         fy = self.K[1, 1]
#                         cx = self.K[0, 2]
#                         cy = self.K[1, 2]

#                         x = (u - cx) * distance_m / fx
#                         y = (v - cy) * distance_m / fy
#                         z = distance_m

#                         label += f" ({distance_m:.2f}m)"

#                         # 카메라 좌표계의 PointStamped 생성
#                         camera_point = PointStamped()
#                         camera_point.header.stamp = msg.header.stamp
#                         camera_point.header.frame_id = "camera_link"  # ⚠ 실제 프레임 이름으로 바꿔야 함
#                         camera_point.point.x = x
#                         camera_point.point.y = y
#                         camera_point.point.z = z

#                         try:
#                             map_point = self.tf_buffer.transform(
#                                 camera_point,
#                                 'map',
#                                 timeout=rclpy.duration.Duration(seconds=0.5)
#                             )
#                             self.get_logger().info(f"[Map absolute pos] x={map_point.point.x:.2f}, y={map_point.point.y:.2f}, z={map_point.point.z:.2f}")
#                         except Exception as e:
#                             self.get_logger().warn(f"TF transform to map failed: {e}")

#                 if conf > CONF:
#                     cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
#                     cv2.putText(display_img, f"{label}: {conf}", (x1, y1 - 10),
#                                 cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
#                     object_count += 1

#         cv2.putText(display_img, f"Objects: {object_count}", (10, 30),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
#         cv2.imshow("YOLO + Depth Viewer", cv2.resize(display_img, (img.shape[1]*2, img.shape[0]*2)))
#         key = cv2.waitKey(1)
#         if key == ord('q'):
#             self.should_shutdown = True
#             self.get_logger().info("Q pressed. Shutting down...")


#     def transform_point(self):
#         try:
#             point_base = PointStamped()
#             point_base.header.stamp = rclpy.time.Time().to_msg()
#             point_base.header.frame_id = 'base_link'
#             point_base.point.x = 5.0
#             point_base.point.y = 0.0
#             point_base.point.z = 0.0

#             point_map = self.tf_buffer.transform(
#                 point_base,
#                 'map',
#                 timeout=rclpy.duration.Duration(seconds=0.5)
#             )

#             self.get_logger().info(f"[Base_link] ({point_base.point.x:.2f}, {point_base.point.y:.2f}, {point_base.point.z:.2f})")
#             self.get_logger().info(f"[Map]       ({point_map.point.x:.2f}, {point_map.point.y:.2f}, {point_map.point.z:.2f})")

#         except Exception as e:
#             self.get_logger().warn(f"TF transform to map failed: {e}")

# # ========================
# # 메인 함수
# # ========================
# def main():
#     rclpy.init()
#     node = CombinedViewerNode()
#     try:
#         while rclpy.ok() and not node.should_shutdown:
#             rclpy.spin_once(node, timeout_sec=0.1)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()
#         print("Shutdown complete.")
#         sys.exit(0)

# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3
# combined_yolo_tf_point.py
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
import cv2
import numpy as np
import math
import os
import sys
from ultralytics import YOLO

# ========================
# 상수 정의
# ========================
MODEL_PATH = '/home/seoyoon/Disabled_best.pt'
RGB_TOPIC = '/robot2/oakd/rgb/preview/image_raw'
DEPTH_TOPIC = '/robot2/oakd/stereo/image_raw'
CAMERA_INFO_TOPIC = '/robot2/oakd/stereo/camera_info'
TARGET_CLASS_ID = 0
NORMALIZE_DEPTH_RANGE = 3.0
CONF = 0.7

class CombinedViewerNode(Node):
    def __init__(self):
        super().__init__('combined_viewer_node')
        self.bridge = CvBridge()
        self.K = None
        self.should_shutdown = False

        # Load YOLO
        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f"Model not found: {MODEL_PATH}")
            sys.exit(1)
        self.model = YOLO(MODEL_PATH)
        self.class_names = getattr(self.model, 'names', [])

        # Subscribers
        self.rgb_sub = self.create_subscription(Image, RGB_TOPIC, self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 10)

        # Depth storage
        self.latest_depth_mm = None

        # TF listener setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        #self.get_logger().info("Waiting 5s for TF to initialize...")
        #self.start_timer = self.create_timer(5.0, self.start_tf_timer)
    
    def start_tf_timer(self):
        self.get_logger().info("TF initialized. Starting periodic TF transform.")
        self.tf_timer = self.create_timer(2.0, self.transform_point)
        self.start_timer.cancel()
    
    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f"CameraInfo received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")
    
    def depth_callback(self, msg):
        self.latest_depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_vis = np.nan_to_num(self.latest_depth_mm, nan=0.0)
        depth_vis = np.clip(depth_vis, 0, NORMALIZE_DEPTH_RANGE * 1000)
        depth_vis = (depth_vis / (NORMALIZE_DEPTH_RANGE * 1000) * 255).astype(np.uint8)
        depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

        if self.K is not None:
            u = int(self.K[0, 2])
            v = int(self.K[1, 2])
            cv2.circle(depth_colored, (u, v), 5, (0, 0, 0), -1)
            cv2.line(depth_colored, (0, v), (depth_colored.shape[1], v), (0, 0, 0), 1)
            cv2.line(depth_colored, (u, 0), (u, depth_colored.shape[0]), (0, 0, 0), 1)

        cv2.imshow("Depth Image", depth_colored)

    def rgb_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        display_img = img.copy()
        object_count = 0
        results = self.model(img, stream=True)

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                if cls != TARGET_CLASS_ID:
                    continue
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = math.ceil(box.conf[0] * 100) / 100
                label = self.class_names[cls] if cls < len(self.class_names) else f"class_{cls}"

                u = (x1 + x2) // 2
                v = (y1 + y2) // 2

                if self.K is not None and self.latest_depth_mm is not None:
                    h, w = self.latest_depth_mm.shape
                    if 0 <= v < h and 0 <= u < w:
                        distance_m = self.latest_depth_mm[v, u] / 1000.0
                        label += f" ({distance_m:.2f}m)"

                if conf > CONF:
                    cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(display_img, f"{label}: {conf}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                    cv2.circle(display_img, (u, v), 5, (0, 255, 255), -1)  # 노란 점으로 중심점 표시
                    self.transform_point()
                    object_count += 1

        cv2.putText(display_img, f"Objects: {object_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("YOLO + Depth Viewer", cv2.resize(display_img, (img.shape[1]*2, img.shape[0]*2)))
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.should_shutdown = True
            self.get_logger().info("Q pressed. Shutting down...")

    def transform_point(self):
        try:
            point_base = PointStamped()
            point_base.header.stamp = rclpy.time.Time().to_msg()
            point_base.header.frame_id = 'base_link'
            point_base.point.x = 1.0
            point_base.point.y = 0.0
            point_base.point.z = 0.0

            point_map = self.tf_buffer.transform(
                point_base,
                'map',
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            self.get_logger().info(f"[Base_link] ({point_base.point.x:.2f}, {point_base.point.y:.2f}, {point_base.point.z:.2f})")
            self.get_logger().info(f"[Map]       ({point_map.point.x:.2f}, {point_map.point.y:.2f}, {point_map.point.z:.2f})")

        except Exception as e:
            self.get_logger().warn(f"TF transform to map failed: {e}")

# ========================
# 메인 함수
# ========================
def main():
    rclpy.init()
    node = CombinedViewerNode()
    try:
        while rclpy.ok() and not node.should_shutdown:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("Shutdown complete.")
        sys.exit(0)

if __name__ == '__main__':
    main()
'''

#!/usr/bin/env python3
# yolo_depth_map_node.py (통합 버전)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import cv2
import math
import os
import sys
import time
import torch
from ultralytics import YOLO
import threading

# ========================
# 설정 상수
# ========================
MODEL_PATH = '/home/rokey/best.pt'
RGB_TOPIC = '/robot2/oakd/rgb/preview/image_raw'
DEPTH_TOPIC = '/robot2/oakd/stereo/image_raw'
CAMERA_INFO_TOPIC = '/robot2/oakd/stereo/camera_info'
TARGET_CLASS_ID = 0
CONFIDENCE_THRESHOLD = 0.7

class YoloDepthMapNode(Node):
    def __init__(self):
        super().__init__('yolo_depth_map_node')
        self.bridge = CvBridge()
        self.K = None
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_rgb_msg = None
        self.should_shutdown = False
        self.lock = threading.Lock()
        self.marker_id = 0

        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f"모델 경로가 존재하지 않습니다: {MODEL_PATH}")
            sys.exit(1)

        self.model = YOLO(MODEL_PATH)
        self.model.to('cuda' if torch.cuda.is_available() else 'cpu')
        self.class_names = getattr(self.model, 'names', [])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.marker_pub = self.create_publisher(Marker, 'detected_objects_marker', 10)

        self.create_subscription(Image, RGB_TOPIC, self.rgb_callback, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, 10)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 10)

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f"CameraInfo 수신 완료: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"[Depth Error] {e}")

    def rgb_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.latest_rgb = img
                self.latest_rgb_msg = msg
        except Exception as e:
            self.get_logger().error(f"[RGB Error] {e}")

    def transform_to_map(self, pt_camera, label):
        try:
            pt_map = self.tf_buffer.transform(pt_camera, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
            x, y, z = pt_map.point.x, pt_map.point.y, pt_map.point.z
            self.get_logger().info(f"[TF] {label} 위치 (map): x={x:.2f}, y={y:.2f}, z={z:.2f}")
            return x, y, z
        except Exception as e:
            self.get_logger().warn(f"[TF 실패] {label}: {e}")
            return float('nan'), float('nan'), float('nan')

    def publish_marker(self, x, y, z, class_id):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'detected_objects'
        marker.id = self.marker_id  # 클래스 ID를 마커 ID로 사용
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 3
        self.marker_pub.publish(marker)

    def inference_loop(self):
        self.get_logger().info("[YOLO 추론 시작]")
        while rclpy.ok() and not self.should_shutdown:
            with self.lock:
                rgb = self.latest_rgb
                depth = self.latest_depth
                K = self.K
                rgb_msg = self.latest_rgb_msg

            if any(v is None for v in [rgb, depth, K, rgb_msg]):
                time.sleep(0.05)
                continue

            frame = rgb.copy()
            results = self.model(frame)

            for result in results:
                for box in result.boxes:
                    cls = int(box.cls[0])
                    if cls != TARGET_CLASS_ID:
                        continue

                    x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                    conf = float(box.conf[0])
                    if conf < CONFIDENCE_THRESHOLD:
                        continue

                    u = (x1 + x2) // 2
                    v = (y1 + y2) // 2
                    if not (0 <= v < depth.shape[0] and 0 <= u < depth.shape[1]):
                        continue

                    z = float(depth[v, u]) / 1000.0
                    fx, fy = K[0, 0], K[1, 1]
                    cx, cy = K[0, 2], K[1, 2]
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy

                    label = self.class_names[cls] if cls < len(self.class_names) else f'class_{cls}'
                    pt_camera = PointStamped()
                    pt_camera.header.frame_id = rgb_msg.header.frame_id
                    pt_camera.header.stamp = rclpy.time.Time().to_msg()
                    pt_camera.point.x = x
                    pt_camera.point.y = y
                    pt_camera.point.z = z

                    map_x, map_y, map_z = self.transform_to_map(pt_camera, label)
                    if not np.isnan(map_x):
                        self.publish_marker(map_x, map_y, map_z, cls)

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, (u, v), 4, (0, 0, 255), -1)
                    cv2.putText(frame, f"{label} {conf:.2f} {z:.2f}m", (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            cv2.imshow("YOLO + Depth + Map", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.should_shutdown = True
            time.sleep(0.05)

def main():
    rclpy.init()
    node = YoloDepthMapNode()
    try:
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
        threading.Thread(target=node.inference_loop, daemon=True).start()

        while rclpy.ok() and not node.should_shutdown:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("Shutdown complete.")
        sys.exit(0)

if __name__ == '__main__':
    main()
