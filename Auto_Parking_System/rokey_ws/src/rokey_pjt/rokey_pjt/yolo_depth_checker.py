# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# import os
# import sys
# import math
# from ultralytics import YOLO
# from geometry_msgs.msg import PointStamped 

# # ========================
# # 상수 정의
# # ========================
# MODEL_PATH = '/home/rokey/rokey_ws/src/rokey_pjt/rokey_pjt/yolo_detect_2/best.pt'  # YOLO 모델 경로
# RGB_TOPIC = '/robot2/oakd/rgb/preview/image_raw'
# DEPTH_TOPIC = '/robot2/oakd/stereo/image_raw'
# CAMERA_INFO_TOPIC = '/robot2/oakd/stereo/camera_info'
# TARGET_CLASS_IDS = [0, 1, 2, 3]  # YOLO에서 탐지할 클래스
# NORMALIZE_DEPTH_RANGE = 3.0  # 정규화 최대 깊이 (m)
# CONF=0.7

# class MultiSensorViewerNode(Node):
#     def __init__(self):
#         super().__init__('multi_sensor_viewer')
#         self.bridge = CvBridge()
#         self.K = None
#         self.should_shutdown = False

#         # YOLO 모델 로드
#         if not os.path.exists(MODEL_PATH):
#             self.get_logger().error(f"Model not found: {MODEL_PATH}")
#             sys.exit(1)
#         self.model = YOLO(MODEL_PATH)
#         self.class_names = getattr(self.model, 'names', [])

#         # Subscribe
#         self.depth_point_pub = self.create_publisher(PointStamped, 'depth_point', 10)
#         self.rgb_sub = self.create_subscription(Image, RGB_TOPIC, self.rgb_callback, 10)
#         self.depth_sub = self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, 10)
#         self.camera_info_sub = self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 10)

#         # 최근 depth frame
#         self.latest_depth_mm = None

#     def camera_info_callback(self, msg):
#         if self.K is None:
#             self.K = np.array(msg.k).reshape(3, 3)
#             self.get_logger().info(f"CameraInfo received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")

#     def depth_callback(self, msg):
#         self.latest_depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         # depth_colored 시각화
#         '''depth_vis = np.nan_to_num(self.latest_depth_mm, nan=0.0)
#         depth_vis = np.clip(depth_vis, 0, NORMALIZE_DEPTH_RANGE * 1000)
#         depth_vis = (depth_vis / (NORMALIZE_DEPTH_RANGE * 1000) * 255).astype(np.uint8)
#         depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

#         # 중심 십자선
#         if self.K is not None:
#             u = int(self.K[0, 2])
#             v = int(self.K[1, 2])
#             cv2.circle(depth_colored, (u, v), 5, (0, 0, 0), -1)
#             cv2.line(depth_colored, (0, v), (depth_colored.shape[1], v), (0, 0, 0), 1)
#             cv2.line(depth_colored, (u, 0), (u, depth_colored.shape[0]), (0, 0, 0), 1)

#         cv2.imshow("Depth Image", depth_colored)
#         '''
#     def rgb_callback(self, msg):
#         img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         display_img = img.copy()

#         object_count = 0
#         results = self.model(img, stream=True)

#         for r in results:
#             for box in r.boxes:
#                 cls = int(box.cls[0])
#                 if cls not in TARGET_CLASS_IDS:
#                     continue
                
#                 x1, y1, x2, y2 = map(int, box.xyxy[0])
#                 conf = math.ceil(box.conf[0] * 100) / 100
#                 label = self.class_names[cls] if cls < len(self.class_names) else f"class_{cls}"

#                 # 중심 좌표
#                 u = (x1 + x2) // 2
#                 v = (y1 + y2) // 2

#                 # 거리 정보 가져오기
#                 if self.K is not None and self.latest_depth_mm is not None:
#                     h, w = self.latest_depth_mm.shape
#                     if 0 <= v < h and 0 <= u < w:
#                         distance_m = self.latest_depth_mm[v, u] / 1000.0
#                         label += f" ({distance_m:.2f}m)"
#                         # fx, fy, cx, cy를 이용해 3D 좌표계 변환
#                         fx, fy = self.K[0, 0], self.K[1, 1]
#                         cx, cy = self.K[0, 2], self.K[1, 2]
#                         x = (u - cx) * distance_m / fx
#                         y = (v - cy) * distance_m / fy
#                         z = distance_m

#                         # PointStamped 메시지 생성 및 publish
#                         point_msg = PointStamped()
#                         point_msg.header.stamp = self.get_clock().now().to_msg()
#                         point_msg.header.frame_id = msg.header.frame_id  # 예: 'oakd_rgb_optical_frame'
#                         point_msg.point.x = x
#                         point_msg.point.y = y
#                         point_msg.point.z = z

#                 if conf> CONF:
#                     # 박스 + 라벨 시각화
#                     cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
#                     if cls != TARGET_CLASS_IDS[3]:
#                         cv2.putText(display_img, f"{label}: {conf}", (x1, y1 - 10),
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
#                     object_count += 1
#                     self.depth_point_pub.publish(point_msg)

#         # 탐지 수 출력
#         cv2.putText(display_img, f"Objects: {object_count}", (10, 30),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

#         # 출력
#         cv2.imshow("YOLO + Depth Viewer", cv2.resize(display_img, (img.shape[1]*2, img.shape[0]*2)))
#         key = cv2.waitKey(1)
#         if key == ord('q'):
#             self.should_shutdown = True
#             self.get_logger().info("Q pressed. Shutting down...")

# # ========================
# # 메인 함수
# # ========================
# def main():
#     rclpy.init()
#     node = MultiSensorViewerNode()

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

# if __name__ == '__main__depe':
#     main()


# ros2 run yolov8_ros resource_depth --ros-args -p use_compressed:=true

#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import time
import psutil
import csv
import threading
import numpy as np

MODEL_PATH = "/home/rokey/rokey_ws/best_320.pt"
if not os.path.exists(MODEL_PATH):
    raise FileNotFoundError(f"[ERROR] 모델 경로가 존재하지 않습니다: {MODEL_PATH}")

# 로그 파일 이름 중복 방지 함수
def get_unique_filename(base_name):
    """파일이 존재하면 _1, _2, ... 숫자 붙여서 고유 파일명 반환"""
    if not os.path.exists(base_name):
        return base_name
    name, ext = os.path.splitext(base_name)
    i = 1
    while True:
        new_name = f"{name}_{i}{ext}"
        if not os.path.exists(new_name):
            return new_name
        i += 1

class YoloDepthNode(Node):
    def __init__(self):
        super().__init__('yolo_depth_node')

        self.declare_parameter('use_compressed', False)
        self.use_compressed = self.get_parameter('use_compressed').get_parameter_value().bool_value
        self.get_logger().info(f"[설정] 압축 이미지 사용 여부: {self.use_compressed}")

        self.bridge = CvBridge()
        self.model = YOLO(MODEL_PATH)

        if self.use_compressed:
            self.create_subscription(CompressedImage, '/robot2/oakd/rgb/image_raw/compressed', self.rgb_compressed_callback, 10)
        else:
            self.create_subscription(Image, '/robot2/oakd/rgb/image_raw', self.rgb_callback, 10)

        self.pub_image = self.create_publisher(Image, '/yolo/detected_depth_image', 10)
        self.pub_objects = self.create_publisher(String, '/yolo/object_info', 10)

        self.rgb_image = None
        self.prev_time = time.time()
        self.prev_net = psutil.net_io_counters()
        self.last_log_time = time.time()
        psutil.cpu_percent()

        base_csv_name = "yolo_depth_compressed_log.csv" if self.use_compressed else "yolo_depth_raw_log.csv"
        csv_name = get_unique_filename(base_csv_name)
        self.csv_file = open(csv_name, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "timestamp", "FPS", "CPU%", "Mem%", "TX_MB", "RX_MB",
            "detected", "detected_count", "inference_time(ms)", "avg_confidence"
        ])
        self.get_logger().info(f"[LOG] CSV 로그 파일 생성됨: {csv_name}")

        self.fps_list = []
        self.cpu_list = []
        self.mem_list = []
        self.tx_list = []
        self.rx_list = []
        self.infer_time_list = []

        self.get_logger().info(f"YOLO 모델 로드 완료: {MODEL_PATH}")
        self._stop = False
        threading.Thread(target=self.input_thread_func, daemon=True).start()
        self.timer = self.create_timer(0.1, self.process)

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"RGB 이미지 변환 실패: {e}")

    def rgb_compressed_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"압축 이미지 디코딩 실패: {e}")

    def process(self):
        if self._stop:
            self.destroy_node()
            rclpy.shutdown()
            return

        if self.rgb_image is None:
            return

        now = time.time()
        dt = now - self.prev_time
        fps = 1.0 / dt if dt > 0 else 0.0
        self.prev_time = now

        start_infer = time.time()
        results = self.model(self.rgb_image, verbose=False, conf=0.6)
        infer_time_ms = (time.time() - start_infer) * 1000

        annotated_frame = results[0].plot()
        boxes = results[0].boxes.xyxy.cpu().numpy()
        confidences = results[0].boxes.conf.cpu().numpy()

        detected_count = len(boxes)
        detected = "YES" if detected_count > 0 else "NO"
        avg_conf = float(np.mean(confidences)) if detected_count > 0 else 0.0

        if now - self.last_log_time >= 5.0:
            self.last_log_time = now

            cpu = psutil.cpu_percent()
            mem = psutil.virtual_memory().percent
            net_now = psutil.net_io_counters()
            tx = (net_now.bytes_sent - self.prev_net.bytes_sent) / (1024 * 1024)
            rx = (net_now.bytes_recv - self.prev_net.bytes_recv) / (1024 * 1024)
            self.prev_net = net_now

            timestamp = time.strftime("%H:%M:%S")
            self.csv_writer.writerow([
                timestamp, f"{fps:.2f}", cpu, mem, f"{tx:.2f}", f"{rx:.2f}",
                detected, detected_count, f"{infer_time_ms:.2f}", f"{avg_conf:.2f}"
            ])

            self.get_logger().info(
                f"[{timestamp}] FPS: {fps:.2f} | CPU: {cpu}% | MEM: {mem}% | "
                f"TX: {tx:.2f}MB | RX: {rx:.2f}MB | Detected: {detected} ({detected_count}) | "
                f"Inference: {infer_time_ms:.2f}ms | Conf: {avg_conf:.2f}"
            )

            self.fps_list.append(fps)
            self.cpu_list.append(cpu)
            self.mem_list.append(mem)
            self.tx_list.append(tx)
            self.rx_list.append(rx)
            self.infer_time_list.append(infer_time_ms)

        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.pub_image.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"이미지 퍼블리시 실패: {e}")

    def input_thread_func(self):
        while not self._stop:
            key = input()
            if key.strip() == '0':
                self.get_logger().info("0 입력됨. 노드 종료합니다...")
                self._stop = True

    def destroy_node(self):
        if self.fps_list:
            avg_fps = sum(self.fps_list) / len(self.fps_list)
            avg_cpu = sum(self.cpu_list) / len(self.cpu_list)
            avg_mem = sum(self.mem_list) / len(self.mem_list)
            avg_tx = sum(self.tx_list) / len(self.tx_list)
            avg_rx = sum(self.rx_list) / len(self.rx_list)
            avg_infer = sum(self.infer_time_list) / len(self.infer_time_list) if self.infer_time_list else 0.0

            self.csv_writer.writerow([])
            self.csv_writer.writerow([
                "AVERAGE", f"{avg_fps:.2f}", f"{avg_cpu:.2f}",
                f"{avg_mem:.2f}", f"{avg_tx:.2f}", f"{avg_rx:.2f}", 
                f"{avg_infer:.2f}", "" 
            ])
            self.get_logger().info(
                f"종료 - 평균값 기록됨: FPS {avg_fps:.2f}, CPU {avg_cpu:.2f}%, MEM {avg_mem:.2f}%, "
                f"TX {avg_tx:.2f}MB, RX {avg_rx:.2f}MB, Inference {avg_infer:.2f}ms"
            )

        self.csv_file.close()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

