#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2

class LaptopCamPublisher(Node):
    def __init__(self):
        super().__init__('laptop_cam_publisher')

        self.publisher_ = self.create_publisher(
            CompressedImage, '/camera/image_raw/compressed', 10)

        # 카메라 열기 (노트북 기본 카메라: index 0)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FPS, 25)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.cap.set(cv2.CAP_PROP_EXPOSURE, -6) 

        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"Laptop camera resolution: {width} x {height}")

        if not self.cap.isOpened():
            self.get_logger().error("노트북 카메라 열기 실패!")
            exit(1)
        else:
            self.get_logger().info("노트북 카메라 오픈 성공")

        # 타이머 주기 (0.2초 → 5fps 정도)
        self.timer = self.create_timer(0.2, self.publish_image)

    def publish_image(self):
        ret, frame = self.cap.read()

        if ret:
            # 좌우 반전 (좌우반전 적용: 원본처럼 보여주기 위함)
            # frame = cv2.flip(frame, 1)

            # 이미지 크기 로그 출력
            height, width, channels = frame.shape
            self.get_logger().info(f"Captured frame size: {width} x {height}")

            # JPEG 압축
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
            _, compressed_image = cv2.imencode('.jpg', frame, encode_param)

            # CompressedImage ROS 메시지 생성
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "laptop_camera"
            msg.format = "jpeg"
            msg.data = compressed_image.tobytes()

            # 퍼블리시
            self.publisher_.publish(msg)
            self.get_logger().info('Published laptop /camera/image_raw/compressed')
        else:
            self.get_logger().warning('노트북 카메라 프레임 읽기 실패')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LaptopCamPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# # 로봇 카메라 이미지 발행
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# import cv2
# import time

# class CameraImagePublisher(Node):
#     def __init__(self):
#         super().__init__('camera_image_publisher')

#         # 토픽 이름 (autorace system 기준)
#         self.publisher_ = self.create_publisher(
#             CompressedImage, '/camera/image_raw/compressed', 10)

#         self.device_path = '/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0'
#         self.open_camera()

#         # 0.2초마다 이미지 퍼블리시 (약 5fps)
#         self.timer = self.create_timer(0.2, self.publish_image)

#         self.failed_count = 0  # 연속 실패 카운터

#     def open_camera(self):
#         self.cap = cv2.VideoCapture(self.device_path)

#         self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
#         self.cap.set(cv2.CAP_PROP_FPS, 25)
#         self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
#         self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

#         time.sleep(0.5)  # 카메라 open 후 잠시 대기

#         if not self.cap.isOpened():
#             self.get_logger().error("카메라 열기 실패!")
#         else:
#             width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
#             height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
#             self.get_logger().info(f"카메라 오픈 성공: {width} x {height}")

#     def publish_image(self):
#         if not self.cap.isOpened():
#             self.get_logger().warning("카메라 장치가 닫혀있음, 재시도")
#             self.open_camera()
#             return

#         ret, frame = self.cap.read()

#         if ret:
#             self.failed_count = 0  # 성공시 실패카운터 초기화

#             height, width, channels = frame.shape
#             self.get_logger().info(f'프레임 크기: {width}x{height}, 채널: {channels}')

#             encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
#             _, compressed_image = cv2.imencode('.jpg', frame, encode_param)

#             msg = CompressedImage()
#             msg.header.stamp = self.get_clock().now().to_msg()
#             msg.header.frame_id = "camera"
#             msg.format = "jpeg"
#             msg.data = compressed_image.tobytes()

#             self.publisher_.publish(msg)
#             self.get_logger().info('Published /camera/image_raw/compressed')
#         else:
#             self.failed_count += 1
#             self.get_logger().warning(f'카메라 프레임 읽기 실패 (누적 {self.failed_count})')

#             if self.failed_count >= 5:
#                 self.get_logger().warning("연속 실패로 카메라 재연결 시도")
#                 self.cap.release()
#                 time.sleep(0.5)
#                 self.open_camera()


#     def destroy_node(self):
#         if self.cap.isOpened():
#             self.cap.release()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = CameraImagePublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
