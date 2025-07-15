#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration
import time
import threading

class Rotate180(Node):
    def __init__(self):
        super().__init__('rotate_180_node')

        # 퍼블리셔 생성
        self.cmd_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        self.audio_pub = self.create_publisher(AudioNoteVector, '/robot2/cmd_audio', 10)

    def play_audio(self, mode="here"):
        if mode == "here":
            notes = [
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=100, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=100, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=100, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=100, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=100, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=100, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=100, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=100, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=100, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
                
            ]
        elif mode == "here2":
            notes = [
                AudioNote(frequency=660, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=990, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=1320, max_runtime=Duration(nanosec=300_000_000)),
            ]
        else:
            return

        msg = AudioNoteVector()
        msg.append = False
        msg.notes = notes
        self.audio_pub.publish(msg)
        self.get_logger().info(f'🔊 Audio "{mode}" published')

        # ✅ 메시지 전송 보장용 짧은 대기 (사운드 재생 안 되면 0.3 ~ 0.5로 조절)
        time.sleep(0.3)

    def rotate(self):
        twist = Twist()
        twist.angular.z = 0.5
        duration = 3.141592 / 0.5  # 180도 회전 시간

        self.get_logger().info(f"↩️ 제자리 180도 회전 시작 (예상 {duration:.2f}초)")
        time.sleep(1.0)
        # 🔊 회전 시작 알림음 (동기)
        # self.play_audio("here")
        # 🔊 회전 시작 알림음 (비동기)
        threading.Thread(target=self.play_audio, args=("here",), daemon=True).start()
        time.sleep(1.0)
        # 🌀 회전
        start_time = self.get_clock().now().seconds_nanoseconds()[0]

        while self.get_clock().now().seconds_nanoseconds()[0] - start_time < duration:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)

        # ⛔ 정지
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info("✅ 회전 완료, 로봇 정지!")
        time.sleep(1.0)
        # 🔊 완료 사운드
        self.play_audio("here2")

def main():
    rclpy.init()
    node = Rotate180()
    node.rotate()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
