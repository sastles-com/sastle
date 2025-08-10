import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Imu
import redis
import base64
import json
import time

class BridgeNode(Node):
    def __init__(self):
        super().__init__('web_frame_bridge')
        self.pub = self.create_publisher(CompressedImage, 'video_frames', 10)
        self.sub = self.create_subscription(Imu, 'm5atom/imu', self.imu_callback, 10)
        self.redis = redis.Redis(host='localhost', port=6379, db=0)
        self.timer = self.create_timer(0.1, self.check_and_publish_frame)
    
    def check_and_publish_frame(self):
        img_b64 = self.redis.get("video_frame")
        if img_b64:
            img_b64 = img_b64.decode()
            img_bytes = base64.b64decode(img_b64.split(",")[1])
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = img_bytes
            self.pub.publish(msg)
            self.redis.delete("video_frame")
        # UIコマンドもここで取得して処理可能
        cmd_json = self.redis.get("ui_command")
        if cmd_json:
            cmd = json.loads(cmd_json)
            # コマンド処理
            self.redis.delete("ui_command")

    def imu_callback(self, msg: Imu):
        # クォータニオン値をRedisに保存
        quat = {
            "x": msg.orientation.x,
            "y": msg.orientation.y,
            "z": msg.orientation.z,
            "w": msg.orientation.w
        }
        self.redis.set("imu_quat", json.dumps(quat))

def main():
    rclpy.init()
    node = BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()