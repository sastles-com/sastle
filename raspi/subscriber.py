import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import os

SAVE_DIR = "received_frames"
os.makedirs(SAVE_DIR, exist_ok=True)

class FrameSubscriber(Node):
    def __init__(self):
        super().__init__('frame_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'video_frames',
            self.listener_callback,
            10)
        self.frame_count = 0
        self.get_logger().info("Subscribed to 'video_frames' topic.")

    def listener_callback(self, msg):
        print(f"Received frame: {len(msg.data)} bytes")  # ← ここでバイト数を確認
        filename = os.path.join(SAVE_DIR, f"frame_{self.frame_count:06d}.jpg")
        # with open(filename, "wb") as f:
        #     f.write(msg.data)
        self.frame_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = FrameSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()