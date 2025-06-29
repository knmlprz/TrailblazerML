# save as imu_cov_override.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUCovOverride(Node):
    def __init__(self):
        super().__init__('imu_cov_override')
        self.sub = self.create_subscription(Imu, '/imu/data', self.callback, 10)
        self.pub = self.create_publisher(Imu, '/imu/data_cov', 10)

    def callback(self, msg):
        msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUCovOverride()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
