import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')
        self.subscription = self.create_subscription(
            Float32,
            '/heading',  # poprawiony temat
            self.heading_callback,
            10
        )
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        self.heading_received = False

    def heading_callback(self, msg):
        if self.heading_received:
            return  # tylko raz

        heading_deg = msg.data
        yaw = math.radians(heading_deg)  # konwersja stopni -> radiany
        self.get_logger().info(f'Received heading: {heading_deg}°, converted to {yaw:.3f} rad')

        # konwersja yaw -> quaternion
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)

        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = 0.0
        pose.pose.pose.position.y = 0.0
        pose.pose.pose.position.z = 0.0
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw

        # kowariancja
        pose.pose.covariance[0] = 0.25       # x
        pose.pose.covariance[7] = 0.25       # y
        pose.pose.covariance[35] = 0.0685    # yaw

        self.publisher.publish(pose)
        self.get_logger().info('Initial pose published based on heading.')
        self.heading_received = True

def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSetter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
