#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityFilter(Node):
    def __init__(self):
        super().__init__('velocity_filter')
        self.get_logger().info('VelocityFilter node started.')

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel_nav',
            10)

    def clamp_velocity(self, value: float, axis_name: str = "") -> float:
        if value == 0.0:
            return 0.0
        if -0.1 < value < 0.1:
            adjusted = 0.15 * (1 if value > 0 else -1)
            return adjusted
        return value

    def cmd_vel_callback(self, msg: Twist):
        modified_msg = Twist()
        original_linear_x = msg.linear.x
        original_angular_z = msg.angular.z

        # Check for both zero — special case
        if original_linear_x == 0.0 and original_angular_z == 0.0:
            modified_msg.linear.x = 0.0
            modified_msg.angular.z = 0.2
            self.get_logger().info('Both linear.x and angular.z are 0.0 — setting angular.z to 0.2')
        # Check if linear.x == angular.z and outside [-0.1, 0.1]
        elif original_linear_x == original_angular_z and abs(original_linear_x) > 0.1:
            modified_msg.linear.x = 0.2
            modified_msg.angular.z = 0.1
            self.get_logger().info(
                f'linear.x and angular.z are equal ({original_linear_x:.3f}) and outside [-0.1, 0.1] — overriding to linear.x=0.2, angular.z=0.1'
            )
        else:
            # Clamp small values separately
            modified_msg.linear.x = self.clamp_velocity(original_linear_x, 'linear.x')
            modified_msg.angular.z = self.clamp_velocity(original_angular_z, 'angular.z')

        # Copy other components as-is
        modified_msg.linear.y = msg.linear.y
        modified_msg.linear.z = msg.linear.z
        modified_msg.angular.x = msg.angular.x
        modified_msg.angular.y = msg.angular.y

        # Log if anything changed
        if modified_msg.linear.x != original_linear_x or modified_msg.angular.z != original_angular_z:
            self.get_logger().info(
                f'Incoming cmd_vel: linear.x={original_linear_x:.3f}, angular.z={original_angular_z:.3f}'
            )

        self.publisher.publish(modified_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
