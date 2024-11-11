#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class TeleopTwistNode(Node):
    def __init__(self):
        super().__init__('teleop_twist_node')

        self.declare_parameter('axis_linear_rt', 5)
        self.declare_parameter('axis_linear_lt', 2)
        self.declare_parameter('axis_angular', 3)
        self.declare_parameter('button_rb', 5)
        self.declare_parameter('scale_linear', 1.0)
        self.declare_parameter('scale_angular', 1.0)

        # Get parameters
        self.axis_linear_rt = self.get_parameter('axis_linear_rt').value
        self.axis_linear_lt = self.get_parameter('axis_linear_lt').value
        self.axis_angular = self.get_parameter('axis_angular').value
        self.button_rb = self.get_parameter('button_rb').value
        self.scale_linear = self.get_parameter('scale_linear').value
        self.scale_angular = self.get_parameter('scale_angular').value

        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, joy_msg):
        twist = Twist()

        # Map RT (axis 5) to forward speed
        forward_speed = -(joy_msg.axes[self.axis_linear_rt] - 1) / 2  # Map [-1,1] to [0,1]

        # Check if RB button is pressed for backward motion
        rb_pressed = joy_msg.buttons[self.button_rb] == 1

        # backward_speed = (joy_msg.axes[self.axis_linear_rt] + 1) / 2 if rb_pressed else 0.0
        if rb_pressed:
            backward_speed = (joy_msg.axes[self.axis_linear_lt]) / 2
        else:
            backward_speed = 0.0

        # Calculate net linear velocity
        twist.linear.x = self.scale_linear * (forward_speed - backward_speed)

        # Map right joystick horizontal axis to angular velocity
        twist.angular.z = self.scale_angular * joy_msg.axes[self.axis_angular]

        self.get_logger().info(f"Publishing Twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}")

        # Publish the Twist message
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopTwistNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
