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
        self.declare_parameter('button_lb', 4)
        self.declare_parameter('scale_linear', 1.0)
        self.declare_parameter('scale_angular', 1.0)

        # Get parameters
        self.axis_linear_rt = self.get_parameter('axis_linear_rt').value
        self.axis_linear_lt = self.get_parameter('axis_linear_lt').value
        self.axis_angular = self.get_parameter('axis_angular').value
        self.button_rb = self.get_parameter('button_rb').value
        self.button_lb = self.get_parameter('button_lb').value
        self.scale_linear = self.get_parameter('scale_linear').value
        self.scale_angular = self.get_parameter('scale_angular').value

        self.cmd_vel_pub_right = self.create_publisher(Twist, '/diff_drive_controller_right/cmd_vel_unstamped', 10)
        self.cmd_vel_pub_left = self.create_publisher(Twist, '/diff_drive_controller_left/cmd_vel_unstamped', 10)

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, joy_msg):
        twist_right = Twist()
        twist_left = Twist()

        rb_pressed = joy_msg.buttons[self.button_rb] == 1

        if rb_pressed:
            forward_speed = 0.0
            backward_speed = -(joy_msg.axes[self.axis_linear_rt] - 1) / 2
        else:
            forward_speed = -(joy_msg.axes[self.axis_linear_rt] - 1) / 2
            backward_speed = 0.0

        twist_right.linear.x = self.scale_linear * (forward_speed - backward_speed)


        lb_pressed = joy_msg.buttons[self.button_lb] == 1
        if lb_pressed:
            forward_speed_left = 0.0
            backward_speed_left = -(joy_msg.axes[self.axis_linear_lt] - 1) / 2
        else:
            forward_speed_left = -(joy_msg.axes[self.axis_linear_lt] - 1) / 2
            backward_speed_left = 0.0

        twist_left.linear.x = self.scale_linear * (forward_speed_left - backward_speed_left)
        self.get_logger().info(f'LB state: {lb_pressed} value: {twist_left.linear.x} || RB state: {rb_pressed} value: {twist_right.linear.x}')
        self.cmd_vel_pub_right.publish(twist_right)
        self.cmd_vel_pub_left.publish(twist_left)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopTwistNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
