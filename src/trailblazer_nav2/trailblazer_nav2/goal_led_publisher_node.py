import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Bool


class GoalLEDPublisher(Node):
    def __init__(self):
        super().__init__('goal_led_publisher')

        # Publisher to LED state topic
        self.led_pub = self.create_publisher(Int8MultiArray, 'P32_GIZ/led_state_topic', 10)

        # Subscriber to goal-reached status
        self.goal_sub = self.create_subscription(Bool, 'goal_reached', self.goal_callback, 10)

        self.already_sent = False

    def goal_callback(self, msg):
        if msg.data and not self.already_sent:
            led_msg = Int8MultiArray()
            led_msg.data = [1, 0, 0]
            self.led_pub.publish(led_msg)
            self.already_sent = True
            self.get_logger().info('Goal reached — LED message sent.')


def main(args=None):
    rclpy.init(args=args)
    node = GoalLEDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
