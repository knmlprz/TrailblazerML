#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float32

class ManualServiceNode(Node):
    def __init__(self):
        super().__init__('manual_trigger_services')

        # Clienty do uruchamiania usług
        self.task5_client = self.create_client(Trigger, 'task5')
        self.task6_client = self.create_client(Trigger, 'task6')

        # Flaga, żeby nie wywołać /task2 wielokrotnie
        self.task6_called = False

        # Subskrypcja
        self.subscriber = self.create_subscription(
            Float32,
            '/aruco_average',
            self.aruco_callback,
            10
        )

        # Timer do odczekania aż serwis task1 będzie dostępny
        self.timer = self.create_timer(1.0, self.call_task1_once)

        self.task5_called = False

    def call_task5_once(self):
        if not self.task5_called and self.task5_client.service_is_ready():
            self.task5_called = True
            self.get_logger().info("Calling /task5 service...")
            req = Trigger.Request()
            future = self.task1_client.call_async(req)

            def task5_done(fut):
                if fut.result().success:
                    self.get_logger().info("Successfully called /task5.")
                else:
                    self.get_logger().warn("Failed to call /task5.")

            future.add_done_callback(task5_done)

            self.destroy_timer(self.timer)

    def aruco_callback(self, msg):
        self.get_logger().info(f"Received /aruco_average: {msg.data}")
        if msg.data > 15.0 and not self.task6_called and self.task6_client.service_is_ready():
            self.task2_called = True
            self.get_logger().info("Calling /task2 service because /aruco_average > 15.0")
            req = Trigger.Request()
            future = self.task6_client.call_async(req)

            def task6_done(fut):
                if fut.result().success:
                    self.get_logger().info("Successfully called /task6.")
                else:
                    self.get_logger().warn("Failed to call /task6.")

            future.add_done_callback(task6_done)

    # Serwisowe callbacki
    def task5_callback(self, request, response):
        self.get_logger().info("Service /task5 was called")
        response.success = True
        response.message = "Hello from task5!"
        return response

    def task6_callback(self, request, response):
        self.get_logger().info("Service /task6 was called")
        response.success = True
        response.message = "Hello from task6!"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ManualServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
