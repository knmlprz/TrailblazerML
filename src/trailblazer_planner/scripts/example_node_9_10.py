#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float32

class ManualServiceNode(Node):
    def __init__(self):
        super().__init__('manual_trigger_services')

        # Clienty do uruchamiania usług
        self.task9_client = self.create_client(Trigger, 'task9')
        self.task10_client = self.create_client(Trigger, 'task10')

        # Flaga, żeby nie wywołać /task2 wielokrotnie
        self.task10_called = False

        # Subskrypcja
        self.subscriber = self.create_subscription(
            Float32,
            '/aruco_average',
            self.aruco_callback,
            10
        )

        # Timer do odczekania aż serwis task1 będzie dostępny
        self.timer = self.create_timer(1.0, self.call_task1_once)

        self.task9_called = False

    def call_task9_once(self):
        if not self.task9_called and self.task9_client.service_is_ready():
            self.task9_called = True
            self.get_logger().info("Calling /task9 service...")
            req = Trigger.Request()
            future = self.task9_client.call_async(req)

            def task9_done(fut):
                if fut.result().success:
                    self.get_logger().info("Successfully called /task9.")
                else:
                    self.get_logger().warn("Failed to call /task9.")

            future.add_done_callback(task9_done)

            self.destroy_timer(self.timer)

    def aruco_callback(self, msg):
        self.get_logger().info(f"Received /aruco_average: {msg.data}")
        if msg.data > 15.0 and not self.task10_called and self.task10_client.service_is_ready():
            self.task10_called = True
            self.get_logger().info("Calling /task10 service because /aruco_average > 15.0")
            req = Trigger.Request()
            future = self.task10_client.call_async(req)

            def task10_done(fut):
                if fut.result().success:
                    self.get_logger().info("Successfully called /task10.")
                else:
                    self.get_logger().warn("Failed to call /task10.")

            future.add_done_callback(task10_done)

    # Serwisowe callbacki
    def task9_callback(self, request, response):
        self.get_logger().info("Service /task9 was called")
        response.success = True
        response.message = "Hello from task9!"
        return response

    def task10_callback(self, request, response):
        self.get_logger().info("Service /task10 was called")
        response.success = True
        response.message = "Hello from task10!"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ManualServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
