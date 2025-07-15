#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float32

class ManualServiceNode(Node):
    def __init__(self):
        super().__init__('manual_trigger_services')

        # Clienty do uruchamiania usług
        self.task7_client = self.create_client(Trigger, 'task7')
        self.task8_client = self.create_client(Trigger, 'task8')

        # Flaga, żeby nie wywołać /task2 wielokrotnie
        self.task8_called = False

        # Subskrypcja
        self.subscriber = self.create_subscription(
            Float32,
            '/aruco_average',
            self.aruco_callback,
            10
        )

        # Timer do odczekania aż serwis task1 będzie dostępny
        self.timer = self.create_timer(1.0, self.call_task1_once)

        self.task7_called = False

    def call_task7_once(self):
        if not self.task7_called and self.task7_client.service_is_ready():
            self.task7_called = True
            self.get_logger().info("Calling /task7 service...")
            req = Trigger.Request()
            future = self.task7_client.call_async(req)

            def task7_done(fut):
                if fut.result().success:
                    self.get_logger().info("Successfully called /task7.")
                else:
                    self.get_logger().warn("Failed to call /task7.")

            future.add_done_callback(task7_done)

            self.destroy_timer(self.timer)

    def aruco_callback(self, msg):
        self.get_logger().info(f"Received /aruco_average: {msg.data}")
        if msg.data > 15.0 and not self.task8_called and self.task8_client.service_is_ready():
            self.task8_called = True
            self.get_logger().info("Calling /task8 service because /aruco_average > 15.0")
            req = Trigger.Request()
            future = self.task8_client.call_async(req)

            def task8_done(fut):
                if fut.result().success:
                    self.get_logger().info("Successfully called /task8.")
                else:
                    self.get_logger().warn("Failed to call /task8.")

            future.add_done_callback(task8_done)

    # Serwisowe callbacki
    def task1_callback(self, request, response):
        self.get_logger().info("Service /task7 was called")
        response.success = True
        response.message = "Hello from task7!"
        return response

    def task8_callback(self, request, response):
        self.get_logger().info("Service /task8 was called")
        response.success = True
        response.message = "Hello from task8!"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ManualServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
