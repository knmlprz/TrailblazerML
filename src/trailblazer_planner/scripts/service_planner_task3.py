#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float32

class ManualServiceNode(Node):
    def __init__(self):
        super().__init__('manual_trigger_services')

        # Clienty do uruchamiania usług
        self.task3_client = self.create_client(Trigger, 'task3')
        self.task4_client = self.create_client(Trigger, 'task4')

        # Flaga, żeby nie wywołać /task2 wielokrotnie
        self.task4_called = False
        self.task3_called = True 
        # Subskrypcja
        self.subscriber = self.create_subscription(
            Float32,
            '/aruco_average',
            self.aruco_callback,
            10
        )

        # Timer do odczekania aż serwis task1 będzie dostępny
        self.timer = self.create_timer(1.0, self.call_task3_once)


    def call_task3_once(self):
        if not self.task3_called and self.task3_client.service_is_ready():
            self.task3_called = True
            self.get_logger().info("Calling /task3 service...")
            req = Trigger.Request()
            future = self.task3_client.call_async(req)

            def task3_done(fut):
                if fut.result().success:
                    self.get_logger().info("Successfully called /task3.")
                else:
                    self.get_logger().warn("Failed to call /task3.")

            future.add_done_callback(task3_done)

            self.destroy_timer(self.timer)

    def aruco_callback(self, msg):
        self.get_logger().info(f"Received /aruco_average: {msg.data}")
        if msg.data > 25.0 and not self.task4_called and self.task4_client.service_is_ready():
            self.task4_called = True
            self.get_logger().info("Calling /task4 service because /aruco_average > 25.0")
            req = Trigger.Request()
            future = self.task4_client.call_async(req)

            def task4_done(fut):
                if fut.result().success:
                    self.get_logger().info("Successfully called /task4.")
                else:
                    self.get_logger().warn("Failed to call /task4.")

            future.add_done_callback(task4_done)

    # Serwisowe callbacki



def main(args=None):
    rclpy.init(args=args)
    node = ManualServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
