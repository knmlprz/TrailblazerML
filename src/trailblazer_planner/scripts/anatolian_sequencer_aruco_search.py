#!/usr/bin/env python3
"""
ROS2 Service Sequencer Node

This node:
  1. On startup, calls `/aruco_searching_start` service.
  2. Provides a server for `/aruco_searching_stop`, and upon receiving it,
     waits 3s, then calls `/driving_to_aruco_start`.
  3. Provides a server for `/driving_to_aruco_stop`, and upon receiving it,
     waits 3s, then calls `/autonomy_start`.
  4. Provides a server for `/autonomy_stop`, and upon receiving it, shuts down.

Usage:
  ros2 run <your_package> service_sequencer.py

"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import threading
import time

class ServiceSequencer(Node):
    def __init__(self):
        super().__init__('service_sequencer')

        # Create clients
        self.cli_search_start = self.create_client(Trigger, '/aruco_searching_start')

        # Create servers for stop signals
        self.srv_search_stop = self.create_service(Trigger, '/aruco_searching_stop', self.handle_search_stop)

        # Ensure service availability then call first
        self.get_logger().info('Waiting for /aruco_searching_start service...')
        self.cli_search_start.wait_for_service()
        self.call_service(self.cli_search_start, 'aruco_searching_start')
        self.get_logger().info('Called /aruco_searching_start')

    def call_service(self, client, name):
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f"Service {name} succeeded: {future.result().message}")
        else:
            self.get_logger().error(f"Service {name} failed or no response")

    def handle_search_stop(self, request, response):
        self.get_logger().info('Received /aruco_searching_stop, launching driving_to_aruco_start in 3s')
        threading.Thread(target=self.delayed_call, args=(self.cli_drive_start, 'driving_to_aruco_start')).start()
        response.success = True
        response.message = 'Scheduled driving_to_aruco_start'
        return response

    def handle_drive_stop(self, request, response):
        self.get_logger().info('Received /driving_to_aruco_stop, launching autonomy_start in 3s')
        threading.Thread(target=self.delayed_call, args=(self.cli_auto_start, 'autonomy_start')).start()
        response.success = True
        response.message = 'Scheduled autonomy_start'
        return response

    def handle_auto_stop(self, request, response):
        self.get_logger().info('Received /autonomy_stop, shutting down node')
        response.success = True
        response.message = 'Shutting down'
        # shutdown after sending response
        threading.Thread(target=self.shutdown_delayed).start()
        return response

    def delayed_call(self, client, name):
        # wait for client availability
        self.get_logger().info(f'Waiting for /{name} service...')
        client.wait_for_service()
        time.sleep(3.0)
        self.call_service(client, name)

    def shutdown_delayed(self):
        time.sleep(0.5)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ServiceSequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

