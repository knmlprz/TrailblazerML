#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from aruco_opencv_msgs.msg import ArucoDetection
from std_srvs.srv import Trigger

angular_speed = 0.6

class CmdVelNavPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_nav_publisher')

        self.publisher = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        self.subscription = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.aruco_callback,
            10
        )

        self.start_service = self.create_service(
            Trigger,
            '/aruco_searching_start',
            self.start_searching_callback
        )

        self.stop_service = self.create_service(
            Trigger,
            '/aruco_searching_stop',
            self.stop_searching_callback
        )

        # Klient do samowywoływania zatrzymania
        self.stop_client = self.create_client(Trigger, '/aruco_searching_stop')

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        self.searching_active = False
        self.toggle = False
        self.counter = 0
        self.toggle_interval = 15

        self.stop_called = False  # Flaga, żeby nie wołać stopa wiele razy

        self.get_logger().info('Node initialized. Use /aruco_searching_start and /aruco_searching_stop services.')

    def start_searching_callback(self, request, response):
        self.searching_active = True
        self.stop_called = False  # Reset flagi przy starcie
        self.get_logger().info('Search mode activated. Publishing on /cmd_vel_nav.')
        response.success = True
        response.message = 'Started publishing /cmd_vel_nav.'
        return response

    def stop_searching_callback(self, request, response):
        # Najpierw wyślij zatrzymanie (linear=0.0, angular=0.0)
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher.publish(stop_msg)
        self.get_logger().info('Published STOP Twist: linear=0.0 angular=0.0')

        # Następnie dezaktywuj tryb
        self.searching_active = False
        self.get_logger().info('Search mode stopped. No longer publishing /cmd_vel_nav.')
        response.success = True
        response.message = 'Stopped publishing /cmd_vel_nav.'
        return response

    def call_stop_service(self):
        if not self.stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Stop service not available.')
            return

        request = Trigger.Request()
        future = self.stop_client.call_async(request)

        def callback(fut):
            try:
                result = fut.result()
                self.get_logger().info(f"Stop service response: success={result.success}, message='{result.message}'")
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')

        future.add_done_callback(callback)

    def timer_callback(self):
        if not self.searching_active:
            return

        if self.counter % self.toggle_interval == 0:
            self.toggle = not self.toggle

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = angular_speed if self.toggle else 0.0

        self.publisher.publish(msg)
        self.counter += 1

        if self.counter % self.toggle_interval == 0:
            self.get_logger().info(f'Published Twist: linear=0.0 angular={msg.angular.z}')

    def aruco_callback(self, msg: ArucoDetection):
        if not msg.markers:
            self.get_logger().info("No markers detected.")
            return

        for i, marker in enumerate(msg.markers):
            x = marker.pose.position.x
            y = marker.pose.position.y
            z = marker.pose.position.z
            self.get_logger().info(f'Marker {i}: position x={x:.2f}, y={y:.2f}, z={z:.2f}')

        if (
            len(msg.markers) >= 2 and 
            self.searching_active and 
            not self.stop_called
        ):
            # Oblicz średnią pozycji X dwóch pierwszych markerów
            x1 = msg.markers[0].pose.position.x
            x2 = msg.markers[1].pose.position.x
            avg_x = (x1 + x2) / 2.0

            self.get_logger().info(f'Average X between two markers: {avg_x:.3f}')

            # Jeśli średnia bliska 0 (kamera skierowana między markerami)
            if abs(avg_x) < 0.05 or avg_x >= 0.0:
                self.get_logger().info("Camera is centered between two markers — calling stop service.")
                self.call_stop_service()
                self.stop_called = True


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelNavPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down node...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
