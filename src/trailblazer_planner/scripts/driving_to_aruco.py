#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from aruco_opencv_msgs.msg import ArucoDetection
from std_srvs.srv import Trigger

angular_gain = 0.5
max_linear_speed = 0.2
max_angular_speed = 0.08
max_missed_detections = 15  # liczba wiadomości z rzędu bez 2 markerów, po której zatrzyma się
max_save_missed_detections = 60

class CmdVelNavPublisher(Node):
    def __init__(self):
        super().__init__('driving_to_aruco_node')

        self.publisher = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        self.subscription = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.aruco_callback,
            10
        )

        self.start_service = self.create_service(
            Trigger,
            '/driving_to_aruco_start',
            self.start_searching_callback
        )

        self.stop_service = self.create_service(
            Trigger,
            '/driving_to_aruco_stop',
            self.stop_searching_callback
        )

        self.stop_client = self.create_client(Trigger, '/driving_to_aruco_stop')

        self.searching_active = False
        self.save_mode = False
        self.missed_counter = 0  # licznik nieudanych detekcji

        self.get_logger().info('Node initialized.')

    def start_searching_callback(self, request, response):
        self.searching_active = True
        self.missed_counter = 0
        self.get_logger().info('Activated ArUco navigation.')
        response.success = True
        response.message = 'Started ArUco navigation.'
        return response

    def stop_searching_callback(self, request, response):
        self.stop_robot()
        self.searching_active = False
        self.get_logger().info('Deactivated ArUco navigation.')
        response.success = True
        response.message = 'Stopped ArUco navigation.'
        return response

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)

    def aruco_callback(self, msg: ArucoDetection):
        if not self.searching_active:
            return

        if len(msg.markers) < 2:
            self.missed_counter += 1
            self.get_logger().warn(f'Less than 2 markers detected. Missed count: {self.missed_counter}/{max_missed_detections}')
            if self.missed_counter >= max_missed_detections:
                self.get_logger().warn('Too many missed detections. Save mode activating.')
                self.stop_robot()
                if self.missed_counter >= max_save_missed_detections:
                    self.get_logger().warn('Too many missed detections. Stopping robot.')
                    self.call_stop_service()
                    self.searching_active = False
            return

        # Wykryto co najmniej 2 markery — zeruj licznik błędów
        self.missed_counter = 0

        x1 = msg.markers[0].pose.position.x
        x2 = msg.markers[1].pose.position.x
        avg_x = (x1 + x2) / 2.0

        self.get_logger().info(f'Detected 2 markers. avg_x = {avg_x:.3f}')

        # Poruszaj się do przodu i skręcaj zależnie od avg_x
        twist = Twist()
        twist.linear.x = max_linear_speed
        twist.angular.z = -avg_x * angular_gain
        twist.angular.z = max(min(twist.angular.z, max_angular_speed), -max_angular_speed)
        self.publisher.publish(twist)

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
