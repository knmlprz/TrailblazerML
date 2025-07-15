#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ServicePoller(Node):
    def __init__(self):
        super().__init__('service_poller')

        #self.stop_client = self.create_client(Trigger, 'task2')
        #self.start_client = self.create_client(Trigger, 'task3')

        #stop trigger jest tym co zatrzymuje poprzedni task
        self.task2_trigger = self.create_service(Trigger, 'task2', self.stop_srv_callback)
        self.task3_trigger = self.create_service(Trigger, 'task3', self.handle_task3)

        self.task3_timer = None

    def stop_srv_callback(self, request, response):
        self.get_logger().info("task2 called. Will call /task3 in 3 sec...")
        
        self.task3_timer = self.create_timer(3.0, self.call_task3_after_delay)

        response.success = True
        response.message = "Srv task3 will be called in 3 secs..."
        return response


    def call_task3_after_delay(self):
        self.destroy_timer(self.task3_timer)

        self.get_logger().info("Calling task3 after delay")
        # Symulujemy wywołanie lokalnego serwisu
        req = Trigger.Request()
        res = Trigger.Response()
        res = self.handle_task3(req, res)  # bezpośrednie wywołanie callbacka
        if res.success:
            self.get_logger().info("task3 executed successfully.")
        else:
            self.get_logger().warn("task3 execution failed.")

    def handle_task3(self, request, response):
        # Tu Twoja logika zadania task3
        self.get_logger().info("task3 service callback triggered!")
        response.success = True
        response.message = "Task3 completed."
        return response
    

def main(args=None):
    rclpy.init(args=args)
    node = ServicePoller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()