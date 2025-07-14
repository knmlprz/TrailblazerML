import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2DArray 

class Aruco1Publisher(Node):
    def __init__(self):
        super().__init__('aruco1_publisher')
        self.publisher_ = self.create_publisher(Float32, '/aruco1', 10)
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/aruco_detections',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        for detection in msg.detections:
            if detection.id == 1:
                pose_x = detection.bbox.center.x 
                self.publisher_.publish(Float32(data=pose_x))

def main(args=None):
    rclpy.init(args=args)
    node = Aruco1Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
