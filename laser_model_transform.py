import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
import math
import numpy as np


class TransformLaserScanNode(Node):
    def __init__(self):
        super().__init__("transform_laser_scan_node")

        # Parameter for the target frame
        self.declare_parameter("target_frame", "amcl/base_link")
        self.target_frame = self.get_parameter("target_frame").value

        # Subscriber to /scan topic
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        # Publisher for the transformed scan
        self.transformed_scan_pub = self.create_publisher(
            LaserScan, "/transformed_scan", 10
        )

        # TF2 Buffer and TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timeout = Duration(seconds = 1)
        self.get_logger().info(f"TransformLaserScanNode initialized. Target frame: {self.target_frame}")

    def scan_callback(self, scan_msg):
       # try:
       #     # Lookup the transform from source frame to target frame
       #     transform = self.tf_buffer.lookup_transform(
       #         self.target_frame, scan_msg.header.frame_id, scan_msg.header.stamp
       #     )
       #     
       #     
       # except Exception as e:
       #     self.get_logger().warn(f"Could not transform: {e}")
       #     return

        # Transform LaserScan ranges
        transformed_scan = LaserScan()
        transformed_scan.header.stamp = scan_msg.header.stamp
        transformed_scan.header.frame_id = self.target_frame
        transformed_scan.angle_min = scan_msg.angle_min
        transformed_scan.angle_max = scan_msg.angle_max
        transformed_scan.angle_increment = scan_msg.angle_increment
        transformed_scan.time_increment = scan_msg.time_increment
        transformed_scan.scan_time = scan_msg.scan_time
        transformed_scan.range_min = scan_msg.range_min
        transformed_scan.range_max = scan_msg.range_max
        transformed_scan.ranges = scan_msg.ranges

        # Publish the transformed LaserScan
        self.transformed_scan_pub.publish(transformed_scan)

def main(args=None):
    rclpy.init(args=args)
    node = TransformLaserScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
