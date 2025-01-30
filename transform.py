import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped


class TransformSLAM(Node):
    def __init__(self):
        super().__init__('transform_listener_node')
        
        # TF2 Buffer 및 Listener 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # TF2 Broadcaster 설정
        self.tf_broadcaster = TransformBroadcaster(self)

        # 타이머 생성 (200 Hz)
        self.timer = self.create_timer(0.005, self.timer_callback)

    def timer_callback(self):
        try:
            # /map -> ego_racecar/base_link 간 Transform 가져오기
            transform_stamped = self.tf_buffer.lookup_transform(
                "map",                                      # Target frame
                "ego_racecar/base_link",                    # Source frame
                rclpy.time.Time())       # 최신 Transform 사용
            
            # Transform 생성
            slam_transform = TransformStamped()
            slam_transform.header.stamp = self.get_clock().now().to_msg()
            slam_transform.header.frame_id = "amcl_odom"  # 부모 프레임
            slam_transform.child_frame_id = "amcl/base_link"    # 자식 프레임

            # Translation
            slam_transform.transform.translation.x = transform_stamped.transform.translation.x
            slam_transform.transform.translation.y = transform_stamped.transform.translation.y
            slam_transform.transform.translation.z = transform_stamped.transform.translation.z

            # Rotation
            slam_transform.transform.rotation.x = transform_stamped.transform.rotation.x
            slam_transform.transform.rotation.y = transform_stamped.transform.rotation.y
            slam_transform.transform.rotation.z = transform_stamped.transform.rotation.z
            slam_transform.transform.rotation.w = transform_stamped.transform.rotation.w

            # Transform 퍼블리시
            self.tf_broadcaster.sendTransform(slam_transform)

        except Exception as ex:
            self.get_logger().warn(f"Could not get transform: {str(ex)}")


def main(args=None):
    rclpy.init(args=args)
    node = TransformSLAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
