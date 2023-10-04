import rclpy
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('dynamic_base_footprint_base_link_broadcaster')

    broadcaster = TransformBroadcaster(node)
    static_transformStamped = TransformStamped()

    static_transformStamped.header.stamp = node.get_clock().now().to_msg()
    static_transformStamped.header.frame_id = "base_footprint"
    static_transformStamped.child_frame_id = "base_link"

    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.0

    static_transformStamped.transform.rotation.x = 0.0
    static_transformStamped.transform.rotation.y = 0.0
    static_transformStamped.transform.rotation.z = 0.0
    static_transformStamped.transform.rotation.w = 1.0

    broadcaster.sendTransform(static_transformStamped)

    rclpy.spin(node)

if __name__ == '__main__':
    main()