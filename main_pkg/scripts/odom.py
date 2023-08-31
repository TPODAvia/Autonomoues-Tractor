#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odometry/global', 10)
        self.timer_ = self.create_timer(1.0, self.publish_odometry)
        self.get_logger().info('Odometry publisher node initialized')

    def publish_odometry(self):
        odometry_msg = Odometry()
        odometry_msg.header = Header()
        odometry_msg.header.stamp = self.get_clock().now().to_msg()
        odometry_msg.header.frame_id = "odom"
        odometry_msg.child_frame_id = "base_link"
        odometry_msg.pose.pose.position.x = 0.0
        odometry_msg.pose.pose.position.y = 0.0
        odometry_msg.pose.pose.position.z = 0.0
        odometry_msg.pose.pose.orientation.w = 1.0
        odometry_msg.pose.pose.orientation.x = 0.0
        odometry_msg.pose.pose.orientation.y = 0.0
        odometry_msg.pose.pose.orientation.z = 0.0
        odometry_msg.twist.twist.linear.x = 0.0
        odometry_msg.twist.twist.linear.y = 0.0
        odometry_msg.twist.twist.linear.z = 0.0
        odometry_msg.twist.twist.angular.x = 0.0
        odometry_msg.twist.twist.angular.y = 0.0
        odometry_msg.twist.twist.angular.z = 0.0

        self.publisher_.publish(odometry_msg)
        # self.get_logger().info('Odometry message published')
        
def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()