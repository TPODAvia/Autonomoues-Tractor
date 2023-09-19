#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from sensor_msgs.msg import Imu

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odometry/global', 10)
        self.subscriber_ = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.timer_ = self.create_timer(0.1, self.publish_odometry)
        self.imu_data = None  # Initialize imu_data
        self.get_logger().info('Odometry publisher node initialized')


    def imu_callback(self, msg):
        self.imu_data = msg

    def publish_odometry(self):
        odometry_msg = Odometry()
        odometry_msg.header = Header()
        odometry_msg.header.stamp = self.get_clock().now().to_msg()
        odometry_msg.header.frame_id = "odom"
        odometry_msg.child_frame_id = "base_link"
        odometry_msg.pose.pose.position.x = 0.0
        odometry_msg.pose.pose.position.y = 0.0
        odometry_msg.pose.pose.position.z = 0.0
        if self.imu_data is not None:
            odometry_msg.pose.pose.orientation = self.imu_data.orientation
        odometry_msg.twist.twist.linear.x = 0.0
        odometry_msg.twist.twist.linear.y = 0.0
        odometry_msg.twist.twist.linear.z = 0.0
        # odometry_msg.twist.twist.angular.x = 0.0
        # odometry_msg.twist.twist.angular.y = 0.0
        # odometry_msg.twist.twist.angular.z = 0.0

        self.publisher_.publish(odometry_msg)
        # self.get_logger().info('Odometry message published')
        
def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    try:
        rclpy.spin(odometry_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        odometry_publisher.destroy_node()

if __name__ == '__main__':
    main()