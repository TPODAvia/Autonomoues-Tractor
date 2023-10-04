#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FakeScanNode(Node):
    def __init__(self):
        super().__init__('fake_scan_node')
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.2, self.publish_scan)

    def publish_scan(self):
        scan = LaserScan()
        # Set the necessary fields of the scan message
        scan.header.frame_id = 'base_link'
        scan.angle_min = 0.0
        scan.angle_max = 2.0 * 3.14159
        scan.angle_increment = 0.01
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.0
        scan.range_max = 10.0
        scan.ranges = [9.0] * int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        self.scan_publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    fake_scan_node = FakeScanNode()
    rclpy.spin(fake_scan_node)
    fake_scan_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()