#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

class EmptyMapPublisher(Node):
    def __init__(self):
        super().__init__('empty_map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        self.empty_map = OccupancyGrid()
        self.define_empty_map()
        self.timer = self.create_timer(0.1, self.publish_empty_map)

    def define_empty_map(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        self.empty_map.header = header
        self.empty_map.info.resolution = 1.0
        self.empty_map.info.width = 1000
        self.empty_map.info.height = 1000
        self.empty_map.data = [1] * (self.empty_map.info.width * self.empty_map.info.height)

    def publish_empty_map(self):
        self.publisher_.publish(self.empty_map)

def main(args=None):
    rclpy.init(args=args)

    empty_map_publisher = EmptyMapPublisher()

    rclpy.spin(empty_map_publisher)

    # Destroy the node explicitly
    empty_map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()