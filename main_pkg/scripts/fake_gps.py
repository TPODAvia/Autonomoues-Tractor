#!/usr/bin/env python3
import rclpy
import os
from rclpy.node import Node
import random

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import Header


class GpsNode(Node):

    def __init__(self):
        super().__init__('fake_gps_node')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
            msg = NavSatFix()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"

            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS

            # Position in degrees.
            msg.latitude = 57.047218 + random.uniform(-0.0001, 0.0001)
            msg.longitude = 9.920100 + random.uniform(-0.0001, 0.0001)

            # Altitude in metres.
            msg.altitude = 1.15 + random.uniform(-0.0001, 0.0001)

            msg.position_covariance[0] = 0
            msg.position_covariance[4] = 0
            msg.position_covariance[8] = 0
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

            self.publisher_.publish(msg)
            self.best_pos_a = None


def main(args=None):
    rclpy.init(args=args)

    gps_node = GpsNode()

    try:
        rclpy.spin(gps_node)
    except KeyboardInterrupt:
        pass
    finally:
        gps_node.destroy_node()


if __name__ == '__main__':
    main()