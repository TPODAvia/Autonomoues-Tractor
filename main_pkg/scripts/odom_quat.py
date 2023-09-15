#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class OdometrySubscriber(Node):

    def __init__(self):
        super().__init__('odometry_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',  # replace with your odometry topic
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        # Extract quaternion
        quaternion = (msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w)
        # Convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.get_logger().info('Roll: "%f", Pitch: "%f", Yaw: "%f"' % (roll, pitch, yaw))

def main(args=None):
    rclpy.init(args=args)

    odometry_subscriber = OdometrySubscriber()

    try:
        rclpy.spin(odometry_subscriber)
    except:
        pass
    finally:
        odometry_subscriber.destroy_node()

if __name__ == '__main__':
    main()