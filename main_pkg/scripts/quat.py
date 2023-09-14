#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion

class ImuSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        # Extract quaternion
        quaternion = (msg.orientation.x,
                      msg.orientation.y,
                      msg.orientation.z,
                      msg.orientation.w)
        # Convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.get_logger().info('Roll: "%f", Pitch: "%f", Yaw: "%f"' % (roll, pitch, yaw))
        pass

def main(args=None):
    rclpy.init(args=args)

    imu_subscriber = ImuSubscriber()

    rclpy.spin(imu_subscriber)

    imu_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()