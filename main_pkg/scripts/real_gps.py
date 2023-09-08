#!/usr/bin/env python3
import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import String
import re
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import Header
import serial
import time

class GpsNode(Node):

    def __init__(self):
        super().__init__('gps_node')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.string_publisher = self.create_publisher(String, 'gps/velocity', 10)

        # Add the parameter
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        # Get the parameter value
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value

        self.ser = serial.Serial(self.serial_port)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.gps_callback)

    def gps_callback(self):
        while True:
            #time.sleep() 
            non_decimal = re.compile(r'[^\d.]+')
            ser_bytes = self.ser.readline()
            if not ser_bytes:
                return
            decoded_bytes = ser_bytes.decode('utf-8')
            print(decoded_bytes)

            if "GNGGA" in decoded_bytes:
                # print("decoded_bytes: ")
                # print(decoded_bytes)
                try:
                    print(int(decoded_bytes.split(",")[7]))
                    if int(decoded_bytes.split(",")[7]) < 3:
                        self.get_logger().info('Not enought sattelite: %s' % decoded_bytes.split(",")[7])
                        return
                except: 
                    self.get_logger().info('Bad connection')
                    return

                msg = NavSatFix()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "base_link"

                msg.status.status = NavSatStatus.STATUS_FIX
                msg.status.service = NavSatStatus.SERVICE_GPS

                # Position in degrees.
                if decoded_bytes.split(",")[2] == '':

                    print("no gps data")
                    return
                    # msg.latitude = -1.0
                    # msg.longitude = -1.0
                    # msg.altitude = -1.0
                
                else:
                    msg.latitude = float(non_decimal.sub('',decoded_bytes.split(",")[2]))*0.01
                    msg.longitude = float(non_decimal.sub('',decoded_bytes.split(",")[4]))*0.01
                    msg.altitude = float(non_decimal.sub('',decoded_bytes.split(",")[9]))

                msg.position_covariance[0] = 0
                msg.position_covariance[4] = 0
                msg.position_covariance[8] = 0
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

                self.publisher_.publish(msg)
                self.best_pos_a = None
                break

            if "GNRMC" in decoded_bytes:
                # print(decoded_bytes)
                gps_vel = String()

                if decoded_bytes.split(",")[7] == '':
                    print("no gps velocity data")
                    gps_vel.data = "0.0"
                else:
                    gps_vel.data = decoded_bytes.split(",")[7]
                    print(decoded_bytes.split(",")[7])
                self.string_publisher.publish(gps_vel)
                break


def main(args=None):
    rclpy.init(args=args)

    gps_node = GpsNode()

    rclpy.spin(gps_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
