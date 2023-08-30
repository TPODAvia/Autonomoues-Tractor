#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading
import time

from sensor_msgs.msg import NavSatFix, Imu

shutdown = False

class WebPublisherNode(Node):
    def __init__(self):
        super().__init__('web_publisher')
        self.subscription_gps = self.create_subscription(NavSatFix, 'gps/filtered', self.gps_callback, 10)
        self.subscription_imu = self.create_subscription(Imu, 'imu', self.imu_callback, 10)

    def gps_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.position_covariance)
        global_data = f"GPS: [Lat, Long, Alt]=[{msg.latitude},{msg.longitude},{msg.altitude}] \n [N, E, Up]=[]"
        WebRequestHandler.update_latest_data(global_data)

    def imu_callback(self, msg):
        global_data = f"IMU: Linear Acceleration [XYZ]=[{msg.linear_acceleration.x},{msg.linear_acceleration.y},{msg.linear_acceleration.z}]"
        WebRequestHandler.update_latest_data(global_data)

class WebRequestHandler(BaseHTTPRequestHandler):
    latest_gps_data = None
    latest_imu_data = None

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(b'<html><body>')
            self.wfile.write(b'<h1>ROS2 Web Publisher</h1>')
            self.wfile.write(b'<p>Received data:</p>')
            self.wfile.write(bytes(f'<p>{WebRequestHandler.latest_gps_data}</p>', 'utf-8'))
            self.wfile.write(bytes(f'<p>{WebRequestHandler.latest_imu_data}</p>', 'utf-8'))
            self.wfile.write(b'</body></html>')
        else:
            self.send_response(404)
            self.end_headers()

    @classmethod
    def update_latest_data(cls, data):
        if data.startswith("GPS"):
            cls.latest_gps_data = data
        elif data.startswith("IMU"):
            cls.latest_imu_data = data

def start_server():
    server_address = ('', 8000)
    httpd = HTTPServer(server_address, WebRequestHandler)
    httpd.RequestHandlerClass = WebRequestHandler  # Set the request handler class explicitly
    # while not shutdown:
    #     httpd.handle_request()
    # httpd.serve_forever()
    while not shutdown:
        httpd.handle_request()
        time.sleep(1)  # Adjust the sleep interval as needed
    httpd.server_close()

def main(args=None):
    global shutdown
    rclpy.init(args=args)
    node = WebPublisherNode()
    server_thread = threading.Thread(target=start_server)
    server_thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        shutdown = True
        server_thread.join()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
