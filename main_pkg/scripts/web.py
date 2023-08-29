#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from http.server import BaseHTTPRequestHandler, HTTPServer
import _thread as thread

global_data = "Nondefverfve"

class WebPublisherNode(Node):
    def __init__(self):
        super().__init__('web_publisher')
        self.publisher_ = self.create_publisher(String, 'web_topic', 10)
        self.subscription_ = self.create_subscription(
            String,
            'web_topic',
            self.callback,
            10
        )

    def callback(self, msg):
        global global_data
        self.get_logger().info('Received: "%s"' % msg.data)
        global_data = msg.data
        WebRequestHandler.update_latest_data(msg.data)

class WebRequestHandler(BaseHTTPRequestHandler):
    latest_data = None
    def do_GET(self):
        if self.path == '/':
            global global_data
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(b'<html><body>')
            self.wfile.write(b'<h1>ROS2 Web Publisher</h1>')
            self.wfile.write(b'<p>Received data:</p>')
            self.wfile.write(bytes(f'<p>{global_data}</p>', 'utf-8'))
            self.wfile.write(b'</body></html>')
        else:
            self.send_response(404)
            self.end_headers()

    @classmethod
    def update_latest_data(cls, data):
        cls.latest_data = data

def start_server():
    server_address = ('', 8000)
    httpd = HTTPServer(server_address, WebRequestHandler)
    httpd.RequestHandlerClass = WebRequestHandler  # Set the request handler class explicitly
    httpd.serve_forever()

def main(args=None):
    rclpy.init(args=args)
    node = WebPublisherNode()
    thread.start_new_thread(start_server, ())

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()