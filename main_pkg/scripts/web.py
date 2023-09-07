#!/usr/bin/env python3 
import rclpy  
from rclpy.node import Node  
from std_msgs.msg import String  
from sensor_msgs.msg import NavSatFix, Imu  
from flask import Flask, jsonify  
import threading  
import time  
from flask_cors import CORS, cross_origin

shutdown = False  
  
app = Flask(__name__)
cors = CORS(app)
latest_gps_data = None 
latest_imu_data = None 
  
class WebPublisherNode(Node):  
 
    def __init__(self):  
        super().__init__('web_publisher')  
        self.subscription_gps = self.create_subscription(NavSatFix, 'gps/filtered', self.gps_callback, 10)  
        self.subscription_imu = self.create_subscription(Imu, 'imu', self.imu_callback, 10)  
  
    def gps_callback(self, msg):  
        pos_cov = [str(i) for i in msg.position_covariance]  
        global_data = {  
            'data_GPS': {  
                'latitude': msg.latitude,  
                'longitude': msg.longitude,  
                'altitude': msg.altitude,  
                'position_covariance': pos_cov  
            }  
        }  
        global latest_gps_data 
        latest_gps_data = global_data 
  
    def imu_callback(self, msg):  
        global_data = {  
            'data_IMU': {  
                    'x': msg.linear_acceleration.x,  
                    'y': msg.linear_acceleration.y,  
                    'z': msg.linear_acceleration.z, 
                    'qx':msg.orientation.x, 
                    'qy':msg.orientation.y, 
                    'qz':msg.orientation.z, 
                    'qw':msg.orientation.w 
            }  
        } 
        global latest_imu_data 
        latest_imu_data = global_data  
         
  
 
@app.route('/')  
def get_latest_data():  
    global latest_gps_data  
    global latest_imu_data
    try:
        if {**latest_gps_data, **latest_imu_data} is not None:  
            return jsonify({**latest_gps_data, **latest_imu_data})  
        else:  
            return jsonify({'message': 'No data available'})
    except:
        return jsonify({'data_GPS': {
                'latitude': -1,
                'longitude': -1,
                'altitude': -1,
                'position_covariance': [-1,-1,-1,-1,-1,-1,-1,-1,-1]
            },
            'data_IMU': {
                    'x': 0,
                    'y': 0,
                    'z': 0,
                    'qx':0,
                    'qy':0,
                    'qz':0,
                    'qw':0
            }
	})


def start_flask_server():  
    app.run(host='0.0.0.0', port=8000, threaded=True)  

def main(args=None):  
    global shutdown  
    rclpy.init(args=args)  
    node = WebPublisherNode()  
    server_thread = threading.Thread(target=start_flask_server)
    server_thread.setDaemon(True)  
    server_thread.start()  
    try:  
        rclpy.spin(node)  
    except KeyboardInterrupt:  
        shutdown = True  
        exit(0)  
    finally:  
        node.destroy_node()  
        rclpy.shutdown()  
  
if __name__ == '__main__':  
    main()
