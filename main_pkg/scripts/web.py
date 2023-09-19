#!/usr/bin/env python3 
import rclpy  
from rclpy.node import Node  
from std_msgs.msg import String  
from sensor_msgs.msg import NavSatFix, Imu  
from flask import Flask, jsonify  
import threading  
import time  
from flask_cors import CORS, cross_origin
from tf_transformations import euler_from_quaternion
  
app = Flask(__name__)
cors = CORS(app)
latest_gps_data = None 
latest_gps_vel_data = None 
latest_imu_data = None 
  
class WebPublisherNode(Node):  
 
    def __init__(self):  
        super().__init__('web_publisher')  
        self.subscription_gps = self.create_subscription(NavSatFix, 'gps/filtered', self.gps_callback, 10)  
        self.subscription_gps_vel = self.create_subscription(String, 'gps/velocity', self.gps_vel_callback, 10) 
        self.subscription_imu = self.create_subscription(Imu, 'imu', self.imu_callback, 10)  

    def gps_vel_callback(self,msg):
        global_data = {'data_GPS_velocity':  msg.data}
        global latest_gps_vel_data 
        latest_gps_vel_data = float(global_data) 

    def gps_callback(self, msg):  
        pos_cov = [float(i) for i in msg.position_covariance]  

        # #cast from degrees, min, sec to decemial degrees
        # latitude_degrees = int(msg.latitude)
        # latitude_decimial_part = msg.latitude - latitude_degrees
        # real_latitude = latitude_degrees + (latitude_decimial_part*100)/60

        # #cast from degrees, min, sec to decemial degrees
        # longitude_degrees = int(msg.longitude)
        # longitude_decimial_part = msg.longitude - longitude_degrees
        # real_longitude = longitude_degrees + (longitude_decimial_part*100)/60

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

        # Extract quaternion
        quaternion = (msg.orientation.x,
                      msg.orientation.y,
                      msg.orientation.z,
                      msg.orientation.w)
        # Convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        global_data = {  
            'data_IMU': {  
                    'acc_x': msg.linear_acceleration.x,  
                    'acc_y': msg.linear_acceleration.y,  
                    'acc_z': msg.linear_acceleration.z, 
                    'roll':roll, 
                    'pitch':pitch, 
                    'yaw':yaw
            }  
        } 
        global latest_imu_data 
        latest_imu_data = global_data 

@app.route('/')  
def get_latest_data():  
    global latest_gps_data
    global latest_gps_vel_data 
    global latest_imu_data

    if latest_gps_data is None:
        latest_gps_data = {'data_GPS': {
                'latitude': -1,
                'longitude': -1,
                'altitude': -1,
                'position_covariance': [-1,-1,-1,-1,-1,-1,-1,-1,-1],
            }}
    if latest_imu_data is None: 
        latest_imu_data = {'data_IMU': {
                    'acc_x': 0.0,
                    'acc_y': 0.0,
                    'acc_z': 0.0,
                    'roll':0.0,
                    'pitch':0.0,
                    'yaw':0.0,
           }}
    if latest_gps_vel_data is None:
        latest_gps_vel_data ={'data_GPS_velocity': 0.0 }
    return jsonify({**latest_gps_data, **latest_gps_vel_data, **latest_imu_data})


def start_flask_server():  
    app.run(host='0.0.0.0', port=8000, threaded=True)  

def main(args=None):  
    rclpy.init(args=args)  
    node = WebPublisherNode()  
    server_thread = threading.Thread(target=start_flask_server)
    server_thread.setDaemon(True)  
    server_thread.start()  
    try:  
        rclpy.spin(node)  
    except KeyboardInterrupt:  
        pass
    finally:  
        node.destroy_node()  

if __name__ == '__main__':  
    main()
