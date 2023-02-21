import sys
sys.path.append('..')
from deprecaded.motor import motor
import grpc
from feature_extractor import feature_extractor
import robot_control_service_pb2
import robot_control_service_pb2_grpc
import video_streaming_service_pb2
import video_streaming_service_pb2_grpc
from utils import *
from concurrent import futures
from PIL import Image
import io
import cv2
import time
import pickle

cam = cv2.VideoCapture(0)

feature_extractor_handler = feature_extractor()
with open('./camera_0_calibration.p', 'rb') as f:
    cfg = pickle.load(f)
MOTOR_LEFT = motor.motor(12,18) 
MOTOR_RIGHT = motor.motor(19,13) 

def convert_points(img):
    return np.asarray(np.vstack([img, np.ones(img.shape[1])]))

def get_slam_points(img):
    p1, p2, kpts, matches = fe.match_frames(img)
    p1 = convert_points(p1)
    p2 = convert_points(p2)

    img_h, img_w, img_ch = img.shape

    intrinsic = np.array([[3000,0,img_w/2],
                [0,3000,img_h/2],
                [0,0,1]])
    tripoints3d = []
    points1_norm = np.dot(np.linalg.inv(intrinsic), p1)
    points2_norm = np.dot(np.linalg.inv(intrinsic), p2)

    E = compute_essential_normalized(points1_norm, points2_norm)

    P1 = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0]])
    P2s = compute_P_from_essential(E)

    ind = -1
    for i, P2 in enumerate(P2s):
        d1 = reconstruct_one_point(points1_norm[:, 0], points2_norm[:, 0], P1, P2)

        P2_homogenous = np.linalg.inv(np.vstack([P2, [0,0,0,1]]))

        d2 = np.dot(P2_homogenous[:3, :4], d1)

        if d1[2] > 0 and d2[2] > 0:
            ind = i

        P2 = np.linalg.inv(np.vstack([P2s[ind], [0, 0, 0, 1]]))[:3, :4]
        tripoints3d = triangulation(points1_norm, points2_norm, P1, P2)
        print("tripoints3d is :")
        # print(tripoints3d[0])

    return img, tripoints3d, kpts,



class video_streamer_handlerServicerServicer(video_streaming_service_pb2_grpc.video_streamer_handlerServicer):
    def send_image_chunk(self, request, context):
        ret, image = cam.read()
        image_bytes = 0
        while 1:
            if ret:
                image_bytes = cv2.imencode('.jpg', image)[1].tobytes()
                break
        return video_streaming_service_pb2.image_chunk(image=image_bytes)


class robot_control_handlerServicerServicer(robot_control_service_pb2_grpc.robot_control_handlerServicer):
    def robot_move_forward(self, request, context):
        MOTOR_LEFT.motor_break()
        MOTOR_RIGHT.motor_break()
        MOTOR_LEFT.forward() 
        MOTOR_RIGHT.forward() 
        return robot_control_service_pb2.Empty()

    def robot_move_backward(self, request, context):
        MOTOR_LEFT.motor_break()
        MOTOR_RIGHT.motor_break()
        MOTOR_LEFT.reverse() 
        MOTOR_RIGHT.reverse() 
        return robot_control_service_pb2.Empty()

    def robot_break(self, request, context):
        MOTOR_LEFT.motor_break()
        MOTOR_RIGHT.motor_break()
        MOTOR_LEFT.motor_break()
        MOTOR_RIGHT.motor_break()
        return robot_control_service_pb2.Empty()

    def robot_turn_right(self, request, context):
        MOTOR_LEFT.motor_break()
        MOTOR_RIGHT.motor_break()
        MOTOR_LEFT.forward() 
        return robot_control_service_pb2.Empty()

    def robot_turn_left(self, request, context):
        MOTOR_LEFT.motor_break()
        MOTOR_RIGHT.motor_break()
        MOTOR_RIGHT.forward() 
        return robot_control_service_pb2.Empty()

    def robot_set_speed(self, request, context):
        MOTOR_LEFT.motor_break()
        MOTOR_RIGHT.motor_break()
        MOTOR_LEFT.set_speed(request.speed)
        MOTOR_RIGHT.set_speed(request.speed)
        return robot_control_service_pb2.Empty()


def main():
    print("starting server")
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=5))
    robot_control_service_pb2_grpc.add_robot_control_handlerServicer_to_server(robot_control_handlerServicerServicer(),server)
    video_streaming_service_pb2_grpc.add_video_streamer_handlerServicer_to_server(video_streamer_handlerServicerServicer(),server)
    server.add_insecure_port("[::]:50052")
    server.start()
    server.wait_for_termination()

main()
# GPIO.cleanup()
