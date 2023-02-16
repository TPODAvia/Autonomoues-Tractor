import grpc
from screen import screen 
import threading
from sample import sample
import os
import video_streaming_service_pb2 
import video_streaming_service_pb2_grpc
import robot_control_service_pb2
import robot_control_service_pb2_grpc
import numpy as np
import cv2
from PIL import Image
import time 
import io
import matplotlib.pyplot as plt
import pickle
import pygame
from pygame.locals import *
from feature_extractor import feature_extractor
from utils import *

def bubble_sort(arr):
    n = len(arr)
    swapped = False
    for i in range(n-1):
        for j in range(0, n-i-1):
            if int(arr[j].split(".")[0]) > int(arr[j + 1].split(".")[0]):
                swapped = True
                arr[j], arr[j + 1] = arr[j + 1], arr[j]
        if not swapped:
            return

def get_photo_name():
    listing = os.listdir(CALIBRATION_SAMPLES_PATH)
    good_listing = []
    for i in listing:
        if ".DS_Store" not in i:
            
            good_listing.append(i)
    bubble_sort(good_listing) 
    print(good_listing)
    if len(good_listing):
        index = int(good_listing[-1].split(".")[0])+1
        return f"{index}.jpeg"
    else:
        return "0.jpeg"

def camera_get_shoot(img,filename):
    img = cv2.resize(img,IMAGE_SHAPE)
    cv2.imwrite(filename, img)

image_size = (900,900)
screen_size =(1920,1080)
window = pygame.display.set_mode(screen_size)
fe = feature_extractor()
with open('./camera_0_calibration.p', 'rb') as f:
    cfg = pickle.load(f)
pygame.init()
class client:
    def __init__(self,channel):
        self.video_stub = video_streaming_service_pb2_grpc.video_streamer_handlerStub(channel)
        self.robot_stub = robot_control_service_pb2_grpc.robot_control_handlerStub(channel)

    def get_sample(self):
        response = self.video_stub.send_image_chunk(video_streaming_service_pb2.Empty())
        img = sample(response.image,image_size,cfg)
        return img

    def event_loop(self,key):
        if key == "w":
            self.robot_stub.robot_move_forward(robot_control_service_pb2.Empty())
        if key == "s":
            self.robot_stub.robot_move_backward(robot_control_service_pb2.Empty())
        if key == "b":
            self.robot_stub.robot_break(robot_control_service_pb2.Empty())
        if key == "a":
            self.robot_stub.robot_turn_left(robot_control_service_pb2.Empty())
        if key == "d":
            self.robot_stub.robot_turn_right(robot_control_service_pb2.Empty())

def display_image(img,position):
    surf = pygame.surfarray.make_surface(img)
    surf = pygame.transform.rotate(surf, 270)
    window.blit(surf, position)
    pygame.display.update()

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
        print(tripoints3d[0])
    return img, tripoints3d, kpts,

def display_2d(img):
    img = img.get_image()
    img,tripoints3d,kpts =get_slam_points(img)
    try:
        img = np.zeros_like(img)
        if kpts != 0:
            for kpt in kpts:
                cv2.circle(img, (int(kpt.pt[0]), int(kpt.pt[1])), radius=2, color=(0,255,0), thickness=-1)
    except Exception as e:
        print(e)
    finally:
        display_image(img,(screen_size[0]//2,0))

def orb_displaying_thread_control(img):
    t = threading.Thread(target=display_orbs,args=(img,)) 
    t.start()
    t.join()

def display_2d_points_thread_control(img):
    t = threading.Thread(target=display_2d,args=(img,)) 
    t.start()
    t.join()

frames = []
def main():
    image_list = []
    detector_flag = False
    with grpc.insecure_channel('10.100.190.244:50052') as channel:
        client_handler = client(channel)
        running = True
        while running:
            img = client_handler.get_sample() 
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                   key = pygame.key.name(event.key)
                   if key == "q":
                       running = not running
                   else:
                       client_handler.event_loop(key)
            img.opencv_2_pygame()
            display_image(img.get_image(),(0,0))
            frames.append(img)
            display_2d_points_thread_control(img)
        pygame.quit()

main()
