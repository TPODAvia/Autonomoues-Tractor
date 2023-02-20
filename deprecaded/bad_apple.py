import cv2 
import numpy as np 
from sample import sample 
import pickle
from feature_extractor import feature_extractor 
from utils import *
import matplotlib.pyplot as plt
import pygame
import numpy as np
import pandas as pd
import time
import pygame
import numpy as np
from random import randint
from pygame.locals import *
from OpenGL.GL import *
# from OpenGL.GLU import pi 
from OpenGL.GLUT import *
import sys
import pandas as pd


video_path = "Dataset.mp4"

cap = cv2.VideoCapture(video_path)
fe = feature_extractor()
coords = []
edges = []
points = []
z = 1
w = 1
whg =0.05 #Размер сетки
screen_size = (1000,1000)
def init_display_open_gl():
    window = pygame.display.set_mode(screen_size,DOUBLEBUF|OPENGL)
    gluPerspective(118, (screen_size[0]/screen_size[1]), 0.1, 150)

    glTranslatef(0,-z/3, -z*1.2)
    pygame.display.flip()
    pygame.key.set_repeat(1,16)
    return window

def init_display_2d():
    window = pygame.display.set_mode(screen_size)
    return window
with open('./camera_0_calibration.p', 'rb') as f:
    cfg = pickle.load(f)

def Grid(coord,points):
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            n3 = coord[vertex][2]/z*0.75
            # glColor3f(1, 1 , 1)
            glColor3f(n3,n3 , n3)
            glVertex3fv(coord[vertex]) 
    glEnd()
    glPointSize(2)
    glBegin(GL_POINTS)
    for point in points:
        glColor3f(1,0,0)
        glVertex3d(point[0],point[1],point[2])
    glEnd()

def init_grid():
    for i in range(0,int(w/whg)+1):
        coords.append((i*whg - w/2,0,0))
        coords.append((i*whg - w/2,0,z))
        edges.append((i*2, i*2+1))

    x_points_count = len(coords)

    for i in range(0,int(z/whg)+1):
        coords.append((-w/2,0,i*whg))
        coords.append((w/2,0,i*whg))
        edges.append((x_points_count+i*2,x_points_count+i*2+1))

def update_3d_grid(d3points):
    for i in d3points:
        points.append((i[0],i[1],i[2]))
    Grid(coords,d3points)
    pygame.display.flip()

def get_3d_points(tri,fname):
        if len(tri) > 0:
            array_to_project = np.array([0,0,0])

            x_points = [pt for pt in tri[0]]
            y_points = [-pt for pt in tri[1]]
            z_points = [-pt for pt in tri[2]]

            for i in range(tri.shape[1]):
                curr_array = np.array([x_points[i], y_points[i], z_points[i]])
                array_to_project = np.vstack((array_to_project, curr_array))

            array_to_project = array_to_project[1:, :]
           #df = pd.DataFrame(array_to_project,columns=["x","y","z"])
           #df.to_csv(f"./frames/frame_{fname}.csv")

            return array_to_project        

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

    return img, tripoints3d, kpts, matches


def display_image(img,position):
    surf = pygame.surfarray.make_surface(img)
    surf = pygame.transform.rotate(surf, 270)
    window.blit(surf, position)
    pygame.display.flip()
    pygame.display.update()

def convert_points(img):
    return np.asarray(np.vstack([img, np.ones(img.shape[1])]))

def display_2d(img,i):
    img = img.get_image()
    img,tripoints3d,kpts,matches =get_slam_points(img)
    p3 = get_3d_points(tripoints3d,i)
    print(len(kpts),np.array(p3).shape)
    df = pd.DataFrame(p3,columns=None)
    for col in list(df.columns):
        q_low = df[col].quantile(0.01)
        q_hi  = df[col].quantile(0.99)

        df = df[(df[col] < q_hi) & (df[col] > q_low)]
    p3 = (df-df.mean())/df.std()
    p3 = p3.values
    for i in p3:
        points.append((i[0],i[1],i[2]))
    try:
        if kpts != 0:
            for kpt in kpts:
                cv2.circle(img, (int(kpt.pt[0]), int(kpt.pt[1])), radius=2, color=(0,255,0), thickness=-1)
    except Exception as e: 
        print(e)
    finally:
        cv2.imwrite(f"./frames/frame_{i}.jpg",img)
        display_image(img,(0,0))
def get_matplot_3dsurface():
    x_p = []
    y_p = []
    z_p = []
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    step = 0.1
    for i in points:
        x_p.append(i[0]+step)
        y_p.append((i[1])*-1)
        z_p.append((i[2])*-1)
    ax.scatter(x_p,z_p,y_p,marker='o')
    fig.savefig('test.jpg')
    img = cv2.imread("test.jpg")
    img = cv2.imencode('.jpg', img)[1].tobytes()
    img = sample(img,(500,500),cfg) 
    display_image(img.get_image(),(screen_size[0]//2,0))
    
running = True
last = None
i = 0
window = init_display_2d()
init_grid()
while cap.isOpened():
    ret,frame = cap.read()
    if ret:
        frame = cv2.imencode('.jpg', frame)[1].tobytes()
        img = sample(frame,(500,500),cfg)
        print("Hello")
        try:
            display_2d(img,i)
            get_matplot_3dsurface()
            print("Hello1")
        except Exception as e:
            print(e)
            print("Hello2")
    for event in pygame.event.get():
        print("Hello3")
        if event.type == pygame.KEYDOWN:
            key = pygame.key.name(event.key)
            if key == "q":
                running = not running
            else:
                print(key)
