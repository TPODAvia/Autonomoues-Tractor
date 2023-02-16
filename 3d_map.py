import time
import pygame
import numpy as np
from random import randint
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import sys
import pandas as pd




file_path = r"./test.csv"

points =[]

df = pd.read_csv(file_path)
print(df)
df= (df-df.mean())/df.std()
for i in df.iloc():
    points.append((i.y,i.z,i.x))

print(points)    
z = 10
w = 10
whg =0.05 #Размер сетки
coord = []
edges = []
# for i in range(int(w/whg)):
#     for j in range(int(z/whg)):
#         coord.append((i*whg - w/2 ,0,j*whg))
#         if(j<int(z/whg)-1):
#             edges.append((i*z+j-z,i*z+j-z+1))
#         if(j<int(z/whg)):
#             edges.append((i*z+j-z,i*z+j))

for i in range(0,int(w/whg)+1):
    coord.append((i*whg - w/2,0,0))
    coord.append((i*whg - w/2,0,z))
    edges.append((i*2, i*2+1))

x_points_count = len(coord)

for i in range(0,int(z/whg)+1):
    coord.append((-w/2,0,i*whg))
    coord.append((w/2,0,i*whg))
    edges.append((x_points_count+i*2,x_points_count+i*2+1))

# coord.append((-w/2,0,z))
# coord.append((w/2,0,z))
# edges.append((len(coord)-2, len(coord)-1))


# points_n = 100
# h = 40
# points =[]

# for i in range(points_n):
#     points.append((randint(-w/2, w/2), randint(0, z) , randint(0, z)))



def Grid():
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




pygame.init()
display = (1920,1080)
pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
gluPerspective(118, (display[0]/display[1]), 0.1, 150)

glTranslatef(0,-z/3, -z*1.2)
Grid()
pygame.display.flip()
pygame.key.set_repeat(1,16)
while True:
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            key = pygame.key.name(event.key)
            print(key)
            if(key == "right"):
                glRotatef(0.4,0,1, 0)
            if(key == "left"):
                glRotatef(-0.4,0,1, 0)
            if(key == "up"):
                glTranslatef(0,0,0.1)
            if(key == "down"):
                glTranslatef(0,0,-0.1)
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()
    Grid()
    pygame.display.flip()
    
       
                

#      glRotatef(1,0,1, 0)

    
    # pygame.time.wait(10)


