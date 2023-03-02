import cv2
import pickle
import numpy as np
import os

square_side_length_mm = 15.40
square_size_mm = square_side_length_mm * 4
square_size = square_size_mm / 1000 
num_intersections_in_x = 7
num_intersections_in_y = 7
obj_points = []
img_points = []

# Prepare expected object 3D object points (0,0,0), (1,0,0) ...
object_points = np.zeros((num_intersections_in_x*num_intersections_in_y,3), np.float32)
object_points[:,:2] = np.mgrid[0:num_intersections_in_x, 0:num_intersections_in_y].T.reshape(-1,2)
object_points = object_points*square_size

base_dir = "./cam_calib/samples"
bad_file_listing = os.listdir(base_dir)
file_listing = []
for i in bad_file_listing:
    if ".DS_Store" not in i:
        file_listing.append(i)

for i,file in enumerate(file_listing):
    image_path = os.path.join(base_dir,file)
    img = cv2.imread(image_path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img =cv2.resize(img,(1920,1080))
    img_size = (img.shape[1], img.shape[0])
    ret, corners = cv2.findChessboardCorners(img, (num_intersections_in_x, num_intersections_in_y), None)
    if ret:
        img_points.append(corners)
        obj_points.append(object_points)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, img_size, None, None)

dist_pickle = {}
dist_pickle["mtx"] = mtx
dist_pickle["dist"] = dist
pickle.dump(dist_pickle, open("/cam_calib/camera_0_calibration.p", "wb"))




    



    



