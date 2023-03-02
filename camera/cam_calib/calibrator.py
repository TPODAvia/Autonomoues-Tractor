import cv2
import pickle
import numpy as np
import os

from . import sample

class calibrator:
    def __init__(self,path_to_images):

        self.square_side_length_mm = 15.40
        self.square_size_mm = self.square_side_length_mm * 4
        self.square_size = self.square_size_mm / 1000 
        self.num_intersections_in_x = 7
        self.num_intersections_in_y = 7
        self.obj_points = []
        self.img_points = []
        self.object_points = np.zeros((self.num_intersections_in_x*self.num_intersections_in_y,3), np.float32)
        self.object_points[:,:2] = np.mgrid[0:self.num_intersections_in_x, 0:self.num_intersections_in_y].T.reshape(-1,2)
        self.object_points = self.object_points*self.square_size
        self.base_dir = path_to_images
        self.file_listing = []

    # def prepare_files():
    #     bad_file_listing = os.listdir(self.base_dir)
    #     for i in bad_file_listing:
    #         if ".DS_Store" not in i:
    #             self.file_listing.append(i)

    def calibrate(self,path_to_save):
        for i,file in enumerate(self.file_listing):
            image_path = os.path.join(self.base_dir,file)
            img = cv2.imread(image_path)
            img_str = cv2.imencode('.jpg', img)[1].tostring()
            img = sample(img_str)
            img_size = (img.image.shape[1], img.image.shape[0])
            ret, corners = cv2.findChessboardCorners(img.image, (self.num_intersections_in_x, self.num_intersections_in_y), None)
            if ret:
                self.img_points.append(corners)
                self.obj_points.append(self.object_points)
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.obj_points, self.img_points, img_size, None, None)
        dist_pickle = {}
        dist_pickle["mtx"] = mtx
        dist_pickle["dist"] = dist
        pickle.dump(dist_pickle, open(path_to_save, "wb"))







    



    



