import numpy as np
import cv2
"""
class for image processing with methods making image good for system
    - parse from byte string for telemetry
    - grey scale
    - grey tresholding
    - resize
    - recolor for sdl2
    - process for slam detector

"""
class sample:
    def __init__(self,buff,image_size,cfg):
        self.cfg = cfg
        self.image_size = image_size
        self.image = self.process_image(self.decode_from_bytes(buff))
        #self.undistort_frame()
        self.opencv_2_pygame()

    def decode_from_bytes(self,image_in_bytes):
        buff = np.fromstring(image_in_bytes, np.uint8)
        buff = buff.reshape(1, -1)
        img = cv2.imdecode(buff, cv2.IMREAD_COLOR)        
        return img
    def image_rotate_custom_angle(self,angle,scale):
        width,height = self.image_size
        image_center = (width / 2,height / 2)
        rotation_matrix = cv2.getRotationMatrix2D(image_center, angle, scale)
        return cv2.warpAffine(self.image, rotation_matrix, (height, width))

    def opencv_2_pygame(self):
        self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
    def undistort_frame(self):
        self.image = cv2.undistort(self.image, self.cfg['mtx'], self.cfg['dist'], None, self.cfg['mtx'])


    def process_image(self,img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)        
        img = cv2.resize(img,self.image_size)        
        return img
    def get_image(self):
        return self.image

    
