import cv2
import numpy as np

class feature_extractor:
    def __init__(self):
        self.orb_extractor = cv2.ORB_create(nfeatures=1,scoreType = cv2.ORB_FAST_SCORE)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.last = None
        self.params = dict(algorithm=1,trees =5)
        self.flann = cv2.FlannBasedMatcher(self.params,dict())
        # self.sift = cv2.SIFT_create()

    def get_orb(self,img,kpts): 
        return self.orb_extractor.compute(img, kpts)


    def get_keypoints(self,pts):
        return [cv2.KeyPoint(p[0][0],p[0][1], size=30) for p in pts]
    
    def get_good_features_to_track(self,img):
        pts = None
        if len(img.shape) > 2:
            pts = cv2.goodFeaturesToTrack(image=np.mean(img, axis=2).astype(np.uint8), maxCorners=4500,qualityLevel=0.02, minDistance=3)
        else:
            pts = cv2.goodFeaturesToTrack(image=np.array(img).astype(np.uint8), maxCorners=4500,qualityLevel=0.02, minDistance=3)
        return pts

    def match_frames(self,img):
        pts = self.get_good_features_to_track(img)
        kpts = self.get_keypoints(pts)
        gkpts,des = self.get_orb(img,kpts)
        ret = []
        if self.last != None:
            matches = self.bf.knnMatch(des, self.last["des"], k=2)
            for m, n in matches:
                if m.distance < 0.55* n.distance:
                    if m.distance < 64:
                        k1 = kpts[m.queryIdx]
                        k2 = self.last["kpts"][m.trainIdx]
                        ret.append((k1,k2))
            c1 = np.asarray([kpts[m.queryIdx].pt for m,n in matches])
            c2 = np.asarray([self.last["kpts"][m.trainIdx].pt for m,n in matches])
            retval, mask = cv2.findHomography(c1, c2, cv2.RANSAC, 100.0)
            mask = mask.ravel()

            p1 = c1[mask==1]
            p2 = c2[mask==1]

            self.last = {"kpts":gkpts,"des":des}
            return p1.T, p2.T, kpts, ret
        else:
            self.last = {"kpts":gkpts,"des":des}
            return np.array([0]),np.array([0]), 0, 0

    def display_keypoints(self,img,kpts):
        img = cv2.drawKeypoints(img, kpts, None, color=(255,0,0), flags=0)
        return img


        
        
