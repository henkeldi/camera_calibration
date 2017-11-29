# -*- coding: UTF-8 -*-
import cv2
import numpy as np
import pickle
import urllib
import progressbar
import os

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.0001)

objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:6,0:9].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

folder = '/media/hdd0/diplom/src/meshrenderer/calibration_img'

bar = progressbar.ProgressBar()

print 'Processing images ..'
for file in bar(os.listdir(folder)):
    img = cv2.imread(os.path.join(folder, file))
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    patternfound, corners = cv2.findChessboardCorners(gray, (6,9), None)

    # If found, add object points, image points (after refining them)
    if patternfound == True:
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners2)
        '''
        img = cv2.drawChessboardCorners(img, (6,9), corners2, patternfound)
        cv2.imshow('', img)
        cv2.waitKey(0)
        '''

print 10*'-'

ret2, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
with open('camera_calib.pickle','w') as f:
    pickle.dump((ret2, mtx, dist, rvecs, tvecs, objpoints, imgpoints), f)
#print rvecs
#print tvecs
#print ret2
print mtx
print dist