# -*- coding: UTF-8 -*-
import logging as log; import sys
log.basicConfig(stream=sys.stdout, format='[%(levelname)s]: %(message)s', level=log.INFO)
import cyglfw3 as glfw
from OpenGL.GL import *
from OpenGL.GL.NV.bindless_texture import *
import cv2
import thread
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import pickle

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((7*9,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:9].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

bridge = CvBridge()
k = -1
def callback(data):
    global k
    try:
        img = bridge.imgmsg_to_cv2(data, "passthrough")
        img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (7,9),None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            if k == 32:
                objpoints.append(objp)
                imgpoints.append(corners2)
                print '{} appended'.format(len(imgpoints))
            if len(imgpoints) == 14:
                ret2, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
                with open('camera_calib.pickle','w') as f:
                    pickle.dump((ret2, mtx, dist, rvecs, tvecs, objpoints, imgpoints), f)
                print ret2
                print mtx
                print dist
                print rvecs
                print tvecs
                rospy.signal_shutdown('Terminated.')
            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (7,9), corners2, ret)
        
        cv2.imshow('img',img)
        k = cv2.waitKey(1)
        if k == 27:
            rospy.signal_shutdown('Interrupted by user.')
    except CvBridgeError as e:
      print(e)

rospy.init_node('listener', anonymous=True, disable_signals=True)
rospy.Subscriber("/rgb/image", Image, callback, queue_size=1)
rospy.spin()    




