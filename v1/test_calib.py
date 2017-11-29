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

objp = np.zeros((7*9,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:9].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

bridge = CvBridge()
with open('camera_calib.pickle','r') as f:
    ret, mtx, dist, rvecs, tvecs, objpoints, imgpoints = pickle.load(f)

once = True
newcameramtx = None; roi = None
mapx, mapy = None, None

mean_error = 0
for i in xrange(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error

print "total error: ", mean_error/len(objpoints)

print 'Matrix'
print mtx

def callback(data):
    global k, once, newcameramtx, roi, mapx, mapy
    try:
        img = bridge.imgmsg_to_cv2(data, "passthrough")
        img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        if once:
            h,  w = img.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
            mapx, mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
            once = False
            return
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        dstRemap = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)
        # crop the image
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]

        cv2.imshow('img', img)
        cv2.imshow('undistorted', dst)
        cv2.imshow('remap undistorted', dstRemap)
        k = cv2.waitKey(1)
        if k == 27:
            rospy.signal_shutdown('Interrupted by user.')
    except CvBridgeError as e:
      print(e)

rospy.init_node('listener', anonymous=True, disable_signals=True)
rospy.Subscriber("/rgb/image", Image, callback, queue_size=1)
rospy.spin()    




