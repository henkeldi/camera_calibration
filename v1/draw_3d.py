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

bridge = CvBridge()
with open('camera_calib.pickle','r') as f:
    ret, mtx, dist, rvecs, tvecs, objpoints, imgpoints = pickle.load(f)

once = True
newcameramtx = None; roi = None
mapx, mapy = None, None

tot_error = 0
for i in xrange(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    tot_error += error

print "total error: ", tot_error/len(objpoints)

cube = False


def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img

def drawCube(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1,2)
    # draw ground floor in green
    img = cv2.drawContours(img, [imgpts[:4]],-1,(0,255,0),-3)
    # draw pillars in blue color
    for i,j in zip(range(4),range(4,8)):
        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)
    # draw top layer in red color
    img = cv2.drawContours(img, [imgpts[4:]],-1,(0,0,255),3)
    return img

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((9*7, 1, 3), np.float32) 
objp[:,:,:2] = np.mgrid[0:7,  0:9].T.reshape(-1,1,2)

def callback(data):
    global k, once, newcameramtx, roi, mapx, mapy
    try:
        img = bridge.imgmsg_to_cv2(data, "passthrough")
        img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        if once:
            h,  w = img.shape[:2]
            #newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
            print 'Matrix = '
            print mtx
            #print 'Optimal Matrix = ', newcameramtx
            #mapx, mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
            once = False
            return
        #dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (7,9), cv2.CALIB_CB_FAST_CHECK)

        if ret == True:
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            # img = cv2.drawChessboardCorners(img, (7,9), corners2, ret)
            # Find the rotation and translation vectors.
            ret, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)
            
            T = np.eye(4,dtype=np.float32)
            T[:3,:3] = cv2.Rodrigues(rvecs)[0]
            T[:3,3] = tvecs[:,0]
            print 'Mat = '
            print T

            #print rvecs
            #print tvecs

            # ret,rvecs, tvecs, inliers = cv2.solvePnP(objp, corners2, mtx, dist)
            # project 3D points to image plane
            
            if cube:
                axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
                       [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])
                imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
                img = drawCube(img,corners2,imgpts)
            else:
                axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)  
                imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
                print 'PTS: '
                print imgpts
                img = draw(img,corners2,imgpts)

        cv2.imshow('scene', img)
        k = cv2.waitKey(1)
        if k == 27:
            cv2.destroyAllWindows()
            rospy.signal_shutdown('Interrupted by user.')
    except CvBridgeError as e:
      print(e)

rospy.init_node('listener', anonymous=True, disable_signals=True)
rospy.Subscriber("/rgb/image", Image, callback, queue_size=1)
rospy.spin()    




