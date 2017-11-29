# -*- coding: UTF-8 -*-
import cv2
import numpy as np
import pickle
import urllib

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((7*9,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:9].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

stream=urllib.urlopen('http://172.16.1.4:8080/video')

bytes = ''

while True:
    bytes+=stream.read(1024)
    a = bytes.find('\xff\xd8')
    b = bytes.find('\xff\xd9')
    if a!=-1 and b!=-1:
        jpg = bytes[a:b+2]
        bytes= bytes[b+2:]
        img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        
        # cv2.imshow('i',img)
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
            cv2.imshow('i',img)
        if cv2.waitKey(1) == 27:
            exit(0)   

while True:
        buf+=stream.read(1024)
        a = buf.find('\xff\xd8')
        b = buf.find('\xff\xd9')
        if a!=-1 and b!=-1:
            jpg = buf[a:b+2]
            buf= buf[b+2:]
            img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8).copy(),cv2.IMREAD_COLOR)
            cv2.imshow('i',img)
            continue
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
                break