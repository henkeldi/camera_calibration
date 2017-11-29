# -*- coding: utf-8 -*-
import glob
import numpy as np
import urllib

import cv2
from cv2 import aruco
from gl_utils import tiles


img_path = 'calibration_img'
images = [cv2.imread(img_file_path) for img_file_path in glob.glob(img_path+'/*.png')]
image = images[0]

dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250) # DICT_ARUCO_ORIGINAL
#dictionary = aruco.Dictionary_create(nMarkers=36, markerSize=5);

markersX = 5
markersY = 7
markerLength = 0.04
markerSeparation=0.01

board = aruco.GridBoard_create(markersX, markersY, markerLength, markerSeparation, dictionary);

markerLength = 0.05 # meter
length = 0.1

def get_markers(dictionary, sidePixels):
    markers = []
    for id in xrange(len(dictionary.bytesList)):
        img = aruco.drawMarker(dictionary, id, sidePixels)
        markers.append(img)
    return np.array(markers)

#markers = get_markers(dictionary, 200)

cameraMatrix =np.array(\
    [[ 600.01501741,    0.        ,  318.69345836],
     [   0.        ,  599.38953918,  240.02065304],
     [   0.        ,    0.        ,    1.        ]])
distCoeff = np.array([[1.36034082e-01,  1.42879740e-01, 2.25706325e-03, -5.52548854e-04, -2.26230477e+00]])

stream=urllib.urlopen('http://172.16.1.4:8080/video')
bytes=''

while True:
    bytes+=stream.read(1024)
    a = bytes.find('\xff\xd8')
    b = bytes.find('\xff\xd9')
    if a!=-1 and b!=-1:
        jpg = bytes[a:b+2]
        bytes= bytes[b+2:]
        image = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.IMREAD_COLOR)
        
        markerCorners, markerIds, rejectedImgPoints = aruco.detectMarkers(image, dictionary, cameraMatrix=cameraMatrix, distCoeff=distCoeff)
        if len(markerCorners) > 0:
            image = aruco.drawDetectedMarkers(image, markerCorners, markerIds)
            
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(markerCorners, markerLength, cameraMatrix, distCoeff)
            for rvec, tvec in zip(rvecs, tvecs):
                image = aruco.drawAxis(image, cameraMatrix, distCoeff, rvec, tvec, length)
            '''
            
            valid, rvec, tvec = aruco.estimatePoseBoard(markerCorners, markerIds, board, cameraMatrix, distCoeff)
            if valid:
                image = aruco.drawAxis(image, cameraMatrix, distCoeff, rvec, tvec, length)
            '''
        else:
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(image,'No Markers Found',(30,250), font, 2, (255,255,255),2,cv2.LINE_AA)


        cv2.imshow('', image)
        cv2.waitKey(1)