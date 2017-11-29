# -*- coding: utf-8 -*-
import numpy as np

import cv2
from cv2 import aruco
import urllib


dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
# dictionary = aruco.Dictionary_create(nMarkers=36, markerSize=5)

markersX = 5
markersY = 7
markerLength = 0.04
markerSeparation=0.01

board = aruco.GridBoard_create(markersX, markersY, markerLength, markerSeparation, dictionary)

markerLength = 0.05 # meter
length = 0.1

cameraMatrix =np.array(\
    [[ 600.01501741,    0.        ,  318.69345836],
     [   0.        ,  599.38953918,  240.02065304],
     [   0.        ,    0.        ,    1.        ]])
distCoeff = np.array([[1.36034082e-01,  1.42879740e-01, 2.25706325e-03, -5.52548854e-04, -2.26230477e+00]])

for image in images:
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