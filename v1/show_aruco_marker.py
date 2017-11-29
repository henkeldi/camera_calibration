# -*- coding: utf-8 -*-
import glob
import numpy as np
import urllib

import cv2
from cv2 import aruco
from gl_utils import tiles

dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250) # DICT_ARUCO_ORIGINAL

def get_markers(dictionary, sidePixels):
    markers = []
    for id in xrange(len(dictionary.bytesList)):
        img = aruco.drawMarker(dictionary, id, sidePixels)
        markers.append(img)
    return np.array(markers)

markers = get_markers(dictionary, 200)

for marker in markers:
	cv2.imshow('', marker)
	cv2.waitKey(0)