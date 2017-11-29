# -*- coding: utf-8 -*-
import os
import cv2
import urllib 
import numpy as np

stream=urllib.urlopen('http://172.16.1.4:8080/video')
bytes=''

img_path = 'calibration_img'

if not os.path.exists(img_path):
	os.makedirs(img_path)

i = 0

while True:
    bytes+=stream.read(1024)
    a = bytes.find('\xff\xd8')
    b = bytes.find('\xff\xd9')
    if a!=-1 and b!=-1:
        jpg = bytes[a:b+2]
        bytes= bytes[b+2:]
        img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.IMREAD_COLOR)
        cv2.imshow('img',img)
        k = cv2.waitKey(1)
        if k ==32:
        	img_save_path = os.path.join( img_path, '{:03d}.png'.format(i) )
        	print 'saving ', img_save_path
        	cv2.imwrite( img_save_path, img )
        	i += 1
        if k == 27:
            exit(0)