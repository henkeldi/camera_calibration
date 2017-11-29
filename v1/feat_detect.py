# -*- coding: utf-8 -*-
import numpy as np
import cv2
import os
import progressbar

img_path = 'corner_imgs'

fast = cv2.FastFeatureDetector_create(threshold=25)

patches = []

patch_size = 11

descriptor = cv2.BRISK_create()

matcher = cv2.BFMatcher_create()

bar = progressbar.ProgressBar()

for file in bar(sorted(os.listdir(img_path))[20:]):
    img = cv2.imread(os.path.join(img_path, file))
    H, W = img.shape[:2]
    kp = fast.detect(img, None)
    #img2 = np.empty_like(img)
    #cv2.drawKeypoints(img, kp, img2, color=(102,32,0))
    for k in kp:
        x, y = k.pt
        left = int(x - int(patch_size / 2.0))
        right = int(x + int(np.ceil(patch_size / 2.0)))
        top = int(y - int(patch_size / 2.0))
        bottom = int(y + int(np.ceil(patch_size / 2.0)))
        if left >= 0 and right < W and top >= 0 and bottom < H:
            patch = img[top:bottom, left:right]
            patches.append(patch.astype(np.float32) /255.0 )
    k1, d1 = descriptor.compute(img, kp)

patches = np.array(patches)
print patches.shape
print patches.mean()
np.save('patches.npy', patches)

'''
# -*- coding: utf-8 -*-
import os
import cv2
import urllib 
import numpy as np

stream=urllib.urlopen('http://172.16.1.4:8080/video')
bytes=''


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
        img_save_path = os.path.join( img_path, '{:09d}.png'.format(i) )
        cv2.imshow('img',img)
        cv2.imwrite( img_save_path, img )
        k = cv2.waitKey(1)
        i += 1
'''