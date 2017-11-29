# -*- coding: utf-8 -*-
import numpy as np
import cv2
import os
import progressbar

import experiment_management as exm

experiment_name = 'fast_patches2'
cfg_file = '/media/hdd0/diplom/src/experiment_management/bin/log/{name}/{name}.cfg'.format(name=experiment_name)
report_dir = 'report'
log_dir = 'log'
em = exm.ExperimentManagement(cfg_file, experiment_name, report_dir, log_dir, all_mode=False)
em.try_reload_checkpoint()

img_path = 'corner_imgs'

fast = cv2.FastFeatureDetector_create(threshold=5)

patches = []

patch_size = 11

matcher = cv2.BFMatcher_create()
bar = progressbar.ProgressBar()

files = sorted(os.listdir(img_path))[20:]

def compute(img, kp):
    patches = []
    for k in kp:
        x, y = k.pt
        left = int(x - int(patch_size / 2.0))
        right = int(x + int(np.ceil(patch_size / 2.0)))
        top = int(y - int(patch_size / 2.0))
        bottom = int(y + int(np.ceil(patch_size / 2.0)))
        if left >= 0 and right < W and top >= 0 and bottom < H:
            patch = img2[top:bottom, left:right]
        else:
            patch = np.zeros((patch_size, patch_size, 3))
        patches.append(patch.astype(np.float32) / 255.0)
    patches = np.array(patches)

    descr = em.project(patches)

    return kp, descr


for file1, file2 in bar(zip(files[:-1], files[1:])):
    img1 = cv2.imread(os.path.join(img_path, file1))
    img2 = cv2.imread(os.path.join(img_path, file2))
    H, W = img1.shape[:2]
    kp1 = fast.detect(img1, None)
    kp2 = fast.detect(img2, None)

    kp1, d1 = compute(img1, kp1)
    kp2, d2 = compute(img2, kp2)

    matches = matcher.knnMatch(d1, d2, k=2)
    #img3 = cv2.drawMatchesKnn(img1, kp1, img2, kp2, matches, img3, flags=2)
    
    good = []
    for m,n in matches:
        if m.distance < 0.06*n.distance:
            print n.distance
            good.append(m)

    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

    matchesMask = mask.ravel().tolist()

    h,w = img1.shape[:2]
    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    dst = cv2.perspectiveTransform(pts,M)

    img2 = cv2.polylines(img2, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)

    draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                       singlePointColor = None,
                       matchesMask = matchesMask, # draw only inliers
                       flags = 2)

    # img3 = np.empty_like(img1)
    img3 = cv2.drawMatches(img1, kp1, img2, kp2, good, None, **draw_params)
    # img3 = cv2.drawMatchesKnn(img1, kp1, img2, kp2, good, img3, flags=2)
    
    i1 = np.empty_like(img1)
    cv2.drawKeypoints(img1, kp1, i1, color=(102,32,0))
    i2 = np.empty_like(img2)
    cv2.drawKeypoints(img2, kp2, i2, color=(102,32,0))
 
    cv2.imshow('', img3)
    cv2.imshow('1', i1)
    cv2.imshow('2', i2)

    k = cv2.waitKey(0)
    if k == 27:
        break


'''
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
    matchesMask = mask.ravel().tolist()

    h,w = img1.shape
    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    dst = cv2.perspectiveTransform(pts,M)

    img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)
'''