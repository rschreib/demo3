import numpy as np
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import glob
import math
from determine_location import *

def take_picture(camera, f):
    rawCapture = PiRGBArray(camera)

    camera.capture(rawCapture, format = "bgr")
    img = rawCapture.array

    cv2.imwrite(f,img)

    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    lower_red_u = np.array([152,15,114])
    upper_red_u = np.array([175, 120, 255])
    mask_red_u = cv2.inRange(hsv,lower_red_u,upper_red_u)

    lower_red_l = np.array([0,10,114])
    upper_red_l = np.array([17,120,255])
    mask_red_l = cv2.inRange(hsv,lower_red_l,upper_red_l)

    mask_red = cv2.bitwise_or(mask_red_l, mask_red_u)

    lower_blue = np.array([98,30,114])
    upper_blue = np.array([125, 120, 255])
    mask_blue = cv2.inRange(hsv,lower_blue,upper_blue)

    lower_green = np.array([50,20,114])
    upper_green = np.array([80, 120, 255])
    mask_green = cv2.inRange(hsv,lower_green,upper_green)

    lower_white = np.array([100,0,230])
    upper_white = np.array([255,7,255])
    mask_white = cv2.inRange(hsv,lower_white,upper_white)

    kernal = np.ones((5,5),np.uint8)
    mask_red = cv2.dilate(mask_red,kernal,iterations=2)
    mask_blue = cv2.dilate(mask_blue,kernal,iterations=2)
    mask_green = cv2.dilate(mask_green,kernal,iterations=2)
    mask_white = cv2.dilate(mask_white,kernal, iterations = 2)
    
    params = cv2.SimpleBlobDetector_Params()

    params.minThreshold = 20;
    params.maxThreshold = 200;

    params.filterByArea = True
    params.minArea = 6

    params.filterByColor= False
    params.blobColor = 200

    params.filterByCircularity = False
    params.minCircularity = 0.1

    params.filterByConvexity = False
    params.minConvexity = 0.5

    params.filterByInertia = False
    params.minInertiaRatio = 0.01

    mask = cv2.bitwise_or(mask_red, mask_blue)
    mask = cv2.bitwise_or(mask, mask_green)

    img1 = cv2.resize(img,None,fx=.5,fy=.5,interpolation = cv2.INTER_CUBIC)
    mask = cv2.resize(mask,None,fx=.5,fy=.5,interpolation = cv2.INTER_CUBIC)
    #cv2.imshow('image',img1)
    #cv2.imshow('mask',mask)
    cv2.imwrite('mask.jpg',mask)

    detector = cv2.SimpleBlobDetector(params)
    keypoints_red = detector.detect(mask_red)
    keypoints_blue = detector.detect(mask_blue)
    keypoints_green = detector.detect(mask_green)
    keypoints_white = detector.detect(mask_white)
    keypoints = []

    print len(keypoints_white)
    
    #for key in keypoints_white:
     #   print key.pt[0], key.pt[1]

    try:
        for key in keypoints_red:
            keypoints.append([key.pt[0], key.pt[1],0])
    except:
        print "Red LED not found"
    try:
        for key in keypoints_blue:
            keypoints.append([key.pt[0], key.pt[1],1])
    except:
        print 'Blue LED not found'
    try:
        for key in keypoints_green:
            keypoints.append([key.pt[0], key.pt[1],2])
    except:
        print 'Green LED not found'

    return keypoints

def find_beacons(keypoints):
    leds = []

    keys = sorted(keypoints, key=lambda x: x[0])

    leds.append([keys[0]])
    count = 0
    for k in range(len(keys)-1):
        if abs(keys[k+1][0] - leds[count][0][0]) < 100:
            leds[count].append(keys[k+1])
        else:
            leds.append([keys[k+1]])
            count = count + 1
            
    return leds


#resX = 1440
#resY = 816

#camera = PiCamera(resolution=(resX,resY))
#camera.iso = 50
#camera.shutter_speed = 1500

#f = 'led05.jpg'
#keypoints = take_picture(camera, f)



