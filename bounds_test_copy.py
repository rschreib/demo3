import numpy as np
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import glob
import math
from determine_location import *
from beacon_manipulation import *

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

    lower_blue = np.array([105,50,114])
    upper_blue = np.array([125, 178, 255])
    mask_blue = cv2.inRange(hsv,lower_blue,upper_blue)

    lower_green = np.array([50,25,114])
    upper_green = np.array([80, 178, 255])
    mask_green = cv2.inRange(hsv,lower_green,upper_green)

    lower_white = np.array([10,0,230])
    upper_white = np.array([180,20,255])
    mask_white = cv2.inRange(hsv,lower_white,upper_white)

    kernal = np.ones((5,5),np.uint8)
    mask_red = cv2.dilate(mask_red,kernal,iterations=2)
    mask_blue = cv2.dilate(mask_blue,kernal,iterations=2)
    mask_green = cv2.dilate(mask_green,kernal,iterations=2)
    mask_white = cv2.dilate(mask_white,kernal, iterations = 1)
    
    params = cv2.SimpleBlobDetector_Params()

    params.minThreshold = 1;
    params.maxThreshold = 200;

    params.filterByArea = False
    params.minArea = .1

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
    cv2.imwrite('mask_white.jpg',mask_white)
    cv2.imwrite('mask.jpg', mask)

    detector = cv2.SimpleBlobDetector(params)
    keypoints_red = detector.detect(mask_red)
    keypoints_blue = detector.detect(mask_blue)
    keypoints_green = detector.detect(mask_green)
    keypoints_white = detector.detect(mask_white)
    keypoints = []

    print len(keypoints_white)

    try:
        print len(keypoints_red)
        for key in keypoints_red:
            print key.pt[0],key.pt[1]
            if key.pt[1] < 500:
                minX = 999
                minY = 999
                l = 0
                for k in keypoints_white:
                    if abs(key.pt[0] - k.pt[0]) < minX and abs(key.pt[1] - k.pt[1]):
                        minX = abs(key.pt[0] - k.pt[0])
                        minY = abs(key.pt[1] - k.pt[1])
                        l = k
                keypoints.append([l.pt[0], l.pt[1],0])
            
    except:
        print "Red LED not found"
    try:
        print len(keypoints_blue)
        for key in keypoints_blue:
            if key.pt[1] > 500:
                    keypoints_blue.remove(key)
                    continue
            print key.pt[0],key.pt[1]
            minX = 999
            minY = 999
            l = 0
            for k in keypoints_white:
                
                if abs(key.pt[0] - k.pt[0]) < minX and abs(key.pt[1] - k.pt[1]):
                    minX = abs(key.pt[0] - k.pt[0])
                    minY = abs(key.pt[1] - k.pt[1])
                    l = k
            keypoints.append([l.pt[0], l.pt[1],1])
    except:
        print 'Blue LED not found'
    try:
        print len(keypoints_green)
        for key in keypoints_green:
            print key.pt[0],key.pt[1]
            if key.pt[1] > 500:
                keypoints_green.remove(key)
                continue
            
            print key.pt[0],key.pt[1]
            minX = 999
            minY = 999
            l = 0
            for k in keypoints_white:
                if abs(key.pt[0] - k.pt[0]) < minX and abs(key.pt[1] - k.pt[1]):
                    minX = abs(key.pt[0] - k.pt[0])
                    minY = abs(key.pt[1] - k.pt[1])
                    l = k
            keypoints.append([l.pt[0], l.pt[1],2])
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

#f = 'led.jpg'
#keypoints = take_picture(camera, f)

#led = find_beacons(keypoints)

#for l in led:
#    print l



