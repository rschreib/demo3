# Necessary Imports
import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')

import numpy as np
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import glob
import math
import Adafruit_CharLCD as LCD
import struct
from determine_location import *
#import smbus

# Resolution Values
resX = 1920
resY = 1088

#finds distance (cm) from LEDs to piCamera
def get_distance(Height,num): #input height of LED triangle
    if num == 3:
        #Distance = -5.0/4.0*Height+160
        Distance = 5.2*float(resY)/Height 
        return Distance
    elif num == 2:
        distance = 4.9*float(resY)/Height
        return distance
    else: return 0


#38, 21.5 (adjacent, opposite) (used to calculate camera angle)
#total degrees of range is -30 (left) +30 (right) or use -29.5 & +29.5
def get_angle(LED_pixel_X): #input the x value of one of the LED Pixels (or center of triangle pixel)
    pixel_middle_of_screen_X = resX/2.0
    max_camera_angle = 30.0
    #approximate angle
    angle = (LED_pixel_X - pixel_middle_of_screen_X) / pixel_middle_of_screen_X * max_camera_angle
    return -1*angle

#Inputs: LED triangle pixel Height, LED pixel location from left of screen
def get_distance_from_LED(Height, LED_pixel_X,distance,angle):
    #angle = get_angle(LED_pixel_X)
    #distance = get_distance(Height)
    hypotenuse = distance / math.cos(math.radians(angle))
    return hypotenuse

def determine_beacon():
    return 0

# Main function that captures and processes an image.
def detect(camera):
    rawCapture = PiRGBArray(camera)

    beacons = [['G','R','B']]

    camera.capture(rawCapture, format="bgr")
    img = rawCapture.array

    cv2.imwrite('led05.jpg',img)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_red_u = np.array([152,15,114])
    upper_red_u = np.array([175, 120, 255])
    mask_red_u = cv2.inRange(hsv,lower_red_u,upper_red_u)

    lower_red_l = np.array([0,10,114])
    upper_red_l = np.array([17,120,255])
    mask_red_l = cv2.inRange(hsv,lower_red_l,upper_red_l)

    mask_red = cv2.bitwise_or(mask_red_l, mask_red_u)

    lower_blue = np.array([96,10,114])
    upper_blue = np.array([125, 120, 255])
    mask_blue = cv2.inRange(hsv,lower_blue,upper_blue)

    lower_green = np.array([50,10,114])
    upper_green = np.array([80, 120, 255])
    mask_green = cv2.inRange(hsv,lower_green,upper_green)

    kernal = np.ones((5,5),np.uint8)
    mask_red = cv2.dilate(mask_red,kernal,iterations=2)
    mask_blue = cv2.dilate(mask_blue,kernal,iterations=2)
    mask_green = cv2.dilate(mask_green,kernal,iterations=2)

    params = cv2.SimpleBlobDetector_Params()

    params.minThreshold = 20;
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

    detector = cv2.SimpleBlobDetector(params)
    keypoints_red = detector.detect(mask_red)
    keypoints_blue = detector.detect(mask_blue)
    keypoints_green = detector.detect(mask_green)
    keypoints = []
    
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
            

    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    return keypoints
    
# Initialize camera
camera = PiCamera(resolution=(resX,resY))
camera.iso = 50
camera.shutter_speed = 1500

# Initialize LCD Plate
lcd = LCD.Adafruit_CharLCDPlate()
time.sleep(1)

# Capture and process imiage
keypoints = detect(camera)

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

for l in leds:
    print l

print '# of Beacons:', len(leds)

count = 1
angles = []
hyps = []
for l in leds:
    #print '----- Beacon:',count, '-----'
    #print len(l)
    height = abs(l[0][1]-l[1][1])
    angle = get_angle(l[1][0])
    distance = get_distance(height, len(l))
    hypotenuse = get_distance_from_LED(height, l[1][0],distance,angle)

    #print 'Height:', height,'pixels'
    #print 'Angle:', angle,'degrees'
    #print 'Distance:',distance,'cm'
    #print 'Hypotenuse:', hypotenuse, 'cm'

    angles.append(angle)
    hyps.append(hypotenuse)

    count = count + 1
    
# Beacon 1 information
x1 = feet_to_cm(0.0)         # Given from Beacon in feet
y1 = feet_to_cm(0.0)         # Given from Beacon in feet
hypotenuse1 = hyps[0]        # Given from piCamera in cm
angle1 = angles[0]           # Given from piCamera in degrees

# Beacon 2 information
x2 = feet_to_cm(2.0)        # Given from Beacon in feet
y2 = feet_to_cm(0.0)        # Given from Beacon in feet
hypotenuse2 = hyps[1]       # Given from piCamera in cm

# Target Destination (use computer monitor coordinate graph system)
xT = feet_to_cm(2.0)
yT = feet_to_cm(3.0)

angleForArduino, magnitudeForArduino = Get_Angle_Distance_For_Arduino(x1,y1,hypotenuse1,angle1,x2,y2,hypotenuse2,xT,yT)

#print(angleForArduino)
#print(magnitudeForArduino)

#lcd.message("Angle: " + str(angle))
#lcd.message("Angle: %.2f" % (angle))
