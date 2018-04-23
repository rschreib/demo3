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
#import smbus

# Resolution Values
resX = 1280
resY = 640

#finds distance (cm) from LEDs to piCamera
def get_distance(Height): #input height of LED triangle
    #Distance = -5.0/4.0*Height+160
    Distance = 96.2*.0625*float(resY)/Height 
    return Distance


#38, 21.5 (adjacent, opposite) (used to calculate camera angle)
#total degrees of range is -30 (left) +30 (right) or use -29.5 & +29.5
def get_angle(LED_pixel_X): #input the x value of one of the LED Pixels (or center of triangle pixel)
    pixel_middle_of_screen_X = resX/2.0
    max_camera_angle = 30.0
    #approximate angle
    angle = (LED_pixel_X - pixel_middle_of_screen_X) / pixel_middle_of_screen_X * max_camera_angle
    return -1*angle

#Inputs: LED triangle pixel Height, LED pixel location from left of screen
def get_distance_from_LED(Height, LED_pixel_X):
    angle = get_angle(LED_pixel_X)
    distance = get_distance(Height)
    hypotenuse = distance / math.cos(math.radians(angle))
    return hypotenuse

# Main function that captures and processes an image.
def detect(camera):

    # Capture Image
    rawCapture = PiRGBArray(camera)
    camera.capture(rawCapture, format="bgr")
    img = rawCapture.array

    # Save image
    cv2.imwrite('led.jpg',img)
    
    # Convert image to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # Bounds and mask initialization for thre colors
    lower_red_u = np.array([150,10,114])
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
    upper_green = np.array([85, 120, 255])
    mask_green = cv2.inRange(hsv,lower_green,upper_green)

    # Dilation of masks
    kernal = np.ones((5,5),np.uint8)
    mask_red = cv2.dilate(mask_red,kernal,iterations=2)
    mask_blue = cv2.dilate(mask_blue,kernal,iterations=2)
    mask_green = cv2.dilate(mask_green,kernal,iterations=2)

    # Initialization for blob detect
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

    
    # Combining the masks
    mask = cv2.bitwise_or(mask_red, mask_blue)
    mask = cv2.bitwise_or(mask, mask_green)

    # Detecting blobs for the three masks
    detector = cv2.SimpleBlobDetector(params)
    keypoints_red = detector.detect(mask_red)
    keypoints_blue = detector.detect(mask_blue)
    keypoints_green = detector.detect(mask_green)
    keypoints = []

    # Adding all keypoints from three masks into one list
    try:
        keypoints.append(keypoints_red[0])
    except:
        print "Red LED not found"
    try:
        keypoints.append(keypoints_blue[0])
    except:
        print 'Blue LED not found'
    try:
        keypoints.append(keypoints_green[0])
    except:
        print 'Green LED not found'

    # Try/Catch for testing
    if len(keypoints_red)>1:
        print len(keypoints_red), "red objects found"
    if len(keypoints_blue)>1:
        print len(keypoints_blue), "blue objects found"
    if len(keypoints_green)>1:
        print len(keypoints_green), "green objects found"
            
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

print 'Height:',keypoints[0].pt[1]-keypoints[2].pt[1]

# Call to angle and distance function
angle = get_angle(keypoints[2].pt[0]) #pixel 160 from left
distance = get_distance(keypoints[0].pt[1]-keypoints[2].pt[1]) #49.2 pixel height of LED triangle

print "Angle:",angle, 'degrees'
print "Distance:",distance, 'cm'

# Hypotenuse distance calculation
pixels = keypoints[2].pt[0]
pixel_height = keypoints[0].pt[1]-keypoints[2].pt[1]
hypotenuse = get_distance_from_LED(pixel_height, pixels)
print "Hypotenuse:",hypotenuse

# Pringint angle to LCD
lcd.message("Angle: %.2f" % (angle))
