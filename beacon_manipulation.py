import math

def get_distance(height, num, resY):
    distance = 0
    if num == 3 or num == 4:
        distance = 5.3*float(resY)/height
    elif num == 2:
        distance = 4.9 * float(resY)/height
    return distance

def get_height(beacon):
    return abs(beacon[0][1] - beacon[1][1])

def get_angle(pixel, resX):
    pixel_middle_of_screen_X = resX/2.0
    max_camera_angle = 30.0
    angle = (pixel - pixel_middle_of_screen_X)
    angle = angle / pixel_middle_of_screen_X
    angle = angle * max_camera_angle
    return angle * -1

def get_distance_from_LED(Height, LED_pixel_X,distance,angle):
    hypotenuse = distance / math.cos(math.radians(angle))
    return hypotenuse



        
        
