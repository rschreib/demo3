import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')

from picamera import PiCamera
from bounds_test_copy import *
from beacon_manipulation import *
from determine_location import *
from robs_code import *

from pprint import pprint
import smbus

# Setup Bus
bus = smbus.SMBus(1)
address = 0x04

usingTerminal = False
try:
    color = sys.stdout.shell
except:
    usingTerminal = True


def run_it_all():
    valid = True

    resX = 1440
    resY = 816

    camera = PiCamera(resolution=(resX,resY))
    camera.iso = 50
    camera.shutter_speed = 1500

    f = 'led05.jpg'
    g = open('test.txt','w')

    keypoints = take_picture(camera, f)

    if len(keypoints) == 0:
        print 'No Beacons Found'
        return [False,[]]

    leds = find_beacons(keypoints)
    count = 0

    g.write('Number of beacons: ' + str(len(leds)) + '\n')

    if len(leds)>2:
        find_two = True
    else:
        find_two = False


    dist_from_center_one = 999
    dist_from_center_two = 999
    middle = resX/2
    
    beacons = []
    print leds

    for l in leds:
        middle_pixel = l[1][0]

        if len(l) == 2:
            l = sorted(l, key=lambda x:x[1])

        if find_two:
            dist_one = abs(middle-middle_pixel)
            if dist_one > dist_from_middle_one and dist_one >dist_from_middle_two:
                continue
            elif dist_one < dist_from_middle_one and dist_one > dist_from_middle_two:
                dist_from_middle_one = dist_one
            else:
                dist_from_middle_two = dist_one
            
        
        beacon = []
        g.write('#######################################################\n')
        count = count + 1
        length = len(l)
        if length <2 or length>3:
            leds.remove(l)
        
        g.write('Beacon ' + str(count) + ':\n')
        g.write('Number of LEDS: ' + str(length) + '\n')
        colors = []
        for k in l:
            g.write(str(k)+'\n')
            if k[2] == 0:
                colors.append('R')
            elif k[2] == 1:
                colors.append('B')
            elif k[2] == 2:
                colors.append('G')
        middle_pixel = l[1][0]
        height = get_height(l)
        print l
        dist_cm = get_distance(height,length, resY)
        #if colors == ['B','G','R']:
        #    dist_cm = dist_cm * .9

        
        dist_ft = cm_to_feet(dist_cm)
        angle = get_angle(middle_pixel, resX)
        hyp_cm = get_distance_from_LED(height, middle_pixel, dist_cm, angle)
        hyp_ft = cm_to_feet(hyp_cm)
        if len(colors)==2:
            colors.append('N')
    
        g.write('Height: ' + str(height) + ' Pixels\n')
        g.write('Distance in cm: ' + str(dist_cm) + ' cm\n')
        g.write('Distance in ft: ' + str(dist_ft) + ' ft\n')
        g.write('Angle: ' + str(angle) + 'degrees\n')
        g.write('Colors: ' + str(colors) +'\n')
        g.write('Hyp: ' + str(hyp_ft) + '\n')
        beacon.append(hyp_ft)
        beacon.append(angle)
        beacon.append(colors)
        beacons.append(beacon)

    if len(leds) < 2:
        print 'No Real Beacons'
        return [False,[]]
    

    g.close()

    return [valid,beacons]

v = [False, []]

while(v[0] == False):
    v = run_it_all()

DestinationPoints = [Point(0,3), Point(1,4), Point(5,2), Point(6,5)]
DestinationOrder = [1,2,3,4]
#m1, m2, m3, m4, a1, a2, a3, a4, pi1, pi2, pi3, pi4  = GetArduinoAnglesAndMagnitudes(B1color, B2color, B1hypotenuse, B2hypotenuse, B1angle, DestinationPoints, DestinationOrder)

#if (usingTerminal):
#    print("\n\t\t\tx%d\tx%d\tx%d\tx%d\t" % (DestinationOrder[0],DestinationOrder[1],DestinationOrder[2],DestinationOrder[3]))
#else:
#    color.write("\n\t\t\tx%d\tx%d\tx%d\tx%d\t\5n" % (DestinationOrder[0],DestinationOrder[1],DestinationOrder[2],DestinationOrder[3]),"KEYWORD")
#print("Magnitudes (Travel):\t%.1f\t%.1f\t%.1f\t%.1f" % (m1,m2,m3,m4)) # This Shows magnitude car must travel to get to this location
#print("Angles (Car Rotation):\t%.1f\t%.1f\t%.1f\t%.1f" % (a1,a2,a3,a4)) # This shows how much car must rotate to get to this location
#print("PI Angle:\t\t%.1f\t%.1f\t%.1f\t%.1f" % (pi1,pi2,pi3,pi4))


#if (usingTerminal):
#    print("\nCOLOR\tITEM\tLOCATION    M_FROM_CAR  ANGLE_FROM_CAR")
#else:
#    color.write("\nCOLOR\tITEM\tLOCATION    M_FROM_CAR  ANGLE_FROM_CAR\n","KEYWORD")

if not v[0]:
    print 'Failed'

elif len(v[1]) > 1:
    B1color = v[1][0][2]
    B1color = ''.join(B1color)
    B2color = v[1][1][2]
    B2color = ''.join(B2color)
    B1hypotenuse = v[1][0][0]
    B2hypotenuse = v[1][1][0]
    B1angle = v[1][0][1]
    #print B1color, B2color
    #print 'Beacon 1: ' + str(B1color) + ', ' + str(B1hypotenuse) + ',' + str(B1angle)
    #print 'Beacon 2: ' + str(B2color) + ', ' + str(B2hypotenuse)


    if (usingTerminal):
        pass
       #print("\nCOLOR\tITEM\tLOCATION    M_FROM_CAR  ANGLE_FROM_CAR")
    else:
        pass
        #color.write("\nCOLOR\tITEM\tLOCATION    M_FROM_CAR  ANGLE_FROM_CAR\n","KEYWORD")
    m1, m2, m3, m4, a1, a2, a3, a4, pi1, pi2, pi3, pi4  = GetArduinoAnglesAndMagnitudes(B1color, B2color, B1hypotenuse, B2hypotenuse, B1angle, DestinationPoints, DestinationOrder)

    g = open('output.txt','w')
    g.write(str(m1) + '\n' + str(m2) + '\n' + str(m3) + '\n' + str(m4) +'\n')
    g.write(str(a1) + '\n' + str(a2) + '\n' + str(a3) + '\n' + str(a4) +'\n')
    g.close()

else:
    print 'not enough beacons.. oops'

if (usingTerminal):
    pass
    #print("\n\t\t\tx%d\tx%d\tx%d\tx%d\t" % (DestinationOrder[0],DestinationOrder[1],DestinationOrder[2],DestinationOrder[3]))
else:
    pass
    color.write("\n\t\t\tx%d\tx%d\tx%d\tx%d\t\n" % (DestinationOrder[0],DestinationOrder[1],DestinationOrder[2],DestinationOrder[3]),"KEYWORD")
#print("Magnitudes (Travel):\t%.1f\t%.1f\t%.1f\t%.1f" % (m1,m2,m3,m4)) # This Shows magnitude car must travel to get to this location
#print("Angles (Car Rotation):\t%.1f\t%.1f\t%.1f\t%.1f" % (a1,a2,a3,a4)) # This shows how much car must rotate to get to this location
#print("PI Angle:\t\t%.1f\t%.1f\t%.1f\t%.1f" % (pi1,pi2,pi3,pi4))

distance1 = m1
angle1 = a1

distance2 = m2
angle2 = a2

distance3 = m3
angle3 = a3

distance4 = m4
angle4 = a4

angle1 = float(angle1)
if angle1 < 0:
    isNegative1 = 1
    angle1 *= -1
else:
    isNegative1 = 0
distance1 = float(distance1)

angle2 = float(angle2)
if angle2 < 0:
    isNegative2 = 1
    angle2 *= -1
else:
    isNegative2 = 0
distance2 = float(distance2)

angle3 = float(angle3)
if angle3 < 0:
    isNegative3 = 1
    angle3 *= -1
else:
    isNegative3 = 0
distance3 = float(distance3)

angle4 = float(angle4)
if angle4 < 0:
    isNegative4 = 1
    angle4 *= -1
else:
    isNegative4 = 0
distance4 = float(distance4)

angleWhole1 = int(angle1)
angleDecimal1 = int(100*(angle1 - angleWhole1))
distanceWhole1 = int(distance1)
distanceDecimal1 = int(100*(distance1 - distanceWhole1))

angleWhole2 = int(angle2)
angleDecimal2 = int(100*(angle2 - angleWhole2))
distanceWhole2 = int(distance2)
distanceDecimal2 = int(100*(distance2 - distanceWhole2))

angleWhole3 = int(angle3)
angleDecimal3 = int(100*(angle3 - angleWhole3))
distanceWhole3 = int(distance3)
distanceDecimal3 = int(100*(distance3 - distanceWhole3))

angleWhole4 = int(angle4)
angleDecimal4 = int(100*(angle4 - angleWhole4))
distanceWhole4 = int(distance4)
distanceDecimal4 = int(100*(distance4 - distanceWhole4))

values = [distanceWhole1, distanceDecimal1, angleWhole1, angleDecimal1, isNegative1,
          distanceWhole2, distanceDecimal2, angleWhole2, angleDecimal2, isNegative2,
          distanceWhole3, distanceDecimal3, angleWhole3, angleDecimal3, isNegative3,
          distanceWhole4, distanceDecimal4, angleWhole4, angleDecimal4, isNegative4]

def writeValues(values):
    bus.write_i2c_block_data(address, 0, values)

#while(True):
#    try:
writeValues(values)
time.sleep(1)
        #break
    #except:
       # print 'fail'
       # continue
    





