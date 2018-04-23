import math # Example: math.degrees(math.atan(math.sqrt(3)/1))
from pprint import pprint
import sys

usingTerminal = False
try:
    color = sys.stdout.shell
except:
    usingTerminal = True

# Functions
def magnitude(x, y):
    return math.sqrt(x*x + y*y)
def feet_to_cm(feet):
    return feet*12.0*2.54
def cm_to_feet(cm):
    return cm/12.0/2.54
class Point:
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)
    def __repr__(self):
        return("(%.2f,%.2f) ft" % (self.x,self.y))
class Beacon:
    global usingTerminal
    car = Point(0,0)
    def __init__(self, color, name, x, y, radius = 0, angle = 0):
        self.x = x                      # Location of Beacon on Grid
        self.y = y                      # Location of Beacon on Grid
        if radius == 0:
            self.radius = magnitude(self.car.x - x, self.car.y - y)     # distance from car
        else:
            self.radius = radius
        if angle == 0:
            self.angle = math.degrees(math.atan2(self.car.x - x, self.car.y - y))
        else:
            self.angle = angle
        self.color = color
        self.name = name
        self.highlight = False
    def __repr__(self):
        if self.name == 'Car':
            if (usingTerminal):
                return ("\033[1;35;40m%s\t%s\t(%.1f,%.1f) ft\t%.1f ft\t%.1f deg\033[0m\t\t<-CAR-" % (self.color,self.name,self.x,self.y,self.radius,self.angle))
            else:
                return ("%s\t%s\t(%.1f,%.1f) ft\t%.1f ft\t%.1f deg\t\t<-CAR-" % (self.color,self.name,self.x,self.y,self.radius,self.angle))
        elif self.highlight == True:
            if (usingTerminal):
                return ("\033[1;31;40m%s\t%s\t(%.1f,%.1f) ft\t%.1f ft\t%.1f deg\033[0m\t<-----" % (self.color,self.name,self.x,self.y,self.radius,self.angle))
            else:
                return ("%s\t%s\t(%.1f,%.1f) ft\t%.1f ft\t%.1f deg\t<-----" % (self.color,self.name,self.x,self.y,self.radius,self.angle))
        return ("%s\t%s\t(%.1f,%.1f) ft\t%.1f ft\t%.1f deg" % (self.color,self.name,self.x,self.y,self.radius,self.angle))
def get_angle_and_distance(p, offset_angle):
    angle = math.degrees(math.atan(p.y / (p.x + 0.000001))) - 90 + offset_angle
    if angle < -90: ##########################################################################################################################  Issues
        angle += 180
    elif angle > 90: #########################################################################################################################  Issues
        angle -= 180
    magnitude = math.sqrt(p.x**2 + p.y**2)
    return angle, magnitude
def distance_to_travel(car_Location, destination):
    dx = destination.x - car_Location.x
    dy = car_Location.y - destination.y
    return Point(dx, dy)
def Intersect_Points(b1, b2):
    # https://gamedev.stackexchange.com/questions/7172/haow-do-i-find-the-intersections-between-colliding-circles
    # Determines the points at which two circles intersect.
    # Based on an algorithm described by Paul Bourke:
    # http://local.wasp.uwa.edu.au/~pbourke/geometry/2circle/
    # Arguments:
    #   P0 (complex): the centre point of the first circle
    #   P1 (complex): the centre point of the second circle
    #   r0 (numeric): radius of the first circle
    #   r1 (numeric): radius of the second circle
    grid_y = feet_to_cm(20.0)

    P0 = complex(b1.x, (grid_y - b1.y))
    P1 = complex(b2.x, (grid_y - b2.y))
    r0 = b1.radius
    r1 = b2.radius

    # print r0, r1

    d = math.sqrt((P1.real - P0.real)**2 + (P1.imag - P0.imag)**2)   # d = distance, note: d = a + b
    a = (r0**2 - r1**2 + d**2) / (2 * d)
    b = d - a
    h = (r0**2 - a**2)**(0.5)
    P2 = P0 + a * (P1 - P0) / d
    i1x = P2.real + h * (P1.imag - P0.imag) / d
    i1y = P2.imag - h * (P1.real - P0.real) / d
    i2x = P2.real - h * (P1.imag - P0.imag) / d
    i2y = P2.imag + h * (P1.real - P0.real) / d

    # computer monitor graph (positive y-direction is down)
    car_location1 = Point(round(i1x), round(grid_y - i1y))
    car_location2 = Point(round(i2x), round(grid_y - i2y))
    car_location1 = Point((i1x), (grid_y - i1y))
    car_location2 = Point((i2x), (grid_y - i2y))
    # print("Circle Intersection:",car_location1, car_location2)
    if (car_location1.y > car_location2.y):
        return car_location1
    else:
        return car_location2
def Get_Angle_Distance_For_Arduino(beacon1,beacon2,b1_angleFromCar,xT,yT):
    b1_y = beacon1.y
    b1_x = beacon1.x

    car_Location = Intersect_Points(beacon1, beacon2)
    gridAngle = 90 - math.degrees(math.atan(float(car_Location.y - b1_y)/(float(b1_x - car_Location.x)+0.000001)))
    destination = Point(xT,yT)
    car_travel = distance_to_travel(car_Location, destination)
    angle_offset = gridAngle + b1_angleFromCar
    car_travel_angle, car_travel_distance = get_angle_and_distance(car_travel, angle_offset)#- imageAngle))

    if (destination.y > car_Location.y) and (destination.x > car_Location.x):
        car_travel_angle = car_travel_angle - 180

    Beacon.car = car_Location
    # print("\033[1;31;40m%s" % beacon1)
    # print("%s\033[0m" % beacon2)
    print("Car Location:\t{}".format(car_Location))
    # print("Destination:\t{}".format(destination))
    # print("Car Travel:\t{}".format(car_travel))
    #     # print("Grid angle:\t%.2f" % gridAngle)
    #     # print("Angle Offset:\t%.2f degrees" % angle_offset)
    # print("Angle:\t\t%.2f degrees" % car_travel_angle)
    # print("Magnitude:\t%.2f ft" % car_travel_distance)
    return car_travel_angle, car_travel_distance, destination
def highlight(BeaconArray,B1color,B2color):
    for i in range(len(BeaconArray)):
        color = BeaconArray[i].color
        if color == B1color or color == B2color:
            BeaconArray[i].highlight = True
    return(BeaconArray)
def NextAngle(prevAngCarIsFacing, cP, nP):
    dy = cP.y - nP.y
    dx = nP.x - cP.x
    angleCarWillbeFacing = math.degrees(math.atan2(dy,dx))
    ArduinoAngleToTurn = angleCarWillbeFacing - prevAngCarIsFacing
    return ArduinoAngleToTurn, angleCarWillbeFacing
def NextMag(cP, nP):
    dy = cP.y - nP.y
    dx = nP.x - cP.x
    return magnitude(dx,dy)
def GetArduinoAnglesAndMagnitudes(B1color,B2color,B1hypotenuse, B2hypotenuse, B1angle, DestinationPoints, DestinationOrder):
    # Given this
    ColorsArray = ['NNN','NNN','NNN','NNN','NNN','GRB','BRN','RGB','RBN','BRG','RGN','GBR','GRN','BGR','GBN','RBG','BGN']
    ColorsArray = ['NNN','NNN','NNN','NNN','NNN','RGB','BGR','NNN','NNN','NNN','NNN','NNN','NNN','NNN','NNN','NNN','NNN']
    BeaconNames = ['Car','x1','x2','x3','x4','B1','B2','B3','B4','B5','B6','B7','B8','B9','B10','B11','B12']
    # Xarray = [4,2,7,2,6,2.5,5,7.5,10,10,10,7.5,5,2.5,0,0,0] # has the x-coordinates of the car, x1(destination1), x2, x3, x4, B1, B2, ...
    # Yarray = [5,2,3,7,8,0,0,0,1.5,3.5,5.5,7,7,7,5.5,3.5,1.5] # has the y-coordinates of the car, x1(destination1), x2, x3, x4, B1, B2, ...

    x1 = DestinationPoints[0].x
    x2 = DestinationPoints[1].x
    x3 = DestinationPoints[2].x
    x4 = DestinationPoints[3].x
    y1 = DestinationPoints[0].y
    y2 = DestinationPoints[1].y
    y3 = DestinationPoints[2].y
    y4 = DestinationPoints[3].y

    Xarray = [4,x1,x2,x3,x4,2.5,5,7.5,10,10,10,7.5,5,2.5,0,0,0] # has the x-coordinates of the car, x1(destination1), x2, x3, x4, B1, B2, ...
    Xarray = [4,x1,x2,x3,x4,3,5,7.5,10,10,10,7.5,5,2.5,0,0,0] # has the x-coordinates of the car, x1(destination1), x2, x3, x4, B1, B2, ...

    Yarray = [5,y1,y2,y3,y4,0,0,0,1.5,3.5,5.5,7,7,7,5.5,3.5,1.5] # has the y-coordinates of the car, x1(destination1), x2, x3, x4, B1, B2, ...
    Yarray = [5,y1,y2,y3,y4,0,0,0,1.5,3.5,5.5,7,7,7,5.5,3.5,1.5] # has the y-coordinates of the car, x1(destination1), x2, x3, x4, B1, B2, ...
    BeaconArray = [] # will eveentually hold Beacon objects. Each Beacon object will have data from the above arrays

    # the two arrays below were for testing purposes
    # MagArray = [0,3.61,3.61,2.83,3.61,5.39,5.10,5.83,5.83,5.00,5.83,6.71,6.00,6.71,5.00,4.00,5.00] # Beacon Distance from Car
    # AngleArray = [0,33.7,-56.3,135.0,-146.3,21.8,-11.3,-31.0,-59.0,-90.0,-121.0,-153.4,180.0,153.4,126.9,90.0,53.1] # Beacon Angle from Car

    # Input from PiCamera
    # B1color = 'RGB'
    # B2color = 'RBG'

    # Use first 2 Beacons for Creating the Grid math
    indexB1 = ColorsArray.index(B1color)
    indexB2 = ColorsArray.index(B2color)

    x1 = Xarray[indexB1]
    y1 = Yarray[indexB1]
    B1name = BeaconNames[indexB1]
    # hypotenuse1 = MagArray[indexB1]
    # angle1 = AngleArray[indexB1]
    # hypotenuse1 = hyps [0]        # Given from piCamera in cm
    # angle1 = angles[0]           # Given from piCamera in degrees

    x2 = Xarray[indexB2]
    y2 = Yarray[indexB2]
    B2name = BeaconNames[indexB2]
    # hypotenuse2 = MagArray[indexB2]
    # angle2 = AngleArray[indexB2]  # Do not need this, can be deleted
    # hypotenuse2 = hyps[1]       # Given from piCamera in cm

    # beacon1 = Beacon(B1color,B1name,x1,y1,hypotenuse1,angle1)
    # beacon2 = Beacon(B2color,B2name,x2,y2,hypotenuse2,angle2)
    beacon1 = Beacon(B1color,B1name,x1,y1,B1hypotenuse,B1angle)
    beacon2 = Beacon(B2color,B2name,x2,y2,B2hypotenuse)

    # Target Destination
    xT = Xarray[DestinationOrder[0]]
    yT = Yarray[DestinationOrder[0]]
    xT2 = Xarray[DestinationOrder[1]]
    yT2 = Yarray[DestinationOrder[1]]
    xT3 = Xarray[DestinationOrder[2]]
    yT3 = Yarray[DestinationOrder[2]]
    xT4 = Xarray[DestinationOrder[3]]
    yT4 = Yarray[DestinationOrder[3]]

    angleForArduino, magnitudeForArduino, CarDestination = Get_Angle_Distance_For_Arduino(beacon1,beacon2,B1angle,xT,yT)

    Xarray[0] = Beacon.car.x
    Yarray[0] = Beacon.car.y
    for i in range(len(Xarray)):
        color = ColorsArray[i]
        name = BeaconNames[i]
        x = Xarray[i]
        y = Yarray[i]
        BeaconArray.append(Beacon(color,name,x,y))

    highlight(BeaconArray, B1color, B2color)
    #pprint(BeaconArray)

    point1 = Point(xT,yT)
    point2 = Point(xT2,yT2)
    point3 = Point(xT3,yT3)
    point4 = Point(xT4,yT4)

    arduinoAngleToTurn2, angleCarWillbeFacing2 = NextAngle((angleForArduino+90), point1, point2)
    arduinoAngleToTurn3, angleCarWillbeFacing3 = NextAngle((angleCarWillbeFacing2), point2, point3)
    arduinoAngleToTurn4, angleCarWillbeFacing4 = NextAngle((angleCarWillbeFacing3), point3, point4)

    magnitude2 = NextMag(point1, point2)
    magnitude3 = NextMag(point2, point3)
    magnitude4 = NextMag(point3, point4)

    a1 = angleForArduino
    a2 = arduinoAngleToTurn2
    a3 = arduinoAngleToTurn3
    a4 = arduinoAngleToTurn4

    a = [a1,a2,a3,a4]
    for i in range(len(a)):
        while(a[i] > 180):
            a[i] -= 360
        while(a[i] < -180):
            a[i] += 360

    return  magnitudeForArduino, magnitude2, magnitude3, magnitude4, a[0], a[1], a[2], a[3], angleForArduino+90, angleCarWillbeFacing2, angleCarWillbeFacing3, angleCarWillbeFacing4
# Green "STRING", red "COMMENT", orange "KEYWORD"


###############################################################################
# INSTRUCITONS:
###############################################################################
# 1) COPY THE CODE BELOW INTO JOHN'S FILE
# 2) USE THE COMMENTED-OUT IMPORTS

# from pprint import pprint
# import sys

# 3) PUT THIS AT THE TOP OF THE FILE IMMEDIATELY BELOW THE IMPORTS
usingTerminal = False
try:
    color = sys.stdout.shell
except:
    usingTerminal = True


# 4) PUT THE REST OF THIS CODE AT THE VERY BOTTOM OF JOHN'S FILE (RIGHT ABOVE THE I2C STUFF)
#if (usingTerminal):
#    print("\nCOLOR\tITEM\tLOCATION    M_FROM_CAR  ANGLE_FROM_CAR")
#else:
#    color.write("\nCOLOR\tITEM\tLOCATION    M_FROM_CAR  ANGLE_FROM_CAR\n","KEYWORD")

# EXAMPLE OF HOW TO CONVERT JOHN'S ARRAY TO A STRING or integer (comment this out after JOHN figures it out)
#a = ['6','5','a', 'b', 'c', 'd','e','f']
#a = ''.join(a)
#a1 = a[2:-3]    # abc
#a2 = a[:2]      # 65
#a3 = a[:-2]     # 65abcd
#a4 = a[2:]      # abcdef
#number1 = float(a[0]) # 6
#print("%s, %s, %s, %s, %.2f" % (a1,a2,a3,a4, number1))

# Inputs from JOHN (PiCamera data)
B1color = 'GRB'
B2color = 'BRN'
B1hypotenuse = 5.39     # Given from piCamera in cm *use cm_to_feet()
B2hypotenuse = 5.10     # Given from piCamera in cm *use cm_to_feet()
B1angle = 21.8          # Given from piCamera in degrees

# Parameters to  Test with (you can change these around but make sure to reference the google drive image)
#                       x1          x2          x3          x4
#DestinationPoints = [Point(2,2), Point(7,3), Point(2,7), Point(6,8)] #Location of Destination1, Destination2,  Destination3, Destination4

#You can customize this order EXAMPLE: [4,3,2,1] will travel to Point(6,8) then Point(2,7) then Point(7,3) then Point(2,2)
#DestinationOrder = [1,2,3,4]    # Look at the image in the shared google drive to understance the destination order

# i2c will only need m1, m2, m3, m4, a1, a2, a3, a4
#m1, m2, m3, m4, a1, a2, a3, a4, pi1, pi2, pi3, pi4  = GetArduinoAnglesAndMagnitudes(B1color, B2color, B1hypotenuse, B2hypotenuse, B1angle, DestinationPoints, DestinationOrder)

#if (usingTerminal):
#    print("\n\t\t\tx%d\tx%d\tx%d\tx%d\t" % (DestinationOrder[0],DestinationOrder[1],DestinationOrder[2],DestinationOrder[3]))
#else:
#    color.write("\n\t\t\tx%d\tx%d\tx%d\tx%d\t\n" % (DestinationOrder[0],DestinationOrder[1],DestinationOrder[2],DestinationOrder[3]),"KEYWORD")
#print("Magnitudes (Travel):\t%.1f\t%.1f\t%.1f\t%.1f" % (m1,m2,m3,m4)) # This Shows magnitude car must travel to get to this location
#print("Angles (Car Rotation):\t%.1f\t%.1f\t%.1f\t%.1f" % (a1,a2,a3,a4)) # This shows how much car must rotate to get to this location
#print("PI Angle:\t\t%.1f\t%.1f\t%.1f\t%.1f" % (pi1,pi2,pi3,pi4))  # This shows the Pi Circle angle the car will be pointing after going to that location with 0 degrees pointing EAST
