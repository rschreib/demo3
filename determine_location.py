import math # math.degrees(math.atan(math.sqrt(3)/1))
def magnitude(x, y):
    return math.sqrt(x*x + y*y)
def feet_to_cm(feet):
    return feet*12.0*2.54
def cm_to_feet(cm):
    return cm/12.0/2.54
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __repr__(self):
        return "(x,y) = (%.2f,%.2f)cm   \t(%.2f,%.2f)ft" % (self.x,self.y,cm_to_feet(self.x),cm_to_feet(self.y))
class Beacon:
    def __init__(self,x, y, radius):
        self.x = x                      # Location of Beacon on Grid
        self.y = y                      # Location of Beacon on Grid
        self.radius = radius            # distance from car
    def __repr__(self):
        return "(x,y) = (%.2f,%.2f)cm   \t(%.2f,%.2f)ft" % (self.x,self.y,cm_to_feet(self.x),cm_to_feet(self.y))
def get_angle_and_distance(p, offset_angle):
    angle = math.degrees(math.atan(p.y / p.x)) - 90 + offset_angle
    if angle < -90:
        angle += 180
    elif angle > 90:
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

    print r0, r1

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
    car_location1 = Point(i1x, (grid_y - i1y))
    car_location2 = Point(i2x, (grid_y - i2y))
    # print("Circle Intersection:",car_location1, car_location2)
    if (car_location1.y > car_location2.y):
        return car_location1
    else:
        return car_location2

def Get_Angle_Distance_For_Arduino(b1_x,b1_y,b1_distanceFromCar,b1_angleFromCar,b2_x,b2_y,b2_distanceFromCar,xT,yT):

    beacon1 = Beacon(b1_x, b1_y, b1_distanceFromCar)
    beacon2 = Beacon(b2_x, b2_y, b2_distanceFromCar)
    car_Location = Intersect_Points(beacon1, beacon2)
    gridAngle = 90 - math.degrees(math.atan(float(car_Location.y - b1_y)/float(b1_x - car_Location.x)))
    destination = Point(xT,yT)
    car_travel = distance_to_travel(car_Location, destination)
    angle_offset = gridAngle + b1_angleFromCar
    car_travel_angle, car_travel_distance = get_angle_and_distance(car_travel, angle_offset)#- imageAngle))

    if (destination.y > car_Location.y) and (destination.x > car_Location.x):
        car_travel_angle = car_travel_angle + 180

    print("Beacon 1:\t{}".format(beacon1))
    print("Beacon 2:\t{}".format(beacon2))
    print("Car Location:\t{}".format(car_Location))
    print("Destination:\t{}".format(destination))
    print("Car Travel:\t{}".format(car_travel))
    # print("Grid angle:\t%.2f" % gridAngle)
    # print("Angle Offset:\t%.2f degrees" % angle_offset)
    print("Angle: %.2f (%.3f)\tMagnitude: %.2f cm\t%.2f ft" % (car_travel_angle,math.radians(car_travel_angle),car_travel_distance,cm_to_feet(car_travel_distance)))

    return car_travel_angle, car_travel_distance


