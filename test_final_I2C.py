import smbus
import time

bus = smbus.SMBus(1)

address = 0x04

distance1 = raw_input("Enter a distance1 in ft: ")
angle1 = raw_input("Enter an angle1 in deg: ")

distance2 = raw_input("Enter a distance2 in ft: ")
angle2 = raw_input("Enter an angle2 in deg: ")

distance3 = raw_input("Enter a distance3 in ft: ")
angle3 = raw_input("Enter an angle3 in deg: ")

distance4 = raw_input("Enter a distance4 in ft: ")
angle4 = raw_input("Enter an angle4 in deg: ")

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

writeValues(values)
time.sleep(1)
