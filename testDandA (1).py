import smbus
import time

bus = smbus.SMBus(1)

address = 0x04

distance = raw_input("Enter a distance in ft: ")
angle = raw_input("Enter an angle in rad: ")

angle = float(angle)
if angle < 0:
    isNegative = 1
    angle = angle * -1
else:
    isNegative = 0
distance = float(distance)

angleWhole = int(angle)
angleDecimal = int(100*(angle-angleWhole))

distanceWhole = int(distance)
distanceDecimal = int(100*(distance-distanceWhole))

values = [distanceWhole, distanceDecimal, angleWhole, angleDecimal, isNegative]

def writeValues(values):
    bus.write_i2c_block_data(address, 0, values)

writeValues(values)
time.sleep(1)
