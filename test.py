#!/usr/bin/env python3
import serial
import scipy
from scipy.optimize import least_squares
x1,y1,z1 =0,0,0
x2,y2,z2 =5,0,0
x3,y3,z3 =10,-3.5,0
dist1 = 0
dist2 = 0
dist3 = 0
count = 0
ser = serial.Serial(port='/dev/ttyACM0', baudrate = 115200)
def equations( guess ):
    x,y,z,r = guess
    return((x - x1)**2 + (y - y1)**2 + (z - z1)**2 - (dist1 - r )**2,
        (x - x2)**2 + (y - y2)**2 + (z - z2)**2 - (dist2 - r )**2,
        (x - x3)**2 + (y - y3)**2 + (z - z3)**2 - (dist3 - r )**2)
while count<= 10000:
    data = ser.readline()
    print(data)
    if "B1" in str(data):
        dist1 = float(str(data)[7:15])
    if "B2" in str(data):
        dist2 = float(str(data)[7:15])
    if "B3" in str(data):
        dist3 = float(str(data)[7:15])
    count = count+1
    if 0 not in (dist1,dist2,dist3):
        results = least_squares(equations,(0,-1,0,0))
        print("est pos:" + str(results.x))
    print(count)
ser.close()
