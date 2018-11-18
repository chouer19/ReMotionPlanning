import numpy as np
import sys

name = '/home/chouer/workspace/expr2/ReMotionPlanning/src/data/thirdWaypoints/2.waypoints'

f = open(name,'r')
line1 = f.readline()
line2 = f.readline()

while line2:
    contents = line1.split(',')
    x1,y1 = float(contents[0]), float(contents[1])
    contents = line2.split(',')
    x2,y2 = float(contents[0]), float(contents[1])
    dis = np.linalg.norm([x1-x2, y1-y2])
    if int(sys.argv[2]) == 1:
        if dis > int(sys.argv[1]):
            print(dis)
            print(x1,y1)
            print(x2,y2)
    if int(sys.argv[2]) == 2:
        if dis < float(sys.argv[1]):
            print(dis)
    line1 = line2
    line2 = f.readline()
