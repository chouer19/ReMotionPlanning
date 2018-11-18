

import os
import sys
import random
import numpy as np
import math
import matplotlib.pyplot as plt
import pygame
import time
from scipy import interpolate


class Waypoint:
    def __init__(self,x = 0, y = 0, fine = 0):
        self._x = round(x,2)
        self._y = round(y,2)
        self._fine = round(fine,2)

    def X(self, x=None):
        if x is None:
            return self._x
        self._x = x

    def Y(self, y=None):
        if y is None:
            return self._y
        self._y =y 

    def Fine(self, fine=None):
        if fine is None:
            return self._fine
        self._fine = fine

    def getWaypoint(self):
        return (self._x, self._y, self._fine)

    def toStr(self):
        return str(self.X()) + ',' + str(self.Y()) + ',' + str(self.Fine()) + '\n'

    def info(self):
        print("********************waypoint info******************")
        print("\tX:" + str(self.X()))
        print("\tY:" + str(self.Y()))
        print("\tFine:" + str(self.Fine()))
        print("")

class Node:
    def __init__(self,ID = 0, x = 0, y = 0, ID1 = 999, ID2 = 999, ID3 = 999):
        self._id = ID
        self._x = x
        self._y = y
        self._idd = []
        if ID1 != 999:
            self._idd.append(ID1)
        if ID2 != 999:
            self._idd.append(ID2)
        if ID3 != 999:
            self._idd.append(ID3)
        pass

    def ID(self,ID=None):
        if ID is None:
            return self._id
        self._id = ID

    def X(self, x=None):
        if x is None:
            return self._x
        self._x = x

    def Y(self, y=None):
        if y is None:
            return self._y
        self._y =y 

    def IDD(self, idd=None):
        if idd is None:
            return self._idd
        self._idd = idd

    def getNode(self):
        return (self.ID(), self.X(), self.Y(), self.IDD())

    def info(self):
        print("==============node info===================")
        print("\tID : %d" % (self.ID()))
        print("\t(X,Y) : (%d, %d)" % (self.X(),(self.Y()) ) )
        print("\tIDD : "+str(self.IDD()))
        print("")

def angBetweenVector(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'    """
    cosang = np.dot(v1, v2)
    sinang = np.linalg.norm(np.cross(v1, v2))
    if np.cross(v1,v2) < 0:
        return -np.arctan2(sinang, cosang) * 180 / math.pi
    return np.arctan2(sinang, cosang) * 180 / math.pi


def getWaypoints(out):
    point = Waypoint(1,2,3)
    waypoints = []
    x0 = out[0][0]
    y0 = out[1][0]
    x1 = out[0][1]
    y1 = out[1][1]
    waypoints.append(Waypoint(x0,y0, angBetweenVector([x0,y0],[x1,y1])))
    #waypoints[0].info()
    for i in range(1, np.shape(out)[1] - 1):
        waypoints.append(Waypoint(out[0][i], out[1][i], \
                angBetweenVector([1,0],[out[0][i+1]-out[0][i-1], out[1][i+1]-out[1][i-1]])))
        #print(angBetweenVector([1,0],[out[0][i+1]-out[0][i-1], out[1][i+1]-out[1][i-1]]))
    return waypoints
    pass

def getSparse(n0,n1):
    distance = np.linalg.norm([n1.X()-n0.X(), n1.Y()-n0.Y()])
    sparseNum = math.floor(distance / 10) - 1
    if sparseNum <= 0:
        return [n0.X()] , [n0.Y()]
    
    #waypoints = []
    Xs = []
    Ys = []

    for i in range(0,int(sparseNum)):
        X = n0.X() + i/sparseNum * (n1.X() - n0.X()) + round(np.random.random() * 4 - 2,2)
        Y = n0.Y() + i/sparseNum * (n1.Y() - n0.Y()) + round(np.random.random() * 4 - 2,2)
        Xs = Xs + [X]
        Ys = Ys + [Y]
    return Xs,Ys
    pass

def getDense(nodes):
    Xs = []
    Ys = []
    distance = 0
    for i in range(0,len(nodes)-1):
        X,Y = getSparse(nodes[i], nodes[i+1])
        Xs += X
        Ys += Y
        distance += np.linalg.norm([nodes[i].X() - nodes[i+1].X(), nodes[i].Y() - nodes[i].Y()])
    interNum = int( (distance) * 2 ) + 1
    tck,u = interpolate.splprep([Xs,Ys],k=2,s=0)
    u=np.linspace(0,1,num=interNum,endpoint=True)
    out = interpolate.splev(u,tck)
    return getWaypoints(out)
    pass

def main(args):
    #render = Render(500, 500)
    ### topu map file
    waypoints = []
    # start point
    point0 = Waypoint(14,2)
    point1 = Waypoint(22,0)
    point2 = Waypoint(78,0)
    Xs, Ys = getSparse(point1, point2)
    Xs = [point0.X()] + Xs + [point2.X()]
    Ys = [point0.Y()] + Ys + [point2.Y()]
    tck,u = interpolate.splprep([Xs,Ys],k=2,s=0)
    u=np.linspace(0,1,num=130,endpoint=True)
    out = interpolate.splev(u,tck)
    #plt.figure()
    #plt.plot(Xs, Ys, 'ro', out[0], out[1], 'b')
    #plt.legend(['Points', 'Interpolated B-spline', 'True'],loc='best')
    ##plt.axis([min(x)-1, max(x)+1, min(y)-1, max(y)+1])
    #plt.title('B-Spline interpolation')
    #plt.xlim(-3,3)
    #plt.axis("equal")
    #plt.grid(True)
    #plt.show()


    waypoints = getWaypoints(out)
    # start node point included, but not end node point
    startFile = '/home/chouer/workspace/expr2/ReMotionPlanning/src/data/thirdWaypoints/start.waypoints'
    f = open(startFile,'w')
    for point in waypoints:
        f.write(point.toStr())
    f.close()
    
if __name__ == "__main__":
    main(sys.argv[1:])
