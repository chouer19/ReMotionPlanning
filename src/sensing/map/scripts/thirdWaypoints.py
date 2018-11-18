import os
import sys
import random
import numpy as np
import math
import matplotlib.pyplot as plt
import pygame
import time
from scipy import interpolate

BLACK = (0,0,0)
WHITE = (255,255,255)
RED = (255,0,0)
DARKPINK = (255,20,147)
DARKRED = (138,0,0)
PURPLE = (160,32,240)
YELLOW = (255,255,0)
GREEN = (00,255,0)
BLUE = (0,0,255)
LIGHTBLUE = (176,226,255)
ORANGE = (139,69,0)
class Render:
    def __init__(self, WIDTH = 400, HEIGHT = 400, OFFSETX = 20, OFFSETY = 20, KX = 1, KY = 1):
        pygame.init()
        self._width = WIDTH
        self._height = HEIGHT
        self._offsetX = OFFSETX
        self._offsetY = OFFSETY
        self._kX = KX
        self._kY = KY
        self._display = pygame.display.set_mode( (WIDTH, HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)

    def renderWaypoints(self,waypoints, num):
        self._display.fill(BLACK)
        points = []
        for i in range(1,num):
            index = len(waypoints) - i
            if  index < 0:
                break
            points.append((int(waypoints[index].X() * self._kX + self._offsetX), \
                    int(waypoints[index].Y() * self._kY + self._offsetY)))
        pygame.draw.lines(self._display,GREEN,False,points, 2)
        pygame.display.flip()
        pygame.display.update()
        pass


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

    def toStr(self):
        return 'ID:' + str(self.ID()) + '\t' + str(self.X()) + ' , ' +  str(self.Y())

    def info(self):
        print("==============node info===================")
        print("\tID : %d" % (self.ID()))
        print("\t(X,Y) : (%d, %d)" % (self.X(),(self.Y()) ) )
        print("\tIDD : "+str(self.IDD()))
        print("")

def getN2(topuInfo, n0, n1):
    def ok(n):
        if np.dot([n.X() - n1.X(), n.Y()-n1.Y() ],[ n1.X() - n0.X(), n1.Y() - n0.Y() ]) > 0:
            return True
        return False
    n2 = topuInfo[random.choice(n1.IDD())]
    while not ok(n2):
        n2 = topuInfo[random.choice(n1.IDD())]
    return n2
    pass


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
    return waypoints
    pass

def getSparse(n0,n1):
    distance = np.linalg.norm([n1.X()-n0.X(), n1.Y()-n0.Y()])
    sparseNum = math.floor(distance / 8.8)
    if sparseNum <= 0:
        return [n0.X()] , [n0.Y()]
    
    #waypoints = []
    Xs = []
    Ys = []

    for i in range(0,int(sparseNum)):
        X = n0.X() + i/sparseNum * (n1.X() - n0.X()) + round(np.random.random() * 3.4 - 1.7,1)
        Y = n0.Y() + i/sparseNum * (n1.Y() - n0.Y()) + round(np.random.random() * 3.4 - 1.7,1)
        #X = n0.X() + i/sparseNum * (n1.X() - n0.X())
        #Y = n0.Y() + i/sparseNum * (n1.Y() - n0.Y()
        if i < 3 or sparseNum - i < 3:
            X = n0.X() + i/sparseNum * (n1.X() - n0.X())
            Y = n0.Y() + i/sparseNum * (n1.Y() - n0.Y())
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
    interNum = int( (distance) * 3 ) + 1
    tck,u = interpolate.splprep([Xs,Ys],k=2,s=0)
    u=np.linspace(0,1,num=interNum,endpoint=True)
    out = interpolate.splev(u,tck)
    return getWaypoints(out)
    pass

def logToFile(waypoints, fileName):
    pass

def main(args):
    ### topu map file
    topuName = '/home/chouer/workspace/expr2/ReMotionPlanning/src/data/topu.map'
    if len(args) > 0:
        topuName = args[0]
    assert os.path.isfile(topuName),topuName + 'topu file is not exist'

    ### read from topu map file, topu topu = []
    topuInfo = []
    topu = open(topuName,'r')
    line = topu.readline()
    while line:
        contents = line.split(',')
        ID = int(contents[0])
        X = int(contents[1])
        Y = int(contents[2])
        ID1 = int(contents[3])
        ID2 = int(contents[4])
        ID3 = 999
        if len(contents) > 5:
            ID3 = int(contents[5])
        node = Node(ID, X, Y, ID1, ID2, ID3)
        topuInfo.append(node)
        ### test output all topu list
        #topuInfo[len(topuInfo)-1].info()
        ## read new line for next loop
        line = topu.readline()
    pass

    waypoints = []
    # start point
    n0 = topuInfo[2]
    n1 = topuInfo[3]
    nodes = [n0, n1]
    #sparse = getSparse()
    # start node point included, but not end node point
    waypointsDir = '/home/chouer/workspace/expr2/ReMotionPlanning/src/data/thirdWaypoints/'
    waypointsNum = 0
    
    while waypointsNum < 1000000:
        for i in range(0,1000):
            newNode = getN2(topuInfo, n0, n1)
            nodes.append(newNode)
            n1,n0 = newNode, n1
        waypoints += getDense(nodes)
        #render.renderWaypoints(waypoints,1000)
        if True:
            fileName = waypointsDir + str(waypointsNum) + '.waypoints'
            dest = open(fileName ,'w')
            print('writing file ' + str(waypointsNum) + '.............\n')
            waypointsNum += 1
            for point in waypoints:
                dest.write(point.toStr())
            dest.close()
            waypoints = []
        nodes = [n0, n1]

if __name__ == "__main__":
    main(sys.argv[1:])
