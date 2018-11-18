#!/usr/bin/env python

import time
import math
import utils
import rospy
import random
import argparse
import numpy as np
from carla_msgs.msg import VehicleState
from carla_msgs.msg import VehicleCmd
from path_msgs.msg import Path
from path_msgs.msg import Waypoint

global receivedState
global state
def stateCallback(VehicleState):
    global receivedState
    global state
    #print('debugdebugdebugdebugdebugdebugdebugdebugdebugdebug')
    state = VehicleState
    receivedState = True
    pass

def findNearest(path):
    dis = 20
    index = -1
    sigma = 180
    for i,content in enumerate(path):
        diss = np.linalg.norm([content.x - state.point.x,content.y - state.point.y])
        sig = ang(content.fine - state.rotation.x)
        if abs(sig) > 90:
            continue
        if dis <= diss:
            continue
        dis = diss
        sigma = sig
        index = i
    return index,dis,sigma
    pass

def ang(an):
    while abs(an) > 180:
        an = 360 - abs(an)
    return an

co = math.cos
si = math.sin
def getLocal(path):
    angle = state.rotation.x / 180 * math.pi
    local = Path()
    rotateMatrix = [ 
            [co(angle),-si(angle)],
            [si(angle),co(angle)]
            ]
    # km/h
    v = 36
    for i in range(0,50):
        if i >= len(path):
            break
        [y,x] = np.dot([path[i].x - state.point.x, path[i].y - state.point.y] , rotateMatrix)
        if i == len(path) - 1:
            pass
        else:
            v = 10 / (1 + abs(ang(path[i+1].fine - path[i].fine))/180 * math.pi * 5)
        point = Waypoint(x,y, path[i].fine, v)
        local.points.append(point)
    return local

def output(trajectory):
    for content in trajectory:
        print(content.x, content.y)

def main():
    global receivedState
    global state
    state = None
    receivedState = False
    rospy.init_node('path_planner', anonymous=True)
    pub = rospy.Publisher('localPath',Path , queue_size=10)
    rospy.Subscriber('carlaPlayerState', VehicleState, stateCallback)
    rate = rospy.Rate(10) # 10hz

    reader = utils.MapReader('/home/chouer/workspace/expr2/ReMotionPlanning/src/data/thirdWaypoints/')
    path = reader.next()

    while not rospy.is_shutdown():
        # prepare new current state
        if not receivedState:
            continue
        # prepare path
        if len(path) < 500:
            path += reader.next()
        index,dis,sigma = findNearest(path[0:200])
        if index < 0:
            path = path[200:]
            continue
        print('*******************')
        if dis > 10:
            path = path[200:]
            continue
        path = path[index:]
        print(index)
        print(dis)
        output(path[0:10])
        trajectory = getLocal(path)
        #print(trajectory.points[0:5])
        output(trajectory.points[0:5])
        print('===================')
        pub.publish(trajectory)
        receivedState = False
        rate.sleep()
    pass

if __name__ == '__main__':
    main()
