#!/usr/bin/env python
import math
import time
import rospy
import numpy as np
from carla_msgs.msg import VehicleState
from carla_msgs.msg import VehicleCmd
from path_msgs.msg import Path
from path_msgs.msg import Waypoint

def ang(an):
    while abs(an) > math.pi:
        an = math.pi * 2 - abs(an)
    return an

class PurePursuit:
    def __init__(self):
        self.receivedState = False
        self._state = None

def stateCallback(data, args):
    args[0]._state = data
    args[0].receivedState = True
    pass

def stateToStr(state):
    return str(round(state.point.x,2)) + ',' + str(round(state.point.y,2)) + ',' + str(round(state.rotation.x,2))+'\n'

def main():
    pp = PurePursuit()
    rospy.init_node('pp_control', anonymous=True)
    rospy.Subscriber('carlaPlayerState', VehicleState, stateCallback, (pp,))
    rate = rospy.Rate(10)
    i = 0
    fiN = '/home/chouer/workspace/expr2/ReMotionPlanning/src/data/log/' +  str(i) + '.waypoints'
    f = open(fiN, 'w')
    lastX,lastY = 14,2
    count = 1
    print('writing %5d st waypoints file.................' % (i))
    while not rospy.is_shutdown():
        if not pp.receivedState:
            continue
        if np.linalg.norm([pp._state.point.x - lastX, pp._state.point.y - lastY]) > 0.49:
            f.write(stateToStr(pp._state))
            lastX , lastY = pp._state.point.x, pp._state.point.y
            count += 1
        if count % 499 == 0:
            print("\t{:.0%}".format(count/20000.0))
        if count > 20000:
            i += 1
            count = 1
            fiN = '/home/chouer/workspace/expr2/ReMotionPlanning/src/data/log/' +  str(i) + '.waypoints'
            f.close()
            f = open(fiN, 'w')
            print('writing %5d st waypoints file.................' % (i))
        rate.sleep()

if __name__ == '__main__':
    main()
