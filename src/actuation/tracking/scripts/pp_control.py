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
        self.receivedTrajectory = False
        self._trajectory = None
        self.Kv = 0.3   # look forward gain
        self.Lfc = 6.0    # look ahead distance
        self.WB = 3.6     # wheel base
        pass

    def lookAhead(self):
        return max(math.log1p(max(0,self._state.forward_speed)) + 4/5 * self._state.forward_speed + 2 , self.Lfc )

    def compute(self):
        steer = 0
        velocity = 0
        Lf = self.lookAhead()
        L = 0
        i = 1
        while Lf > L and i < len(self._trajectory.points):
            L += np.linalg.norm([self._trajectory.points[i].x - self._trajectory.points[i-1].x,\
                    self._trajectory.points[i].y - self._trajectory.points[i-1].y])
            i += 1
        point = self._trajectory.points[i]
        tx,ty,tyaw,tsp = point.x, point.y, point.fine, point.velocity
        print(tx,ty)
        x,y,yaw,sp = self._state.point.x, self._state.point.y, self._state.rotation.x, self._state.forward_speed

        alpha = ang( math.atan2(tx, ty) )
        #alpha = ang(( alpha - ang(yaw/ 180 * math.pi) ))
        delta = math.atan2(4.0 * self.WB * math.sin(alpha) / Lf, 1.0)
        delta /= (math.pi )
        if delta > 0:
            delta = (1 + delta) ** 2 - 1
        else:
            delta = 1 - (delta - 1)**2
        delta = max(-1, min(1, delta))
        return delta, tsp 


class PID:
    def __init__(self, P = 0.6, I = 0.5, D = 0.2):
        self._p = P
        self._i = I
        self._d = D
        self._pTerm = 0
        self._iTerm = 0
        self._dTerm = 0
        self._window = 2
        self._last_error = 0

    def update(self,real, target):
        error = target - real
        delta_error = error - self._last_error
        self._last_error = error
        self._pTerm = error
        self._iTerm += error
        self._iTerm = max(-self._window, self._iTerm)
        self._iTerm = min(self._window, self._iTerm)
        self._dTerm = delta_error
        return self._pTerm * self._p + self._iTerm * self._i + self._dTerm * self._d

def stateCallback(data, args):
    args[0]._state = data
    args[0].receivedState = True
    pass


def pathCallback(data, args):
    args[0]._trajectory = data
    args[0].receivedTrajectory = True
    pass

def main():
    pp = PurePursuit()
    pid = PID()
    rospy.init_node('pp_control', anonymous=True)
    pub = rospy.Publisher('carlaControl', VehicleCmd , queue_size=10)
    rospy.Subscriber('carlaPlayerState', VehicleState, stateCallback, (pp,))
    rospy.Subscriber('localPath', Path, pathCallback,(pp,))
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if not pp.receivedState:
            continue
        if not pp.receivedTrajectory:
            continue
        steer, tsp = pp.compute()
        accel = pid.update(pp._state.forward_speed, tsp)
        control = VehicleCmd()
        control.steer_cmd.steer = steer
        control.accel_cmd.accel = max(0, accel)
        control.accel_cmd.accel = min(1, accel)
        control.brake_cmd.brake = max(-1, accel)
        control.brake_cmd.brake = min(0, accel)
        control.hand_brake = False
        control.reverse = False
        control.restart = False
        pub.publish(control)

        print(steer)
        print('**************************************')
        #print(round(pp._state.forward_speed,2))
        pp.receivedState = False
        pp.receivedTrajectory = False
        rate.sleep()

if __name__ == '__main__':
    main()
