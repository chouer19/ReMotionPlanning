#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from carla_msgs.msg import VehicleCmd

"""
Sample Python/Pygame Programs
Simpson College Computer Science
http://programarcadegames.com/
http://simpson.edu/computer-science/

Show everything we can pull off the joystick
"""
import pygame

def talker():
    pygame.init()
    # Initialize the joysticks
    pygame.joystick.init()

    pub = rospy.Publisher('carlaControl', VehicleCmd , queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(50) # 10hz
    control = VehicleCmd()
    control.steer_cmd.steer = 0
    control.accel_cmd.accel = 0.2
    control.brake_cmd.brake = 0
    control.hand_brake = False
    control.reverse = False
    control.restart = False
    while not rospy.is_shutdown():
        control.accel_cmd.accel = 0
        control.brake_cmd.brake = 0
        control.steer_cmd.steer = 0
        control.restart = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
    
            # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN
            # JOYBUTTONUP JOYHATMOTION
            if event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.")
            if event.type == pygame.JOYBUTTONUP:
                print("Joystick button released.")
    
        # DRAWING STEP
        # First, clear the screen to white. Don't put other drawing commands
        # above this, or they will be erased with this command.
    
        # Get count of joysticks
        joystick_count = pygame.joystick.get_count()
        assert joystick_count > 0,"not a joystick on"
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        # Get the name from the OS for the controller/joystick
        name = joystick.get_name()
        # Usually axis run in pairs, up/down for one, and left/right for
        # the other.
        axes = joystick.get_numaxes()
        #L-
        axis_0 = joystick.get_axis(0)
        #L|
        axis_1 = joystick.get_axis(1)
        #Lt:brake
        axis_2 = joystick.get_axis(2)/2 + 0.5
        axis_2 =  axis_2 * -1
        #R-:steer
        axis_3 = joystick.get_axis(3)
        control.steer_cmd.steer = axis_3 / 2
        #R|:nothing
        axis_4 = joystick.get_axis(4)
        if axis_1 > 0:
            control.brake_cmd.brake = axis_1 / 5
        if axis_1 < 0:
            control.accel_cmd.accel = abs(axis_1) / 1.41
        #Rt:accel
        axis_5 = joystick.get_axis(5)/2 + 0.5

        buttons = joystick.get_numbuttons()
        #A:
        button_0 = joystick.get_button(0)
        #B:
        button_1 = joystick.get_button(1)
        #X:
        button_2 = joystick.get_button(2)
        #Y:
        button_3 = joystick.get_button(3)
        #LB:hand_brake(false)
        button_4 = joystick.get_button(4)
        if button_4 == 1:
            control.hand_brake = False
        #RB:hand_brake(true)
        button_5 = joystick.get_button(5)
        if button_5 == 1:
            control.hand_brake = True
        #BACK:backward(revers = true)
        button_6 = joystick.get_button(6)
        if button_6 == 1:
            control.reverse = True
        #START:forward(revers = false)
        button_7 = joystick.get_button(7)
        if button_7 == 1:
            control.reverse = False
        button_8 = joystick.get_button(8)
        button_9 = joystick.get_button(9)
        button_10 = joystick.get_button(10)
        # (right/left,up/down)
        hats = joystick.get_numhats()
        hat = joystick.get_hat(0)
        print(hat)
        if abs(hat[0]) + abs(hat[1]) + button_0 > 2:
            control.restart = True
        rospy.loginfo(control)
        pub.publish(control)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    # Close the window and quit.
    # If you forget this line, the program will 'hang'
    # on exit if running from IDLE.
    pygame.quit()
