#!/usr/bin/env python

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Keyboard controlling for CARLA. Please refer to client_example.py for a simpler
# and more documented example.

"""
Welcome to CARLA manual control.

Read state and info from CARLA simulator and Publish those
Type:parla_msgs/VehicleState
Topic:"carlaPlayerState"

Subscribe from control and send to CARLA simulator

STARTING in a moment...
"""

from __future__ import print_function

import argparse
import logging
import random
import time

class Timer(object):
    def __init__(self):
        self.step = 0
        self._lap_step = 0
        self._lap_time = time.time()

    def tick(self):
        self.step += 1

    def lap(self):
        self._lap_step = self.step
        self._lap_time = time.time()

    def ticks_per_second(self):
        return float(self.step - self._lap_step) / self.elapsed_seconds_since_lap()

    def elapsed_seconds_since_lap(self):
        return time.time() - self._lap_time

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')
from carla import image_converter
from carla import sensor
from carla.client import make_carla_client, VehicleControl
from carla.planner.map import CarlaMap
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError
from carla.util import print_over_same_line

import rospy
from carla_msgs.msg import VehicleState
from carla_msgs.msg import VehicleCmd

def make_carla_settings(args):
    """Make a CarlaSettings object with the settings we need."""
    settings = CarlaSettings()
    settings.set(
        #SynchronousMode=False,
        SynchronousMode=True,
        SendNonPlayerAgentsInfo=False,
        NumberOfVehicles=0,
        NumberOfPedestrians=1,
        #WeatherId=random.choice([1, 3, 7, 8, 14]),
        #WeatherId=random.choice(1),
        WeatherId=1,
        QualityLevel=args.quality_level)
    settings.randomize_seeds()
    return settings

class CarlaGame(object):
    def __init__(self, carla_client, args):
        self.client = carla_client
        self._carla_settings = make_carla_settings(args)
        self._timer = None
        self._is_on_reverse = False
        self._position = None
        self._agent_positions = None
        self._vehicle_state = VehicleState()
        self._vehicle_control = VehicleCmd()
        def controlCallback(VehicleControl):
            self._vehicle_control.steer_cmd.steer = VehicleControl.steer_cmd.steer 
            self._vehicle_control.accel_cmd.accel = VehicleControl.accel_cmd.accel 
            self._vehicle_control.brake_cmd.brake = VehicleControl.brake_cmd.brake 
            self._vehicle_control.hand_brake =      VehicleControl.hand_brake
            self._vehicle_control.reverse =         VehicleControl.reverse
            self._vehicle_control.restart =         VehicleControl.restart
            pass
        self._pub = rospy.Publisher('carlaPlayerState', VehicleState, queue_size=10)
        rospy.init_node('driver', anonymous=True)
        rospy.Subscriber('carlaControl', VehicleCmd, controlCallback)
        self._rate = rospy.Rate(10) # 10hz


    def pubState(self, measurements):
        self._vehicle_state.point.x = measurements.transform.location.x
        self._vehicle_state.point.y = measurements.transform.location.y
        self._vehicle_state.point.z = measurements.transform.location.z
        self._vehicle_state.orientation.x = measurements.transform.orientation.x
        self._vehicle_state.orientation.y = measurements.transform.orientation.y
        self._vehicle_state.orientation.z = measurements.transform.orientation.z
        self._vehicle_state.acceleration.x = measurements.acceleration.x
        self._vehicle_state.acceleration.y = measurements.acceleration.y
        self._vehicle_state.acceleration.z = measurements.acceleration.z
        self._vehicle_state.rotation.x = measurements.transform.rotation.yaw
        self._vehicle_state.rotation.y = measurements.transform.rotation.pitch
        self._vehicle_state.rotation.z = measurements.transform.rotation.roll
        self._vehicle_state.forward_speed         = measurements.forward_speed
        self._vehicle_state.collision_vehicles    = measurements.collision_vehicles
        self._vehicle_state.collision_pedestrians = measurements.collision_pedestrians
        self._vehicle_state.collision_other       = measurements.collision_other
        self._vehicle_state.intersection_otherlane = measurements.intersection_otherlane
        self._vehicle_state.intersection_offroad  = measurements.intersection_offroad
        self._pub.publish(self._vehicle_state)
        #rospy.loginfo(self._vehicle_state)
        pass

    def execute(self):
        self._initialize_game()
        while not rospy.is_shutdown():
            self._on_loop()
            #self._rate.sleep()

    def _initialize_state(self):
        pass

    def _initialize_control(self):
        self._vehicle_control.steer_cmd.steer = 0
        self._vehicle_control.accel_cmd.accel = 0
        self._vehicle_control.brake_cmd.brake = 0
        self._vehicle_control.hand_brake = False
        self._vehicle_control.reverse = False
        pass

    def _initialize_game(self):
        self._initialize_state()
        self._initialize_control()
        self._on_new_episode()

    def _on_new_episode(self):
        #self._carla_settings.randomize_seeds()
        #self._carla_settings.randomize_weather()
        scene = self.client.load_settings(self._carla_settings)
        #number_of_player_starts = len(scene.player_start_spots)
        #player_start = np.random.randint(number_of_player_starts)
        ## (14,2)-->
        player_start = 45
        print('Starting new episode...')
        self.client.start_episode(player_start)
        self._timer = Timer()

        # how to send a control
        '''
        control = VehicleControl()
        if keys[K_LEFT] or keys[K_a]:
            control.steer = -1.0
        control.reverse = self._is_on_reverse
        return control
        '''
        
    def _on_loop(self):
        self._timer.tick()
        measurements,_ = self.client.read_data()
        self.pubState(measurements.player_measurements)
        # Print measurements every second.
        if self._timer.elapsed_seconds_since_lap() > 1.0:
            self._print_player_measurements(measurements.player_measurements)
            # Plot position on the map as well.
            self._timer.lap()
        if self._vehicle_control.restart:
            self._on_new_episode()
        else:
            control = VehicleControl()
            control.throttle = self._vehicle_control.accel_cmd.accel
            control.brake = self._vehicle_control.brake_cmd.brake
            control.steer = self._vehicle_control.steer_cmd.steer
            control.hand_brake = self._vehicle_control.hand_brake
            control.reverse = self._vehicle_control.reverse
            self.client.send_control(control)

    def _print_player_measurements(self, player_measurements):
        message = 'Step {step} ({fps:.1f} FPS): '
        message += '{speed:.2f} km/h, '
        message += '{other_lane:.0f}% other lane, {offroad:.0f}% off-road'
        message = message.format(
            step=self._timer.step,
            fps=self._timer.ticks_per_second(),
            speed=player_measurements.forward_speed * 3.6,
            other_lane=100 * player_measurements.intersection_otherlane,
            offroad=100 * player_measurements.intersection_offroad)
        print_over_same_line(message)

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host server (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '-l', '--lidar',
        action='store_true',
        help='enable Lidar')
    argparser.add_argument(
        '-q', '--quality-level',
        choices=['Low', 'Epic'],
        type=lambda s: s.title(),
        default='Low',
        help='graphics quality level, a lower level makes the simulation run considerably faster.')
    argparser.add_argument(
        '-m', '--map-name',
        metavar='M',
        default=None,
        help='plot the map of the current city (needs to match active map in '
             'server, options: Town01 or Town02)')
    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    #while True:
    try:
        with make_carla_client(args.host, args.port) as client:
            game = CarlaGame(client, args)
            game.execute()

    except TCPConnectionError as error:
        logging.error(error)
        time.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
