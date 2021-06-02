import glob
import sys
import argparse
import os

# import controller module
try:
    sys.path.append(glob.glob('/opt/carla-simulator/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg')[0])    
    print("successfully found Carla module")
except IndexError:
    print("error finding Carla module")
    pass

import carla

from py_controller import Waypoint, Controller2D, Commands, State

class Controller2D_interface():
    def __init__(self, vehicle, args_lateral, args_longitudinal, offset=0, max_throttle=0.75, max_brake=0.3,
                 max_steering=0.8):
        """
        Constructor method.

        :param vehicle: actor to apply to local planner logic onto
        :param args_lateral: dictionary of arguments to set the lateral PID controller
        using the following semantics:
            K_P -- Proportional term
            K_D -- Differential term
            K_I -- Integral term
        :param args_longitudinal: dictionary of arguments to set the longitudinal
        PID controller using the following semantics:
            K_P -- Proportional term
            K_D -- Differential term
            K_I -- Integral term
        :param offset: If different than zero, the vehicle will drive displaced from the center line.
        Positive values imply a right offset while negative ones mean a left one. Numbers high enough
        to cause the vehicle to drive through other lanes might break the controller.
        """
        self.max_brake = max_brake
        self.max_throt = max_throttle
        self.max_steer = max_steering

        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        self.past_steering = self._vehicle.get_control().steer

        # generate starting commands
        self.vehicle_controller = Controller2D()

my_waypoint = Waypoint(1.1, 2.2, 3.3)
print(my_waypoint)

my_state = State()
print(my_state)

my_commands = Commands()
print(my_commands)

my_controller = Controller2D([my_waypoint, my_waypoint], my_commands)
print(my_controller)