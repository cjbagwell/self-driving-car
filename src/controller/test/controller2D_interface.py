import glob
import sys
import argparse
import os
import numpy as np

# import controller module
try:
    sys.path.append(glob.glob('/opt/carla-simulator/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg')[0])    
except IndexError:
    print("error finding Carla module in controller2D_interface.py")
    pass

# TODO: fix importing so no warnings
import carla
from py_controller import Waypoint, Controller2D, Commands, State

class Controller2DInterface():
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
        past_app = self._vehicle.get_control().throttle
        past_bpp = self._vehicle.get_control().brake

        # generate starting commands
        ini_commands = Commands(past_app, past_bpp, self.past_steering)
        print("ini_commands {}".format(ini_commands))
        kp = 0.7
        ki = 0.2
        kd = 0.1
        ks = 0.1
        kcte = 1.0

        self.vehicle_controller = Controller2D(ini_commands, kp, ki, kd, ks, kcte)
    
    def run_step(self, target_speed, waypoint):
        """
        Execute one step of control invoking both lateral and longitudinal
        PID controllers to reach a target waypoint
        at a given target_speed.

            :param target_speed: desired vehicle speed
            :param waypoint: target location encoded as a waypoint
            :return: distance (in meters) to the waypoint
        """
        # carla variables
        physics_control = self._vehicle.get_physics_control()
        wheels = physics_control.wheels
        w0x = wheels[0].position.x/100
        w0y = wheels[0].position.y/100
        w1x = wheels[1].position.x/100
        w1y = wheels[1].position.y/100
        front_ax_x = (w0x+w1x)/2
        front_ax_y = (w0y+w1y)/2

        # build state for c++ code
        curr_transform = self._vehicle.get_transform()
        curr_rotation = curr_transform.rotation
        curr_velocity = self._vehicle.get_velocity()

        # debug info 
        # print("Wheel 0 location: {}, {}".format(w0x, w0y))
        # print("Wheel 1 location: {}, {}".format(w1x, w1y))
        # print("Center of Front : {}, {}".format(front_ax_x, front_ax_y))
        # print("Vehicle location: {}, {}, {}".format(curr_location.x, curr_location.y, curr_rotation.yaw))
        # print("Center  of  Mass: {}, {}".format(com.x, com.y))
        
        curr_speed = np.sqrt(np.square(curr_velocity.x) + np.square(curr_velocity.y))
        curr_snapshot = self._world.get_snapshot()
        curr_time = curr_snapshot.timestamp.platform_timestamp # TODO: not sure about this (using os time stampe)
        curr_frame = curr_snapshot.timestamp.frame
        dt = curr_snapshot.timestamp.delta_seconds
        curr_state = State(front_ax_x, front_ax_y, np.deg2rad(curr_rotation.yaw), curr_speed, curr_time, curr_frame)

        # build waypoints for c++ code
        curr_w_location = waypoint.transform.location
        curr_w_rotation = waypoint.transform.rotation
        curr_waypoint = Waypoint(curr_w_location.x, curr_w_location.y, np.deg2rad(curr_w_rotation.yaw), target_speed) 
        
        # run step using c++ controller
        new_commands = self.vehicle_controller.run_step(curr_state, curr_waypoint, dt)
        
        # format return controls
        new_controls = carla.VehicleControl(new_commands.get_app(), 
                                            new_commands.get_steering_angle_rate(), 
                                            new_commands.get_bpp())
        
        return new_controls


# my_waypoint = Waypoint(1.1, 2.2, 3.3)
# print(my_waypoint)

# my_state = State()
# print(my_state)

# my_commands = Commands()
# print(my_commands)

# my_controller = Controller2D([my_waypoint, my_waypoint], my_commands)
# print(my_controller)