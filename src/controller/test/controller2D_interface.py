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
import carla

try:
    sys.path.append(os.path.abspath("/home/jordan/Projects/self-driving-car"))
except:
    print("Error while locating the project")

from build.src.controller.py_controller import Waypoint, Controller2D, Commands
from build.src.localization.py_localization import State, Quaternion

class Controller2DInterface():
    def __init__(self, vehicle, opt_dict):
        """
        Constructor method.

        :param vehicle: actor to apply to local planner logic onto
        :param opt_dict: dictionary of arguments to set the vehicle controller.
        Stanley Controller using the following semantics:
            K_S -- Velocity-Softening term
            K_CTE -- Crosstrack-Error term
        PID controller using the following semantics:
            K_P -- Proportional term
            K_D -- Differential term
            K_I -- Integral term
        """

        self._vehicle = vehicle
        self._world = self._vehicle.get_world()

        # generate starting commands and controller parameters
        kp = opt_dict['K_P']
        ki = opt_dict['K_I']
        kd = opt_dict['K_D']
        ks = opt_dict['K_S']
        kcte = opt_dict['K_CTE']

        self.vehicle_controller = Controller2D(kp, ki, kd, ks, kcte)
    
    def run_step(self, target_speed, waypoint):
        """
        Execute one step of control invoking both lateral and longitudinal
        PID controllers to reach a target waypoint
        at a given target_speed.

            :param target_speed: desired vehicle speed
            :param waypoint: target location encoded as a waypoint
            :return: required controls for the ego vehicle
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
        curr_transform  = self._vehicle.get_transform()
        curr_rotation   = curr_transform.rotation
        curr_velocity   = self._vehicle.get_velocity()        
        curr_snapshot   = self._world.get_snapshot()
        curr_time       = curr_snapshot.timestamp.platform_timestamp 
        curr_frame      = curr_snapshot.timestamp.frame
        
        dt  = curr_snapshot.timestamp.delta_seconds
        pos = [front_ax_x, front_ax_y, curr_transform.location.z]
        q   = Quaternion([np.deg2rad(curr_rotation.roll), 
                        np.deg2rad(curr_rotation.pitch), 
                        np.deg2rad(curr_rotation.yaw)], False)
        vel = [curr_velocity.x, curr_velocity.y, curr_velocity.z]
        curr_state = State(pos, vel, q.as_vector(), curr_time, curr_frame)

        # build waypoints for c++ code
        curr_w_location = waypoint.transform.location
        curr_w_rotation = waypoint.transform.rotation
        curr_waypoint   = Waypoint(curr_w_location.x, 
                                   curr_w_location.y, 
                                   np.deg2rad(curr_w_rotation.yaw), 
                                   target_speed) 
        
        # run step using c++ controller
        new_commands = self.vehicle_controller.run_step(curr_state, curr_waypoint, dt)
        
        # format return controls
        new_controls = carla.VehicleControl(new_commands.get_app(), 
                                            new_commands.get_steering_angle_rate(), 
                                            new_commands.get_bpp())
        
        return new_controls
