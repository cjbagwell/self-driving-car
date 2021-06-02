import glob
import sys
import os

# import controller module
try:
    sys.path.append(glob.glob('/opt/carla-simulator/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg')[0])    
    print("successfully found Carla module")
except IndexError:
    print("error finding Carla module")
    pass

from py_controller import Waypoint, Controller2D, Commands, State


my_waypoint = Waypoint(1.1, 2.2, 3.3)
print(my_waypoint)

my_state = State()
print(my_state)

my_commands = Commands()
print(my_commands)

my_controller = Controller2D([my_waypoint, my_waypoint], my_commands)
print(my_controller)