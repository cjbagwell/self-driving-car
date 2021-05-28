import glob
import sys
import os

# import controller module
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    print("error")
    pass

from py_controller import Waypoint

my_waypoint = Waypoint(1.1, 2.2, 3.3)
print(my_waypoint)