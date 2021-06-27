#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
 
"""Example of automatic vehicle control from client side."""
from __future__ import print_function
import argparse
import glob
import logging
import os
import sys


# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('/opt/carla-simulator/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg')[0])
except IndexError:
    print("error finding Carla module")
    pass

# ==============================================================================
# -- Add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath('/opt/carla-simulator/PythonAPI'))) + '/carla')
except IndexError:
    print("error while adding PythonAPI for release mode")
    pass

# ==============================================================================
# -- My Code -------------------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.abspath("/home/jordan/Projects/self-driving-car"))
except:
    print("error while locating the project")

from src.controller.test.test_agent import TestAgent
from src.controller.test.carla_util import game_loop

def main():
    """Main method"""

    argparser = argparse.ArgumentParser(
        description='CARLA Automatic Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='Print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='Window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.tesla.model3',
        help='Actor filter (default: "vehicle.tesla.model3")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '-l', '--loop',
        action='store_true',
        dest='loop',
        help='Sets a new random destination upon reaching the previous one (default: False)')
    # argparser.add_argument(
    #     '-b', '--behavior', type=str,
    #     choices=["cautious", "normal", "aggressive"],
    #     help='Choose one of the possible agent behaviors (default: normal) ',
    #     default='normal')
    argparser.add_argument("-a", "--agent", type=str,
                           choices=["Behavior", "Roaming", "Basic", "Test"],
                           help="select which agent to run",
                           default="Test")
    argparser.add_argument(
        '-s', '--seed',
        help='Set seed for repeating executions (default: None)',
        default=None,
        type=int)
    argparser.add_argument(
        '--K_P',
        help='Set the Proportional Gain for the Longitudinal Controller',
        default=0.8,
        type=float)
    argparser.add_argument(
        '--K_I',
        help='Set the Integral Gain for the Longitudinal Controller',
        type=float,
        default=0.15)
    argparser.add_argument(
        '--K_D',
        help='Set the Derivative Gain for the Longitudinal Controller', 
        type=float,
        default=0.2)
    argparser.add_argument(
        '--K_S', 
        help="Set the Velocity Softening Term for the Lateral Stanley Controller.",
        type=float,
        default=0.1)
    argparser.add_argument(
        '--K_CTE',
        help="Set the Crosstrack-Error gain for the Lateral Stanley Controller.",
        type=float,
        default=3.0)
    argparser.add_argument(
        '-t', '--town',
        help="Select the town to load into",
        type=str,
        default='Town02')
    argparser.add_argument(
        '--target_speed',
        help="Set the target speed for the vehicle [km/hr]",
        default=20,
        type=int)

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:
        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
