#!/usr/bin/env python3
"""
Wrapper script to run the TurtleBot 3 Waypoint Navigator with environment variables.

This script reads environment variables and constructs the appropriate ROS 2 arguments
before running the navigator node.
"""

import os
import sys


def main():
    """Main entry point for the waypoint navigator wrapper."""
    # Set up sys.argv with ROS 2 arguments
    sys.argv = ['waypoint_navigator_twist', '--ros-args']

    # Read parameters from environment variables
    params = {
        'speed': os.getenv('WAYPOINT_SPEED', '0.2'),
        'angular_speed': os.getenv('WAYPOINT_ANGULAR_SPEED', '0.5'),
        'repetitions': os.getenv('WAYPOINT_REPETITIONS', '1'),
        'pattern': os.getenv('WAYPOINT_PATTERN', 'default'),
        'position_tolerance': os.getenv('WAYPOINT_POSITION_TOLERANCE', '0.1'),
        'angle_tolerance': os.getenv('WAYPOINT_ANGLE_TOLERANCE', '0.1'),
        'namespace': os.getenv('WAYPOINT_NAMESPACE', ''),
    }

    # Build sys.argv with parameters
    for param, value in params.items():
        # Skip namespace if empty
        if param == 'namespace' and not value:
            continue
        sys.argv.extend(['-p', f'{param}:={value}'])

    # Add installed packages to path
    sys.path.insert(0, '/ws/install/turtlebot3_waypoint_navigator/lib/python3.10/site-packages')

    # Import and run the navigator
    from turtlebot3_waypoint_navigator.navigator_twist import main as navigator_main
    navigator_main()

if __name__ == '__main__':
    main()
