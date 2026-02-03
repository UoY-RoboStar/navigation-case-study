#!/usr/bin/env python3
"""
Wrapper script to run the TurtleBot 3 Waypoint Navigator (Nav2) with environment variables.

This script reads environment variables and constructs the appropriate ROS 2 arguments
before running the Nav2-based navigator node.
"""

import os
import sys


def main():
    """Main entry point for the Nav2 waypoint navigator wrapper."""
    # Set up sys.argv with ROS 2 arguments
    sys.argv = ['waypoint_navigator_nav2', '--ros-args']

    # Read parameters from environment variables
    max_loops = os.getenv('WAYPOINT_REPETITIONS', '1')
    pause_at_waypoint = os.getenv('WAYPOINT_PAUSE', '1.0')
    namespace = os.getenv('WAYPOINT_NAMESPACE', '')

    # Build sys.argv with parameters
    sys.argv.extend(['-p', f'max_loops:={max_loops}'])
    sys.argv.extend(['-p', f'pause_at_waypoint:={pause_at_waypoint}'])
    
    if namespace:
        sys.argv.extend(['-p', f'namespace:={namespace}'])

    # Add installed packages to path
    sys.path.insert(0, '/ws/install/turtlebot3_waypoint_navigator/lib/python3.10/site-packages')

    # Import and run the Nav2 navigator
    from turtlebot3_waypoint_navigator.navigator_nav2 import main as navigator_main
    navigator_main()

if __name__ == '__main__':
    main()
