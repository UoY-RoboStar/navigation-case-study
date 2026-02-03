"""TurtleBot 3 Waypoint Navigator Package.

This package provides ROS 2 nodes for autonomous waypoint-based navigation
of TurtleBot 3 robots.

Two implementations are available:
1. navigator_nav2: Full-featured Navigation2 integration
2. navigator_twist: Lightweight velocity command-based navigation
"""

__version__ = '1.0.0'
__author__ = 'RoboSapiens'
__email__ = 'contact@robosapiens.org'

__all__ = ['navigator_nav2', 'navigator_twist']
