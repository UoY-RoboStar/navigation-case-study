#!/usr/bin/env python3
"""
TurtleBot3 Live Command-Line Map Visualizer

This script subscribes to ROS topics and displays a live ASCII visualization
of the robot's current position, map, navigation status, and sensor data.

Usage:
    python3 map_visualizer.py

ROS Topics Subscribed:
    - /map (nav_msgs/OccupancyGrid) - Map data
    - /odom (nav_msgs/Odometry) - Robot odometry
    - /amcl_pose (geometry_msgs/PoseWithCovarianceStamped) - Localized pose
    - /scan (sensor_msgs/LaserScan) - Lidar data
    - /cmd_vel (geometry_msgs/Twist) - Velocity commands
    - /goal_pose (geometry_msgs/PoseStamped) - Current navigation goal
"""

import math
import sys
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class MapVisualizer(Node):
    """
    ROS2 Node that visualizes robot navigation state in the terminal.
    """

    def __init__(self):
        super().__init__('map_visualizer')

        # Data storage
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.odom_linear_vel = 0.0
        self.odom_angular_vel = 0.0
        self.odom_received = False

        self.amcl_x = 0.0
        self.amcl_y = 0.0
        self.amcl_theta = 0.0
        self.amcl_received = False

        self.goal_x = None
        self.goal_y = None
        self.goal_received = False

        self.cmd_vel_linear = 0.0
        self.cmd_vel_angular = 0.0

        self.scan_ranges = []
        self.scan_min_range = 0.0
        self.scan_received = False

        self.last_update = time.time()

        # Statistics
        self.message_counts = {
            'map': 0,
            'odom': 0,
            'amcl': 0,
            'scan': 0,
            'cmd_vel': 0,
            'goal': 0
        }

        # Create subscriptions
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.get_logger().info('Map Visualizer Node Started')
        self.get_logger().info('Subscribing to ROS topics...')

    def map_callback(self, msg):
        """Handle map updates."""
        self.message_counts['map'] += 1
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_data = list(msg.data)

    def odom_callback(self, msg):
        """Handle odometry updates."""
        self.message_counts['odom'] += 1
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        self.odom_theta = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.odom_linear_vel = msg.twist.twist.linear.x
        self.odom_angular_vel = msg.twist.twist.angular.z
        self.odom_received = True
        self.last_update = time.time()

    def amcl_callback(self, msg):
        """Handle AMCL pose updates."""
        self.message_counts['amcl'] += 1
        self.amcl_x = msg.pose.pose.position.x
        self.amcl_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        self.amcl_theta = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.amcl_received = True

    def scan_callback(self, msg):
        """Handle laser scan updates."""
        self.message_counts['scan'] += 1
        self.scan_ranges = list(msg.ranges)
        if self.scan_ranges:
            valid_ranges = [r for r in self.scan_ranges if not math.isinf(r) and not math.isnan(r)]
            if valid_ranges:
                self.scan_min_range = min(valid_ranges)
        self.scan_received = True

    def cmd_vel_callback(self, msg):
        """Handle velocity command updates."""
        self.message_counts['cmd_vel'] += 1
        self.cmd_vel_linear = msg.linear.x
        self.cmd_vel_angular = msg.angular.z

    def goal_callback(self, msg):
        """Handle navigation goal updates."""
        self.message_counts['goal'] += 1
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_received = True

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def world_to_map(self, world_x, world_y):
        """Convert world coordinates to map grid coordinates."""
        if self.map_data is None:
            return None, None

        map_x = int((world_x - self.map_origin_x) / self.map_resolution)
        map_y = int((world_y - self.map_origin_y) / self.map_resolution)

        return map_x, map_y

    def get_map_value(self, map_x, map_y):
        """Get occupancy value at map coordinates."""
        if self.map_data is None:
            return -1

        if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
            index = map_y * self.map_width + map_x
            if 0 <= index < len(self.map_data):
                return self.map_data[index]

        return -1

    def render_mini_map(self, width=60, height=30):
        """Render a mini ASCII map centered on the robot."""
        if not self.amcl_received:
            return ["No AMCL pose data available yet..."]

        lines = []

        # Use AMCL pose as center
        center_x = self.amcl_x
        center_y = self.amcl_y

        # Calculate world bounds for the view
        cell_size = 0.1  # meters per character
        view_width = width * cell_size
        view_height = height * cell_size

        min_world_x = center_x - view_width / 2
        max_world_x = center_x + view_width / 2
        min_world_y = center_y - view_height / 2
        max_world_y = center_y + view_height / 2

        # Render from top to bottom (y decreases)
        for row in range(height):
            line = ""
            world_y = max_world_y - (row * cell_size)

            for col in range(width):
                world_x = min_world_x + (col * cell_size)

                # Check if robot is at this position
                robot_dist = math.sqrt((world_x - self.amcl_x)**2 + (world_y - self.amcl_y)**2)

                if robot_dist < 0.15:  # Robot marker
                    # Show direction arrow based on theta
                    angle = self.amcl_theta
                    if -math.pi/8 < angle <= math.pi/8:
                        line += "→"
                    elif math.pi/8 < angle <= 3*math.pi/8:
                        line += "↗"
                    elif 3*math.pi/8 < angle <= 5*math.pi/8:
                        line += "↑"
                    elif 5*math.pi/8 < angle <= 7*math.pi/8:
                        line += "↖"
                    elif angle > 7*math.pi/8 or angle <= -7*math.pi/8:
                        line += "←"
                    elif -7*math.pi/8 < angle <= -5*math.pi/8:
                        line += "↙"
                    elif -5*math.pi/8 < angle <= -3*math.pi/8:
                        line += "↓"
                    else:
                        line += "↘"
                    continue

                # Check if goal is at this position
                if self.goal_received and self.goal_x is not None:
                    goal_dist = math.sqrt((world_x - self.goal_x)**2 + (world_y - self.goal_y)**2)
                    if goal_dist < 0.15:
                        line += "★"
                        continue

                # Get map occupancy value
                if self.map_data is not None:
                    map_x, map_y = self.world_to_map(world_x, world_y)
                    occupancy = self.get_map_value(map_x, map_y)

                    if occupancy == -1:  # Unknown
                        line += "?"
                    elif occupancy > 50:  # Occupied
                        line += "█"
                    elif occupancy > 0:  # Possibly occupied
                        line += "▓"
                    else:  # Free
                        line += "·"
                else:
                    line += " "

            lines.append(line)

        return lines

    def render_lidar_view(self, width=60):
        """Render a simple top-down lidar visualization."""
        if not self.scan_received or not self.scan_ranges:
            return ["No scan data available yet..."]

        lines = []
        height = 15

        # Create empty grid
        grid = [[' ' for _ in range(width)] for _ in range(height)]

        # Robot at center
        robot_x = width // 2
        robot_y = height // 2
        grid[robot_y][robot_x] = 'R'

        # Plot lidar points
        num_ranges = len(self.scan_ranges)
        for i, r in enumerate(self.scan_ranges):
            if math.isinf(r) or math.isnan(r) or r < 0.1:
                continue

            # Angle for this measurement
            angle = (i / num_ranges) * 2 * math.pi + self.amcl_theta

            # Scale and convert to grid coordinates
            scale = 5.0  # Scale factor
            dx = r * math.cos(angle) / scale * (width / 2)
            dy = -r * math.sin(angle) / scale * (height / 2)

            point_x = int(robot_x + dx)
            point_y = int(robot_y + dy)

            if 0 <= point_x < width and 0 <= point_y < height:
                grid[point_y][point_x] = '•'

        # Convert grid to lines
        for row in grid:
            lines.append(''.join(row))

        return lines

    def render_status(self):
        """Render the complete status display."""
        lines = []

        # Header
        lines.append("=" * 80)
        lines.append("  TurtleBot3 Live Navigation Status".center(80))
        lines.append("=" * 80)
        lines.append("")

        # Position Information
        lines.append("┌─ POSITION ─────────────────────────────────────────────────────────────────┐")

        if self.amcl_received:
            lines.append(f"│ AMCL Pose:    X: {self.amcl_x:7.3f} m  Y: {self.amcl_y:7.3f} m  θ: {math.degrees(self.amcl_theta):6.1f}°" + " " * 13 + "│")
        else:
            lines.append("│ AMCL Pose:    [Waiting for localization...]" + " " * 32 + "│")

        if self.odom_received:
            lines.append(f"│ Odometry:     X: {self.odom_x:7.3f} m  Y: {self.odom_y:7.3f} m  θ: {math.degrees(self.odom_theta):6.1f}°" + " " * 13 + "│")
        else:
            lines.append("│ Odometry:     [No data]" + " " * 52 + "│")

        if self.goal_received and self.goal_x is not None:
            goal_dist = math.sqrt((self.goal_x - self.amcl_x)**2 + (self.goal_y - self.amcl_y)**2)
            lines.append(f"│ Goal:         X: {self.goal_x:7.3f} m  Y: {self.goal_y:7.3f} m  Dist: {goal_dist:6.3f} m" + " " * 13 + "│")
        else:
            lines.append("│ Goal:         [No active goal]" + " " * 45 + "│")

        lines.append("└────────────────────────────────────────────────────────────────────────────┘")
        lines.append("")

        # Velocity Information
        lines.append("┌─ VELOCITY ─────────────────────────────────────────────────────────────────┐")
        lines.append(f"│ Linear:       {self.odom_linear_vel:6.3f} m/s    (cmd: {self.cmd_vel_linear:6.3f} m/s)" + " " * 24 + "│")
        lines.append(f"│ Angular:      {self.odom_angular_vel:6.3f} rad/s  (cmd: {self.cmd_vel_angular:6.3f} rad/s)" + " " * 21 + "│")
        lines.append("└────────────────────────────────────────────────────────────────────────────┘")
        lines.append("")

        # Sensor Information
        lines.append("┌─ SENSORS ──────────────────────────────────────────────────────────────────┐")
        if self.scan_received and self.scan_ranges:
            lines.append(f"│ Lidar:        {len(self.scan_ranges)} points   Min distance: {self.scan_min_range:5.3f} m" + " " * 27 + "│")
        else:
            lines.append("│ Lidar:        [No scan data]" + " " * 47 + "│")
        lines.append("└────────────────────────────────────────────────────────────────────────────┘")
        lines.append("")

        # Mini Map
        lines.append("┌─ MAP VIEW ─────────────────────────────────────────────────────────────────┐")
        map_lines = self.render_mini_map(width=78, height=20)
        for map_line in map_lines:
            # Pad to fit in box
            padded = map_line + " " * (78 - len(map_line))
            lines.append(f"│{padded}│")
        lines.append("│                                                                              │")
        lines.append("│  Legend:  → Robot   ★ Goal   █ Obstacle   · Free   ? Unknown                │")
        lines.append("└────────────────────────────────────────────────────────────────────────────┘")
        lines.append("")

        # Topic Statistics
        lines.append("┌─ TOPIC STATISTICS ─────────────────────────────────────────────────────────┐")
        lines.append(f"│ /map: {self.message_counts['map']:6d}  /odom: {self.message_counts['odom']:6d}  /amcl_pose: {self.message_counts['amcl']:6d}" + " " * 26 + "│")
        lines.append(f"│ /scan: {self.message_counts['scan']:5d}  /cmd_vel: {self.message_counts['cmd_vel']:5d}  /goal_pose: {self.message_counts['goal']:5d}" + " " * 26 + "│")

        time_since_update = time.time() - self.last_update
        status = "🟢 ACTIVE" if time_since_update < 1.0 else "🔴 STALE"
        lines.append(f"│ Status: {status}  (Last update: {time_since_update:.1f}s ago)" + " " * (50 - len(f"{time_since_update:.1f}")) + "│")
        lines.append("└────────────────────────────────────────────────────────────────────────────┘")
        lines.append("")
        lines.append("Press Ctrl+C to exit")

        return "\n".join(lines)

    def display_loop(self):
        """Continuously update the display."""
        try:
            while rclpy.ok():
                # Clear screen (ANSI escape sequence)
                print("\033[2J\033[H", end="")

                # Render and print status
                output = self.render_status()
                print(output)

                # Update rate
                time.sleep(0.2)  # 5 Hz update rate
        except KeyboardInterrupt:
            pass


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = MapVisualizer()

    # Create executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    # Start ROS2 spinning in a separate thread
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("Starting Map Visualizer...")
    print("Waiting for ROS topics...")
    time.sleep(2)

    try:
        # Run display loop in main thread
        node.display_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down Map Visualizer...')
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
