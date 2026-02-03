#!/usr/bin/env python3
"""
TurtleBot 3 Waypoint Navigator - Twist (Velocity Command) Implementation

A ROS 2 node that navigates TurtleBot 3 through a predefined sequence of waypoints
using direct velocity commands (geometry_msgs/Twist).

This lightweight implementation does not require Navigation2 stack.

Usage:
    ros2 run turtlebot3_waypoint_navigator waypoint_navigator_twist

With parameters:
    ros2 run turtlebot3_waypoint_navigator waypoint_navigator_twist \
        --ros-args -p speed:=0.3 -p repetitions:=5 -p pattern:=figure_eight

Launch with turtlebot3_world (no Nav2 needed):
    Terminal 1: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    Terminal 2: ros2 run turtlebot3_waypoint_navigator waypoint_navigator_twist
"""

import argparse
import math
import time
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class TurtleBot3WaypointNavigatorTwist(Node):
    """ROS 2 Node for TurtleBot 3 waypoint navigation using velocity commands."""

    def __init__(self):
        super().__init__('turtlebot3_waypoint_navigator_twist')

        # Declare parameters
        self.declare_parameter('speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('repetitions', 1)
        self.declare_parameter('pattern', 'default')
        self.declare_parameter('position_tolerance', 0.1)
        self.declare_parameter('angle_tolerance', 0.1)
        self.declare_parameter('namespace', '')

        # Get parameters
        self.linear_speed = self.get_parameter('speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.repetitions = self.get_parameter('repetitions').value
        self.pattern = self.get_parameter('pattern').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.namespace = self.get_parameter('namespace').value

        # Publisher for velocity commands
        self.cmd_vel_topic = f"{self.namespace}/cmd_vel" if self.namespace else "/cmd_vel"
        self.vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Subscriber for odometry
        self.odom_topic = f"{self.namespace}/odom" if self.namespace else "/odom"
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        # Current robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Control parameters
        self.control_loop_rate = 0.05  # 20 Hz

        # Navigation state
        self.current_rep = 0
        self.current_waypoint_index = 0
        self.log_counter = 0  # For periodic logging

        self.get_logger().info(f'TurtleBot 3 Waypoint Navigator (Twist) initialized')
        self.get_logger().info(f'Publishing velocity commands to: {self.cmd_vel_topic}')
        self.get_logger().info(f'Subscribing to odometry from: {self.odom_topic}')
        self.get_logger().info(f'Speed: {self.linear_speed} m/s')
        self.get_logger().info(f'Pattern: {self.pattern}')
        self.get_logger().info(f'Repetitions: {self.repetitions if self.repetitions > 0 else "infinite"}')

        # Debug: List available topics
        import subprocess
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=5)
            topics = result.stdout.strip().split('\n')
            self.get_logger().info(f'Available ROS 2 topics: {topics}')
        except Exception as e:
            self.get_logger().warn(f'Could not list ROS 2 topics: {e}')

    def odom_callback(self, msg: Odometry) -> None:
        """Update current robot pose from odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        quat = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(quat)

        # Log odometry every 5 updates (~0.25 seconds at 20Hz)
        self.log_counter += 1
        if self.log_counter >= 5:
            self.get_logger().info(
                f"[ODOM] Position: ({self.current_x:.2f}, {self.current_y:.2f}), "
                f"Yaw: {math.degrees(self.current_yaw):.1f}°"
            )
            self.log_counter = 0

    @staticmethod
    def quaternion_to_yaw(quat: Quaternion) -> float:
        """Convert quaternion to yaw angle in radians."""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi] range."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def distance_to_point(self, target_x: float, target_y: float) -> float:
        """Calculate Euclidean distance to a target point."""
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        return math.sqrt(dx**2 + dy**2)

    def angle_to_point(self, target_x: float, target_y: float) -> float:
        """Calculate desired angle to face a target point."""
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        return math.atan2(dy, dx)

    def rotate_to_angle(self, target_yaw: float) -> None:
        """Rotate the robot to a target yaw angle."""
        self.get_logger().info(
            f"Rotating to angle: {math.degrees(target_yaw):.1f}° "
            f"(current: {math.degrees(self.current_yaw):.1f}°)"
        )

        while rclpy.ok():
            angle_diff = self.normalize_angle(target_yaw - self.current_yaw)

            # Check if we've reached the target angle
            if abs(angle_diff) < self.angle_tolerance:
                break

            # Proportional control for rotation
            cmd_vel = Twist()
            cmd_vel.angular.z = math.copysign(self.angular_speed, angle_diff)
            self.vel_publisher.publish(cmd_vel)

            rclpy.spin_once(self, timeout_sec=self.control_loop_rate)

        # Stop rotation
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
        self.get_logger().info("Rotation complete")

    def move_to_point(self, target_x: float, target_y: float) -> None:
        """Move the robot to a target point using proportional control."""
        self.get_logger().info(f"Moving to point: ({target_x:.2f}, {target_y:.2f})")

        # Wait for first odometry message
        self.get_logger().info("Waiting for first odometry update...")
        odom_wait_timeout = 10.0  # 10 seconds max wait
        odom_wait_start = time.time()
        initial_x = self.current_x
        initial_y = self.current_y

        while rclpy.ok() and (time.time() - odom_wait_start) < odom_wait_timeout:
            if self.current_x != initial_x or self.current_y != initial_y:
                self.get_logger().info(f"Odometry received! Current position: ({self.current_x:.2f}, {self.current_y:.2f})")
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_x == initial_x and self.current_y == initial_y:
            self.get_logger().warn("No odometry updates received! Robot position may be unknown.")

        self.get_logger().info(f"Starting movement. Current position: ({self.current_x:.2f}, {self.current_y:.2f})")

        start_time = time.time()
        timeout = 120.0  # 2 minute timeout per waypoint
        loop_count = 0

        while rclpy.ok():
            # Check timeout
            if time.time() - start_time > timeout:
                self.get_logger().warn(
                    f"Timeout reaching waypoint ({target_x:.2f}, {target_y:.2f})"
                )
                break

            # Calculate distance and angle to target
            distance = self.distance_to_point(target_x, target_y)
            target_angle = self.angle_to_point(target_x, target_y)
            angle_diff = self.normalize_angle(target_angle - self.current_yaw)

            # Log every 20 iterations (~1 second)
            loop_count += 1
            if loop_count % 20 == 0:
                self.get_logger().info(
                    f"[MOVE_STATUS] Distance: {distance:.3f}m, Current: ({self.current_x:.2f}, {self.current_y:.2f}), "
                    f"Angle diff: {math.degrees(angle_diff):.1f}°"
                )

            self.get_logger().debug(
                f"Distance: {distance:.3f}m, Angle diff: {math.degrees(angle_diff):.1f}°"
            )

            # Check if we've reached the target
            if distance < self.position_tolerance:
                self.get_logger().info(f"Reached point: ({target_x:.2f}, {target_y:.2f})")
                break

            # Create velocity command
            cmd_vel = Twist()

            # Linear motion with proportional control
            cmd_vel.linear.x = min(self.linear_speed, distance * 0.5)

            # Angular motion to correct heading
            cmd_vel.angular.z = math.copysign(
                min(self.angular_speed, abs(angle_diff) * 0.5),
                angle_diff
            )

            if loop_count % 20 == 0:
                self.get_logger().info(
                    f"[CMD_VEL] Publishing: linear={cmd_vel.linear.x:.3f} m/s, "
                    f"angular={cmd_vel.angular.z:.3f} rad/s"
                )

            self.vel_publisher.publish(cmd_vel)
            rclpy.spin_once(self, timeout_sec=self.control_loop_rate)

        # Stop the robot
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)

    def go_to_waypoint(self, x: float, y: float, theta: float = None) -> None:
        """Navigate to a waypoint and optionally orient to a target heading."""
        self.get_logger().info(f"Navigating to waypoint: ({x:.2f}, {y:.2f})")

        # Move to the point
        self.move_to_point(x, y)

        # Brief pause while processing callbacks
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)

        # If theta is specified, rotate to that orientation
        if theta is not None:
            self.rotate_to_angle(theta)

        # Brief pause at waypoint while processing callbacks
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

    def get_waypoints(self) -> List[Tuple[float, float, float]]:
        """Get waypoints based on selected pattern."""
        if self.pattern == "figure_eight":
            return self.get_figure_eight_waypoints()
        elif self.pattern == "spiral":
            return self.get_spiral_waypoints()
        else:
            return self.get_default_waypoints()

    @staticmethod
    def get_default_waypoints() -> List[Tuple[float, float, float]]:
        """Default rectangular patrol path for turtlebot3_world.

        Navigates around the obstacle grid starting from spawn point (-2.0, -0.5).
        The world has a 3x3 grid of cylindrical obstacles (radius 0.15) at positions:
        (-1.1, -1.1), (-1.1, 0), (-1.1, 1.1), (0, -1.1), (0, 0), (0, 1.1),
        (1.1, -1.1), (1.1, 0), (1.1, 1.1)

        This path creates a large rectangle in clear space around the perimeter,
        keeping at least 0.5m clearance from all obstacles.
        """
        return [
            # From spawn (-2.0, -0.5), first move FURTHER WEST to be extra safe
            (-2.2, -0.5, math.pi),
            # Now safely move south along far west
            (-2.2, -2.2, -math.pi/2),
            # Move to standard west corridor
            (-2.0, -2.2, 0.0),
            # Move east along bottom corridor (south of all obstacles)
            (-1.0, -2.2, 0.0),
            (0.0, -2.2, 0.0),
            (1.0, -2.2, 0.0),
            (2.0, -2.2, 0.0),
            # Move north along east corridor
            (2.0, -1.0, math.pi/2),
            (2.0, 0.0, math.pi/2),
            (2.0, 1.0, math.pi/2),
            (2.0, 2.2, math.pi/2),
            # Move west along top corridor (north of all obstacles)
            (1.0, 2.2, math.pi),
            (0.0, 2.2, math.pi),
            (-1.0, 2.2, math.pi),
            (-2.0, 2.2, math.pi),
            # Move south along west corridor (return to spawn area)
            (-2.0, 1.0, -math.pi/2),
            (-2.0, 0.0, -math.pi/2),
            (-2.0, -0.5, -math.pi/2),
        ]

    @staticmethod
    def get_figure_eight_waypoints() -> List[Tuple[float, float, float]]:
        """Figure-eight pattern using circular arcs."""
        waypoints = []

        # Right loop
        for angle in range(0, 360, 20):
            rad = math.radians(angle)
            x = 1.0 + 0.5 * math.cos(rad)
            y = 0.0 + 0.5 * math.sin(rad)
            waypoints.append((x, y, rad))

        # Left loop
        for angle in range(0, 360, 20):
            rad = math.radians(angle)
            x = -1.0 + 0.5 * math.cos(rad)
            y = 0.0 + 0.5 * math.sin(rad)
            waypoints.append((x, y, rad))

        return waypoints

    @staticmethod
    def get_spiral_waypoints() -> List[Tuple[float, float, float]]:
        """Spiral pattern expanding outward."""
        waypoints = []

        for i in range(12):
            angle = i * 0.5
            radius = i * 0.15
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            waypoints.append((x, y, angle))

        return waypoints

    def follow_waypoints(self, waypoints: List[Tuple[float, float, float]]) -> None:
        """Follow a sequence of waypoints."""
        for rep in range(self.repetitions if self.repetitions > 0 else 1):
            self.get_logger().info(
                f"======== Repetition {rep + 1}/{self.repetitions if self.repetitions > 0 else '∞'} ========"
            )

            for idx, (x, y, theta) in enumerate(waypoints):
                if not rclpy.ok():
                    break

                self.get_logger().info(
                    f"Waypoint {idx + 1}/{len(waypoints)}: ({x:.2f}, {y:.2f}), "
                    f"heading: {math.degrees(theta):.1f}°"
                )
                self.go_to_waypoint(x, y, theta)

            self.get_logger().info(
                f"Completed repetition {rep + 1}/{self.repetitions if self.repetitions > 0 else '∞'}"
            )

            # Pause between repetitions while processing callbacks
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.1)

            # For infinite repetitions, continue loop
            if self.repetitions <= 0:
                rep = 0  # Reset to keep loop going

    def stop_robot(self) -> None:
        """Stop the robot by publishing zero velocity."""
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
        self.get_logger().info("Robot stopped")

    def run(self) -> None:
        """Main navigation loop."""
        # Get waypoints based on pattern
        waypoints = self.get_waypoints()

        self.get_logger().info(f"Starting navigation with {len(waypoints)} waypoints")
        self.get_logger().info(f"Pattern: {self.pattern}")

        try:
            # Follow the waypoints
            self.follow_waypoints(waypoints)
            self.get_logger().info("Navigation complete!")
        except KeyboardInterrupt:
            self.get_logger().info("Navigation interrupted by user")
        finally:
            self.stop_robot()


def main(args=None):
    """Main entry point for the waypoint navigator node."""
    rclpy.init(args=args)

    navigator = TurtleBot3WaypointNavigatorTwist()

    try:
        navigator.run()
    except Exception as e:
        navigator.get_logger().error(f"Error during navigation: {e}")
    finally:
        navigator.stop_robot()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
