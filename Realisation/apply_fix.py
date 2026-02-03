#!/usr/bin/env python3
"""
Script to apply the initial pose fix to navigator_nav2.py

This script modifies the navigator_nav2.py file to:
1. Add AMCL pose subscription for verification
2. Wait for AMCL convergence before starting navigation
3. Increase wait time and add proper logging

Usage:
    python3 apply_fix.py
"""

import os
import re
import sys


def apply_fix():
    """Apply the fix to navigator_nav2.py"""

    file_path = "turtlebotrossim/src/turtlebot3_waypoint_navigator/turtlebot3_waypoint_navigator/navigator_nav2.py"

    if not os.path.exists(file_path):
        print(f"Error: File not found: {file_path}")
        print("Please run this script from the repository root directory")
        return False

    print(f"Reading {file_path}...")
    with open(file_path, 'r') as f:
        content = f.read()

    # Backup original file
    backup_path = file_path + ".backup"
    print(f"Creating backup at {backup_path}...")
    with open(backup_path, 'w') as f:
        f.write(content)

    # Fix 1: Add AMCL pose tracking in __init__
    print("Applying Fix 1: Adding AMCL pose subscriber...")
    init_addition = """
        # Subscriber for AMCL pose to verify localization
        self.amcl_pose_received = False
        self.latest_amcl_pose = None
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

"""

    # Find the location after initial_pose_pub creation
    pattern1 = r"(self\.initial_pose_pub = self\.create_publisher\([^)]+\))\s*\n"
    if re.search(pattern1, content):
        content = re.sub(
            pattern1,
            r"\1\n" + init_addition,
            content
        )
        print("  ✓ Added AMCL pose subscriber to __init__")
    else:
        print("  ✗ Could not find initial_pose_pub location")
        return False

    # Fix 2: Add AMCL pose callback method
    print("Applying Fix 2: Adding AMCL pose callback...")
    callback_method = """
    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        \"\"\"Callback when AMCL publishes pose estimate.\"\"\"
        if not self.amcl_pose_received:
            self.get_logger().info(
                f'AMCL pose received: '
                f'({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})'
            )
        self.amcl_pose_received = True
        self.latest_amcl_pose = msg

"""

    # Add callback after _wait_for_nav2_server method
    pattern2 = r"(self\.get_logger\(\)\.info\('NavigateToPose action server is available!'\)\s*\n)"
    if re.search(pattern2, content):
        content = re.sub(
            pattern2,
            r"\1" + callback_method,
            content
        )
        print("  ✓ Added AMCL pose callback method")
    else:
        print("  ✗ Could not find location for callback method")
        return False

    # Fix 3: Update set_initial_pose to publish multiple times
    print("Applying Fix 3: Publishing initial pose multiple times...")
    old_publish = r"for _ in range\(3\):\s+self\.initial_pose_pub\.publish\(initial_pose\)\s+time\.sleep\(0\.1\)"
    new_publish = """for i in range(10):
            self.initial_pose_pub.publish(initial_pose)
            time.sleep(0.1)"""

    if re.search(old_publish, content):
        content = re.sub(old_publish, new_publish, content)
        print("  ✓ Updated initial pose publishing (3 -> 10 times)")
    else:
        print("  ✗ Could not find initial pose publishing loop")

    # Fix 4: Replace start_navigation method with improved version
    print("Applying Fix 4: Replacing start_navigation method...")

    new_start_navigation = '''    def start_navigation(self) -> None:
        """Start the waypoint navigation sequence."""
        self.get_logger().info('Starting waypoint navigation...')

        # Set initial pose for AMCL localization (robot spawn position)
        self.set_initial_pose(x=-2.0, y=-0.5, theta=0.0)

        # Wait for AMCL to converge
        self.get_logger().info('Waiting for AMCL to converge...')
        timeout = 30.0  # 30 second timeout
        start_time = time.time()
        wait_count = 0

        # Actively wait for AMCL pose
        while not self.amcl_pose_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            wait_count += 1
            if wait_count % 10 == 0:  # Log every second
                elapsed = time.time() - start_time
                self.get_logger().info(f'Still waiting for AMCL... ({elapsed:.1f}s elapsed)')

        if not self.amcl_pose_received:
            self.get_logger().error(
                'AMCL did not publish pose within timeout! '
                'Navigation may fail. Check if AMCL is running and map is loaded.'
            )
            # Continue anyway, but warn user
            self.get_logger().info('Waiting 10 more seconds before attempting navigation...')
            time.sleep(10.0)
        else:
            self.get_logger().info(
                f'AMCL converged successfully! '
                f'Robot localized at ({self.latest_amcl_pose.pose.pose.position.x:.2f}, '
                f'{self.latest_amcl_pose.pose.pose.position.y:.2f})'
            )
            # Additional wait for stability
            self.get_logger().info('Waiting 3 more seconds for particle filter to stabilize...')
            time.sleep(3.0)

        self.get_logger().info('Starting navigation to first waypoint...')
        self.navigate_to_waypoint(0)
'''

    # Find and replace the start_navigation method
    pattern4 = r'def start_navigation\(self\) -> None:.*?self\.navigate_to_waypoint\(0\)'
    if re.search(pattern4, content, re.DOTALL):
        content = re.sub(
            pattern4,
            new_start_navigation.strip(),
            content,
            flags=re.DOTALL
        )
        print("  ✓ Replaced start_navigation method with improved version")
    else:
        print("  ✗ Could not find start_navigation method")
        return False

    # Write the modified content
    print(f"Writing modified content to {file_path}...")
    with open(file_path, 'w') as f:
        f.write(content)

    print("\n" + "="*60)
    print("SUCCESS! Fix applied successfully!")
    print("="*60)
    print(f"\nOriginal file backed up to: {backup_path}")
    print("\nNext steps:")
    print("1. Rebuild the package:")
    print("   cd turtlebotrossim")
    print("   colcon build --packages-select turtlebot3_waypoint_navigator")
    print("   source install/setup.bash")
    print("\n2. Test the fix:")
    print("   ros2 run turtlebot3_waypoint_navigator waypoint_navigator_nav2")
    print("\n3. Verify AMCL convergence in the logs")
    print("   You should see: 'AMCL converged successfully!'")
    print("\n")

    return True

if __name__ == "__main__":
    print("="*60)
    print("Initial Pose Fix Application Script")
    print("="*60)
    print()

    if apply_fix():
        sys.exit(0)
    else:
        print("\n" + "="*60)
        print("FAILED! Could not apply fix")
        print("="*60)
        print("\nPlease check the error messages above.")
        print("You may need to apply the fix manually.")
        sys.exit(1)
