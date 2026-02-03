# TurtleBot 3 Waypoint Navigator

This document provides complete instructions for using the TurtleBot 3 waypoint navigator script to make your robot follow a repeating path through the default `turtlebot3_world`.

## Overview

The `turtlebot3_waypoint_navigator.py` script uses the ROS 2 Navigation2 stack to navigate TurtleBot 3 through a predefined sequence of waypoints in a continuous loop. The robot will:

1. Navigate to each waypoint in sequence
2. Orient itself to a specified heading at each waypoint
3. Pause briefly at each waypoint
4. Loop back to the start and repeat the pattern

## Prerequisites

### Required Packages

- **ROS 2** (Humble or Iron recommended)
- **TurtleBot 3 packages**:
  - `turtlebot3`
  - `turtlebot3_gazebo`
  - `turtlebot3_navigation2`
- **Navigation2 stack**
- **Gazebo simulator**
- **Python packages**:
  - `rclpy` (usually comes with ROS 2)
  - `tf_transformations` - Install with: `pip install tf-transformations`
  - `geometry_msgs` (comes with ROS 2)

### Installation

```bash
# Install TurtleBot 3 packages (if not already installed)
sudo apt-get install ros-humble-turtlebot3
sudo apt-get install ros-humble-turtlebot3-gazebo
sudo apt-get install ros-humble-turtlebot3-navigation2
sudo apt-get install ros-humble-navigation2

# Install Python dependencies
pip install tf-transformations
```

### Environment Setup

Set up the ROS 2 environment variables:

```bash
# Add to your .bashrc or run in each terminal
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
# or: export TURTLEBOT3_MODEL=burger
```

## Launch Sequence

You need to launch three separate processes in order:

### Terminal 1: Start Gazebo Simulation with TurtleBot 3 World

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Wait for Gazebo to fully load (you should see the TurtleBot 3 robot in the world).

### Terminal 2: Start Navigation2 Stack

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=/path/to/map.yaml
```

**Note**: If you don't have a map, you'll need to use SLAM first:

```bash
# Option A: Use the included demo map
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=$(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/maps/map.yaml
```

Or build a map using SLAM:

```bash
# Terminal 2A: Start SLAM
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
```

### Terminal 3: Run the Waypoint Navigator

```bash
source /opt/ros/humble/setup.bash
python3 /path/to/turtlebot3_waypoint_navigator.py
```

## Usage

### Basic Usage

```bash
python3 turtlebot3_waypoint_navigator.py
```

This will start navigation with default settings.

### Command Line Arguments

The script supports the following command-line arguments:

```
--repetitions N          Number of times to repeat the path (default: 1)
--speed M                Linear speed in m/s (default: 0.2)
--angular-speed R        Angular speed in rad/s (default: 0.5)
--namespace NS           Robot namespace for multi-robot setups (e.g., /tb3_0)
--pattern NAME           Waypoint pattern: "default" or "figure_eight" (default: default)
--position-tolerance T   Position tolerance in meters (default: 0.1)
--angle-tolerance A      Angle tolerance in radians (default: 0.1)
```

### Examples

**Navigate through default pattern 5 times at 0.3 m/s:**
```bash
python3 turtlebot3_waypoint_navigator.py --repetitions 5 --speed 0.3
```

**Use figure-eight pattern 3 times:**
```bash
python3 turtlebot3_waypoint_navigator.py --pattern figure_eight --repetitions 3
```

**Multi-robot scenario:**
```bash
python3 turtlebot3_waypoint_navigator.py --namespace /tb3_0 --speed 0.2
```

**Custom tolerances for precise navigation:**
```bash
python3 turtlebot3_waypoint_navigator.py --position-tolerance 0.05 --angle-tolerance 0.05
```

**Continuous navigation with faster speed:**
```bash
python3 turtlebot3_waypoint_navigator.py --repetitions 0 --speed 0.4
```

## Modifying Waypoints

To customize the path your robot follows, edit the waypoint definitions in the script.

### Default Waypoints

The default waypoints form a rectangular patrol pattern. They are defined in the `get_default_waypoints()` function:

```python
def get_default_waypoints() -> List[Tuple[float, float, float]]:
    return [
        (0.5, 0.5, 0.0),           # x, y, theta (yaw in radians)
        (1.5, 0.5, 0.0),
        (1.5, 1.5, math.pi/2),
        (0.5, 1.5, math.pi),
        (0.5, 0.5, -math.pi/2),
    ]
```

Each tuple contains:
- **x**: X coordinate in meters
- **y**: Y coordinate in meters
- **theta**: Target heading/yaw in radians (0 = facing East, π/2 = facing North, π = facing West, etc.)

### Adding Custom Waypoints

Edit the script and modify the waypoint list:

```python
def get_default_waypoints():
    return [
        (0.0, 0.0, 0.0),           # Start position
        (2.0, 0.0, 0.0),           # Move right
        (2.0, 2.0, math.pi/2),     # Turn and move forward
        (0.0, 2.0, math.pi),       # Turn and move left
        (0.0, 0.0, -math.pi/2),    # Return to start
    ]
```

### Finding Waypoint Coordinates

1. **In Gazebo**: Move the robot manually using teleoperation or drag it in the GUI
2. **From ROS Topics**: Subscribe to `/amcl_pose` (or `/odom`) to get current position
3. **Using RViz**: 
   - Open RViz: `rviz2`
   - Add the Map display
   - Use "Publish Point" tool to click on desired locations
   - The coordinates will be printed to console

Command to check current robot position:

```bash
ros2 topic echo /amcl_pose
```

## Understanding Angles/Theta

The `theta` parameter represents the robot's desired heading (yaw angle) in radians:

```
North (↑):    π/2  or 1.571
East  (→):    0
West  (←):    ±π   or ±3.142
South (↓):   -π/2  or -1.571
```

### Convert degrees to radians:

```python
import math
degrees = 45
radians = math.radians(degrees)  # Result: 0.785
```

Common angles:
- 0° = 0 rad (East)
- 45° = 0.785 rad (NE)
- 90° = 1.571 rad (North)
- 180° = 3.142 rad (West)
- 270° = -1.571 rad (South)

## Monitoring Navigation

### ROS 2 Topics to Monitor

```bash
# Robot position and orientation
ros2 topic echo /amcl_pose

# Odometry feedback
ros2 topic echo /odom

# Navigation goal status
ros2 topic echo /navigate_to_pose/_action/goal

# Navigation feedback
ros2 topic echo /navigate_to_pose/_action/feedback
```

### RViz Visualization

Visualize the robot's navigation:

```bash
rviz2 -c $(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/rviz/nav2_default_view.rviz
```

## Stopping the Navigator

Press `Ctrl+C` in the terminal running the waypoint navigator script to stop and cancel the current navigation goal.

## Troubleshooting

### Issue: "Waiting for NavigateToPose action server..."

**Problem**: The Navigation2 stack hasn't fully started.

**Solution**: 
- Ensure Terminal 2 has fully launched the navigation stack (wait 30-60 seconds)
- Check that Gazebo is running in Terminal 1
- Verify map is loaded correctly

### Issue: Robot doesn't move or gets stuck

**Possible causes**:
1. Map not loaded or outdated
2. Robot initial position not aligned with map
3. Obstacles in path
4. Navigation2 parameters not optimized

**Solutions**:
- Restart Gazebo and Navigation2
- Use RViz to manually set the robot's initial pose (2D Pose Estimate tool)
- Rebuild the map using SLAM
- Check `/tf` and `/map` topics are being published

### Issue: Robot overshoots or navigates inefficiently

**Solutions**:
- Reduce linear speed: `--speed 0.15`
- Increase position tolerance: `--position-tolerance 0.2`
- Adjust Navigation2 parameters in `/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml`

### Issue: "Failed to load map"

**Solution**: Ensure the map file exists and path is correct. Use the included demo map:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  map:=$(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/maps/map.yaml
```

### Issue: tf2 transformation errors

**Solution**: Ensure `tf_transformations` is installed:

```bash
pip install tf-transformations
```

## Performance Tuning

### Speed Adjustment

- **Slow (0.1 m/s)**: Very safe, precise navigation, good for tight spaces
- **Normal (0.2-0.3 m/s)**: Recommended default
- **Fast (0.4-0.5 m/s)**: Faster navigation, requires well-tuned parameters

### Tolerances

- **Strict (0.05 m, 0.05 rad)**: Precise positioning, slower overall speed
- **Normal (0.1 m, 0.1 rad)**: Balanced approach
- **Relaxed (0.2 m, 0.2 rad)**: Faster navigation, less precise

### Navigation2 Parameter Tuning

Edit `/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml` for advanced tuning:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    use_sim_time: true
    
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    max_controller_duration: 15.0
```

## Advanced Usage

### Multi-Robot Navigation

For multiple TurtleBots:

```bash
# Terminal 3A: Robot 1
python3 turtlebot3_waypoint_navigator.py --namespace /tb3_0 --speed 0.2

# Terminal 3B: Robot 2
python3 turtlebot3_waypoint_navigator.py --namespace /tb3_1 --speed 0.2
```

### Custom Waypoint Patterns

Extend the script to add more patterns:

```python
def get_spiral_waypoints():
    waypoints = []
    for i in range(10):
        angle = i * 0.5
        radius = i * 0.2
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        waypoints.append((x, y, angle))
    return waypoints
```

## Performance Metrics

The script logs:
- **Loop count**: Number of complete cycles through all waypoints
- **Distance to waypoint**: Helps verify navigation accuracy
- **Total waypoints traversed**: Useful for monitoring robot activity

Example output:
```
Starting navigation with 5 waypoints
[Loop 1] Navigating to Start (0.50, 0.50)
Distance to waypoint: 0.52m
Distance to waypoint: 0.25m
Reached waypoint: (0.50, 0.50)
Pausing at waypoint for 1.0 second(s)
[Loop 1] Navigating to Point 1 (1.50, 0.50)
...
```

## References

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Navigation2 Documentation](https://navigation.ros.org/)
- [TurtleBot 3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- [Gazebo Documentation](https://gazebosim.org/)

## Support

For issues or questions:
1. Check the Troubleshooting section above
2. Review ROS 2 logs: `ros2 topic list` and `ros2 node list`
3. Check Navigation2 action feedback for specific errors
4. Consult the TurtleBot 3 documentation
