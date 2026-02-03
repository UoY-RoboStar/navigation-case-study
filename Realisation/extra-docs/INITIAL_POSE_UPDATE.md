# Initial Pose Estimate Feature - TurtleBot3 Waypoint Navigator

## Overview

The TurtleBot3 waypoint navigator nodes have been updated to automatically set an initial pose estimate for the robot before starting navigation. This is crucial for proper localization using AMCL (Adaptive Monte Carlo Localization) in the Navigation2 stack.

## Why Initial Pose is Important

When using Nav2 with AMCL for localization, the robot needs to know its approximate starting position on the map. Without setting an initial pose:
- AMCL may take a long time to converge on the correct position
- The robot might get lost or fail to navigate properly
- Navigation goals may be rejected or fail

By setting an initial pose estimate, you tell AMCL approximately where the robot is located and its orientation, allowing for faster and more reliable localization.

## Changes Made

### 1. Standalone Script (`turtlebot3_waypoint_navigator.py`)

**New Features:**
- Added `PoseWithCovarianceStamped` publisher to `/initialpose` topic
- Created `set_initial_pose()` method to publish initial pose with covariance
- Added configurable initial pose parameters in `__init__()` method
- Automatic initial pose publishing before navigation starts
- Command-line arguments for customizing initial pose

**Default Initial Pose:**
- X: 0.0 m
- Y: 0.0 m
- Theta: 0.0 rad (facing East)

### 2. Package Version (`navigator_nav2.py`)

The packaged version already had initial pose functionality and has been verified to work correctly with the following defaults:
- X: -2.0 m
- Y: -0.5 m
- Theta: 0.0 rad

## Usage

### Basic Usage (Default Pose)

```bash
# Launch the navigator with default initial pose (0, 0, 0)
python3 turtlebot3_waypoint_navigator.py
```

### Custom Initial Pose

You can specify a custom initial pose using command-line arguments:

```bash
# Set initial pose to x=1.5, y=2.0, theta=1.57 (90 degrees)
python3 turtlebot3_waypoint_navigator.py \
    --initial-x 1.5 \
    --initial-y 2.0 \
    --initial-theta 1.57
```

### For Different Robot Spawn Locations

If your robot spawns at a different location in Gazebo, you should set the initial pose to match:

```bash
# Example: Robot spawns at (-2.0, -0.5) facing right
python3 turtlebot3_waypoint_navigator.py \
    --initial-x -2.0 \
    --initial-y -0.5 \
    --initial-theta 0.0
```

### Help Information

```bash
python3 turtlebot3_waypoint_navigator.py --help
```

## Technical Details

### Initial Pose Message

The node publishes a `PoseWithCovarianceStamped` message to the `/initialpose` topic with:

**Header:**
- `frame_id`: "map"
- `stamp`: Current ROS time

**Pose:**
- Position (x, y, z)
- Orientation (quaternion converted from theta/yaw)

**Covariance Matrix:**
The covariance represents uncertainty in the initial pose estimate:
- X variance: 0.25 m² (±0.5 m standard deviation)
- Y variance: 0.25 m² (±0.5 m standard deviation)
- Theta variance: 0.06853 rad² (±0.26 rad or ±15° standard deviation)

### Timing

After publishing the initial pose, the node:
1. Publishes the message to `/initialpose`
2. Waits 2 seconds for AMCL to process the pose
3. Starts navigation to the first waypoint

This delay ensures AMCL has time to update its particle filter before navigation commands are sent.

## Integration with Nav2

The initial pose is consumed by the AMCL node in Nav2, which uses it to:
1. Initialize the particle filter around the specified pose
2. Begin tracking the robot's position based on odometry and laser scans
3. Continuously refine the pose estimate as the robot moves

## Complete Startup Sequence

1. **Terminal 1**: Launch Gazebo simulation
   ```bash
   export TURTLEBOT3_MODEL=waffle
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **Terminal 2**: Launch Navigation2 stack (wait 30 seconds for full startup)
   ```bash
   export TURTLEBOT3_MODEL=waffle
   ros2 launch turtlebot3_navigation2 navigation2.launch.py \
       use_sim_time:=true \
       map:=$(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/maps/map.yaml
   ```

3. **Terminal 3**: Run the waypoint navigator (now automatically sets initial pose)
   ```bash
   python3 turtlebot3_waypoint_navigator.py
   ```

## Verifying Initial Pose

You can verify the initial pose was set correctly by:

1. **Check logs**: Look for the message "Initial pose published successfully"
2. **RViz**: Open RViz and observe the particle cloud converge around the initial pose
3. **Topic echo**: Monitor the `/initialpose` topic:
   ```bash
   ros2 topic echo /initialpose
   ```

## Troubleshooting

### Robot doesn't move after setting initial pose

- **Check AMCL**: Ensure AMCL node is running in Nav2 stack
- **Verify map frame**: Make sure the map frame exists and transforms are publishing
- **Wait longer**: Increase the sleep time after `set_initial_pose()` if needed

### Initial pose doesn't match robot location

- **Update coordinates**: Use `--initial-x`, `--initial-y`, `--initial-theta` arguments
- **Check spawn location**: Verify where the robot actually spawns in Gazebo
- **Use RViz tool**: Manually set initial pose in RViz to find correct coordinates

### Navigation fails immediately

- **Covariance too small**: If the initial pose is very uncertain, increase covariance values
- **Wrong orientation**: Make sure theta is in radians, not degrees (90° = 1.57 rad)

## Customizing Initial Pose in Code

To modify the default initial pose in the code, edit the `__init__()` method:

```python
self.initial_pose = {
    "x": -2.0,     # Your X coordinate
    "y": -0.5,     # Your Y coordinate
    "theta": 0.0   # Your orientation in radians
}
```

Or adjust the command-line argument defaults in the `main()` function.

## Best Practices

1. **Match spawn location**: Set initial pose to match where the robot spawns in simulation
2. **Reasonable covariance**: Use covariance values that reflect your actual uncertainty
3. **Wait for AMCL**: Always allow time for AMCL to process the initial pose
4. **Verify in RViz**: Check that particle cloud converges before starting navigation
5. **Test different poses**: If navigation fails, try slightly different initial poses

## Related Topics

- `/initialpose` - Topic for setting initial pose
- `/amcl_pose` - Topic for current AMCL pose estimate
- `/particlecloud` - Visualization of AMCL particle filter
- AMCL configuration in Nav2 parameter files

## References

- [Nav2 AMCL Documentation](https://navigation.ros.org/configuration/packages/configuring-amcl.html)
- [PoseWithCovarianceStamped Message](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)
- [TurtleBot3 Navigation Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/)
