# Quick Start: Initial Pose Estimate

## What's New

The TurtleBot3 waypoint navigator now **automatically sets an initial pose estimate** before starting navigation. This ensures AMCL (localization) knows where the robot is on the map, resulting in reliable navigation from the start.

## Quick Usage

### Default Initial Pose (0, 0, 0)

```bash
# Just run the script - initial pose is set automatically!
python3 turtlebot3_waypoint_navigator.py
```

The robot will:
1. Set initial pose to (0, 0) facing East (0 radians)
2. Wait 2 seconds for AMCL to process
3. Begin navigating waypoints

### Custom Initial Pose

If your robot spawns at a different location, specify it:

```bash
# Robot spawns at (-2.0, -0.5) facing right
python3 turtlebot3_waypoint_navigator.py \
    --initial-x -2.0 \
    --initial-y -0.5 \
    --initial-theta 0.0
```

```bash
# Robot spawns at (1.5, 2.0) facing up (90 degrees = 1.57 radians)
python3 turtlebot3_waypoint_navigator.py \
    --initial-x 1.5 \
    --initial-y 2.0 \
    --initial-theta 1.57
```

## Finding Your Robot's Spawn Location

### Method 1: Check Gazebo
1. Launch Gazebo simulation
2. Look at robot's position in the world
3. Use those coordinates as `--initial-x` and `--initial-y`

### Method 2: Check Launch File
Look in your Gazebo launch file for spawn parameters:
```xml
<arg name="x_pos" default="-2.0"/>
<arg name="y_pos" default="-0.5"/>
<arg name="yaw" default="0.0"/>
```

### Method 3: Use RViz
1. Launch Nav2 and RViz
2. Use "2D Pose Estimate" tool to manually set pose
3. Watch the terminal output for coordinates
4. Use those values in your script

## Common Scenarios

### Scenario 1: TurtleBot3 World (Default Spawn)
```bash
# Default spawn at origin
python3 turtlebot3_waypoint_navigator.py
```

### Scenario 2: TurtleBot3 House
```bash
# Typical house world spawn
python3 turtlebot3_waypoint_navigator.py \
    --initial-x -2.0 \
    --initial-y -0.5 \
    --initial-theta 0.0
```

### Scenario 3: Custom World
```bash
# Check your world's spawn location first!
python3 turtlebot3_waypoint_navigator.py \
    --initial-x <your_x> \
    --initial-y <your_y> \
    --initial-theta <your_theta>
```

## Orientation (Theta) Reference

Theta is in **radians**, not degrees!

| Direction | Degrees | Radians | Theta Value |
|-----------|---------|---------|-------------|
| East (→)  | 0°      | 0       | `0.0`       |
| North (↑) | 90°     | π/2     | `1.57`      |
| West (←)  | 180°    | π       | `3.14`      |
| South (↓) | 270°    | 3π/2    | `4.71`      |

### Quick Conversion
```python
import math
theta_radians = math.radians(degrees)  # Convert degrees to radians
```

## Complete Launch Sequence

```bash
# Terminal 1: Gazebo
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Nav2 (wait 30 seconds after launching)
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=$(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/maps/map.yaml

# Terminal 3: Waypoint Navigator with initial pose
python3 turtlebot3_waypoint_navigator.py --initial-x 0.0 --initial-y 0.0 --initial-theta 0.0
```

## Verification in RViz

After the script starts, check RViz:

1. **Particle Cloud**: Should converge around the robot (red particles)
2. **Robot Position**: Green robot model should align with Gazebo position
3. **Global Costmap**: Should show obstacles correctly oriented
4. **Navigation Path**: Should plan from correct starting position

## Troubleshooting

### ❌ Robot doesn't move after initial pose
- **Wait longer**: AMCL may need more time to localize
- **Check coordinates**: Verify initial pose matches spawn location
- **Check Nav2**: Ensure Navigation2 is fully started (wait 30 seconds)

### ❌ Initial pose seems wrong in RViz
- **Check frame**: Make sure using "map" frame in RViz
- **Verify spawn**: Double-check robot spawn location in Gazebo
- **Try manually**: Use RViz "2D Pose Estimate" tool to set correct pose

### ❌ Navigation fails immediately
- **Wrong coordinates**: Initial pose doesn't match actual position
- **Wrong orientation**: Make sure theta is in radians (not degrees!)
- **AMCL not ready**: Wait longer for AMCL to fully initialize

### ❌ Particle cloud is scattered everywhere
- **Covariance too large**: Edit `set_initial_pose()` method to reduce uncertainty
- **Wrong pose**: Initial pose is far from actual position
- **Need time**: Give AMCL more time to converge (increase sleep time)

## Advanced: Modifying Default Pose in Code

Edit the `__init__` method to change default values:

```python
def __init__(self, initial_x=0.0, initial_y=0.0, initial_theta=0.0):
    super().__init__('turtlebot3_waypoint_navigator')
    
    # Change these defaults
    self.initial_pose = {
        "x": initial_x,
        "y": initial_y,
        "theta": initial_theta
    }
```

Or modify command-line argument defaults in `main()`:

```python
parser.add_argument(
    '--initial-x',
    type=float,
    default=-2.0,  # Change this
    help='Initial X position for pose estimate'
)
```

## Testing Different Poses

Quick test to find the right initial pose:

```bash
# Try different positions quickly
for x in 0.0 -1.0 -2.0; do
    python3 turtlebot3_waypoint_navigator.py --initial-x $x --initial-y 0.0 --initial-theta 0.0
    sleep 5  # Run for 5 seconds
    # Press Ctrl+C to try next
done
```

## Package Version (ROS 2 node)

The package version `navigator_nav2.py` also has initial pose functionality:

```bash
ros2 run turtlebot3_waypoint_navigator waypoint_navigator_nav2
```

It uses hardcoded initial pose (-2.0, -0.5, 0.0) which you can modify in the code.

## Key Takeaways

✅ **Automatic**: Initial pose is set automatically when script starts  
✅ **Configurable**: Use `--initial-x`, `--initial-y`, `--initial-theta` arguments  
✅ **Essential**: Required for AMCL localization to work properly  
✅ **Radians**: Always use radians for theta, not degrees  
✅ **Match spawn**: Initial pose should match robot's spawn location  

## Need Help?

View all options:
```bash
python3 turtlebot3_waypoint_navigator.py --help
```

See detailed documentation:
- `INITIAL_POSE_UPDATE.md` - Complete technical documentation
- `README_WAYPOINT_NAVIGATOR.md` - Full waypoint navigator guide
- `QUICKSTART_WAYPOINT.md` - General waypoint navigation quick start
