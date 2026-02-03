# Quick Start: TurtleBot 3 Waypoint Navigation

Get your TurtleBot 3 moving around waypoints in 5 minutes!

## 30-Second Setup

```bash
# Terminal 1: Launch simulation
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch Nav2 (wait 30 seconds for full startup)
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=$(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/maps/map.yaml

# Terminal 3: Run navigator
python3 turtlebot3_waypoint_navigator.py
```

That's it! Your robot is now patrolling the default rectangular path.

## Two Implementation Options

### Option 1: Nav2 Action Server (Recommended)
**File:** `turtlebot3_waypoint_navigator.py`

**Pros:**
- Robust path planning with obstacle avoidance
- Autonomous navigation with localization
- Production-ready

**Requirements:**
- Navigation2 stack running
- Map file available
- AMCL localization working

**Launch:**
```bash
python3 turtlebot3_waypoint_navigator.py
```

### Option 2: Direct Velocity Commands (Lightweight)
**File:** `turtlebot3_waypoint_navigator_twist.py`

**Pros:**
- No Nav2 needed
- Simpler, faster startup
- Good for testing and simple patterns

**Cons:**
- No built-in obstacle avoidance
- Less robust to localization errors

**Launch:**
```bash
python3 turtlebot3_waypoint_navigator_twist.py
```

## Common Commands

### Run for a fixed number of repetitions
```bash
python3 turtlebot3_waypoint_navigator.py --repetitions 5
```

### Change speed (0.1-0.5 m/s)
```bash
python3 turtlebot3_waypoint_navigator.py --speed 0.3
```

### Use figure-eight pattern
```bash
python3 turtlebot3_waypoint_navigator.py --pattern figure_eight
```

### Combine options
```bash
python3 turtlebot3_waypoint_navigator.py --pattern figure_eight --repetitions 3 --speed 0.25
```

## Modifying Waypoints

Edit the script to change the path. For `turtlebot3_waypoint_navigator.py`, find this section:

```python
def get_default_waypoints() -> List[Tuple[float, float, float]]:
    return [
        (0.5, 0.5, 0.0),           # x, y, theta (heading in radians)
        (1.5, 0.5, 0.0),
        (1.5, 1.5, math.pi/2),
        (0.5, 1.5, math.pi),
        (0.5, 0.5, -math.pi/2),
    ]
```

### Common Heading Values
```
0.0        → Facing East (→)
π/2 ≈ 1.57 → Facing North (↑)
π ≈ 3.14   → Facing West (←)
-π/2       → Facing South (↓)
```

### Example: Square with 5 waypoints
```python
def get_default_waypoints():
    return [
        (0.0, 0.0, 0.0),           # Corner 1
        (2.0, 0.0, 0.0),           # Corner 2
        (2.0, 2.0, math.pi/2),     # Corner 3
        (0.0, 2.0, math.pi),       # Corner 4
        (0.0, 0.0, -math.pi/2),    # Back to start
    ]
```

## Troubleshooting

### "Waiting for NavigateToPose action server..."
**Fix:** Navigation2 stack hasn't fully started. Wait 30-60 seconds and check Terminal 2.

### Robot doesn't move
**Fix:** 
1. Check that Gazebo is running (Terminal 1)
2. Verify map is loaded: `ros2 topic echo /map` should show data
3. Use RViz to set initial pose with "2D Pose Estimate" button

### Robot gets stuck at a waypoint
**Fix:** 
- Increase position tolerance: `--position-tolerance 0.2`
- Move waypoints further apart
- Restart Navigation2 stack

### Transform errors
**Fix:** Install `tf_transformations`:
```bash
pip install tf-transformations
```

## Monitoring the Robot

### Watch robot position in real-time
```bash
ros2 topic echo /amcl_pose
```

### Visualize in RViz
```bash
rviz2 -c $(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/rviz/nav2_default_view.rviz
```

### Check navigation status
```bash
ros2 topic echo /navigate_to_pose/_action/feedback
```

## Stop Navigation

Press `Ctrl+C` in the terminal running the navigator.

## Next Steps

- **Read full docs:** See `WAYPOINT_NAVIGATOR_README.md` for detailed configuration
- **Add custom patterns:** Create spiral or custom shapes by editing waypoint functions
- **Multi-robot:** Run separate navigator instances with `--namespace` flag
- **Integration:** Extend the script to trigger actions when reaching waypoints

## Script Files

- **`turtlebot3_waypoint_navigator.py`** - Full Nav2 implementation (recommended)
- **`turtlebot3_waypoint_navigator_twist.py`** - Lightweight velocity-based version
- **`WAYPOINT_NAVIGATOR_README.md`** - Complete documentation
- **`QUICKSTART_WAYPOINT.md`** - This file

## Support

For detailed setup instructions, troubleshooting, and advanced usage, see:
- `WAYPOINT_NAVIGATOR_README.md` - Complete guide
- ROS 2 docs: https://docs.ros.org/
- Navigation2 docs: https://navigation.ros.org/
- TurtleBot 3 manual: https://emanual.robotis.com/docs/en/platform/turtlebot3/

---

**Ready to go?** Run the 30-second setup above and your robot will start patrolling! 🤖
