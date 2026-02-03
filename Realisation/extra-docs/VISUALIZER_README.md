# TurtleBot3 Live Map Visualizer

A real-time command-line visualization tool for monitoring TurtleBot3 navigation status, displaying the robot's position, map, sensors, and navigation goals in an ASCII art interface.

## Overview

The Map Visualizer provides a live, terminal-based dashboard for monitoring your TurtleBot3 robot during autonomous navigation. It subscribes to key ROS2 topics and renders an intuitive ASCII visualization that updates in real-time.

## Features

- **Live Position Tracking**: Real-time display of robot position from both odometry and AMCL localization
- **ASCII Map View**: Top-down map visualization showing:
  - Robot position and orientation (with direction arrows)
  - Navigation goals (★)
  - Obstacles (█)
  - Free space (·)
  - Unknown areas (?)
- **Velocity Monitoring**: Current and commanded linear/angular velocities
- **Sensor Status**: LiDAR scan data and minimum obstacle distance
- **Topic Statistics**: Message counts and update rates for all subscribed topics
- **Health Indicators**: Visual status indicators showing data freshness

## Quick Start

### Using Docker (Recommended)

Start your waypoint navigator as usual, then in a new terminal:

```bash
./run_waypoint_navigator_docker.sh visualize
```

### Manual Usage (Inside Container)

```bash
# Enter the container
./run_waypoint_navigator_docker.sh shell

# Run the visualizer
/workspace/docker/scripts/run_visualizer.sh

# Or run directly with Python
source /opt/ros/humble/setup.bash
python3 /workspace/docker/scripts/map_visualizer.py
```

### Standalone ROS2 Environment

If running outside Docker with a native ROS2 installation:

```bash
cd docker/scripts
python3 map_visualizer.py
```

## ROS Topics Subscribed

The visualizer subscribes to the following ROS2 topics:

| Topic | Message Type | Purpose |
|-------|-------------|---------|
| `/map` | `nav_msgs/OccupancyGrid` | Occupancy grid map for visualization |
| `/odom` | `nav_msgs/Odometry` | Raw odometry data from robot |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Localized pose estimate |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan data |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands being sent to robot |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Current navigation goal |

## Display Elements

### Header Section
```
================================================================================
                  TurtleBot3 Live Navigation Status
================================================================================
```

### Position Information Box
Shows the robot's current position from multiple sources:
- **AMCL Pose**: Localized position (X, Y, θ) - most accurate
- **Odometry**: Raw wheel encoder position - may drift over time
- **Goal**: Target waypoint position and distance to goal

```
┌─ POSITION ─────────────────────────────────────────────────────────────────┐
│ AMCL Pose:    X:   1.234 m  Y:   0.567 m  θ:   45.0°                       │
│ Odometry:     X:   1.240 m  Y:   0.570 m  θ:   45.2°                       │
│ Goal:         X:   2.000 m  Y:   1.500 m  Dist:  1.123 m                   │
└────────────────────────────────────────────────────────────────────────────┘
```

### Velocity Information Box
Real-time velocity data:
- Current velocity (from odometry)
- Commanded velocity (from `/cmd_vel` topic)

```
┌─ VELOCITY ─────────────────────────────────────────────────────────────────┐
│ Linear:        0.150 m/s    (cmd:  0.150 m/s)                              │
│ Angular:       0.000 rad/s  (cmd:  0.000 rad/s)                            │
└────────────────────────────────────────────────────────────────────────────┘
```

### Sensor Information Box
LiDAR data summary:

```
┌─ SENSORS ──────────────────────────────────────────────────────────────────┐
│ Lidar:        360 points   Min distance: 0.523 m                           │
└────────────────────────────────────────────────────────────────────────────┘
```

### ASCII Map View
Live top-down visualization centered on the robot:

```
┌─ MAP VIEW ─────────────────────────────────────────────────────────────────┐
│???????????????????????···················································│
│???????????????????????···················································│
│???????????????????????···················································│
│???????????????????????···················································│
│???????????????????????···················································│
│??????????????????█████████··············································│
│??????????????????█████████··············································│
│??????????????????███→███··············································│
│??????????????????█████████··············································│
│??????????????????█████████··············································│
│???????????????????????···················································│
│???????????????????????················★······························│
│???????????????????????···················································│
│???????????????????????···················································│
│                                                                              │
│  Legend:  → Robot   ★ Goal   █ Obstacle   · Free   ? Unknown                │
└────────────────────────────────────────────────────────────────────────────┘
```

**Map Symbols:**
- `→ ↑ ↓ ← ↗ ↖ ↘ ↙` - Robot (arrow shows orientation)
- `★` - Navigation goal
- `█` - Occupied space (obstacles, walls)
- `▓` - Possibly occupied
- `·` - Free space
- `?` - Unknown/unexplored area

### Topic Statistics Box
Shows message counts and data freshness:

```
┌─ TOPIC STATISTICS ─────────────────────────────────────────────────────────┐
│ /map:     42  /odom:   1234  /amcl_pose:    567                            │
│ /scan:   1234  /cmd_vel:  1234  /goal_pose:     1                          │
│ Status: 🟢 ACTIVE  (Last update: 0.1s ago)                                  │
└────────────────────────────────────────────────────────────────────────────┘
```

**Status Indicators:**
- 🟢 **ACTIVE**: Data received within last 1 second
- 🔴 **STALE**: No data for more than 1 second

## Usage Examples

### Example 1: Basic Monitoring

```bash
# Terminal 1: Start navigation
./run_waypoint_navigator_docker.sh start

# Terminal 2: Monitor with visualizer
./run_waypoint_navigator_docker.sh visualize
```

### Example 2: Debugging Navigation Issues

The visualizer is particularly useful for debugging:

1. **Localization Issues**: Check if AMCL pose is updating and matches odometry
2. **Goal Tracking**: Verify the goal position and distance are reasonable
3. **Obstacle Detection**: See if obstacles are being detected in the map
4. **Velocity Verification**: Confirm commands are being sent and executed

### Example 3: Multi-Terminal Setup

Recommended terminal layout:

```
┌─────────────────┬─────────────────┐
│   Simulator     │   Visualizer    │
│   (logs)        │   (live map)    │
├─────────────────┴─────────────────┤
│          RViz (optional)          │
└───────────────────────────────────┘
```

Commands:
```bash
# Terminal 1: View simulation logs
./run_waypoint_navigator_docker.sh logs

# Terminal 2: Run visualizer
./run_waypoint_navigator_docker.sh visualize

# Terminal 3: Launch RViz (optional)
./run_waypoint_navigator_docker.sh start --rviz
```

## Configuration

The visualizer can be customized by editing `/docker/scripts/map_visualizer.py`:

### Update Rate
Change the display refresh rate (default: 5 Hz):

```python
time.sleep(0.2)  # 5 Hz update rate (line ~450)
```

### Map Size
Adjust the ASCII map dimensions:

```python
map_lines = self.render_mini_map(width=78, height=20)  # Default size
```

### Cell Resolution
Change the meters-per-character ratio:

```python
cell_size = 0.1  # meters per character (default)
```

## Troubleshooting

### No Data Appearing

**Problem**: Visualizer shows "No AMCL pose data available yet..."

**Solutions**:
1. Verify simulation is running:
   ```bash
   ./run_waypoint_navigator_docker.sh status
   ```

2. Check ROS topics are publishing:
   ```bash
   ./run_waypoint_navigator_docker.sh monitor
   ```

3. Verify ROS connectivity inside container:
   ```bash
   ./run_waypoint_navigator_docker.sh shell
   ros2 topic list
   ros2 topic echo /amcl_pose --once
   ```

### Garbled Display

**Problem**: ASCII characters not rendering correctly

**Solutions**:
1. Ensure terminal supports UTF-8:
   ```bash
   echo $LANG  # Should show UTF-8
   export LANG=en_US.UTF-8
   ```

2. Use a modern terminal emulator (avoid basic terminals)

3. Increase terminal size (minimum 80 columns recommended)

### Status Shows "STALE"

**Problem**: Red "STALE" indicator appears

**Causes**:
- Navigation paused or stopped
- Robot waiting at waypoint
- Network issues between containers

**Solutions**:
1. Check if navigation is active:
   ```bash
   ./run_waypoint_navigator_docker.sh logs waypoint-navigator
   ```

2. Verify containers are healthy:
   ```bash
   docker compose -f docker/docker-compose.waypoint-navigator.yaml ps
   ```

### Map Not Visible

**Problem**: Map shows only `?` characters

**Solutions**:
1. Wait for map to be published (can take 10-30 seconds after startup)
2. Check map topic:
   ```bash
   ros2 topic echo /map --once
   ```
3. Verify SLAM/navigation is running

### Slow Performance

**Problem**: Visualizer updates slowly or lags

**Solutions**:
1. Reduce update rate (increase sleep time)
2. Decrease map size
3. Check system resources:
   ```bash
   ./run_waypoint_navigator_docker.sh status
   ```

## Technical Details

### Dependencies

- Python 3.8+
- ROS2 Humble
- rclpy (ROS2 Python client library)
- Standard Python libraries: math, time, threading, collections

### Architecture

The visualizer uses a multi-threaded architecture:

1. **ROS Thread**: Runs `rclpy.spin()` to receive messages from topics
2. **Display Thread**: Runs the rendering loop to update the terminal

This ensures smooth display updates even if message rates vary.

### Performance

- Update rate: 5 Hz (200ms between frames)
- Map rendering: ~10-20ms per frame
- Memory footprint: ~50-100 MB
- CPU usage: <5% on modern systems

## Integration with Other Tools

### RViz Complement

The visualizer complements RViz:
- **Visualizer**: Quick, lightweight, terminal-based monitoring
- **RViz**: Full 3D visualization with rich graphics

Use both together:
```bash
# Start with RViz
./run_waypoint_navigator_docker.sh start --rviz

# Add terminal visualizer
./run_waypoint_navigator_docker.sh visualize
```

### Logging

Combine with log monitoring:
```bash
# Terminal 1: Logs
./run_waypoint_navigator_docker.sh logs | grep -E "(Goal|Waypoint|AMCL)"

# Terminal 2: Visualizer
./run_waypoint_navigator_docker.sh visualize
```

## Advanced Usage

### Custom Topics

To monitor additional topics, edit `map_visualizer.py` and add subscriptions:

```python
self.custom_sub = self.create_subscription(
    MessageType,
    '/your_topic',
    self.custom_callback,
    10
)
```

### Export Screenshots

Capture the display output:
```bash
./run_waypoint_navigator_docker.sh visualize | tee visualization.txt
```

### Remote Monitoring

Monitor a robot over SSH:
```bash
ssh user@robot-hostname
cd /path/to/project
./run_waypoint_navigator_docker.sh visualize
```

## See Also

- [WAYPOINT_NAVIGATOR_README.md](WAYPOINT_NAVIGATOR_README.md) - Main navigation documentation
- [RUN_WAYPOINT_DOCKER.md](RUN_WAYPOINT_DOCKER.md) - Docker setup guide
- [QUICKSTART_WAYPOINT.md](QUICKSTART_WAYPOINT.md) - Quick start guide

## Contributing

To improve the visualizer:

1. **Add Features**: Edit `/docker/scripts/map_visualizer.py`
2. **Test Changes**: Run with `./run_waypoint_navigator_docker.sh visualize`
3. **Update Docs**: Add notes here

Potential enhancements:
- Path history visualization
- Battery/resource monitoring
- Multi-robot support
- Color themes
- Performance graphs

## License

This tool is part of the RoboSapiens Adaptive Platform project and shares the same license.

---

**Questions or Issues?**

If you encounter problems with the visualizer:
1. Check this troubleshooting guide
2. Review the ROS2 logs: `./run_waypoint_navigator_docker.sh logs`
3. Open an issue with the output of: `./run_waypoint_navigator_docker.sh status`
