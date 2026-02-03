# TurtleBot 3 Waypoint Navigator - Docker Setup Guide

This guide explains how to run the TurtleBot 3 Waypoint Navigator using Docker Compose.

## Prerequisites

Before running the waypoint navigator in Docker, ensure:

1. **TurtleBot 3 Gazebo simulation is running** on the host machine
   ```bash
   # Terminal 1: Start the simulation
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **Docker and Docker Compose are installed** on your system
   ```bash
   docker --version
   docker compose version
   ```

3. **ROS 2 Humble is installed** on the host machine (for sourcing)

4. **The workspace is built** (will be auto-built on first run if not present)
   ```bash
   cd turtlebotrossim
   source /opt/ros/humble/setup.bash
   colcon build --packages-select turtlebot3_waypoint_navigator
   ```

## Quick Start

### Method 1: Using the Helper Script (Recommended)

From the project root directory:

```bash
# Basic usage with defaults
./docker/run-waypoint-navigator.sh

# With custom parameters
./docker/run-waypoint-navigator.sh \
  --speed 0.15 \
  --repetitions 1 \
  --position-tolerance 0.05 \
  --angle-tolerance 0.05

# Figure-eight pattern
./docker/run-waypoint-navigator.sh --pattern figure_eight --repetitions 5

# Infinite spiral at high speed
./docker/run-waypoint-navigator.sh --pattern spiral --repetitions 0 --speed 0.4

# Show help
./docker/run-waypoint-navigator.sh --help
```

### Method 2: Using Docker Compose Directly

From the project root directory:

```bash
# Basic usage
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator

# With environment variables
WAYPOINT_SPEED=0.15 \
WAYPOINT_POSITION_TOLERANCE=0.05 \
WAYPOINT_ANGLE_TOLERANCE=0.05 \
WAYPOINT_REPETITIONS=1 \
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator

# Detached mode (runs in background)
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up -d waypoint_navigator

# Stop the service
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml down
```

## Configuration Parameters

All parameters can be set via environment variables:

| Parameter | Environment Variable | Default | Description |
|-----------|----------------------|---------|-------------|
| Speed | `WAYPOINT_SPEED` | 0.2 | Linear velocity in m/s |
| Angular Speed | `WAYPOINT_ANGULAR_SPEED` | 0.5 | Angular velocity in rad/s |
| Repetitions | `WAYPOINT_REPETITIONS` | 1 | Number of times to repeat path (0 = infinite) |
| Pattern | `WAYPOINT_PATTERN` | default | Path pattern: `default`, `figure_eight`, `spiral` |
| Position Tolerance | `WAYPOINT_POSITION_TOLERANCE` | 0.1 | Distance tolerance to waypoint in meters |
| Angle Tolerance | `WAYPOINT_ANGLE_TOLERANCE` | 0.1 | Angle tolerance in radians |
| Namespace | `WAYPOINT_NAMESPACE` | (empty) | ROS namespace for topics |
| ROS Domain ID | `ROS_DOMAIN_ID` | 0 | ROS 2 Domain ID for communication |

## Examples

### Example 1: Slow Exploration (Safety)
```bash
WAYPOINT_SPEED=0.15 \
WAYPOINT_ANGULAR_SPEED=0.3 \
WAYPOINT_POSITION_TOLERANCE=0.05 \
WAYPOINT_ANGLE_TOLERANCE=0.05 \
WAYPOINT_REPETITIONS=1 \
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### Example 2: Fast Figure-Eight Pattern
```bash
WAYPOINT_SPEED=0.4 \
WAYPOINT_PATTERN=figure_eight \
WAYPOINT_REPETITIONS=3 \
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### Example 3: Continuous Spiral Navigation
```bash
WAYPOINT_SPEED=0.25 \
WAYPOINT_PATTERN=spiral \
WAYPOINT_REPETITIONS=0 \
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### Example 4: Using the Helper Script with Options
```bash
./docker/run-waypoint-navigator.sh \
  --speed 0.2 \
  --angular-speed 0.4 \
  --pattern figure_eight \
  --repetitions 2 \
  --position-tolerance 0.08 \
  --angle-tolerance 0.08
```

## Navigation Patterns

### Default Pattern
A rectangular patrol path suitable for the TurtleBot3 World simulation:
- (0.5, 0.5) → (1.5, 0.5) → (1.5, 1.5) → (0.5, 1.5) → back to start

### Figure-Eight Pattern
A smooth figure-eight path with circular arcs:
- Right loop followed by left loop
- Good for testing curved navigation

### Spiral Pattern
An expanding spiral starting from the origin:
- Gradually expands outward in a spiral
- Good for area exploration

## Monitoring the Navigation

### View Docker Logs
```bash
# Live logs
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml logs -f waypoint_navigator

# Last 100 lines
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml logs --tail 100 waypoint_navigator
```

### Check Robot Status in Separate Terminal
```bash
# List ROS 2 nodes
ros2 node list

# Show active topics
ros2 topic list

# Monitor odometry
ros2 topic echo /odom

# Monitor velocity commands
ros2 topic echo /cmd_vel
```

## Troubleshooting

### Issue: "ROS 2 environment did not become ready"
**Cause**: The Gazebo simulation is not running on the host  
**Solution**: Start the simulation before running the container:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Issue: "turtlebot3_waypoint_navigator package not found"
**Cause**: The package hasn't been built yet  
**Solution**: The Docker container should auto-build it. If it doesn't, build manually:
```bash
cd turtlebotrossim
source /opt/ros/humble/setup.bash
colcon build --packages-select turtlebot3_waypoint_navigator
```

### Issue: Robot not moving
**Cause**: Could be several issues:
1. Gazebo simulation not running
2. Network connectivity between host and container
3. `/cmd_vel` topic not being published

**Solution**: Check logs and verify ROS 2 communication:
```bash
# In a host terminal, check if cmd_vel commands are reaching Gazebo
ros2 topic echo /cmd_vel

# Check odometry updates
ros2 topic echo /odom
```

### Issue: Container can't connect to ROS 2
**Cause**: ROS Domain ID mismatch  
**Solution**: Ensure ROS_DOMAIN_ID matches on host and container:
```bash
# On host, check your ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# Use the same ID when running:
ROS_DOMAIN_ID=0 docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### Issue: Permission denied when running helper script
**Cause**: Script is not executable  
**Solution**: Make it executable:
```bash
chmod +x docker/run-waypoint-navigator.sh
```

## Advanced Usage

### Running in Detached Mode
```bash
# Start in background
WAYPOINT_SPEED=0.2 docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up -d waypoint_navigator

# Check status
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml ps

# View logs
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml logs -f waypoint_navigator

# Stop when done
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml down
```

### Using Different ROS Domains
```bash
# Run on domain 1
ROS_DOMAIN_ID=1 docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### Custom Namespace
```bash
# Use /robot1 namespace for topics
WAYPOINT_NAMESPACE=/robot1 \
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator

# Robot will use /robot1/cmd_vel and /robot1/odom
```

## Performance Tuning

### Tolerances
- **Tighter tolerances** (e.g., 0.02): More precise navigation, slower
- **Looser tolerances** (e.g., 0.2): Faster navigation, less precise

### Speed
- **Slow** (0.1-0.2 m/s): Safe, good for testing, tight spaces
- **Medium** (0.2-0.4 m/s): Balanced, typical use
- **Fast** (0.4+ m/s): Quick exploration, may skip waypoints if set too high

### Angular Speed
- Usually 2-3x the linear speed for smooth turns
- Increase if robot hesitates at waypoints

## File Structure

```
docker/
├── docker-compose.waypoint-navigator-simple.yaml  # Main compose file
├── run-waypoint-navigator.sh                      # Helper script
└── ... (other files)

turtlebotrossim/
├── src/
│   └── turtlebot3_waypoint_navigator/
│       ├── turtlebot3_waypoint_navigator/
│       │   ├── __init__.py
│       │   ├── navigator_twist.py                 # Main navigator implementation
│       │   └── navigator_nav2.py                  # Alternative Nav2 implementation
│       ├── setup.py
│       └── package.xml
├── install/                                        # Generated after build
└── build/                                          # Generated after build
```

## Next Steps

1. **Verify host setup**: Ensure Gazebo simulation is running
2. **Build the workspace** (if not already done):
   ```bash
   cd turtlebotrossim && colcon build --packages-select turtlebot3_waypoint_navigator
   ```
3. **Start the navigator**: Use either the helper script or docker compose command
4. **Monitor the robot**: Watch the Gazebo window and check logs
5. **Adjust parameters** as needed for your use case

## Additional Resources

- [TurtleBot 3 Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Docker Documentation](https://docs.docker.com/)
- [Docker Compose Reference](https://docs.docker.com/compose/compose-file/)

## Support

For issues or questions about the waypoint navigator, refer to:
- `WAYPOINT_NAVIGATOR_README.md` - Detailed navigator documentation
- `QUICKSTART_WAYPOINT.md` - Quick start guide
- `WAYPOINT_EXAMPLES.md` - Additional usage examples
