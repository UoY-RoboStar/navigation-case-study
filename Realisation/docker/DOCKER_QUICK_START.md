# Docker Waypoint Navigator - Quick Start Guide

## Prerequisites

Before running the Docker service, you must have:

1. **Gazebo simulation running** with TurtleBot 3:
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. **ROS 2 environment sourced** (on host):
```bash
source /opt/ros/humble/setup.bash
```

3. **Waypoint navigator package built**:
```bash
cd robosapiens-adaptive-platform-turtlebot/turtlebotrossim
source /opt/ros/humble/setup.bash
colcon build --packages-select turtlebot3_waypoint_navigator
```

## Start the Service

Navigate to the docker directory:
```bash
cd robosapiens-adaptive-platform-turtlebot/docker
```

### Default Usage (0.2 m/s, 1 repetition)

```bash
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### Stop the Service

```bash
docker-compose -f docker-compose.waypoint-navigator-simple.yaml down
```

## 10 Common Use Cases

All commands should be run from `robosapiens-adaptive-platform-turtlebot/docker`

### 1. Default Test (1 pass, normal speed)
```bash
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### 2. Fast Exploration (0.4 m/s, infinite loop)
```bash
WAYPOINT_SPEED=0.4 WAYPOINT_REPETITIONS=0 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### 3. Precision Scanning (0.1 m/s, strict tolerances)
```bash
WAYPOINT_SPEED=0.1 WAYPOINT_POSITION_TOLERANCE=0.05 WAYPOINT_ANGLE_TOLERANCE=0.05 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### 4. Figure-Eight Pattern (3 repetitions)
```bash
WAYPOINT_PATTERN=figure_eight WAYPOINT_REPETITIONS=3 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### 5. Spiral Search
```bash
WAYPOINT_PATTERN=spiral WAYPOINT_SPEED=0.2 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### 6. Security Patrol (continuous, medium speed)
```bash
WAYPOINT_SPEED=0.25 WAYPOINT_REPETITIONS=0 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### 7. Warehouse Scanning (slow, precise, single pass)
```bash
WAYPOINT_SPEED=0.15 WAYPOINT_POSITION_TOLERANCE=0.05 WAYPOINT_ANGLE_TOLERANCE=0.05 WAYPOINT_REPETITIONS=1 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### 8. Multi-Robot Robot 1
```bash
WAYPOINT_NAMESPACE=/tb3_0 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### 9. Multi-Robot Robot 2 (Different Pattern)
```bash
WAYPOINT_NAMESPACE=/tb3_1 WAYPOINT_PATTERN=figure_eight \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### 10. Run in Background
```bash
WAYPOINT_SPEED=0.3 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up -d waypoint_navigator

# Later, view logs:
docker-compose -f docker-compose.waypoint-navigator-simple.yaml logs -f waypoint_navigator

# Later, stop:
docker-compose -f docker-compose.waypoint-navigator-simple.yaml stop waypoint_navigator
```

## Environment Variables

| Variable | Default | Description | Examples |
|----------|---------|-------------|----------|
| `WAYPOINT_SPEED` | 0.2 | Linear velocity (m/s) | 0.1 (slow), 0.2 (normal), 0.4 (fast) |
| `WAYPOINT_ANGULAR_SPEED` | 0.5 | Angular velocity (rad/s) | 0.3, 0.5, 1.0 |
| `WAYPOINT_REPETITIONS` | 1 | Repetitions (0=infinite) | 1 (single), 3 (three times), 0 (infinite) |
| `WAYPOINT_PATTERN` | default | Movement pattern | default, figure_eight, spiral |
| `WAYPOINT_POSITION_TOLERANCE` | 0.1 | Position tolerance (m) | 0.05 (strict), 0.1 (normal), 0.2 (loose) |
| `WAYPOINT_ANGLE_TOLERANCE` | 0.1 | Angle tolerance (rad) | 0.05, 0.1, 0.2 |
| `WAYPOINT_NAMESPACE` | (empty) | Robot namespace | /tb3_0, /tb3_1 |
| `ROS_DOMAIN_ID` | 0 | ROS 2 domain ID | 0, 1, 2, etc |

## Monitoring

### View Live Logs
```bash
docker-compose -f docker-compose.waypoint-navigator-simple.yaml logs -f waypoint_navigator
```

### Check Service Status
```bash
docker-compose -f docker-compose.waypoint-navigator-simple.yaml ps
```

### View Last 50 Lines
```bash
docker-compose -f docker-compose.waypoint-navigator-simple.yaml logs --tail=50 waypoint_navigator
```

## Troubleshooting

### Error: "cmd_vel topic not found"
**Solution:** Make sure Gazebo is running:
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Error: "Package not built"
**Solution:** Build the package:
```bash
cd robosapiens-adaptive-platform-turtlebot/turtlebotrossim
source /opt/ros/humble/setup.bash
colcon build --packages-select turtlebot3_waypoint_navigator
```

### Error: "ROS 2 environment not available"
**Solution:** Source ROS 2 on the host:
```bash
source /opt/ros/humble/setup.bash
```

### Container exits immediately
**Solution:** Check the logs:
```bash
docker-compose -f docker-compose.waypoint-navigator-simple.yaml logs waypoint_navigator
```

## Speed Profiles

### Exploration (Fast)
- Speed: 0.4 m/s
- Position Tolerance: 0.2 m
- Repetitions: 0 (infinite)
```bash
WAYPOINT_SPEED=0.4 WAYPOINT_POSITION_TOLERANCE=0.2 WAYPOINT_REPETITIONS=0 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### Standard (Balanced)
- Speed: 0.2 m/s
- Position Tolerance: 0.1 m
- Repetitions: 1
```bash
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### Precision (Accurate)
- Speed: 0.1 m/s
- Position Tolerance: 0.05 m
- Repetitions: 1
```bash
WAYPOINT_SPEED=0.1 WAYPOINT_POSITION_TOLERANCE=0.05 WAYPOINT_ANGLE_TOLERANCE=0.05 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

## Docker Commands Reference

| Task | Command |
|------|---------|
| Start service | `docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator` |
| Start in background | `docker-compose -f docker-compose.waypoint-navigator-simple.yaml up -d waypoint_navigator` |
| View logs | `docker-compose -f docker-compose.waypoint-navigator-simple.yaml logs -f waypoint_navigator` |
| Check status | `docker-compose -f docker-compose.waypoint-navigator-simple.yaml ps` |
| Stop service | `docker-compose -f docker-compose.waypoint-navigator-simple.yaml stop waypoint_navigator` |
| Remove containers | `docker-compose -f docker-compose.waypoint-navigator-simple.yaml down` |
| Open shell | `docker-compose -f docker-compose.waypoint-navigator-simple.yaml exec waypoint_navigator bash` |

## Real-World Examples

### Security Patrol Route
```bash
WAYPOINT_SPEED=0.25 WAYPOINT_REPETITIONS=0 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```
Continuous patrol at moderate speed.

### Warehouse Inventory
```bash
WAYPOINT_SPEED=0.15 WAYPOINT_POSITION_TOLERANCE=0.05 WAYPOINT_ANGLE_TOLERANCE=0.05 WAYPOINT_REPETITIONS=1 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```
Slow, precise, single pass for scanning.

### Area Surveillance
```bash
WAYPOINT_PATTERN=figure_eight WAYPOINT_SPEED=0.2 WAYPOINT_REPETITIONS=0 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```
Smooth continuous motion in figure-eight pattern.

## File Locations

- Service file: `robosapiens-adaptive-platform-turtlebot/docker/docker-compose.waypoint-navigator-simple.yaml`
- This guide: `robosapiens-adaptive-platform-turtlebot/docker/DOCKER_QUICK_START.md`
- Full guide: `robosapiens-adaptive-platform-turtlebot/docker/DOCKER_SIMPLE_WAYPOINT_GUIDE.md`
- ROS 2 guide: `robosapiens-adaptive-platform-turtlebot/turtlebotrossim/ROS2_WAYPOINT_PACKAGE_GUIDE.md`
- Package: `robosapiens-adaptive-platform-turtlebot/turtlebotrossim/src/turtlebot3_waypoint_navigator/`

## Summary

1. **Ensure Gazebo is running** - Launch it in one terminal
2. **Build the package** - One-time setup with colcon
3. **Run the Docker service** - Use the docker-compose command
4. **Customize** - Use environment variables to adjust behavior
5. **Monitor** - View logs to see progress

**That's it!** Your robot will start navigating waypoints.
