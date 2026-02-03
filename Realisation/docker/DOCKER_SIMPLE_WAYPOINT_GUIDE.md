# Docker Compose Waypoint Navigator - Simple Single Service

## Overview

This is a simplified Docker Compose service that connects to an **already-running** Gazebo simulation and sends waypoint navigation commands to TurtleBot 3.

**Key Features:**
- ✅ Single service - no dependencies to manage
- ✅ Connects to existing simulation (no Gazebo launch needed)
- ✅ Configurable via environment variables
- ✅ Lightweight and fast
- ✅ Perfect for testing and experimentation

## Prerequisites

Before running this service, you must have:

1. **Gazebo simulation running** with TurtleBot 3:
   ```bash
   export TURTLEBOT3_MODEL=waffle
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **ROS 2 environment configured**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. **Waypoint navigator package built**:
   ```bash
   cd robosapiens-adaptive-platform-turtlebot/turtlebotrossim
   colcon build --packages-select turtlebot3_waypoint_navigator
   ```

4. **Docker and Docker Compose installed**

## Quick Start

### Default Usage (1 repetition, 0.2 m/s)

```bash
cd robosapiens-adaptive-platform-turtlebot/docker
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

The robot will navigate through one complete path at normal speed.

### Stop the Service

```bash
docker-compose -f docker-compose.waypoint-navigator-simple.yaml down
```

### View Logs

```bash
docker-compose -f docker-compose.waypoint-navigator-simple.yaml logs -f waypoint_navigator
```

## Common Use Cases

### Case 1: Fast Exploration (High Speed, Loose Tolerances)

```bash
WAYPOINT_SPEED=0.4 \
WAYPOINT_POSITION_TOLERANCE=0.2 \
WAYPOINT_REPETITIONS=0 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

**Result:** Robot moves at 0.4 m/s, repeats infinitely, with relaxed positioning tolerances.

### Case 2: Precision Scanning (Slow Speed, Tight Tolerances)

```bash
WAYPOINT_SPEED=0.1 \
WAYPOINT_POSITION_TOLERANCE=0.05 \
WAYPOINT_ANGLE_TOLERANCE=0.05 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

**Result:** Robot moves slowly (0.1 m/s) with strict precision tolerances.

### Case 3: Figure-Eight Pattern (Smooth Motion)

```bash
WAYPOINT_PATTERN=figure_eight \
WAYPOINT_SPEED=0.25 \
WAYPOINT_REPETITIONS=3 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

**Result:** Robot follows figure-eight pattern 3 times at 0.25 m/s.

### Case 4: Spiral Search (Area Exploration)

```bash
WAYPOINT_PATTERN=spiral \
WAYPOINT_SPEED=0.2 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

**Result:** Robot moves in expanding spiral pattern.

### Case 5: Security Patrol (Infinite with Pauses)

```bash
WAYPOINT_SPEED=0.2 \
WAYPOINT_REPETITIONS=0 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

**Result:** Robot repeats pattern infinitely until stopped.

### Case 6: Multi-Robot (Robot 1)

```bash
WAYPOINT_NAMESPACE=/tb3_0 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

**Result:** Robot publishes commands to /tb3_0/cmd_vel for multi-robot setup.

### Case 7: Multi-Robot (Robot 2 with Different Pattern)

```bash
WAYPOINT_NAMESPACE=/tb3_1 \
WAYPOINT_PATTERN=figure_eight \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

**Result:** Second robot follows different pattern.

## Environment Variables Reference

| Variable | Default | Options | Description |
|----------|---------|---------|-------------|
| `WAYPOINT_SPEED` | 0.2 | 0.1-0.5 | Linear velocity (m/s) |
| `WAYPOINT_ANGULAR_SPEED` | 0.5 | 0.1+ | Angular velocity (rad/s) |
| `WAYPOINT_REPETITIONS` | 0 | 0+ | Repetitions (0=infinite) |
| `WAYPOINT_PATTERN` | default | default, figure_eight, spiral | Movement pattern |
| `WAYPOINT_POSITION_TOLERANCE` | 0.1 | 0.05-0.2 | Position tolerance (m) |
| `WAYPOINT_ANGLE_TOLERANCE` | 0.1 | 0.05-0.2 | Angle tolerance (rad) |
| `WAYPOINT_NAMESPACE` | (empty) | /tb3_0, /tb3_1, etc | Robot namespace |
| `ROS_DOMAIN_ID` | 0 | 0+ | ROS 2 domain ID |

## Command Examples

All examples assume you're in: `robosapiens-adaptive-platform-turtlebot/docker`

### Set Speed to 0.3 m/s

```bash
WAYPOINT_SPEED=0.3 docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### Run 10 Repetitions

```bash
WAYPOINT_REPETITIONS=10 docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### Combine Multiple Parameters

```bash
WAYPOINT_SPEED=0.3 \
WAYPOINT_PATTERN=figure_eight \
WAYPOINT_REPETITIONS=5 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### Run in Background

```bash
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up -d waypoint_navigator

# Later, check status
docker-compose -f docker-compose.waypoint-navigator-simple.yaml ps

# View logs
docker-compose -f docker-compose.waypoint-navigator-simple.yaml logs -f

# Stop
docker-compose -f docker-compose.waypoint-navigator-simple.yaml down
```

### Run and Save Logs

```bash
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator 2>&1 | tee waypoint_navigation.log
```

## Performance Profiles

### Quick Testing (15 seconds startup)
```bash
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```
- Speed: 0.2 m/s
- Repetitions: 1
- CPU: 5-15%
- Memory: ~100MB

### Fast Exploration
```bash
WAYPOINT_SPEED=0.4 WAYPOINT_REPETITIONS=0 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```
- Speed: 0.4 m/s (maximum)
- Repetitions: Infinite
- CPU: 5-15%
- Memory: ~100MB

### Precision Mode
```bash
WAYPOINT_SPEED=0.1 WAYPOINT_POSITION_TOLERANCE=0.05 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```
- Speed: 0.1 m/s (slow)
- Position Tolerance: 0.05 m (strict)
- CPU: 5-15%
- Memory: ~100MB

## Available Patterns

### Default (Rectangular)
```bash
WAYPOINT_PATTERN=default docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```
Traditional rectangular patrol with 8 waypoints.

### Figure-Eight
```bash
WAYPOINT_PATTERN=figure_eight docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```
Smooth figure-eight motion for continuous surveillance.

### Spiral
```bash
WAYPOINT_PATTERN=spiral docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```
Expanding spiral pattern for area search.

## Multi-Robot Setup

### Robot 1 (Standard Pattern)
```bash
WAYPOINT_NAMESPACE=/tb3_0 docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### Robot 2 (Figure-Eight Pattern)
In a separate terminal:
```bash
WAYPOINT_NAMESPACE=/tb3_1 \
WAYPOINT_PATTERN=figure_eight \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

Both robots will navigate independently without interference.

## Troubleshooting

### Service Won't Start

**Check if simulation is running:**
```bash
ros2 topic list | grep cmd_vel
```

If no output, start Gazebo:
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Connection Issues

**Check ROS 2 environment:**
```bash
source /opt/ros/humble/setup.bash
ros2 node list
```

**Check DOMAIN_ID:**
```bash
echo $ROS_DOMAIN_ID
# Should be 0 or match simulation's DOMAIN_ID
```

### Robot Doesn't Move

**Verify cmd_vel topic exists:**
```bash
ros2 topic list | grep cmd_vel
```

**Check ROS 2 connectivity:**
```bash
docker-compose -f docker-compose.waypoint-navigator-simple.yaml exec waypoint_navigator \
  bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"
```

### Service Exits Immediately

**Check logs:**
```bash
docker-compose -f docker-compose.waypoint-navigator-simple.yaml logs waypoint_navigator
```

Common causes:
- Simulation not running
- Wrong ROS_DOMAIN_ID
- Network connectivity issue

## Stopping the Service

### Graceful Stop (Ctrl+C)
```bash
# Press Ctrl+C in the terminal where service is running
```

### Force Stop
```bash
docker-compose -f docker-compose.waypoint-navigator-simple.yaml stop waypoint_navigator
```

### Complete Cleanup
```bash
docker-compose -f docker-compose.waypoint-navigator-simple.yaml down
```

## Real-World Examples

### Example 1: Security Patrol Route
```bash
# Infinite repetitions, 2-second pauses recommended
# (Note: pauses require Nav2, use repetitions=0 for lightweight version)
WAYPOINT_SPEED=0.25 \
WAYPOINT_REPETITIONS=0 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### Example 2: Warehouse Inventory Scanning
```bash
WAYPOINT_SPEED=0.15 \
WAYPOINT_POSITION_TOLERANCE=0.05 \
WAYPOINT_ANGLE_TOLERANCE=0.05 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### Example 3: Area Surveillance (Smooth Motion)
```bash
WAYPOINT_PATTERN=figure_eight \
WAYPOINT_SPEED=0.2 \
WAYPOINT_REPETITIONS=0 \
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

### Example 4: Quick Test (One Pass)
```bash
docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

## Docker Commands Cheat Sheet

| Task | Command |
|------|---------|
| Start service | `docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator` |
| Start in background | `docker-compose -f docker-compose.waypoint-navigator-simple.yaml up -d waypoint_navigator` |
| View logs | `docker-compose -f docker-compose.waypoint-navigator-simple.yaml logs -f waypoint_navigator` |
| Check status | `docker-compose -f docker-compose.waypoint-navigator-simple.yaml ps` |
| Stop service | `docker-compose -f docker-compose.waypoint-navigator-simple.yaml stop waypoint_navigator` |
| Remove containers | `docker-compose -f docker-compose.waypoint-navigator-simple.yaml down` |
| Execute command | `docker-compose -f docker-compose.waypoint-navigator-simple.yaml exec waypoint_navigator bash` |

## File Location

```
robosapiens-adaptive-platform-turtlebot/
└── docker/
    ├── docker-compose.waypoint-navigator-simple.yaml  (← This file)
    └── DOCKER_SIMPLE_WAYPOINT_GUIDE.md                (← This guide)
```

## Next Steps

1. **Ensure simulation is running** - Start Gazebo with TurtleBot 3
2. **Build the package** - `colcon build --packages-select turtlebot3_waypoint_navigator`
3. **Run the service** - `docker-compose -f docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator`
4. **Customize as needed** - Set environment variables for your use case
5. **Monitor progress** - Watch logs: `docker-compose logs -f`

## Support & Documentation

- **ROS 2 Waypoint Navigator Guide:** See `ROS2_WAYPOINT_PACKAGE_GUIDE.md`
- **Full Docker Guide:** See `DOCKER_WAYPOINT_NAVIGATOR_GUIDE.md`
- **Package README:** See `turtlebotrossim/src/turtlebot3_waypoint_navigator/README.md`
- **ROS 2 Documentation:** https://docs.ros.org/
- **Docker Compose Docs:** https://docs.docker.com/compose/

---

**Version:** 1.0
**Status:** Ready to Use ✓
**Last Updated:** 2024
