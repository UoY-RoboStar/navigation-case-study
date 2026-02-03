# TurtleBot 3 Waypoint Navigator - Docker Compose Guide

A comprehensive guide for running the waypoint navigator using Docker Compose with pre-configured services for common use cases.

## Overview

This guide explains how to use the Docker Compose services defined in `docker-compose.waypoint-navigator.yaml` to run various TurtleBot 3 waypoint navigation scenarios.

## Quick Start

### Prerequisites

1. Docker and Docker Compose installed
2. NVIDIA Docker runtime (optional, for GPU acceleration)
3. X11 display server configured (for visualization)
4. Built turtlebot3_waypoint_navigator ROS 2 package

### Build the Package

Before running any services, build the waypoint navigator package:

```bash
cd robosapiens-adaptive-platform-turtlebot/turtlebotrossim
source /opt/ros/humble/setup.bash
colcon build --packages-select turtlebot3_waypoint_navigator
```

### Run a Service

```bash
cd robosapiens-adaptive-platform-turtlebot/docker
docker-compose -f docker-compose.waypoint-navigator.yaml up <service-name>
```

## Available Services

### 1. Single Robot - Full Nav2 (Production)

**Service Name:** `waypoint_nav2_single_robot`

**Description:** Production-grade navigation with path planning and obstacle avoidance using Navigation2 stack.

**Launch:**
```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up waypoint_nav2_single_robot
```

**Configuration:**
- Uses Navigation2 action server
- Full path planning and obstacle avoidance
- AMCL-based localization
- Infinite loops (max_loops=0)
- 1.0 second pause at each waypoint

**Use Case:** Security patrols, production deployments, complex environments

**Best For:** Real-world deployments with dynamic obstacles

---

### 2. Single Robot - Lightweight Twist (Testing)

**Service Name:** `waypoint_twist_single_robot`

**Description:** Simple velocity command-based navigation without Navigation2 dependency.

**Launch:**
```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up waypoint_twist_single_robot
```

**Configuration:**
- Direct Twist velocity commands
- No Navigation2 required
- Speed: 0.2 m/s
- Single repetition
- Standard rectangular pattern
- Position tolerance: 0.1m
- Angle tolerance: 0.1rad

**Use Case:** Testing, learning, simple environments

**Best For:** Quick experimentation and development

---

### 3. Figure-Eight Pattern

**Service Name:** `waypoint_twist_figure_eight`

**Description:** Continuous figure-eight motion for smooth area surveillance.

**Launch:**
```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up waypoint_twist_figure_eight
```

**Configuration:**
- Figure-eight pattern
- Speed: 0.25 m/s
- 3 repetitions
- Smooth continuous motion
- Standard tolerances

**Use Case:** Surveillance, camera coverage optimization, smooth patrol routes

**Best For:** Continuous monitoring applications

---

### 4. Fast Exploration Mode

**Service Name:** `waypoint_twist_fast_exploration`

**Description:** High-speed area coverage with relaxed tolerances.

**Launch:**
```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up waypoint_twist_fast_exploration
```

**Configuration:**
- Speed: 0.4 m/s (max safe speed)
- 5 repetitions
- Position tolerance: 0.2m (relaxed)
- Angle tolerance: 0.2rad (relaxed)
- Fast waypoint transitions

**Use Case:** Quick area surveys, rapid reconnaissance, exploration

**Best For:** Speed-optimized scenarios where precision isn't critical

---

### 5. Precision Navigation Mode

**Service Name:** `waypoint_twist_precision_mode`

**Description:** Slow, accurate navigation for precise positioning tasks.

**Launch:**
```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up waypoint_twist_precision_mode
```

**Configuration:**
- Speed: 0.1 m/s (slow, stable)
- Single repetition
- Position tolerance: 0.05m (strict)
- Angle tolerance: 0.05rad (strict)
- Careful positioning at each waypoint

**Use Case:** Warehouse scanning, precise docking, measurement tasks

**Best For:** Accuracy-critical applications

---

### 6. Multi-Robot Coordinator

**Service Name:** `waypoint_nav2_multi_robot_coordinator`

**Description:** Central monitoring and orchestration node for multi-robot scenarios.

**Launch:**
```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up waypoint_nav2_multi_robot_coordinator
```

**Use Case:** Multi-robot mission planning, centralized monitoring

**Best For:** Complex multi-robot deployments

---

### 7. Multi-Robot Robot 1

**Service Name:** `waypoint_twist_multi_robot_1`

**Description:** First robot in multi-robot patrol (standard rectangular pattern).

**Launch:**
```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up waypoint_twist_multi_robot_1
```

**Configuration:**
- Namespace: /tb3_0
- Pattern: default (rectangular)
- Speed: 0.2 m/s
- Infinite repetitions
- ROS_DOMAIN_ID=0

**Run with Robot 2:**
```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up \
  waypoint_twist_multi_robot_1 \
  waypoint_twist_multi_robot_2
```

---

### 8. Multi-Robot Robot 2

**Service Name:** `waypoint_twist_multi_robot_2`

**Description:** Second robot in multi-robot patrol (figure-eight pattern for offset coverage).

**Launch:**
```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up waypoint_twist_multi_robot_2
```

**Configuration:**
- Namespace: /tb3_1
- Pattern: figure_eight (different from Robot 1)
- Speed: 0.2 m/s
- Infinite repetitions
- ROS_DOMAIN_ID=1

**Note:** Use different ROS_DOMAIN_IDs to prevent interference between robots

---

### 9. Security Patrol Route

**Service Name:** `waypoint_security_patrol`

**Description:** Simulated security patrol with Nav2 (pauses at waypoints for observation).

**Launch:**
```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up waypoint_security_patrol
```

**Configuration:**
- Uses Navigation2 for robust navigation
- Infinite loops
- 2.0 second pause at each waypoint (for observation/monitoring)
- Full obstacle avoidance

**Use Case:** Security applications, checkpoint monitoring

**Best For:** Professional security and monitoring systems

---

### 10. Warehouse Scanning

**Service Name:** `waypoint_warehouse_scanning`

**Description:** Slow, precise navigation for warehouse inventory scanning.

**Launch:**
```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up waypoint_warehouse_scanning
```

**Configuration:**
- Speed: 0.15 m/s (slow for accuracy)
- Single repetition
- Position tolerance: 0.05m (very precise)
- Angle tolerance: 0.05rad (very precise)
- Designed for scanning accuracy

**Use Case:** Inventory management, scanning operations

**Best For:** Warehouse and inventory applications

---

## Supporting Services

### Gazebo Simulation

**Service Name:** `gazebo_turtlebot3_world`

**Description:** Launches Gazebo with TurtleBot 3 in the default world.

**Launch:**
```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up gazebo_turtlebot3_world
```

**Note:** Most services depend on this automatically.

---

### Navigation2 Stack

**Service Name:** `nav2_stack`

**Description:** Launches the Navigation2 stack (used by Nav2-based services).

**Launch:**
```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up nav2_stack
```

**Note:** Automatically started by services that require it.

---

### RViz Visualization

**Service Name:** `rviz_monitor`

**Description:** Starts RViz for visualization and monitoring.

**Launch:**
```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up rviz_monitor
```

**Use:** Visual monitoring of robot navigation in real-time

---

## Complete Scenarios

### Scenario 1: Single Robot Testing (Lightweight)

Quick test without Nav2:

```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up \
  gazebo_turtlebot3_world \
  waypoint_twist_single_robot
```

**Startup time:** ~15 seconds
**Resources:** Minimal (5-15% CPU, ~100MB memory)

---

### Scenario 2: Production Deployment (Full Stack)

Complete setup with Nav2, RViz, and monitoring:

```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up \
  gazebo_turtlebot3_world \
  nav2_stack \
  waypoint_nav2_single_robot \
  rviz_monitor
```

**Startup time:** ~90 seconds
**Resources:** Moderate (30-50% CPU, ~500MB memory)
**Features:** Full navigation stack, visualization, obstacle avoidance

---

### Scenario 3: Multi-Robot Coordination

Two robots with different patterns:

```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up \
  gazebo_turtlebot3_world \
  waypoint_nav2_multi_robot_coordinator \
  waypoint_twist_multi_robot_1 \
  waypoint_twist_multi_robot_2
```

**Setup:** Each robot operates independently with different patterns

---

### Scenario 4: Security Patrol with Monitoring

Security patrol with pauses for observation:

```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up \
  gazebo_turtlebot3_world \
  nav2_stack \
  waypoint_security_patrol \
  rviz_monitor
```

**Features:** Long pauses at waypoints, visual monitoring

---

### Scenario 5: Warehouse Operations

Precise scanning with strict tolerances:

```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up \
  gazebo_turtlebot3_world \
  waypoint_warehouse_scanning
```

**Features:** Slow, precise movement; minimal CPU usage

---

## Docker Compose Commands Reference

### Start a Service

```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up <service-name>
```

### Start Multiple Services

```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up \
  service1 \
  service2 \
  service3
```

### Start in Background

```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up -d <service-name>
```

### View Logs

```bash
# All services
docker-compose -f docker-compose.waypoint-navigator.yaml logs -f

# Specific service
docker-compose -f docker-compose.waypoint-navigator.yaml logs -f <service-name>

# Last 100 lines
docker-compose -f docker-compose.waypoint-navigator.yaml logs --tail=100 <service-name>
```

### Stop All Services

```bash
docker-compose -f docker-compose.waypoint-navigator.yaml stop
```

### Remove All Containers

```bash
docker-compose -f docker-compose.waypoint-navigator.yaml down
```

### Remove Containers and Volumes

```bash
docker-compose -f docker-compose.waypoint-navigator.yaml down -v
```

### Execute Command in Container

```bash
docker-compose -f docker-compose.waypoint-navigator.yaml exec <service-name> bash
```

### Attach to Running Container

```bash
docker-compose -f docker-compose.waypoint-navigator.yaml exec -it <service-name> bash
```

### Check Service Status

```bash
docker-compose -f docker-compose.waypoint-navigator.yaml ps
```

### Rebuild Services

```bash
docker-compose -f docker-compose.waypoint-navigator.yaml build
```

---

## Environment Variables

### Setting Display for Visualization

```bash
export DISPLAY=:0
docker-compose -f docker-compose.waypoint-navigator.yaml up rviz_monitor
```

### GPU Acceleration (NVIDIA)

Uncomment GPU variants in `docker-compose.waypoint-navigator.yaml` or set:

```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up waypoint_nav2_production_gpu
```

### Custom ROS_DOMAIN_ID

For multiple docker-compose runs on same machine:

```bash
ROS_DOMAIN_ID=1 docker-compose -f docker-compose.waypoint-navigator.yaml up <service>
```

---

## Troubleshooting

### "Cannot connect to Docker daemon"

**Solution:**
```bash
sudo usermod -aG docker $USER
# Log out and back in, or:
newgrp docker
```

### "Display not found" (RViz/Gazebo issues)

**Solution:**
```bash
# Allow X11 access
xhost +local:docker

# Or set DISPLAY
export DISPLAY=:0
```

### Service stuck waiting for dependencies

**Solution:**
```bash
# Check service status
docker-compose -f docker-compose.waypoint-navigator.yaml ps

# View logs to see errors
docker-compose -f docker-compose.waypoint-navigator.yaml logs <service-name>

# Restart specific service
docker-compose -f docker-compose.waypoint-navigator.yaml restart <service-name>
```

### Container runs but exits immediately

**Solution:**
```bash
# Check logs
docker-compose -f docker-compose.waypoint-navigator.yaml logs <service-name>

# Run interactively to debug
docker-compose -f docker-compose.waypoint-navigator.yaml run <service-name> bash
```

### Port conflicts

**Solution:**
Services use ports 9000-9101. If conflicts occur:
```bash
# Find what's using the port
sudo lsof -i :9000

# Or use a different network
docker network create waypoint_net
# Then modify docker-compose.yaml to use this network
```

---

## Performance Tips

### For Minimal Resource Usage

Use the Twist-based services instead of Nav2:
```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up waypoint_twist_single_robot
```

**Expected:**
- CPU: 5-15%
- Memory: ~100MB
- Startup: ~15 seconds

### For Better Graphics

Enable GPU acceleration (if available):
```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up waypoint_nav2_production_gpu
```

### For Fastest Startup

Use lightweight services without Nav2:
```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up waypoint_twist_fast_exploration
```

### For Background Operation

Run in detached mode:
```bash
docker-compose -f docker-compose.waypoint-navigator.yaml up -d waypoint_twist_single_robot
# Check status
docker-compose -f docker-compose.waypoint-navigator.yaml ps
# View logs
docker-compose -f docker-compose.waypoint-navigator.yaml logs -f
```

---

## Integration with Existing Stack

To include these services in the main docker-compose setup:

**Option 1: Include in main file**

Edit `docker/docker-compose.yaml`:
```yaml
include:
  - ../turtlebotrossim/docker/docker-compose.yaml
  - docker-compose.waypoint-navigator.yaml
```

Then run:
```bash
docker-compose up waypoint_twist_single_robot
```

**Option 2: Use both files separately**

```bash
docker-compose -f docker/docker-compose.yaml -f docker/docker-compose.waypoint-navigator.yaml up
```

---

## Advanced Configuration

### Custom Waypoints

To use custom waypoints:

1. Edit the ROS 2 node source:
   ```bash
   nano turtlebotrossim/src/turtlebot3_waypoint_navigator/turtlebot3_waypoint_navigator/navigator_twist.py
   ```

2. Modify waypoint lists

3. Rebuild package:
   ```bash
   cd turtlebotrossim
   colcon build --packages-select turtlebot3_waypoint_navigator
   ```

4. The Docker containers will use the updated code via volume mount

### Custom Parameters

Modify the command in services to change parameters:

```yaml
command: >
  bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 run turtlebot3_waypoint_navigator waypoint_navigator_twist \
    --ros-args \
    -p speed:=0.5 \
    -p repetitions:=10
  "
```

### Network Configuration

Services use bridge network. For host network (performance):

```bash
docker-compose -f docker-compose.waypoint-navigator.yaml \
  run --rm --network host waypoint_twist_single_robot
```

---

## Monitoring and Logging

### Real-time Monitoring

```bash
# Watch all services
docker-compose -f docker-compose.waypoint-navigator.yaml logs -f

# Watch specific service
docker-compose -f docker-compose.waypoint-navigator.yaml logs -f waypoint_twist_single_robot

# Watch with timestamps
docker-compose -f docker-compose.waypoint-navigator.yaml logs -f --timestamps
```

### Save Logs to File

```bash
docker-compose -f docker-compose.waypoint-navigator.yaml logs > waypoint_navigation.log
```

### ROS 2 Monitoring

Inside a container:
```bash
docker-compose -f docker-compose.waypoint-navigator.yaml exec waypoint_twist_single_robot bash

# Inside container:
ros2 topic list
ros2 topic echo /cmd_vel
ros2 node list
```

---

## Performance Comparison

### Nav2 Version (Production)
- Startup: ~90 seconds
- CPU: 30-50%
- Memory: ~500MB
- Features: Path planning, obstacle avoidance
- Best for: Production, complex environments

### Twist Version (Testing)
- Startup: ~15 seconds
- CPU: 5-15%
- Memory: ~100MB
- Features: Direct velocity control
- Best for: Testing, simple environments

---

## Next Steps

1. **Quick Start:** Run `waypoint_twist_single_robot` for immediate testing
2. **Production:** Use `waypoint_nav2_single_robot` for deployment
3. **Multi-Robot:** Combine `waypoint_twist_multi_robot_1` and `waypoint_twist_multi_robot_2`
4. **Custom Scenarios:** Modify services for your specific needs
5. **Integration:** Include in main docker-compose.yaml for seamless operation

---

## References

- Docker Compose Documentation: https://docs.docker.com/compose/
- ROS 2 Documentation: https://docs.ros.org/
- TurtleBot 3 Manual: https://emanual.robotis.com/docs/en/platform/turtlebot3/
- Waypoint Navigator Guide: See `ROS2_WAYPOINT_PACKAGE_GUIDE.md`

---

## Support

For issues or questions:

1. Check service logs: `docker-compose logs <service-name>`
2. Verify Docker is running: `docker ps`
3. Check available ports: `sudo lsof -i`
4. Review ROS 2 topics: `ros2 topic list`
5. Check node status: `ros2 node list`

For detailed troubleshooting, see the ROS 2 Waypoint Navigator guide: `turtlebotrossim/ROS2_WAYPOINT_PACKAGE_GUIDE.md`
