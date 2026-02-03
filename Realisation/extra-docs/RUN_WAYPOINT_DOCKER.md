# Running the TurtleBot 3 Waypoint Navigator in Docker

This document provides step-by-step instructions to run the waypoint navigator with your exact command.

## ⚠️ CRITICAL: Gazebo Simulation Requirement

**The TurtleBot 3 Gazebo simulation MUST be running for the navigator to work.**

The Docker container communicates with Gazebo via ROS 2 topics (`/odom`, `/cmd_vel`). Without Gazebo:
- ❌ No odometry updates (robot position unknown)
- ❌ No movement commands executed (robot stays still)
- ❌ Navigator times out waiting for odometry

**You MUST start Gazebo BEFORE running the navigator.**

## Overview

The waypoint navigator has been fully configured for Docker Compose. It will:

1. ✅ Run in a Docker container with ROS 2 Humble
2. ✅ Automatically source and build the workspace if needed
3. ✅ Connect to your running Gazebo simulation via host networking
4. ✅ Navigate the TurtleBot 3 using velocity commands
5. ✅ Accept parameters via environment variables

## Prerequisites

### 1. Install Dependencies

**Docker and Docker Compose:**
```bash
# Check if Docker is installed
docker --version
docker compose version

# If not installed, follow: https://docs.docker.com/get-docker/
```

**ROS 2 Humble on Host:**
```bash
# Check if ROS 2 is installed
echo $ROS_DISTRO

# If not, install from: https://docs.ros.org/en/humble/Installation.html
```

### 2. START GAZEBO SIMULATION FIRST ⚠️

**Terminal 1** - Start the TurtleBot 3 world (KEEP THIS RUNNING throughout):

```bash
# Source ROS 2 first
source /opt/ros/humble/setup.bash

# Launch the world - THIS MUST BE RUNNING BEFORE RUNNING NAVIGATOR
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Wait for Gazebo to fully load before proceeding.** You should see:
- Gazebo window open
- TurtleBot 3 waffle model visible in the empty world
- No errors in the terminal

### 3. Build the Workspace (First Time Only)

**Terminal 2** - Build the package (only needed once):

```bash
cd robosapiens-adaptive-platform-turtlebot/turtlebotrossim
source /opt/ros/humble/setup.bash
colcon build --packages-select turtlebot3_waypoint_navigator
```

This creates the `install/setup.sh` that Docker will use. You should see:
```
Summary: 1 package finished
```

**Note:** After the first build, Docker will reuse the built workspace automatically.

## Running the Navigator

### ⚠️ Important: Terminal Setup

You need **3 terminals running simultaneously**:

| Terminal | Purpose | Status |
|----------|---------|--------|
| Terminal 1 | Gazebo Simulation | KEEP RUNNING ✅ |
| Terminal 2 | Build workspace (first time) | Run once ✅ |
| Terminal 3 | Run Navigator | START HERE ✅ |

### Method 1: Using Your Exact Command (Recommended)

**Terminal 3** - Run from project root (while Gazebo is running in Terminal 1):

```bash
cd robosapiens-adaptive-platform-turtlebot

WAYPOINT_SPEED=0.15 \
WAYPOINT_POSITION_TOLERANCE=0.05 \
WAYPOINT_ANGLE_TOLERANCE=0.05 \
WAYPOINT_REPETITIONS=1 \
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

**Watch Gazebo window while this runs** - you should see the robot move!

### Method 2: Using the Helper Script (Also Recommended)

If you prefer easier parameter passing (Gazebo must still be running):

```bash
cd robosapiens-adaptive-platform-turtlebot

./docker/run-waypoint-navigator.sh \
  --speed 0.15 \
  --position-tolerance 0.05 \
  --angle-tolerance 0.05 \
  --repetitions 1
```

### Method 3: With Default Parameters

```bash
cd robosapiens-adaptive-platform-turtlebot

docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

**Important:** Gazebo must still be running in Terminal 1 for this to work.

## Expected Output

When the navigator starts, you should see:

```
Creating waypoint_navigator ... done
Attaching to waypoint_navigator
waypoint_navigator  | ==========================================
waypoint_navigator  | TurtleBot 3 Waypoint Navigator
waypoint_navigator  | ==========================================
waypoint_navigator  | Speed: 0.15 m/s
waypoint_navigator  | Angular Speed: 0.5 rad/s
waypoint_navigator  | Repetitions: 1
waypoint_navigator  | Pattern: default
waypoint_navigator  | Position Tolerance: 0.05 m
waypoint_navigator  | Angle Tolerance: 0.05 rad
waypoint_navigator  | ROS Domain ID: 0
waypoint_navigator  | ==========================================
waypoint_navigator  | 
waypoint_navigator  | Sourcing ROS 2 environment...
waypoint_navigator  | ✓ ROS 2 environment sourced
waypoint_navigator  | 
waypoint_navigator  | Checking workspace...
waypoint_navigator  | Sourcing workspace at /ws...
waypoint_navigator  | ✓ Workspace sourced successfully
waypoint_navigator  | 
waypoint_navigator  | Waiting for ROS 2 environment to be ready...
waypoint_navigator  | ✓ ROS 2 environment is ready!
waypoint_navigator  | 
waypoint_navigator  | Starting waypoint navigation...
waypoint_navigator  | ============================================
waypoint_navigator  | 
waypoint_navigator  | [INFO] [turtlebot3_waypoint_navigator_twist]: TurtleBot 3 Waypoint Navigator (Twist) initialized
waypoint_navigator  | [INFO] [turtlebot3_waypoint_navigator_twist]: Publishing velocity commands to: /cmd_vel
waypoint_navigator  | [INFO] [turtlebot3_waypoint_navigator_twist]: Subscribing to odometry from: /odom
```

Then the robot should start moving in Gazebo following waypoints!

## Stopping the Navigator

### Option 1: Keyboard Interrupt
Press `Ctrl+C` in the terminal where the navigator is running.

### Option 2: Docker Command (in another terminal)
```bash
cd robosapiens-adaptive-platform-turtlebot
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml down
```

## Validation

To verify everything is set up correctly:

```bash
cd robosapiens-adaptive-platform-turtlebot
./docker/validate-setup.sh
```

This checks:
- ✓ Docker and Docker Compose installation
- ✓ Project structure
- ✓ Docker configuration files
- ✓ ROS 2 environment
- ✓ Workspace files
- ✓ Package configuration

## Common Issues

### Issue: "ROS 2 environment did not become ready"

**Cause:** Gazebo simulation is not running

**Solution:** Make sure Terminal 1 has the Gazebo world running:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Issue: "turtlebot3_waypoint_navigator package not found"

**Cause:** Workspace not built

**Solution:** Build the workspace:
```bash
cd turtlebotrossim
source /opt/ros/humble/setup.bash
colcon build --packages-select turtlebot3_waypoint_navigator
```

### Issue: "Permission denied" when running helper script

**Solution:** Make it executable:
```bash
chmod +x docker/run-waypoint-navigator.sh
```

### Issue: Robot not moving in Gazebo

**Solution:** Verify ROS 2 communication:
```bash
# Check topics exist
ros2 topic list | grep -E "(cmd_vel|odom)"

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Monitor odometry
ros2 topic echo /odom
```

### Issue: Docker image not found

**Solution:** Docker will build the image on first run. If it fails:
```bash
# Try to force rebuild
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml build --no-cache
```

## Configuration Parameters

You can customize the robot's behavior by changing these environment variables:

| Variable | Default | Description |
|----------|---------|-------------|
| `WAYPOINT_SPEED` | 0.2 | Linear speed in m/s (0.1-0.4 recommended) |
| `WAYPOINT_ANGULAR_SPEED` | 0.5 | Angular speed in rad/s |
| `WAYPOINT_REPETITIONS` | 1 | Number of repetitions (0 = infinite) |
| `WAYPOINT_PATTERN` | default | Pattern: `default`, `figure_eight`, or `spiral` |
| `WAYPOINT_POSITION_TOLERANCE` | 0.1 | Distance to consider waypoint reached (m) |
| `WAYPOINT_ANGLE_TOLERANCE` | 0.1 | Angle tolerance for heading (rad) |
| `WAYPOINT_NAMESPACE` | (empty) | ROS namespace for topics (optional) |
| `ROS_DOMAIN_ID` | 0 | ROS 2 Domain ID (0-232) |

### Example Commands

**Slow Exploration (Safe):**
```bash
WAYPOINT_SPEED=0.1 \
WAYPOINT_POSITION_TOLERANCE=0.05 \
WAYPOINT_ANGLE_TOLERANCE=0.05 \
WAYPOINT_REPETITIONS=1 \
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

**Fast Figure-Eight Pattern (3 times):**
```bash
WAYPOINT_SPEED=0.3 \
WAYPOINT_PATTERN=figure_eight \
WAYPOINT_REPETITIONS=3 \
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

**Continuous Spiral (Infinite):**
```bash
WAYPOINT_SPEED=0.25 \
WAYPOINT_PATTERN=spiral \
WAYPOINT_REPETITIONS=0 \
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

## Navigation Patterns

### Default Pattern
A rectangular path around the TurtleBot 3 world:
- Waypoints: (0.5, 0.5) → (1.5, 0.5) → (1.5, 1.5) → (0.5, 1.5) → repeat
- Best for: General testing, closed spaces

### Figure-Eight Pattern
Smooth figure-eight with circular loops:
- Two circular arcs side by side
- Best for: Testing curved navigation

### Spiral Pattern
Expanding spiral from origin:
- Gradually expands outward
- Best for: Area exploration

## Advanced Usage

### Run in Background
```bash
WAYPOINT_SPEED=0.15 \
WAYPOINT_POSITION_TOLERANCE=0.05 \
WAYPOINT_ANGLE_TOLERANCE=0.05 \
WAYPOINT_REPETITIONS=1 \
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up -d waypoint_navigator
```

Then check status:
```bash
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml ps
```

View logs:
```bash
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml logs -f waypoint_navigator
```

### Multiple Robots (Different Namespaces)

**Terminal 3:**
```bash
WAYPOINT_NAMESPACE=/robot1 \
WAYPOINT_SPEED=0.15 \
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up -d waypoint_navigator1
```

**Terminal 4:**
```bash
WAYPOINT_NAMESPACE=/robot2 \
WAYPOINT_SPEED=0.2 \
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up -d waypoint_navigator2
```

### Custom ROS Domain ID
For network isolation:
```bash
ROS_DOMAIN_ID=1 \
WAYPOINT_SPEED=0.15 \
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

## File Structure

```
robosapiens-adaptive-platform-turtlebot/
├── docker/
│   ├── docker-compose.waypoint-navigator-simple.yaml    [Main config]
│   ├── run-waypoint-navigator.sh                        [Helper script]
│   └── validate-setup.sh                                [Validation script]
│
├── turtlebotrossim/
│   ├── src/
│   │   └── turtlebot3_waypoint_navigator/
│   │       ├── turtlebot3_waypoint_navigator/
│   │       │   ├── navigator_twist.py                   [Implementation]
│   │       │   └── navigator_nav2.py
│   │       ├── setup.py
│   │       └── package.xml
│   └── install/                                         [Build output]
│
├── RUN_WAYPOINT_DOCKER.md                               [This file]
├── WAYPOINT_DOCKER_SETUP.md                             [Full documentation]
└── WAYPOINT_DOCKER_COMPLETE.md                          [Setup summary]
```

## Terminal Setup Summary

For easy reference, here's the recommended terminal setup:

**Terminal 1 - Gazebo Simulation (KEEP RUNNING):**
```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2 - Build Workspace (Run once):**
```bash
cd turtlebotrossim
source /opt/ros/humble/setup.bash
colcon build --packages-select turtlebot3_waypoint_navigator
```

**Terminal 3 - Run Navigator:**
```bash
cd robosapiens-adaptive-platform-turtlebot
WAYPOINT_SPEED=0.15 WAYPOINT_POSITION_TOLERANCE=0.05 \
WAYPOINT_ANGLE_TOLERANCE=0.05 WAYPOINT_REPETITIONS=1 \
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

**Terminal 4 - Monitor (Optional):**
```bash
# Monitor logs
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml logs -f waypoint_navigator

# Or monitor ROS 2 topics
ros2 topic echo /cmd_vel
ros2 topic echo /odom
```

## Next Steps

1. ✅ **Verify setup:** Run `./docker/validate-setup.sh`
2. ✅ **Start Gazebo:** Run the command in Terminal 1
3. ✅ **Build workspace:** Run the command in Terminal 2 (first time only)
4. ✅ **Run navigator:** Run the command in Terminal 3
5. ✅ **Monitor robot:** Watch Gazebo and check logs

## Troubleshooting - Robot Not Moving

### Problem: Docker Container Runs but Robot Doesn't Move

**Root Cause:** Gazebo simulation is not running on the host, so there's no `/odom` topic for position data and no `/cmd_vel` connection to send movement commands.

**Solution Checklist:**

1. ✅ **Verify Gazebo is Running**
   ```bash
   # In a separate terminal, check if Gazebo process exists
   ps aux | grep gazebo
   ```
   Should show a running Gazebo process. If not, start it:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. ✅ **Check ROS 2 Topics are Available**
   ```bash
   # List all topics
   ros2 topic list
   ```
   You should see:
   - `/odom` (odometry from Gazebo)
   - `/cmd_vel` (velocity command topic)

   If missing, Gazebo isn't running or didn't load the TurtleBot 3 properly.

3. ✅ **Verify Topic Data is Flowing**
   ```bash
   # Monitor odometry updates
   ros2 topic echo /odom --once
   ```
   Should show position and orientation data. If nothing, Gazebo isn't publishing.

4. ✅ **Check Docker Network Connection**
   ```bash
   # From inside the container, verify ROS 2 communication
   docker exec waypoint_navigator ros2 topic list
   ```
   Should show the same topics as the host.

5. ✅ **Verify ROS_DOMAIN_ID Matches**
   ```bash
   # Check host domain ID
   echo $ROS_DOMAIN_ID
   
   # Default is 0, make sure it matches in docker-compose env vars
   ```

### Problem: "Waiting for ROS 2 environment to be ready" Timeout

**Root Cause:** ROS 2 nodes aren't responding, usually because Gazebo isn't running.

**Solution:**
1. Ensure Gazebo is fully launched in Terminal 1
2. Wait 5-10 seconds after Gazebo window appears before running navigator
3. Verify ROS 2 is working: `ros2 node list` should show Gazebo nodes

### Problem: Navigator Says "No executable found"

**Already Fixed!** This was a build configuration issue that has been resolved by changing the package.xml to use `ament_python`.

If you still see this error:
1. Rebuild the workspace: `rm -rf turtlebotrossim/build turtlebotrossim/install turtlebotrossim/log`
2. Run the navigator again - it will rebuild automatically

## Live Map Visualization 🗺️

**NEW:** Monitor your robot's navigation in real-time with the command-line map visualizer!

The visualizer provides a live ASCII art dashboard showing:
- Robot position and orientation
- Navigation goals
- Map obstacles and free space
- Velocity and sensor data
- Topic statistics and health

### Quick Start with Visualizer

**Terminal 1 - Gazebo (Keep Running):**
```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2 - Start Navigation:**
```bash
./run_waypoint_navigator_docker.sh start
```

**Terminal 3 - Launch Visualizer:**
```bash
./run_waypoint_navigator_docker.sh visualize
```

### What You'll See

```
================================================================================
                  TurtleBot3 Live Navigation Status
================================================================================

┌─ POSITION ─────────────────────────────────────────────────────────────────┐
│ AMCL Pose:    X:   1.234 m  Y:   0.567 m  θ:   45.0°                       │
│ Odometry:     X:   1.240 m  Y:   0.570 m  θ:   45.2°                       │
│ Goal:         X:   2.000 m  Y:   1.500 m  Dist:  1.123 m                   │
└────────────────────────────────────────────────────────────────────────────┘

┌─ MAP VIEW ─────────────────────────────────────────────────────────────────┐
│··········································································│
│··········→·······························★·······························│
│··········································································│
│  Legend:  → Robot   ★ Goal   █ Obstacle   · Free   ? Unknown            │
└────────────────────────────────────────────────────────────────────────────┘
```

**Legend:**
- `→ ↑ ↓ ←` - Robot with direction indicator
- `★` - Current navigation goal
- `█` - Obstacles/walls
- `·` - Free space
- `?` - Unknown/unexplored areas

### Visualizer Commands

```bash
# Start visualizer
./run_waypoint_navigator_docker.sh visualize

# Exit visualizer
# Press Ctrl+C

# Check if data is flowing
./run_waypoint_navigator_docker.sh monitor
```

### Troubleshooting Visualizer

**"No AMCL pose data available":**
- Wait 10-30 seconds for localization to initialize
- Verify simulation is running: `./run_waypoint_navigator_docker.sh status`

**Garbled display:**
- Ensure terminal supports UTF-8: `export LANG=en_US.UTF-8`
- Resize terminal to at least 80 characters wide

**For complete visualizer documentation, see:**
- `VISUALIZER_README.md` - Full feature guide
- `VISUALIZER_QUICKSTART.md` - Quick start guide

## Getting Help

For more detailed information, see:

- `WAYPOINT_DOCKER_SETUP.md` - Comprehensive setup guide
- `WAYPOINT_NAVIGATOR_README.md` - Navigator implementation details
- `QUICKSTART_WAYPOINT.md` - Quick reference
- `WAYPOINT_EXAMPLES.md` - Additional usage examples

## Summary

The waypoint navigator docker-compose setup is now **complete and working**. You can run the exact command:

```bash
WAYPOINT_SPEED=0.15 WAYPOINT_POSITION_TOLERANCE=0.05 \
WAYPOINT_ANGLE_TOLERANCE=0.05 WAYPOINT_REPETITIONS=1 \
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

**Remember:** Gazebo must be running in Terminal 1 for the robot to move!

## Quick Checklist Before Running

- [ ] Gazebo is running: `ps aux | grep gazebo`
- [ ] ROS 2 topics exist: `ros2 topic list | grep -E "cmd_vel|odom"`
- [ ] Docker is working: `docker ps`
- [ ] Workspace built: `ls turtlebotrossim/install/setup.sh`

## Your Command Is Ready! 🚀

```bash
WAYPOINT_SPEED=0.15 WAYPOINT_POSITION_TOLERANCE=0.05 \
WAYPOINT_ANGLE_TOLERANCE=0.05 WAYPOINT_REPETITIONS=1 \
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator
```

Just make sure:
1. ✅ Gazebo is running in Terminal 1
2. ✅ Workspace is built
3. ✅ You're in the project root directory

Then run the command above and watch your robot navigate! 🤖
