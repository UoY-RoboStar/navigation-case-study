# Quick Start Guide - Docker Deployment

## ⚡ 5-Minute Setup

Get the TurtleBot3 Waypoint Navigator running in Docker with full headless simulation and Nav2.

### Prerequisites Check

```bash
# Check you have these installed
docker --version          # Need 20.10+
docker compose version    # Need 2.0+
nvidia-smi               # Optional: for GPU acceleration
```

**Don't have Docker?** Install: `curl -fsSL https://get.docker.com | sh`

---

## 🚀 Three Simple Steps

### Step 1: Build (One-time, ~5 minutes)

```bash
cd robosapiens-adaptive-platform-turtlebot
./run_waypoint_navigator_docker.sh build
```

**This builds:**
- Docker images with all dependencies
- Waypoint navigator package with initial pose fix
- Complete ROS 2 workspace

### Step 2: Start (~2 minutes)

```bash
./run_waypoint_navigator_docker.sh start
```

**This launches:**
- Headless Gazebo simulation (GPU-accelerated if available)
- Navigation2 stack with AMCL localization
- Waypoint navigator with automatic initial pose setting

### Step 3: Monitor

```bash
# Watch the navigator logs
./run_waypoint_navigator_docker.sh logs waypoint-navigator
```

**Look for these SUCCESS indicators:**
```
✓ AMCL subscriber connected! (1 subscriber(s))
✓ AMCL pose received: (-2.00, -0.50)
✓ AMCL converged successfully!
✓ Goal accepted by the action server  <-- SUCCESS!
✓ Goal succeeded!
```

---

## ✅ Success = "Goal accepted"

If you see `"Goal accepted by the action server"` in the logs → **IT'S WORKING!** 🎉

The robot will autonomously navigate through waypoints. You're done!

---

## 🎮 Common Commands

```bash
# Stop everything
./run_waypoint_navigator_docker.sh stop

# Check status
./run_waypoint_navigator_docker.sh status

# Monitor ROS topics
./run_waypoint_navigator_docker.sh monitor

# Restart if needed
./run_waypoint_navigator_docker.sh restart

# View all logs
./run_waypoint_navigator_docker.sh logs

# Clean up everything
./run_waypoint_navigator_docker.sh clean
```

---

## 🐛 Quick Troubleshooting

### ❌ "Docker not found"
```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
# Log out and back in
```

### ❌ "Permission denied" on Docker
```bash
sudo usermod -aG docker $USER
# Log out and back in
```

### ❌ "Goal was rejected"
This means the initial pose fix isn't working. Try:
```bash
./run_waypoint_navigator_docker.sh restart
# Wait 2 minutes and check logs again
```

### ❌ Container keeps restarting
```bash
# Check what's wrong
./run_waypoint_navigator_docker.sh logs

# Rebuild if needed
./run_waypoint_navigator_docker.sh clean
./run_waypoint_navigator_docker.sh build
./run_waypoint_navigator_docker.sh start
```

### ❌ Too slow / laggy
```bash
# Try without GPU (software rendering)
./run_waypoint_navigator_docker.sh stop
./run_waypoint_navigator_docker.sh start --gpu none
```

---

## 🎨 Optional: Add Visualization

Want to see the robot in RViz?

```bash
# Allow X11 display
xhost +local:docker

# Start with RViz
./run_waypoint_navigator_docker.sh start --rviz
```

**RViz will show:**
- Robot model and position
- Particle cloud (AMCL localization)
- Navigation paths
- Costmaps

---

## ⏱️ Expected Timeline

```
0:00  → Run build command
0:00-5:00  Building images and workspace
5:00  → Run start command
5:00  → Gazebo simulation starts
5:30  → Nav2 initializes
6:00  → Waypoint navigator starts
6:00-7:00  Initial pose setup
7:00-7:30  AMCL converges
7:30  → Navigation begins
7:30+  Robot navigates waypoints autonomously
```

**Total:** ~7-8 minutes from build to autonomous navigation

---

## 🎯 Different GPU Options

### NVIDIA GPU (Default - Fastest)
```bash
./run_waypoint_navigator_docker.sh start --gpu nvidia
```

### Intel/AMD GPU
```bash
./run_waypoint_navigator_docker.sh start --gpu mesa
```

### No GPU (Software Rendering)
```bash
./run_waypoint_navigator_docker.sh start --gpu none
```

---

## 📊 Verify It's Working

### Method 1: Check Logs
```bash
./run_waypoint_navigator_docker.sh logs waypoint-navigator | grep "Goal"
```
Should see: `"Goal accepted by the action server"`

### Method 2: Check Topics
```bash
./run_waypoint_navigator_docker.sh monitor
```
Should see:
- `/amcl_pose` publishing at ~2 Hz
- `/cmd_vel` showing robot movement
- Multiple ROS nodes active

### Method 3: Check Status
```bash
./run_waypoint_navigator_docker.sh status
```
Should show:
- Simulation container: Running
- Navigator container: Running
- Both services healthy

---

## 🔄 Daily Usage

**Starting your day:**
```bash
./run_waypoint_navigator_docker.sh start
```

**Checking progress:**
```bash
./run_waypoint_navigator_docker.sh logs waypoint-navigator
```

**Ending your day:**
```bash
./run_waypoint_navigator_docker.sh stop
```

**Making changes:**
```bash
./run_waypoint_navigator_docker.sh shell
# Make changes
# Rebuild: colcon build --packages-select turtlebot3_waypoint_navigator
# Exit and restart
./run_waypoint_navigator_docker.sh restart
```

---

## 🆘 Need More Help?

**Detailed guides:**
- Full Docker guide: `DOCKER_WAYPOINT_NAVIGATOR.md`
- Fix documentation: `FINAL_FIX_SUMMARY.md`
- Troubleshooting: `DIAGNOSIS_INITIAL_POSE.md`

**Interactive debugging:**
```bash
./run_waypoint_navigator_docker.sh shell
```

**View all options:**
```bash
./run_waypoint_navigator_docker.sh help
```

---

## 🎓 What's Actually Happening?

1. **Container 1 (Simulation):** 
   - Runs headless Gazebo with TurtleBot3
   - Starts Nav2 with AMCL localization
   - Waits for navigator to connect

2. **Container 2 (Navigator):**
   - Waits for simulation to be ready
   - Sets initial pose for AMCL (with fix!)
   - Waits for AMCL to converge
   - Sends navigation goals to waypoints
   - Robot navigates autonomously!

---

## ✨ That's It!

**TL;DR:**
```bash
./run_waypoint_navigator_docker.sh build     # One time
./run_waypoint_navigator_docker.sh start     # Every time
./run_waypoint_navigator_docker.sh logs waypoint-navigator  # Monitor
```

**Success = "Goal accepted"** 

You now have a fully autonomous TurtleBot3 navigating waypoints! 🤖🎉

---

*For production deployment, cloud setup, or advanced configuration, see `DOCKER_WAYPOINT_NAVIGATOR.md`*
