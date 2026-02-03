# TurtleBot3 Waypoint Navigator - Docker Deployment Guide

## 📋 Overview

This guide covers running the TurtleBot3 Waypoint Navigator in a fully containerized environment using Docker. The setup includes:

- **Headless Gazebo Simulation** - GPU-accelerated physics simulation without display
- **Navigation2 Stack** - Complete autonomous navigation with AMCL localization
- **Waypoint Navigator** - Automated waypoint-based navigation with initial pose fix
- **Optional RViz** - Visual monitoring and debugging interface

## 🎯 Key Features

- ✅ **Fully Headless** - Run simulation without X11/display (ideal for servers/cloud)
- ✅ **GPU Accelerated** - NVIDIA, Mesa (Intel/AMD), or software rendering
- ✅ **Initial Pose Fix** - Automatic AMCL localization with convergence verification
- ✅ **Health Checks** - Built-in service health monitoring
- ✅ **Easy Management** - Single script controls entire stack
- ✅ **Resource Efficient** - Optimized container configuration
- ✅ **Production Ready** - Suitable for deployment on servers/cloud platforms

## 📦 Prerequisites

### Required

1. **Docker** (20.10 or later)
   ```bash
   # Install Docker
   curl -fsSL https://get.docker.com -o get-docker.sh
   sudo sh get-docker.sh
   sudo usermod -aG docker $USER
   ```

2. **Docker Compose** (v2.0 or later)
   ```bash
   # Usually included with Docker Desktop
   # Or install separately:
   sudo apt-get install docker-compose-plugin
   ```

3. **NVIDIA GPU** (optional but recommended)
   ```bash
   # Install nvidia-docker2
   distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
   curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
   curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
     sudo tee /etc/apt/sources.list.d/nvidia-docker.list
   sudo apt-get update
   sudo apt-get install -y nvidia-docker2
   sudo systemctl restart docker
   ```

### Verify Installation

```bash
# Check Docker
docker --version
docker compose version

# Check NVIDIA GPU (if applicable)
nvidia-smi
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

## 🚀 Quick Start

### 1. Build Docker Images

```bash
cd robosapiens-adaptive-platform-turtlebot
./run_waypoint_navigator_docker.sh build
```

This will:
- Build the Docker images with all dependencies
- Compile the waypoint navigator package with the initial pose fix
- Set up the workspace inside the container

### 2. Start the System

```bash
./run_waypoint_navigator_docker.sh start
```

This will:
- Start headless Gazebo simulation with Nav2
- Wait for simulation to become healthy
- Start the waypoint navigator
- Begin autonomous navigation

### 3. Monitor Progress

```bash
# View waypoint navigator logs
./run_waypoint_navigator_docker.sh logs waypoint-navigator

# Check system status
./run_waypoint_navigator_docker.sh status

# Monitor ROS topics
./run_waypoint_navigator_docker.sh monitor
```

### 4. Stop the System

```bash
./run_waypoint_navigator_docker.sh stop
```

## 📖 Detailed Usage

### Command Reference

```bash
./run_waypoint_navigator_docker.sh [command] [options]
```

| Command | Description |
|---------|-------------|
| `build` | Build Docker images and compile workspace |
| `start` | Start all services (simulation + navigator) |
| `stop` | Stop all running services |
| `restart` | Restart all services |
| `logs [service]` | Show logs (all or specific service) |
| `status` | Show container status and resource usage |
| `monitor` | Monitor ROS topics and nodes in real-time |
| `shell` | Open interactive bash shell in container |
| `clean` | Stop and remove all containers/volumes |
| `help` | Show help message |

### Options

| Option | Description |
|--------|-------------|
| `--gpu nvidia` | Use NVIDIA GPU acceleration (default) |
| `--gpu mesa` | Use Mesa with Intel/AMD GPU |
| `--gpu none` | Software rendering (no GPU) |
| `--rviz` | Enable RViz visualization window |
| `--debug` | Enable debug logging and services |
| `--build` | Force rebuild before starting |
| `--domain N` | Set ROS_DOMAIN_ID (default: 0) |

## 💡 Usage Examples

### Basic Usage

```bash
# Start with default settings (NVIDIA GPU)
./run_waypoint_navigator_docker.sh start

# View logs in real-time
./run_waypoint_navigator_docker.sh logs waypoint-navigator
```

### With Visualization

```bash
# Start with RViz for visual monitoring
./run_waypoint_navigator_docker.sh start --gpu nvidia --rviz

# Note: Requires X11 display access on host
```

### Without GPU

```bash
# Use software rendering (slower but works everywhere)
./run_waypoint_navigator_docker.sh start --gpu none
```

### Development Workflow

```bash
# Build and start in debug mode
./run_waypoint_navigator_docker.sh start --build --debug

# Open shell for debugging
./run_waypoint_navigator_docker.sh shell

# Monitor specific topics
./run_waypoint_navigator_docker.sh monitor
```

### Multiple Instances

```bash
# Run multiple instances with different ROS domains
ROS_DOMAIN_ID=0 ./run_waypoint_navigator_docker.sh start
ROS_DOMAIN_ID=1 ./run_waypoint_navigator_docker.sh start

# Or use the --domain flag
./run_waypoint_navigator_docker.sh start --domain 1
```

## 🔍 Service Details

### Services Included

#### 1. Simulation Service (`sim-*`)

- **Purpose:** Headless Gazebo simulation with Nav2
- **Variants:** `sim-nvidia`, `sim-mesa`, `sim-nogpu`
- **Includes:**
  - Gazebo physics engine
  - TurtleBot3 Waffle model
  - Navigation2 stack with AMCL
  - Costmap generation
  - Path planning
- **Health Check:** Verifies ROS topics are publishing

#### 2. Waypoint Navigator Service (`waypoint-navigator-*`)

- **Purpose:** Autonomous waypoint navigation
- **Variants:** `waypoint-navigator-nvidia`, `-mesa`, `-nogpu`
- **Features:**
  - Automatic initial pose setting
  - AMCL convergence verification
  - Waypoint-based patrol
  - Goal acceptance monitoring
- **Startup:** Waits 60s for Nav2 initialization

#### 3. RViz Service (`rviz-*`) [Optional]

- **Purpose:** Visual monitoring and debugging
- **Variants:** `rviz-nvidia`, `rviz-nogpu`
- **Displays:**
  - Robot model and pose
  - Particle cloud (AMCL)
  - Costmaps
  - Navigation paths
- **Enable:** Use `--rviz` flag

#### 4. Monitor Service [Debug Profile]

- **Purpose:** Topic and node monitoring
- **Usage:** Enabled with `--debug` flag
- **Monitors:** `/amcl_pose`, topic rates, node status

#### 5. Debug Shell [Debug Profile]

- **Purpose:** Interactive debugging
- **Usage:** `./run_waypoint_navigator_docker.sh shell`
- **Environment:** Pre-configured with ROS sourced

## 📊 Expected Behavior

### Startup Sequence

```
0s    → Container starts
0-5s  → Gazebo initializes
5-30s → Nav2 stack initializes (AMCL, planners, controllers)
30s   → Health check passes
30s   → Waypoint navigator starts
30-35s → Initial pose published
35-50s → Waiting for AMCL to subscribe
50-55s → AMCL subscriber connected
55-57s → Publishing initial pose (10 times)
57-90s → Waiting for AMCL convergence
~70s  → AMCL pose received
~73s  → AMCL converged successfully
~76s  → Additional stability wait
~80s  → Navigation to first waypoint begins
~80s  → Goal accepted by action server ✅
```

### Success Indicators

Watch for these log messages:

```
✓ Waiting for AMCL to subscribe to /initialpose...
✓ AMCL subscriber connected! (1 subscriber(s))
✓ First initial pose message sent
✓ Initial pose published to AMCL (10 times over 2 seconds)
✓ Waiting for AMCL to converge...
✓ AMCL pose received: (-2.00, -0.50)
✓ AMCL converged successfully!
✓ Robot localized at (-2.00, -0.50)
✓ Waiting 3 more seconds for particle filter to stabilize...
✓ Starting navigation to first waypoint...
✓ [Loop 0] Navigating to Start West (-2.20, -0.50)
✓ Goal accepted by the action server  <-- KEY SUCCESS!
✓ Goal succeeded!
```

### Failure Indicators

If you see these, something is wrong:

```
✗ Goal was rejected by the action server
✗ AMCL did not publish pose within timeout
✗ No subscribers to /initialpose after 5 seconds
```

## 🐛 Troubleshooting

### Issue: Containers won't start

**Check Docker is running:**
```bash
docker info
```

**Check for port conflicts:**
```bash
docker ps -a
netstat -tuln | grep 11311
```

**Solution:**
```bash
# Clean up old containers
./run_waypoint_navigator_docker.sh clean

# Restart Docker daemon
sudo systemctl restart docker
```

### Issue: NVIDIA GPU not detected

**Check NVIDIA driver:**
```bash
nvidia-smi
```

**Check nvidia-docker:**
```bash
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

**Solution:**
```bash
# Reinstall nvidia-docker2
sudo apt-get install --reinstall nvidia-docker2
sudo systemctl restart docker

# Or use software rendering
./run_waypoint_navigator_docker.sh start --gpu none
```

### Issue: "Goal was rejected" errors

**This means initial pose fix is not working.**

**Check logs:**
```bash
./run_waypoint_navigator_docker.sh logs waypoint-navigator | grep -A 5 "initial pose"
```

**Expected:** Should see "AMCL subscriber connected" and "AMCL converged successfully"

**Solution:**
```bash
# Rebuild with latest fix
./run_waypoint_navigator_docker.sh build
./run_waypoint_navigator_docker.sh restart
```

### Issue: Simulation is slow

**Symptoms:** Low frame rate, high CPU usage

**Solutions:**

1. **Use GPU acceleration:**
   ```bash
   ./run_waypoint_navigator_docker.sh start --gpu nvidia
   ```

2. **Reduce physics iterations** (edit simulation parameters)

3. **Increase Docker resources:**
   - Docker Desktop → Settings → Resources
   - Increase CPU cores and RAM

### Issue: Container exits immediately

**Check logs:**
```bash
docker-compose -f turtlebotrossim/docker/docker-compose.waypoint-navigator.yaml logs
```

**Common causes:**
- Missing dependencies in image
- ROS package build failure
- Configuration errors

**Solution:**
```bash
# Rebuild from scratch
./run_waypoint_navigator_docker.sh clean
./run_waypoint_navigator_docker.sh build
./run_waypoint_navigator_docker.sh start
```

### Issue: RViz won't display

**Check X11 forwarding:**
```bash
echo $DISPLAY
xhost +local:docker
```

**Solution:**
```bash
# Enable X11 access
xhost +local:docker

# Start with RViz
./run_waypoint_navigator_docker.sh start --rviz
```

## 🔧 Configuration

### Environment Variables

Set these before running:

```bash
# ROS Domain ID (for multiple robots)
export ROS_DOMAIN_ID=0

# User ID (for file permissions)
export UID=$(id -u)

# Display for RViz
export DISPLAY=:0
```

### Modifying Waypoints

Edit the waypoints in the navigator code:

```bash
# Open shell in container
./run_waypoint_navigator_docker.sh shell

# Edit waypoints
cd /ws/src/turtlebot3_waypoint_navigator/turtlebot3_waypoint_navigator
vim navigator_nav2.py

# Rebuild
cd /ws
colcon build --packages-select turtlebot3_waypoint_navigator
source install/setup.bash

# Exit and restart
exit
./run_waypoint_navigator_docker.sh restart
```

### Adjusting Wait Times

If AMCL takes longer to converge, adjust timeouts:

```python
# In navigator_nav2.py
timeout = 30.0  # Increase from 30 to 60 seconds
time.sleep(3.0) # Increase stability wait
```

## 📈 Performance Tuning

### For Maximum Performance

```bash
# Use NVIDIA GPU
./run_waypoint_navigator_docker.sh start --gpu nvidia

# Allocate more Docker resources
# Docker Desktop → Settings → Resources
# - CPUs: 4+ cores
# - Memory: 8GB+
# - Swap: 2GB+
```

### For Minimum Resource Usage

```bash
# Use software rendering
./run_waypoint_navigator_docker.sh start --gpu none

# Reduce Docker resource limits
# Edit docker-compose file to add:
# deploy:
#   resources:
#     limits:
#       cpus: '2'
#       memory: 4G
```

## 🌐 Cloud Deployment

### AWS EC2

```bash
# Use g4dn instances for GPU
# Install NVIDIA drivers
sudo apt-get update
sudo apt-get install -y nvidia-driver-470

# Install nvidia-docker
# (see Prerequisites section)

# Clone and run
git clone <repo>
cd robosapiens-adaptive-platform-turtlebot
./run_waypoint_navigator_docker.sh build
./run_waypoint_navigator_docker.sh start --gpu nvidia
```

### Google Cloud Platform

```bash
# Use N1 instances with GPU
# Install NVIDIA drivers via GCP console

# Install Docker and nvidia-docker
# (see Prerequisites section)

# Deploy
./run_waypoint_navigator_docker.sh start --gpu nvidia
```

### Kubernetes

See `k8s/` directory for Kubernetes deployment manifests.

## 📂 File Structure

```
robosapiens-adaptive-platform-turtlebot/
├── run_waypoint_navigator_docker.sh          # Main launcher script
├── DOCKER_WAYPOINT_NAVIGATOR.md              # This file
├── turtlebotrossim/
│   ├── docker/
│   │   ├── docker-compose.waypoint-navigator.yaml  # Service definitions
│   │   ├── Dockerfile                              # Image definition
│   │   ├── entrypoint.sh                           # Container startup
│   │   └── setup.bash                              # ROS environment
│   ├── src/
│   │   └── turtlebot3_waypoint_navigator/
│   │       └── turtlebot3_waypoint_navigator/
│   │           └── navigator_nav2.py               # Navigator with fix
│   └── scan_relay_ws/
│       └── demo_bringup/
│           └── launch/
│               └── sim_headless_demo_tb3.launch.py # Headless sim launch
```

## 🧪 Testing

### Verify Initial Pose Fix

```bash
# Start system
./run_waypoint_navigator_docker.sh start

# Monitor logs (in another terminal)
./run_waypoint_navigator_docker.sh logs waypoint-navigator

# Expected: See "AMCL converged successfully!" and "Goal accepted"
```

### Automated Testing

```bash
# Run test script
./run_waypoint_navigator_docker.sh start
sleep 120  # Wait for full initialization
./run_waypoint_navigator_docker.sh monitor  # Check topic activity

# Should see:
# - /amcl_pose publishing at ~2 Hz
# - /cmd_vel showing robot movement
# - Navigation goals being accepted
```

## 📚 Additional Resources

- **Main README:** `README_FIX_STATUS.md` - Fix status and next steps
- **Build Guide:** `BUILD_AND_TEST_FIX.md` - Detailed build instructions
- **Quick Test:** `QUICK_TEST.md` - Fast testing reference
- **Fix Details:** `FINAL_FIX_SUMMARY.md` - Technical fix documentation
- **Troubleshooting:** `DIAGNOSIS_INITIAL_POSE.md` - Issue diagnosis

## 🤝 Contributing

To modify the Docker setup:

1. Edit `docker-compose.waypoint-navigator.yaml`
2. Test changes: `./run_waypoint_navigator_docker.sh build`
3. Run: `./run_waypoint_navigator_docker.sh start --debug`
4. Submit PR with changes

## 📝 Notes

- **Startup Time:** Full system initialization takes ~60-90 seconds
- **Resource Usage:** Expect 2-4 GB RAM, 50-100% CPU during simulation
- **GPU Memory:** NVIDIA setup uses ~1-2 GB VRAM
- **Network:** Uses host networking for ROS communication
- **Persistence:** Workspace changes persist in volumes

## 🆘 Support

If you encounter issues:

1. Check logs: `./run_waypoint_navigator_docker.sh logs`
2. Verify status: `./run_waypoint_navigator_docker.sh status`
3. Try debug mode: `./run_waypoint_navigator_docker.sh start --debug`
4. Open shell: `./run_waypoint_navigator_docker.sh shell`
5. Clean and rebuild: 
   ```bash
   ./run_waypoint_navigator_docker.sh clean
   ./run_waypoint_navigator_docker.sh build
   ```

## ✅ Quick Checklist

Before deploying, ensure:

- [ ] Docker and Docker Compose installed
- [ ] NVIDIA drivers installed (if using GPU)
- [ ] nvidia-docker2 configured (if using GPU)
- [ ] Images built: `./run_waypoint_navigator_docker.sh build`
- [ ] Enough resources allocated (4 cores, 8GB RAM recommended)
- [ ] No port conflicts on host network
- [ ] ROS_DOMAIN_ID set correctly for your environment

## 🎯 Summary

**To run the complete system:**

```bash
# One-time setup
./run_waypoint_navigator_docker.sh build

# Start everything
./run_waypoint_navigator_docker.sh start

# Monitor progress
./run_waypoint_navigator_docker.sh logs waypoint-navigator

# Stop when done
./run_waypoint_navigator_docker.sh stop
```

**Expected result:** Autonomous waypoint navigation with proper AMCL localization and goal acceptance! 🚀

---

*Last Updated: December 2024*  
*Docker Compose Version: 2.0+*  
*ROS 2 Distro: Humble*
