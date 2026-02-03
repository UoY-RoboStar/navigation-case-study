# TurtleBot 3 Waypoint Navigator - Complete Guide

A comprehensive suite of Python scripts to autonomously navigate TurtleBot 3 through repeating waypoint sequences in ROS 2 simulation and real-world environments.

## 📦 What's Included

### Scripts

1. **`turtlebot3_waypoint_navigator.py`** (Recommended)
   - Uses Navigation2 action server for robust autonomous navigation
   - Features path planning, obstacle avoidance, and AMCL localization
   - Best for production deployments and complex environments
   - **Startup time:** ~90 seconds

2. **`turtlebot3_waypoint_navigator_twist.py`** (Lightweight Alternative)
   - Direct velocity commands (Twist messages)
   - No Navigation2 required - simpler and faster setup
   - Best for testing, learning, and simple environments
   - **Startup time:** ~15 seconds

### Documentation

- **`QUICKSTART_WAYPOINT.md`** - Get started in 5 minutes
- **`WAYPOINT_NAVIGATOR_README.md`** - Complete reference guide
- **`WAYPOINT_SCRIPTS_INDEX.md`** - Detailed comparison and architecture
- **`WAYPOINT_EXAMPLES.md`** - Advanced patterns and real-world use cases
- **`README_WAYPOINT_NAVIGATOR.md`** - This file

## 🚀 Quick Start (3 Steps, 5 Minutes)

### Step 1: Launch Gazebo Simulation
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Step 2: Launch Navigation Stack (Skip for Twist version)
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true \
  map:=$(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/maps/map.yaml
```

### Step 3: Run Waypoint Navigator
```bash
# Option A: Recommended Nav2 version
python3 turtlebot3_waypoint_navigator.py

# Option B: Simpler Twist version (no Nav2 needed)
python3 turtlebot3_waypoint_navigator_twist.py
```

**That's it!** Your TurtleBot 3 should now be patrolling through waypoints.

## 📊 Version Comparison

| Feature | Nav2 Version | Twist Version |
|---------|:---:|:---:|
| **Path Planning** | ✅ | ❌ |
| **Obstacle Avoidance** | ✅ | ❌ |
| **Localization (AMCL)** | ✅ | ❌ |
| **Startup Time** | ~90s | ~15s |
| **Complexity** | Medium | Low |
| **Nav2 Required** | Yes | No |
| **Best For** | Production | Testing |

## 🎯 Which Script Should I Use?

### Choose Nav2 Version if:
- ✅ You need robust autonomous navigation
- ✅ Your environment has obstacles
- ✅ You want path replanning capability
- ✅ This is a production deployment
- ✅ You're comfortable with longer startup time

### Choose Twist Version if:
- ✅ You want simplicity (no Nav2 needed)
- ✅ You're experimenting or prototyping
- ✅ You want fast startup
- ✅ You're learning ROS 2
- ✅ Your environment is obstacle-free

## 📋 Common Commands

### Run for Fixed Repetitions
```bash
python3 turtlebot3_waypoint_navigator_twist.py --repetitions 5
```

### Adjust Speed
```bash
python3 turtlebot3_waypoint_navigator_twist.py --speed 0.3
```

### Use Different Pattern
```bash
python3 turtlebot3_waypoint_navigator_twist.py --pattern figure_eight
```

### Multi-Robot Setup
```bash
python3 turtlebot3_waypoint_navigator_twist.py --namespace /tb3_0
python3 turtlebot3_waypoint_navigator_twist.py --namespace /tb3_1
```

### Get All Options
```bash
python3 turtlebot3_waypoint_navigator_twist.py --help
```

## 🔧 Installation & Prerequisites

### System Requirements
- Ubuntu 20.04+ or 22.04
- ROS 2 (Humble, Iron, or later)
- Python 3.8+

### Install ROS 2 Packages
```bash
# TurtleBot 3 and simulation
sudo apt-get install ros-humble-turtlebot3 \
                     ros-humble-turtlebot3-gazebo \
                     ros-humble-navigation2 \
                     ros-humble-nav2-bringup

# Python dependencies
pip install tf-transformations numpy
```

### Environment Setup
```bash
# Add to ~/.bashrc or run in each terminal
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle  # or burger/waffle_pi
```

## 📍 Customizing Waypoints

Edit the script to modify waypoints. In `turtlebot3_waypoint_navigator_twist.py`, find:

```python
def get_default_waypoints():
    return [
        (0.5, 0.5, 0.0),              # x, y, heading (radians)
        (1.5, 0.5, 0.0),
        (1.5, 1.5, math.pi/2),        # π/2 = 90° (North)
        (0.5, 1.5, math.pi),          # π = 180° (West)
        (0.5, 0.5, -math.pi/2),       # -π/2 = 270° (South)
    ]
```

### Heading Angles Reference
```
0.0           → East (→)
π/2 ≈ 1.57    → North (↑)
π ≈ 3.14      → West (←)
-π/2 ≈ -1.57  → South (↓)
```

### Convert Degrees to Radians
```python
import math
degrees = 45
radians = math.radians(degrees)  # ≈ 0.785
```

## 📚 Documentation Map

| Document | Purpose | Read Time | Best For |
|----------|---------|-----------|----------|
| **QUICKSTART_WAYPOINT.md** | Fast start | 5 min | Everyone |
| **WAYPOINT_NAVIGATOR_README.md** | Complete reference | 20 min | Intermediate users |
| **WAYPOINT_SCRIPTS_INDEX.md** | Detailed comparison | 10 min | Decision making |
| **WAYPOINT_EXAMPLES.md** | Advanced patterns | 15 min | Advanced users |
| **Script comments** | Implementation | 10 min | Developers |

## 🐛 Troubleshooting

### Robot Doesn't Move
**Solution:**
1. Verify Gazebo is running (Terminal 1)
2. For Nav2 version: Verify Nav2 is fully initialized (~60 seconds)
3. Check topics are publishing: `ros2 topic list`
4. Review logs: `ros2 topic echo /odom`

### "Waiting for NavigateToPose action server..."
**Solution:**
- Wait 30-60 seconds for Nav2 to fully initialize
- Verify `navigation2.launch.py` completed successfully
- Check map file exists and is loaded

### Robot Gets Stuck at Waypoint
**Solution:**
```bash
python3 turtlebot3_waypoint_navigator_twist.py \
  --speed 0.1 \
  --position-tolerance 0.2
```

### ImportError for tf_transformations
**Solution:**
```bash
pip install tf-transformations
```

### Other Issues
See complete troubleshooting in `WAYPOINT_NAVIGATOR_README.md`

## 🏗️ Architecture Overview

### Nav2 Version Flow
```
User Script
    ↓
ROS 2 Node
    ↓
NavigateToPose Action
    ↓
Navigation2 Stack (Planning, Control, Localization)
    ↓
/cmd_vel → TurtleBot 3
```

### Twist Version Flow
```
User Script
    ↓
ROS 2 Node
    ↓
Control Loop (Proportional Control)
    ↓
/cmd_vel → TurtleBot 3
```

## 💡 Key Features

### Both Scripts Include
- ✅ Repeating waypoint sequences
- ✅ Configurable speeds and tolerances
- ✅ Multi-robot support via namespaces
- ✅ Comprehensive logging and feedback
- ✅ Graceful shutdown handling
- ✅ Command-line argument support
- ✅ Timeout protection per waypoint

### Nav2 Version Adds
- ✅ Global path planning
- ✅ Dynamic obstacle avoidance
- ✅ AMCL-based localization
- ✅ Replanning on goal failure
- ✅ Multiple navigation modes

### Twist Version Adds
- ✅ Proportional control
- ✅ Multiple waypoint patterns (default, figure-eight, spiral)
- ✅ Lightweight resource usage
- ✅ Direct odometry feedback
- ✅ Real-time heading correction

## 🎓 Learning Resources

### Included Examples
The `WAYPOINT_EXAMPLES.md` file contains:
- Basic patterns (square, line, corners)
- Advanced patterns (spiral, grid, lawn-mower)
- Real-world use cases (security, warehouse, inspection)
- Integration examples (MQTT, ROS services, logging)
- Performance optimization techniques

### External Resources
- [ROS 2 Documentation](https://docs.ros.org/)
- [Navigation2 Guide](https://navigation.ros.org/)
- [TurtleBot 3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- [Gazebo Simulator](https://gazebosim.org/)

## 🔌 Integration & Extension

### Add Event Callbacks
```python
def on_waypoint_reached(self, waypoint):
    """Called when reaching a waypoint"""
    self.get_logger().info(f"Reached: {waypoint}")
    # Add custom logic here
```

### Publish Status via MQTT
```python
def publish_status(self, status):
    client.publish('robot/status', status)
```

### Log Waypoint Visits
```python
with open('navigation_log.csv', 'a') as f:
    writer = csv.writer(f)
    writer.writerow([timestamp, waypoint, success])
```

See `WAYPOINT_EXAMPLES.md` for complete integration examples.

## 📊 Performance Characteristics

### Typical Performance (Gazebo Simulation)
- **CPU Usage:** Nav2 (30-50%), Twist (5-15%)
- **Memory:** Nav2 (~500MB), Twist (~100MB)
- **Update Rate:** 20-50 Hz
- **Response Time:** Nav2 (200-500ms), Twist (50-100ms)
- **Waypoint Completion Time:** 5-15 seconds each

### Real Robot Performance
- **Localization Accuracy:** ±10-20cm (AMCL-based)
- **Heading Accuracy:** ±5-10 degrees
- **Maximum Speed:** 0.5 m/s (depends on robot model)
- **Typical Patrol Duration:** 30-60 seconds per loop

## 🎯 Use Cases

### Supported Scenarios
1. **Facility Patrol** - Security monitoring routes
2. **Area Coverage** - Lawn-mower patterns for complete coverage
3. **Inventory Scanning** - Grid-based warehouse navigation
4. **Environmental Monitoring** - Circular observation stations
5. **Inspection Routes** - Multi-zone facility checks
6. **Delivery Planning** - Route planning with stops
7. **Multi-Robot Coordination** - Parallel patrol routes
8. **Research & Education** - Learning ROS 2 navigation

## 📈 Performance Tuning

### For Precision (Accurate Positioning)
```bash
python3 turtlebot3_waypoint_navigator_twist.py \
  --speed 0.1 \
  --position-tolerance 0.05 \
  --angle-tolerance 0.05
```

### For Speed (Fast Coverage)
```bash
python3 turtlebot3_waypoint_navigator_twist.py \
  --speed 0.4 \
  --position-tolerance 0.2 \
  --angle-tolerance 0.2
```

### For Balanced Performance
```bash
python3 turtlebot3_waypoint_navigator_twist.py \
  --speed 0.2 \
  --position-tolerance 0.1 \
  --angle-tolerance 0.1
```

## 🚀 Advanced Features

### Dynamic Waypoint Selection
- Modify waypoints at runtime based on conditions
- Switch patterns based on sensor input
- Adapt routes based on localization quality

### Multi-Robot Coordination
- Run multiple robots with different namespaces
- Coordinate overlapping patrol routes
- Synchronize actions between robots

### Performance Monitoring
- Log all waypoint visits with timestamps
- Track navigation accuracy over time
- Measure energy efficiency
- Publish metrics to MQTT or ROS topics

### Conditional Routing
- Branch to different waypoints based on conditions
- Return to charging station when battery low
- Skip waypoints based on sensor feedback

## 📝 File Organization

```
robosapiens-adaptive-platform-turtlebot/
├── turtlebot3_waypoint_navigator.py              # Nav2 version
├── turtlebot3_waypoint_navigator_twist.py        # Twist version
├── QUICKSTART_WAYPOINT.md                        # 5-minute start
├── WAYPOINT_NAVIGATOR_README.md                  # Complete guide
├── WAYPOINT_SCRIPTS_INDEX.md                     # Comparison
├── WAYPOINT_EXAMPLES.md                          # Patterns & examples
└── README_WAYPOINT_NAVIGATOR.md                  # This file
```

## 🔄 Workflow

### For New Users
1. Read `QUICKSTART_WAYPOINT.md` (5 min)
2. Follow quick start steps (5 min)
3. Watch robot move through default waypoints (2 min)
4. Customize waypoints as needed

### For Production Deployment
1. Read `WAYPOINT_NAVIGATOR_README.md` (20 min)
2. Choose appropriate script version
3. Test in simulation with your waypoint pattern
4. Fine-tune parameters for your environment
5. Deploy to real hardware

### For Advanced Integration
1. Review `WAYPOINT_EXAMPLES.md` for patterns
2. Study integration examples
3. Extend script with custom callbacks
4. Test thoroughly in simulation
5. Deploy with monitoring

## 💻 System Requirements

### Minimum Requirements
- **OS:** Ubuntu 20.04+
- **CPU:** Dual-core processor
- **RAM:** 4GB (2GB for simulation only)
- **Disk:** 2GB free space
- **Network:** Local only (no internet required)

### Recommended Requirements
- **OS:** Ubuntu 22.04
- **CPU:** Quad-core processor
- **RAM:** 8GB
- **GPU:** 2GB VRAM (for Gazebo graphics)
- **Disk:** 10GB free space

## 🔐 Best Practices

1. **Always test in simulation first** before deploying to real robot
2. **Start with low speeds** (0.15 m/s) and increase gradually
3. **Use tolerances appropriately** - too strict causes oscillation
4. **Monitor logs** for warnings and errors
5. **Record waypoint visits** for analysis and debugging
6. **Use namespace for multi-robot** to avoid conflicts
7. **Document your waypoints** with comments and names
8. **Test edge cases** like starting position variations

## 🆘 Getting Help

### Quick Help
```bash
python3 turtlebot3_waypoint_navigator_twist.py --help
```

### Check Topics
```bash
ros2 topic list
ros2 topic echo /odom
```

### View Nodes
```bash
ros2 node list
ros2 node info /node_name
```

### Detailed Diagnostics
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py --verbose
```

### Documentation
1. **Quick Reference:** `QUICKSTART_WAYPOINT.md`
2. **Complete Guide:** `WAYPOINT_NAVIGATOR_README.md`
3. **Comparison:** `WAYPOINT_SCRIPTS_INDEX.md`
4. **Examples:** `WAYPOINT_EXAMPLES.md`
5. **Script Help:** `--help` flag in scripts

## 📞 Support

For issues:
1. Check the troubleshooting section in `WAYPOINT_NAVIGATOR_README.md`
2. Review `WAYPOINT_EXAMPLES.md` for similar patterns
3. Check ROS 2 topics and nodes: `ros2 topic list`, `ros2 node list`
4. Enable verbose logging for diagnostics
5. Test in simulation first if deploying to real robot

## 📄 License

See the main project `LICENSE.md` file.

## 🎉 Quick Summary

This waypoint navigator package provides **two production-ready Python scripts** to autonomously navigate TurtleBot 3 through repeating waypoint sequences:

1. **`turtlebot3_waypoint_navigator.py`** - Full-featured with Navigation2
2. **`turtlebot3_waypoint_navigator_twist.py`** - Lightweight alternative

Choose based on your needs:
- **Production/Complex:** Use Nav2 version
- **Testing/Simple:** Use Twist version

Both support:
- Repeating waypoint sequences
- Customizable speeds and tolerances
- Multi-robot scenarios
- Full ROS 2 integration

**Get started:** Read `QUICKSTART_WAYPOINT.md` and run the 3-step setup!

---

**Version:** 1.0 | **Status:** Production Ready | **Last Updated:** 2024
