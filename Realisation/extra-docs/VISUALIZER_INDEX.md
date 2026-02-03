# TurtleBot3 Map Visualizer - Documentation Index

Complete guide to the live command-line map visualization tool for TurtleBot3 navigation monitoring.

## 📚 Documentation Overview

This index provides quick access to all visualizer documentation and resources.

## 🚀 Quick Links

| Document | Purpose | Audience |
|----------|---------|----------|
| [VISUALIZER_QUICKSTART.md](VISUALIZER_QUICKSTART.md) | Get started in 2 minutes | New users |
| [VISUALIZER_README.md](VISUALIZER_README.md) | Complete feature guide | All users |
| [RUN_WAYPOINT_DOCKER.md](RUN_WAYPOINT_DOCKER.md) | Integration with waypoint navigator | Developers |
| [docker/scripts/map_visualizer.py](docker/scripts/map_visualizer.py) | Source code | Advanced users |
| [docker/scripts/run_visualizer.sh](docker/scripts/run_visualizer.sh) | Launcher script | Developers |

## 📖 Documentation by Topic

### Getting Started

1. **[VISUALIZER_QUICKSTART.md](VISUALIZER_QUICKSTART.md)**
   - Installation: None required (included in Docker setup)
   - First run: `./run_waypoint_navigator_docker.sh visualize`
   - Basic usage and common commands
   - Quick troubleshooting

### Complete Guide

2. **[VISUALIZER_README.md](VISUALIZER_README.md)**
   - Feature overview and capabilities
   - Display elements explained
   - ROS topics subscribed
   - Configuration options
   - Advanced usage scenarios
   - Troubleshooting guide
   - Technical details and architecture

### Integration

3. **[RUN_WAYPOINT_DOCKER.md](RUN_WAYPOINT_DOCKER.md)** (See "Live Map Visualization" section)
   - Using visualizer with waypoint navigator
   - Multi-terminal setup
   - Integration with RViz
   - Combined workflows

## 🎯 Quick Command Reference

### Essential Commands

```bash
# Start visualization
./run_waypoint_navigator_docker.sh visualize

# Check system status
./run_waypoint_navigator_docker.sh status

# Monitor ROS topics
./run_waypoint_navigator_docker.sh monitor

# View logs
./run_waypoint_navigator_docker.sh logs

# Get help
./run_waypoint_navigator_docker.sh help
```

### Typical Workflow

```bash
# Terminal 1: Start navigation
./run_waypoint_navigator_docker.sh start

# Terminal 2: Launch visualizer
./run_waypoint_navigator_docker.sh visualize

# Terminal 3 (optional): Monitor logs
./run_waypoint_navigator_docker.sh logs -f
```

## 🗂️ File Structure

```
robosapiens-adaptive-platform-turtlebot/
│
├── Documentation (Start Here)
│   ├── VISUALIZER_INDEX.md              ← You are here
│   ├── VISUALIZER_QUICKSTART.md         ← 2-minute quick start
│   ├── VISUALIZER_README.md             ← Complete guide
│   └── RUN_WAYPOINT_DOCKER.md           ← Integration guide
│
├── Scripts (Core Implementation)
│   └── docker/scripts/
│       ├── map_visualizer.py            ← Main visualizer (Python)
│       └── run_visualizer.sh            ← Launcher wrapper (Bash)
│
├── Docker Integration
│   ├── run_waypoint_navigator_docker.sh ← Main control script
│   └── docker/
│       └── docker-compose.waypoint-navigator.yaml
│
└── Related Documentation
    ├── WAYPOINT_NAVIGATOR_README.md     ← Navigator details
    ├── QUICKSTART_WAYPOINT.md           ← Waypoint quick start
    └── DOCKER_WAYPOINT_NAVIGATOR.md     ← Docker setup
```

## 🎓 Learning Path

### For New Users

1. **Start Here:** [VISUALIZER_QUICKSTART.md](VISUALIZER_QUICKSTART.md)
   - Prerequisites check
   - First run
   - Basic understanding

2. **Then Read:** [VISUALIZER_README.md](VISUALIZER_README.md) (Sections: Features, Display Elements)
   - Understand what you're seeing
   - Learn the symbols and layout

3. **Practice:** Run some examples
   - Follow the quick start workflow
   - Try different terminal layouts

### For Developers

1. **Integration:** [RUN_WAYPOINT_DOCKER.md](RUN_WAYPOINT_DOCKER.md)
   - Multi-terminal setup
   - Combining with other tools

2. **Customization:** [VISUALIZER_README.md](VISUALIZER_README.md) (Section: Configuration)
   - Adjust update rates
   - Modify display size
   - Add custom topics

3. **Source Code:** [docker/scripts/map_visualizer.py](docker/scripts/map_visualizer.py)
   - Implementation details
   - Extend functionality

### For Troubleshooting

1. **Quick Fixes:** [VISUALIZER_QUICKSTART.md](VISUALIZER_QUICKSTART.md) (Section: Quick Troubleshooting)
   - Common issues and solutions

2. **Detailed Guide:** [VISUALIZER_README.md](VISUALIZER_README.md) (Section: Troubleshooting)
   - Comprehensive problem diagnosis
   - Advanced debugging

3. **System Diagnostics:**
   ```bash
   ./run_waypoint_navigator_docker.sh status
   ./run_waypoint_navigator_docker.sh monitor
   ```

## 📋 Feature Summary

### What the Visualizer Shows

| Feature | Description | Update Rate |
|---------|-------------|-------------|
| **Position** | AMCL pose, odometry, goal location | Real-time |
| **Map View** | ASCII art top-down map (78×20 chars) | 5 Hz |
| **Velocity** | Linear/angular speed (actual vs commanded) | Real-time |
| **Sensors** | LiDAR data summary | Real-time |
| **Statistics** | Message counts, update timestamps | Real-time |
| **Status** | System health indicator | Real-time |

### ROS Topics Monitored

- `/map` - Occupancy grid
- `/odom` - Robot odometry
- `/amcl_pose` - Localized pose
- `/scan` - LiDAR data
- `/cmd_vel` - Velocity commands
- `/goal_pose` - Navigation goals

## 🔧 Common Use Cases

### 1. Monitoring Navigation Progress

**Goal:** Watch robot navigate waypoints in real-time

**Documents:** 
- [VISUALIZER_QUICKSTART.md](VISUALIZER_QUICKSTART.md) - Basic setup
- [RUN_WAYPOINT_DOCKER.md](RUN_WAYPOINT_DOCKER.md) - Integration

**Commands:**
```bash
./run_waypoint_navigator_docker.sh start
./run_waypoint_navigator_docker.sh visualize
```

### 2. Debugging Navigation Issues

**Goal:** Diagnose why robot isn't reaching waypoints

**Documents:**
- [VISUALIZER_README.md](VISUALIZER_README.md) - Troubleshooting section
- [VISUALIZER_QUICKSTART.md](VISUALIZER_QUICKSTART.md) - Quick fixes

**Diagnostic Steps:**
1. Check AMCL pose is updating (not "Waiting for localization")
2. Verify goal position is reasonable
3. Monitor velocity commands vs actual velocity
4. Check for obstacles in map view
5. Verify topic statistics show active updates

### 3. Remote Monitoring

**Goal:** Monitor robot over SSH without GUI

**Documents:**
- [VISUALIZER_README.md](VISUALIZER_README.md) - Remote monitoring section

**Commands:**
```bash
ssh user@robot-hostname
cd /path/to/project
./run_waypoint_navigator_docker.sh visualize
```

### 4. Development and Testing

**Goal:** Verify navigation behavior during development

**Documents:**
- [VISUALIZER_README.md](VISUALIZER_README.md) - Advanced usage
- [docker/scripts/map_visualizer.py](docker/scripts/map_visualizer.py) - Source

**Workflow:**
1. Make code changes
2. Restart navigator
3. Watch visualizer for behavior changes
4. Check topic statistics for message flow

## ❓ Common Questions

### Q: Do I need to install anything?
**A:** No, the visualizer is included in the Docker setup.

### Q: Can I use this without Docker?
**A:** Yes, run `python3 docker/scripts/map_visualizer.py` in a ROS2 environment.

### Q: Does it work over SSH?
**A:** Yes! It's designed for terminal-only environments.

### Q: Can I customize the display?
**A:** Yes, see [VISUALIZER_README.md](VISUALIZER_README.md) Configuration section.

### Q: What if my terminal doesn't support Unicode arrows?
**A:** The script will fall back to ASCII alternatives. Ensure UTF-8 encoding: `export LANG=en_US.UTF-8`

### Q: How do I record the output?
**A:** Use `script` command: `script -c "./run_waypoint_navigator_docker.sh visualize" session.log`

### Q: Can I monitor multiple robots?
**A:** Currently single robot. Multi-robot support is a potential future enhancement.

## 🐛 Troubleshooting Quick Reference

| Issue | Quick Fix | Details |
|-------|-----------|---------|
| No data displayed | Wait 30s for initialization | [VISUALIZER_QUICKSTART.md](VISUALIZER_QUICKSTART.md#quick-troubleshooting) |
| Garbled display | Check terminal UTF-8 support | [VISUALIZER_README.md](VISUALIZER_README.md#troubleshooting) |
| "Could not start" | Verify simulation running | [VISUALIZER_QUICKSTART.md](VISUALIZER_QUICKSTART.md#quick-troubleshooting) |
| Stale status indicator | Check navigation active | [VISUALIZER_README.md](VISUALIZER_README.md#status-shows-stale) |
| Map shows only "?" | Wait for map publication | [VISUALIZER_README.md](VISUALIZER_README.md#map-not-visible) |

## 🔗 Related Resources

### Core Navigation Documentation
- [WAYPOINT_NAVIGATOR_README.md](WAYPOINT_NAVIGATOR_README.md)
- [QUICKSTART_WAYPOINT.md](QUICKSTART_WAYPOINT.md)
- [WAYPOINT_EXAMPLES.md](WAYPOINT_EXAMPLES.md)

### Docker and Setup
- [DOCKER_WAYPOINT_NAVIGATOR.md](DOCKER_WAYPOINT_NAVIGATOR.md)
- [QUICKSTART_DOCKER.md](QUICKSTART_DOCKER.md)
- [RUN_WAYPOINT_DOCKER.md](RUN_WAYPOINT_DOCKER.md)

### Technical Documentation
- [ROS2_PACKAGE_SUMMARY.md](ROS2_PACKAGE_SUMMARY.md)
- [TESTING_GUIDE.md](TESTING_GUIDE.md)

## 📝 Document Status

| Document | Status | Last Updated |
|----------|--------|--------------|
| VISUALIZER_INDEX.md | ✅ Current | 2024 |
| VISUALIZER_QUICKSTART.md | ✅ Current | 2024 |
| VISUALIZER_README.md | ✅ Current | 2024 |
| map_visualizer.py | ✅ Implemented | 2024 |
| run_visualizer.sh | ✅ Implemented | 2024 |

## 🚦 Getting Started Checklist

Before using the visualizer:

- [ ] Docker and Docker Compose installed
- [ ] TurtleBot3 packages installed (included in Docker)
- [ ] Waypoint navigator working
- [ ] Terminal supports UTF-8
- [ ] Terminal at least 80 characters wide

To verify setup:
```bash
./run_waypoint_navigator_docker.sh status
```

## 💡 Tips for Best Experience

1. **Terminal Size:** Use full-screen or at least 80×40 characters
2. **Multiple Monitors:** Put visualizer on second monitor
3. **Logging:** Combine with log monitoring in split terminal
4. **Screenshots:** Use terminal screenshot feature for documentation
5. **Color Terminals:** Some terminals display Unicode better than others

## 🎬 Next Steps

1. **New to visualizer?** → [VISUALIZER_QUICKSTART.md](VISUALIZER_QUICKSTART.md)
2. **Want details?** → [VISUALIZER_README.md](VISUALIZER_README.md)
3. **Setting up navigation?** → [RUN_WAYPOINT_DOCKER.md](RUN_WAYPOINT_DOCKER.md)
4. **Have questions?** → Check troubleshooting sections in docs above

---

**Ready to visualize?**

```bash
./run_waypoint_navigator_docker.sh visualize
```

**Need help?** Start with [VISUALIZER_QUICKSTART.md](VISUALIZER_QUICKSTART.md)
