# Map Visualizer Quick Start Guide

Get up and running with the TurtleBot3 live command-line map visualizer in 2 minutes.

## What is This?

A real-time ASCII art dashboard that shows your robot's position, map, sensors, and navigation status directly in your terminal. No GUI required!

## Prerequisites

- Docker and Docker Compose installed
- TurtleBot3 waypoint navigator already set up
- Terminal that supports UTF-8 (most modern terminals)

## ⚠️ First Time Setup

**If this is your first time using the visualizer**, you need to rebuild the Docker image to include the visualizer scripts:

```bash
./run_waypoint_navigator_docker.sh build
```

This takes 5-15 minutes. You only need to do this once (or when updating to a new version with visualizer changes).

**Already rebuilt?** Skip to Quick Start below.

**Need detailed update instructions?** See [VISUALIZER_UPDATE.md](VISUALIZER_UPDATE.md)

## Quick Start

### Step 1: Start Navigation

```bash
./run_waypoint_navigator_docker.sh start
```

Wait for the simulation to fully initialize (~30 seconds).

### Step 2: Launch Visualizer

Open a new terminal and run:

```bash
./run_waypoint_navigator_docker.sh visualize
```

That's it! You should now see a live updating display.

## What You'll See

```
================================================================================
                  TurtleBot3 Live Navigation Status
================================================================================

┌─ POSITION ─────────────────────────────────────────────────────────────────┐
│ AMCL Pose:    X:   0.000 m  Y:   0.000 m  θ:    0.0°                       │
│ Odometry:     X:   0.000 m  Y:   0.000 m  θ:    0.0°                       │
│ Goal:         X:   1.500 m  Y:   0.000 m  Dist:  1.500 m                   │
└────────────────────────────────────────────────────────────────────────────┘

┌─ MAP VIEW ─────────────────────────────────────────────────────────────────┐
│··········································································│
│··········································································│
│··········································································│
│·····················→·························★··························│
│··········································································│
│  Legend:  → Robot   ★ Goal   █ Obstacle   · Free   ? Unknown            │
└────────────────────────────────────────────────────────────────────────────┘
```

**Key Elements:**
- `→` = Your robot (arrow shows direction)
- `★` = Current navigation goal
- `█` = Obstacles/walls
- `·` = Free space
- `?` = Unexplored areas

## Common Commands

### View Help
```bash
./run_waypoint_navigator_docker.sh help
```

### Check Status
```bash
./run_waypoint_navigator_docker.sh status
```

### View Logs
```bash
./run_waypoint_navigator_docker.sh logs
```

### Stop Everything
```bash
./run_waypoint_navigator_docker.sh stop
```

### Exit Visualizer
Press `Ctrl+C` in the visualizer terminal.

## Quick Troubleshooting

### "No AMCL pose data available yet..."

**Solution:** Wait 10-30 seconds for localization to initialize.

### Blank or Garbled Display

**Solution:** Ensure your terminal is at least 80 characters wide and supports UTF-8:
```bash
echo $LANG  # Should show UTF-8
export LANG=en_US.UTF-8
```

### "Could not start visualizer"

**Solutions:**
1. Verify simulation is running:
   ```bash
   ./run_waypoint_navigator_docker.sh status
   ```

2. Check if container is healthy:
   ```bash
   docker ps
   ```

3. Try restarting:
   ```bash
   ./run_waypoint_navigator_docker.sh restart
   ```

### Status Shows "🔴 STALE"

This means no data received for >1 second. Usually normal when:
- Robot is paused at a waypoint
- Navigation completed
- Waiting for new commands

Check logs to confirm:
```bash
./run_waypoint_navigator_docker.sh logs waypoint-navigator
```

## Typical Workflow

```bash
# Terminal 1: Start navigation
./run_waypoint_navigator_docker.sh start

# Wait ~30 seconds for initialization...

# Terminal 2: Monitor with visualizer
./run_waypoint_navigator_docker.sh visualize

# Terminal 3 (optional): Watch logs
./run_waypoint_navigator_docker.sh logs -f
```

## Understanding the Display

### Position Box
- **AMCL Pose**: Most accurate (uses sensor fusion)
- **Odometry**: Raw wheel encoders (may drift)
- **Goal**: Current target and distance remaining

### Velocity Box
- Shows current speed vs commanded speed
- Helps verify robot is responding to commands

### Map View
- Updates 5 times per second
- Centered on robot position
- ~6m × 3m viewing area (adjustable)

### Statistics Box
- Message counts from each topic
- Status indicator (🟢 ACTIVE / 🔴 STALE)
- Time since last update

## Next Steps

### Full Documentation
See [VISUALIZER_README.md](VISUALIZER_README.md) for:
- Detailed feature descriptions
- Configuration options
- Advanced usage
- Integration with other tools

### Customize Settings
Edit `docker/scripts/map_visualizer.py` to change:
- Update rate (default: 5 Hz)
- Map size (default: 78×20)
- Display layout

### Run Standalone
Inside the container:
```bash
./run_waypoint_navigator_docker.sh shell
/workspace/docker/scripts/run_visualizer.sh
```

## Tips & Tricks

1. **Resize Terminal**: Bigger terminal = more map detail
2. **Multiple Monitors**: Put visualizer on second screen
3. **Record Session**: `script -c "./run_waypoint_navigator_docker.sh visualize" session.log`
4. **Screenshot**: Use your terminal's screenshot feature
5. **Remote Monitoring**: Works great over SSH!

## Common Use Cases

### Debugging Navigation
Watch the robot's position and goal to verify:
- Navigation is progressing
- Goals are reasonable
- Robot isn't stuck

### Monitoring Autonomous Operation
Leave visualizer running to monitor:
- Waypoint progress
- Obstacle avoidance
- System health

### Demonstrations
Show live navigation status without RViz:
- Lightweight
- Works over SSH/remote
- No GUI required

## Getting Help

If you encounter issues:

1. Check [VISUALIZER_README.md](VISUALIZER_README.md) - Comprehensive guide
2. Check [TROUBLESHOOTING.md](TROUBLESHOOTING.md) - Common issues
3. Run diagnostics: `./run_waypoint_navigator_docker.sh monitor`
4. View logs: `./run_waypoint_navigator_docker.sh logs`

## Example Session

```bash
# Start everything
user@host:~/project$ ./run_waypoint_navigator_docker.sh start
✓ Building workspace...
✓ Starting services...
✓ Simulation ready

# Launch visualizer
user@host:~/project$ ./run_waypoint_navigator_docker.sh visualize

╔════════════════════════════════════════════════════════════════════════╗
║          TurtleBot3 Live Map Visualizer - Starting...                 ║
╚════════════════════════════════════════════════════════════════════════╝

[1/3] Sourcing ROS environment...
✓ ROS Humble environment sourced
✓ Workspace environment sourced
[2/3] Checking visualizer script...
✓ Visualizer script found
[3/3] Checking ROS connectivity...
✓ ROS nodes detected

════════════════════════════════════════════════════════════════════════
Starting Map Visualizer...
════════════════════════════════════════════════════════════════════════

Press Ctrl+C to exit

# [Live display appears and updates]
```

---

**Ready to visualize? Run:**
```bash
./run_waypoint_navigator_docker.sh visualize
```

**Questions?** See [VISUALIZER_README.md](VISUALIZER_README.md)
