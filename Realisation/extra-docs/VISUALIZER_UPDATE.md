# Visualizer Update Guide

## Quick Update Instructions

If you already have the waypoint navigator setup, follow these steps to add the visualizer functionality.

## What Changed

The map visualizer scripts have been added to the Docker image. You need to **rebuild your Docker images** to include them.

## Update Steps

### Step 1: Pull Latest Code

```bash
cd robosapiens-adaptive-platform-turtlebot
git pull  # Or however you update your code
```

### Step 2: Verify Scripts Are Present

```bash
# Check that the visualizer scripts exist
ls -la turtlebotrossim/docker/scripts/

# You should see:
# - map_visualizer.py
# - run_visualizer.sh
```

### Step 3: Rebuild Docker Images

**Option A: Using the main script (Recommended)**

```bash
./run_waypoint_navigator_docker.sh build
```

**Option B: Using docker compose directly**

```bash
cd robosapiens-adaptive-platform-turtlebot/turtlebotrossim
docker compose -f docker/docker-compose.waypoint-navigator.yaml build --no-cache
```

**Note:** The rebuild may take 5-15 minutes depending on your system.

### Step 4: Verify Installation

Start your navigation:

```bash
./run_waypoint_navigator_docker.sh start
```

Once running, test the visualizer:

```bash
./run_waypoint_navigator_docker.sh visualize
```

You should see the live ASCII map display!

## Troubleshooting

### Issue: "No such file or directory: /opt/scripts/run_visualizer.sh"

**Cause:** Docker image wasn't rebuilt with the new scripts.

**Solution:**
```bash
# Force a clean rebuild
./run_waypoint_navigator_docker.sh stop
docker compose -f turtlebotrossim/docker/docker-compose.waypoint-navigator.yaml build --no-cache
./run_waypoint_navigator_docker.sh start
```

### Issue: "map_visualizer.py not found"

**Cause:** Scripts missing from source directory.

**Solution:**
```bash
# Verify scripts exist
ls turtlebotrossim/docker/scripts/

# If missing, the files should be:
# - turtlebotrossim/docker/scripts/map_visualizer.py
# - turtlebotrossim/docker/scripts/run_visualizer.sh

# Make sure they're executable
chmod +x turtlebotrossim/docker/scripts/*.py
chmod +x turtlebotrossim/docker/scripts/*.sh
```

### Issue: Build fails

**Cause:** Docker cache issues or incomplete files.

**Solution:**
```bash
# Clean everything and rebuild
./run_waypoint_navigator_docker.sh clean
docker system prune -f
./run_waypoint_navigator_docker.sh build
```

## What's Included

After rebuilding, your Docker image will contain:

1. **`/opt/scripts/map_visualizer.py`**
   - Main visualizer ROS2 node
   - Subscribes to navigation topics
   - Renders ASCII map display

2. **`/opt/scripts/run_visualizer.sh`**
   - Wrapper script
   - Sources ROS environment
   - Validates setup
   - Launches visualizer

## File Locations

```
Host System:
  robosapiens-adaptive-platform-turtlebot/
  └── turtlebotrossim/
      └── docker/
          ├── Dockerfile                    [UPDATED: Copies scripts]
          └── scripts/
              ├── map_visualizer.py         [NEW]
              └── run_visualizer.sh         [NEW]

Inside Docker Container:
  /opt/
  └── scripts/
      ├── map_visualizer.py                [Copied during build]
      └── run_visualizer.sh                [Copied during build]
```

## Quick Reference

```bash
# Build with visualizer support
./run_waypoint_navigator_docker.sh build

# Start navigation
./run_waypoint_navigator_docker.sh start

# Launch visualizer (new terminal)
./run_waypoint_navigator_docker.sh visualize

# Exit visualizer
# Press Ctrl+C
```

## Verification Checklist

After updating, verify everything works:

- [ ] Scripts exist: `ls turtlebotrossim/docker/scripts/`
- [ ] Docker image rebuilt: `./run_waypoint_navigator_docker.sh build`
- [ ] Navigation starts: `./run_waypoint_navigator_docker.sh start`
- [ ] Status shows running: `./run_waypoint_navigator_docker.sh status`
- [ ] Visualizer launches: `./run_waypoint_navigator_docker.sh visualize`
- [ ] Display updates in real-time
- [ ] Ctrl+C exits cleanly

## Need Help?

If you encounter issues after following these steps:

1. **Check Docker image was rebuilt:**
   ```bash
   docker images | grep turtlebot4
   # Look for recent timestamp
   ```

2. **Verify scripts are in container:**
   ```bash
   ./run_waypoint_navigator_docker.sh shell
   ls -la /opt/scripts/
   # Should show map_visualizer.py and run_visualizer.sh
   exit
   ```

3. **Check logs:**
   ```bash
   ./run_waypoint_navigator_docker.sh logs
   ```

4. **Review documentation:**
   - [VISUALIZER_QUICKSTART.md](VISUALIZER_QUICKSTART.md) - Quick start
   - [VISUALIZER_README.md](VISUALIZER_README.md) - Full guide
   - [VISUALIZER_INDEX.md](VISUALIZER_INDEX.md) - Doc index

## Next Steps

Once updated successfully:

1. Read [VISUALIZER_QUICKSTART.md](VISUALIZER_QUICKSTART.md) for usage guide
2. See [VISUALIZER_README.md](VISUALIZER_README.md) for detailed features
3. Try the examples in the quick start guide

---

**Summary:** Just run `./run_waypoint_navigator_docker.sh build` to update!
