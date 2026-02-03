# Build and Test Initial Pose Fix - Complete Guide

## 🔴 CRITICAL ISSUE IDENTIFIED

**Problem:** The fix has been applied to the source code, but **the package has NOT been rebuilt**.

The old compiled version (without the fix) is still being used when you run the navigator!

---

## 🔍 Quick Diagnosis

Run this to verify:

```bash
cd robosapiens-adaptive-platform-turtlebot

# Check source has the fix
grep -n "Waiting for AMCL to subscribe" turtlebotrossim/src/turtlebot3_waypoint_navigator/turtlebot3_waypoint_navigator/navigator_nav2.py

# Check installed version has the fix
find turtlebotrossim/install -name "navigator_nav2.py" -exec grep -l "Waiting for AMCL to subscribe" {} \;
```

**Expected Results:**
- First command: Should show line number (around 144) ✅
- Second command: Should show the installed file path ✅

**If second command shows nothing:** Package needs to be rebuilt! ❌

---

## 🏗️ Build Instructions

### Option A: Local Build (Recommended if not using Docker)

#### Step 1: Navigate to Workspace
```bash
cd robosapiens-adaptive-platform-turtlebot/turtlebotrossim
```

#### Step 2: Clean Previous Build
```bash
# Remove old build artifacts
rm -rf build/turtlebot3_waypoint_navigator
rm -rf install/turtlebot3_waypoint_navigator  # May need sudo
```

If you get permission denied:
```bash
sudo rm -rf install/turtlebot3_waypoint_navigator
```

#### Step 3: Build Package
```bash
colcon build --packages-select turtlebot3_waypoint_navigator
```

**Expected Output:**
```
Starting >>> turtlebot3_waypoint_navigator
Finished <<< turtlebot3_waypoint_navigator [X.XXs]

Summary: 1 package finished [X.XXs]
```

#### Step 4: Source the Workspace
```bash
source install/setup.bash
```

#### Step 5: Verify Fix is Installed
```bash
find install -name "navigator_nav2.py" -exec grep -l "Waiting for AMCL to subscribe" {} \;
```

**Must return the path!** If not, rebuild failed.

---

### Option B: Docker Build (If using Docker)

#### Step 1: Check if Using Docker
```bash
# Are you running commands inside a Docker container?
# Check with:
cat /proc/1/cgroup | grep docker
```

If output contains "docker", you're in a container.

#### Step 2: Rebuild Docker Image

**From the host machine (outside container):**

```bash
cd robosapiens-adaptive-platform-turtlebot/turtlebotrossim

# Rebuild the Docker image
docker-compose -f docker/docker-compose.yaml build dev
```

This rebuilds the image with the updated source code.

#### Step 3: Restart Container

```bash
# Stop existing container
docker-compose -f docker/docker-compose.yaml down

# Start new container with updated image
docker-compose -f docker/docker-compose.yaml up -d dev
```

#### Step 4: Enter Container and Rebuild

```bash
# Enter the container
docker-compose -f docker/docker-compose.yaml exec dev bash

# Inside container - rebuild package
cd /path/to/workspace  # Adjust to your workspace path
colcon build --packages-select turtlebot3_waypoint_navigator
source install/setup.bash
```

---

### Option C: Use Rebuild Script (Automated)

We've created a script that does everything:

```bash
cd robosapiens-adaptive-platform-turtlebot
./rebuild_navigator.sh
```

This script:
- ✅ Verifies source has the fix
- ✅ Cleans old build
- ✅ Builds package
- ✅ Verifies fix is installed
- ✅ Sources workspace
- ✅ Provides next steps

---

## 🧪 Testing the Fix

### Prerequisites

**Terminal 1 - Gazebo:**
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2 - Nav2 (WAIT 60 SECONDS AFTER LAUNCH!):**
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    use_sim_time:=true \
    map:=$(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/maps/map.yaml
```

⚠️ **CRITICAL:** You must wait at least 60 seconds for Nav2 to fully initialize!

### Run the Navigator

**Terminal 3 - Navigator (FRESH TERMINAL!):**
```bash
cd robosapiens-adaptive-platform-turtlebot/turtlebotrossim
source install/setup.bash  # MUST source in the terminal you're running from!
ros2 run turtlebot3_waypoint_navigator waypoint_navigator_nav2
```

---

## ✅ Success Indicators

### Log Messages (MUST appear in this order)

```
[INFO] TurtleBot 3 Waypoint Navigator (Nav2) initialized
[INFO] Loaded 18 waypoints
[INFO] Max loops: 5
[INFO] Pause at waypoint: 1.0s
[INFO] Waiting for NavigateToPose action server...
[INFO] NavigateToPose action server is available!
[INFO] Starting waypoint navigation...
[INFO] Setting initial pose: (-2.00, -0.50, 0.00 rad)

✓ [INFO] Waiting for AMCL to subscribe to /initialpose...  <-- NEW MESSAGE!
✓ [INFO] Still waiting for AMCL subscriber... (1.0s elapsed)
✓ [INFO] Still waiting for AMCL subscriber... (2.0s elapsed)
✓ [INFO] AMCL subscriber connected! (1 subscriber(s))  <-- NEW MESSAGE!
✓ [INFO] Publishing initial pose...  <-- NEW MESSAGE!
✓ [INFO] First initial pose message sent  <-- NEW MESSAGE!
✓ [INFO] Initial pose published to AMCL (10 times over 2 seconds)  <-- NEW MESSAGE!
✓ [INFO] Giving AMCL 2 more seconds to process initial pose...  <-- NEW MESSAGE!

✓ [INFO] Waiting for AMCL to converge...  <-- NEW MESSAGE!
✓ [INFO] Still waiting for AMCL... (1.0s elapsed)
✓ [INFO] Still waiting for AMCL... (2.0s elapsed)
✓ [INFO] AMCL pose received: (-2.00, -0.50)  <-- NEW MESSAGE!

✓ [INFO] AMCL converged successfully!  <-- NEW MESSAGE!
✓ [INFO] Robot localized at (-2.00, -0.50)  <-- NEW MESSAGE!
✓ [INFO] Waiting 3 more seconds for particle filter to stabilize...
✓ [INFO] Starting navigation to first waypoint...

✓ [INFO] [Loop 0] Navigating to Start West (-2.20, -0.50)
✓ [INFO] Goal accepted by the action server  <-- SUCCESS!!!
✓ [INFO] Goal succeeded!
```

### Key Differences from Old Behavior

| Old (Broken) | New (Fixed) |
|--------------|-------------|
| No "Waiting for AMCL to subscribe" | ✅ Shows subscriber wait |
| No "AMCL subscriber connected" | ✅ Shows connection confirmed |
| No "First initial pose message sent" | ✅ Shows message sent |
| No "AMCL pose received" | ✅ Shows AMCL response |
| No "AMCL converged successfully" | ✅ Shows convergence |
| ❌ "Goal was rejected" | ✅ "Goal accepted" |
| Fails in ~3 seconds | Succeeds in ~20 seconds |

---

## 🚫 If You Still See "Goal was rejected"

The fix is **NOT INSTALLED**. You're still running the old code.

### Checklist:

1. **Did you rebuild the package?**
   ```bash
   colcon build --packages-select turtlebot3_waypoint_navigator
   ```

2. **Did you source the workspace in the terminal you're running from?**
   ```bash
   source install/setup.bash
   ```
   
   ⚠️ Each new terminal needs to source the workspace!

3. **Did you verify the installed version has the fix?**
   ```bash
   find install -name "navigator_nav2.py" -exec grep -l "Waiting for AMCL to subscribe" {} \;
   ```

4. **Are you running the correct executable?**
   ```bash
   which ros2
   ros2 pkg prefix turtlebot3_waypoint_navigator
   ```
   Should point to your workspace install directory.

5. **Is colcon available?**
   ```bash
   which colcon
   ```
   If not found:
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

---

## 🐳 Docker-Specific Issues

### Issue: Changes not in container

**Problem:** You modified source on host but container still has old code.

**Solution:**
```bash
# Exit container
exit

# Rebuild image
cd robosapiens-adaptive-platform-turtlebot/turtlebotrossim
docker-compose -f docker/docker-compose.yaml build dev

# Restart container
docker-compose -f docker/docker-compose.yaml down
docker-compose -f docker/docker-compose.yaml up -d dev

# Enter and rebuild
docker-compose -f docker/docker-compose.yaml exec dev bash
cd /workspace  # or wherever your workspace is
colcon build --packages-select turtlebot3_waypoint_navigator
source install/setup.bash
```

### Issue: Volume mounting

If using volume mounts, the source changes should appear automatically, but you still need to rebuild inside the container.

---

## 📊 Quick Verification Script

Save this as `check_fix.sh`:

```bash
#!/bin/bash

echo "=== Checking Initial Pose Fix Status ==="
echo ""

# Check source
echo "1. Checking source file..."
if grep -q "Waiting for AMCL to subscribe" turtlebotrossim/src/turtlebot3_waypoint_navigator/turtlebot3_waypoint_navigator/navigator_nav2.py; then
    echo "   ✓ Fix present in source"
else
    echo "   ✗ Fix NOT in source - run fix_initial_pose_publisher.py first"
    exit 1
fi

# Check install
echo "2. Checking installed file..."
INSTALLED=$(find turtlebotrossim/install -name "navigator_nav2.py" -exec grep -l "Waiting for AMCL to subscribe" {} \; 2>/dev/null)
if [ -n "$INSTALLED" ]; then
    echo "   ✓ Fix present in installed version"
    echo "   Location: $INSTALLED"
else
    echo "   ✗ Fix NOT in installed version"
    echo "   YOU NEED TO REBUILD THE PACKAGE!"
    echo ""
    echo "   Run: cd turtlebotrossim && colcon build --packages-select turtlebot3_waypoint_navigator"
    exit 1
fi

echo ""
echo "✅ Fix is properly installed and ready to use!"
echo ""
echo "Next: Run the navigator and check for success messages"
```

Run it:
```bash
chmod +x check_fix.sh
./check_fix.sh
```

---

## 🎯 Step-by-Step: Complete Test from Scratch

### Minute 0: Clean Start

```bash
cd robosapiens-adaptive-platform-turtlebot
```

### Minute 1: Verify and Rebuild

```bash
# Verify source has fix
grep "Waiting for AMCL to subscribe" turtlebotrossim/src/turtlebot3_waypoint_navigator/turtlebot3_waypoint_navigator/navigator_nav2.py

# If above shows nothing, run: python3 fix_initial_pose_publisher.py

# Clean and rebuild
cd turtlebotrossim
rm -rf build/turtlebot3_waypoint_navigator
sudo rm -rf install/turtlebot3_waypoint_navigator  # If needed
colcon build --packages-select turtlebot3_waypoint_navigator

# Verify installation
find install -name "navigator_nav2.py" -exec grep -l "Waiting for AMCL to subscribe" {} \;
# MUST show path!
```

### Minute 3: Launch Gazebo (Terminal 1)

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Wait for Gazebo to fully load (robot visible).

### Minute 5: Launch Nav2 (Terminal 2)

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true
```

**WAIT 60 SECONDS!** Set a timer. Go get coffee. ☕

### Minute 6: Run Navigator (Terminal 3 - NEW TERMINAL!)

```bash
cd robosapiens-adaptive-platform-turtlebot/turtlebotrossim
source install/setup.bash  # CRITICAL - source in THIS terminal!
ros2 run turtlebot3_waypoint_navigator waypoint_navigator_nav2
```

### Minute 7: Watch Logs

Look for the NEW messages:
- "Waiting for AMCL to subscribe..."
- "AMCL subscriber connected!"
- "First initial pose message sent"
- "AMCL pose received"
- "AMCL converged successfully!"
- **"Goal accepted by the action server"** ← THIS IS SUCCESS!

### Minute 8: Celebrate! 🎉

If you see "Goal accepted" - the fix is working!

---

## 📞 Still Not Working?

### Try Manual Initial Pose

In a separate terminal, manually publish initial pose:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: -2.0, y: -0.5, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.07]
  }
}"
```

Then check AMCL response:
```bash
ros2 topic echo /amcl_pose
```

If AMCL responds to manual pose but not from the navigator, the issue is with the build/installation.

---

## 📋 Final Checklist

Before running navigator, verify:

- [ ] Source file has fix (grep shows line number)
- [ ] Installed file has fix (find command shows path)
- [ ] Package was rebuilt (colcon build succeeded)
- [ ] Workspace was sourced IN THE TERMINAL YOU'RE USING
- [ ] Gazebo is running
- [ ] Nav2 is running and waited 60 seconds
- [ ] AMCL node is running (`ros2 node list | grep amcl`)
- [ ] Map is loaded (`ros2 topic echo /map --once`)
- [ ] Using a FRESH terminal for navigator (not one from before rebuild)

---

## 🏁 Summary

**The Problem:** Fix was in source but not compiled/installed.

**The Solution:** Rebuild the package:
```bash
cd turtlebotrossim
colcon build --packages-select turtlebot3_waypoint_navigator
source install/setup.bash
```

**The Test:** Run navigator and see "Goal accepted" instead of "Goal was rejected".

**If still failing:** The installed version still doesn't have the fix. Check you're sourcing the correct workspace and using the rebuilt package.

---

*Last updated: 2024*  
*Fix status: Applied to source, requires rebuild to take effect*
