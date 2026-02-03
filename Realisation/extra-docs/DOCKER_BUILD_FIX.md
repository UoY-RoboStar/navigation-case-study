# Docker Build Fix Documentation

## Problem Summary

The script `./run_waypoint_navigator_docker.sh build` was failing with permission errors and confusing PowerShell-related error messages on Linux.

## Root Cause

The Docker Compose configuration had a **fundamental design flaw**: it was mounting the host machine's `build/` and `install/` directories directly into the container:

```yaml
volumes:
  - ../build:/ws/build:rw
  - ../install:/ws/install:rw
```

This created permission conflicts because:

1. **User Mismatch**: The container runs as user `devuser` (UID 1000), but files created on the host by root couldn't be overwritten by the container user
2. **Bidirectional Problem**: Files created in the container had wrong ownership on the host machine
3. **Inconsistent State**: Previous builds created files owned by root that blocked subsequent builds

## Why PowerShell Errors Appeared (On Linux!)

The error messages mentioning PowerShell were **NOT** because the script was trying to run PowerShell scripts. Instead:

- **colcon** (the ROS 2 build tool) has a PowerShell extension installed by default
- This extension generates setup scripts for Windows compatibility (`.ps1` files)
- When colcon couldn't write to `/ws/install/` due to permission denied, it failed on ALL shell extensions:
  - PowerShell (`.ps1`)
  - Bash (`.bash`)
  - Zsh (`.zsh`)
  - Shell (`.sh`)
  - DSV (data-oriented shell variables)

The PowerShell error appeared first in the error output, causing confusion, but it was just one of many failing shell extensions due to the underlying permission problem.

## Solution

### Changes Made

1. **Replaced Host Directory Mounts with Docker Volumes** (`docker-compose.waypoint-navigator.yaml`):
   ```yaml
   volumes:
     - ../src:/ws/src:rw                  # Source code still mounted (read/write)
     - workspace_build:/ws/build          # Build artifacts in Docker volume
     - workspace_install:/ws/install      # Install artifacts in Docker volume
     - /dev/shm:/dev/shm
   ```

2. **Added Volume Permission Initialization** (`run_waypoint_navigator_docker.sh`):
   ```bash
   # Fix permissions on Docker volumes before building
   docker compose run --rm --user root deploy \
       bash -c "chown -R devuser:devuser /ws/build /ws/install"
   ```

3. **Cleaned Up Old Host Directories**:
   ```bash
   pkexec rm -rf turtlebotrossim/build turtlebotrossim/install
   ```

### Why This Works

- **Docker volumes** are managed entirely by Docker and isolated from host filesystem permissions
- The initialization step ensures volumes start with correct ownership for the `devuser` account
- Source code (`src/`) remains mounted from the host for easy editing
- Build artifacts (`build/`, `install/`) stay in Docker volumes where they belong
- No more permission conflicts between host and container

## Best Practice: Build Isolation

The build should happen **entirely in Docker** OR **entirely locally**, not mixed:

- ✅ **Correct (Docker)**: Source mounted from host, build artifacts in Docker volumes
- ✅ **Correct (Local)**: Everything on host, no Docker volumes
- ❌ **Incorrect (Mixed)**: Build artifacts shared between host and Docker with different user contexts

## Verification

After the fix, the build completes successfully:

```bash
$ ./run_waypoint_navigator_docker.sh build

>>> Building ROS 2 Workspace
Starting >>> turtlebot3_waypoint_navigator
Finished <<< turtlebot3_waypoint_navigator [1.28s]

Summary: 1 package finished [1.66s]
✓ Build completed successfully!
```

## Files Modified

1. `turtlebotrossim/docker/docker-compose.waypoint-navigator.yaml` - Changed volume mounts
2. `run_waypoint_navigator_docker.sh` - Added permission initialization step

## Additional Fix: UID Readonly Variable Error

### Problem

When running `./run_waypoint_navigator_docker.sh start`, the script failed with:
```
./run_waypoint_navigator_docker.sh: line 289: UID: readonly variable
```

### Cause

In bash, `UID` is a **read-only shell variable** that cannot be modified or exported. The script was trying to:
```bash
export UID=$(id -u)
```

This fails because bash protects the `UID` variable from being reassigned.

### Solution

Changed the variable name from `UID` to `USER_ID`:

1. **In `run_waypoint_navigator_docker.sh`**:
   ```bash
   export USER_ID=$(id -u)  # Changed from UID
   ```

2. **In `docker-compose.waypoint-navigator.yaml`**:
   ```yaml
   args:
     - UID=${USER_ID:-1000}  # Read from USER_ID environment variable
     - GID=${USER_ID:-1000}
   ```

3. **Updated help documentation** to reflect the correct environment variable name.

### Result

The start command now works without the readonly variable error.

## Additional Fix: Container Health Check Failure

### Problem

When running `./run_waypoint_navigator_docker.sh start`, the system failed with:
```
dependency failed to start: container tb3-sim-headless-nvidia is unhealthy
```

The simulation container was starting successfully but was marked as "unhealthy", preventing the waypoint navigator from starting.

### Cause

The health check in the Docker Compose file was configured as:
```yaml
healthcheck:
  test: ["CMD", "ros2", "topic", "list"]
```

This failed because:
1. The health check runs in a new shell process without the ROS environment
2. The `ros2` command is not in the system PATH by default
3. The ROS environment must be sourced first for `ros2` to be available

Docker health check inspection showed:
```
"Output": "OCI runtime exec failed: exec failed: unable to start container process: exec: \"ros2\": executable file not found in $PATH"
```

### Solution

Changed the health check to source the ROS environment before running the command:

```yaml
healthcheck:
  test:
    [
      "CMD",
      "bash",
      "-c",
      "source /opt/ros/humble/setup.bash && ros2 topic list",
    ]
  interval: 5s
  timeout: 10s
  retries: 10
  start_period: 30s
```

This ensures the health check:
1. Uses bash to run the command
2. Sources the ROS environment first
3. Then executes `ros2 topic list` to verify ROS topics are available

### Result

The containers now start successfully with healthy status:
```bash
$ ./run_waypoint_navigator_docker.sh start
✓ Simulation is ready!
ℹ Starting waypoint navigator...
 Container tb3-sim-headless-nvidia Healthy
 Container tb3-waypoint-navigator Started

NAME                      STATUS
tb3-sim-headless-nvidia   Up 21 seconds (healthy)
tb3-waypoint-navigator    Up 10 seconds
```

## Additional Fix: Gazebo Headless Initialization Hang

### Problem

After fixing all previous issues, the simulation container would start but the robot never spawned, and odometry was never published:
- `/spawn_entity` service unavailable after 30+ seconds
- `/odom` topic showed "Publisher count: 0"
- Gazebo hung at "Init world[default]" in logs

### Cause

Gazebo was attempting to download models from the internet model database (`http://models.gazebosim.org/`) and hanging indefinitely when:
1. The network request timed out or stalled
2. Even though `GAZEBO_MODEL_PATH` was set, Gazebo prioritized internet downloads
3. The spawn_entity.py script would timeout waiting for the `/spawn_entity` service

**Evidence from Gazebo logs**:
```
[Wrn] [ModelDatabase.cc:340] Getting models from[http://models.gazebosim.org/]. This may take a few seconds.
[Msg] Loading world file [/opt/ros/humble/share/nav2_bringup/worlds/world_only.model]
Init world[default]
<hung here indefinitely>
```

### Solution

Disabled the Gazebo model database by setting `GAZEBO_MODEL_DATABASE_URI` to empty in docker-compose.yaml:

```yaml
environment:
  - GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models
  - GAZEBO_MODEL_DATABASE_URI=
```

This forces Gazebo to use only local models from `GAZEBO_MODEL_PATH`.

### Additional Change: Switch to Software Rendering

Changed the default GPU type from `nvidia` to `none` (software rendering) because:
- NVIDIA GPU headless mode had persistent issues with ROS plugin initialization
- Software rendering works reliably in Docker headless environments
- The spawn_entity service becomes available quickly with software rendering

**Modified files**:
1. `run_waypoint_navigator_docker.sh` - Changed default `GPU_TYPE="none"`
2. `docker-compose.waypoint-navigator.yaml` - Added `GAZEBO_MODEL_DATABASE_URI=` to all sim services

### Result

After applying the fix:
- Gazebo initializes in ~1 second
- Robot spawns successfully: "SpawnEntity: Successfully spawned entity [turtlebot3_waffle]"
- Odometry published: `/odom` shows "Publisher count: 1"
- AMCL localizes successfully
- Waypoint navigation starts and robot begins moving

```bash
$ docker logs tb3-sim-headless-nogpu | grep spawn
[INFO] [spawn_entity.py-2]: Spawn status: SpawnEntity: Successfully spawned entity [turtlebot3_waffle]
[INFO] [spawn_entity.py-2]: process has finished cleanly [pid 58]

$ docker logs tb3-waypoint-navigator | tail -5
[INFO] [turtlebot3_waypoint_navigator_nav2]: AMCL converged successfully! Robot localized at (-1.99, -0.47)
[INFO] [turtlebot3_waypoint_navigator_nav2]: Starting navigation to first waypoint...
[INFO] [turtlebot3_waypoint_navigator_nav2]: [Loop 0] Navigating to Start West (-2.20, -0.50)
[INFO] [turtlebot3_waypoint_navigator_nav2]: Goal accepted by the action server
```

## Additional Notes

- The Docker volumes are named `docker_workspace_build` and `docker_workspace_install`
- To completely reset the build, remove these volumes: `docker volume rm docker_workspace_build docker_workspace_install`
- The source code directory remains mounted from the host for convenient development
- Use `USER_ID` environment variable (not `UID`) to override the container user ID if needed
- Default GPU mode is now `none` (software rendering) for reliable headless operation
- Use `--gpu nvidia` flag if you need GPU acceleration and have proper nvidia-docker2 setup
