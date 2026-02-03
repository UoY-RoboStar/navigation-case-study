# Docker Container Communication Fix

## Quick Fix Summary

**TL;DR**: The waypoint navigator couldn't communicate with Gazebo in a separate container. Fixed by:

1. Adding `FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fastdds_no_shared_memory.xml` to force UDP transport
2. Adding `ipc: host` and `/dev/shm` mount to the docker-compose configuration
3. Replacing `time.sleep()` with `rclpy.spin_once()` in the Python code to process ROS 2 callbacks

**Files Changed**:
- `docker/docker-compose.waypoint-navigator-simple.yaml` - Added IPC, shared memory, and FastDDS config
- `turtlebotrossim/src/turtlebot3_waypoint_navigator/turtlebot3_waypoint_navigator/navigator_twist.py` - Fixed callback processing

**Test**: Run `./docker/run-waypoint-navigator.sh --speed 0.15 --position-tolerance 0.05 --angle-tolerance 0.05 --repetitions 1` and the robot should now move!

---

## Problem

The waypoint navigator script (`./docker/run-waypoint-navigator.sh`) was failing to move the robot when Gazebo was running in a separate Docker container. The navigator would start, publish velocity commands to `/cmd_vel`, but never receive odometry updates from the `/odom` topic, causing the robot to remain stationary.

## Root Cause

The issue was a **ROS 2 DDS (Data Distribution Service) communication problem** between Docker containers. Even though both containers were using `network_mode: host`, they couldn't communicate because:

1. **Shared Memory Transport**: By default, ROS 2 Humble uses FastDDS which prefers shared memory transport for inter-process communication on the same host.

2. **Container Isolation**: Even with `network_mode: host`, Docker containers have separate process namespaces. Without proper IPC (Inter-Process Communication) configuration, shared memory segments created by one container are not accessible to another.

3. **Discovery Issues**: The ROS 2 nodes could discover each other and see the topic list (via multicast discovery), but actual data transfer was failing because the default shared memory transport couldn't work across container boundaries.

## Solution

The fix involved three changes to `docker/docker-compose.waypoint-navigator-simple.yaml`:

### 1. Add IPC Host Mode

```yaml
# Use host IPC for shared memory communication
ipc: host
```

This allows containers to share the host's IPC namespace, enabling shared memory communication if needed.

### 2. Mount `/dev/shm`

```yaml
volumes:
  - /dev/shm:/dev/shm
```

This mounts the shared memory filesystem, which ROS 2 uses for efficient data transfer.

### 3. Force UDP Transport with FastDDS Profile (KEY FIX)

```yaml
environment:
  - FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fastdds_no_shared_memory.xml
```

**This was the critical fix.** The FastDDS XML profile disables shared memory transport and forces UDP multicast, which works reliably across Docker containers even with host networking. The profile contains:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles" >
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>CustomUdpTransport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>CustomUdpTransport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
```

### 4. Fix Python Code for ROS 2 Callbacks

Additionally, the waypoint navigator Python code (`navigator_twist.py`) was fixed to properly process ROS 2 callbacks. The original code used `time.sleep()` in control loops, which blocked callback processing. The fix replaced all `time.sleep()` calls with `rclpy.spin_once()` to ensure odometry callbacks are processed:

```python
# Before (blocked callbacks):
time.sleep(self.control_loop_rate)

# After (processes callbacks):
rclpy.spin_once(self, timeout_sec=self.control_loop_rate)
```

## Testing the Fix

### Verify Odometry Reception

Test if the waypoint navigator container can receive odometry:

```bash
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml run --rm waypoint_navigator \
  bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /odom --once"
```

If successful, you should see odometry data with position and orientation information.

### Run the Waypoint Navigator

```bash
./docker/run-waypoint-navigator.sh --speed 0.15 --position-tolerance 0.05 --angle-tolerance 0.05 --repetitions 1
```

The robot should now move and you should see log messages showing:
- Odometry updates: `[ODOM] Position: (x, y), Yaw: angle`
- Movement status: `[MOVE_STATUS] Distance: ...`
- Velocity commands: `[CMD_VEL] Publishing: linear=... angular=...`

## Why This Works

1. **UDP Multicast**: UDP multicast works reliably across containers with `network_mode: host` because it uses the host's network stack without process isolation issues.

2. **No Shared Memory Dependencies**: By disabling shared memory transport, we avoid all the IPC complexity and container isolation issues.

3. **Proper Callback Processing**: Using `rclpy.spin_once()` ensures the ROS 2 executor processes incoming messages (like odometry) while the control loop runs.

## Alternative Solutions (Not Recommended)

Other potential solutions that were considered but not used:

1. **Single Container**: Run both Gazebo and the navigator in the same container - eliminates communication issues but reduces modularity.

2. **ROS 2 Zenoh Bridge**: Use a different DDS implementation or bridge - adds complexity.

3. **Custom FastDDS Config**: Create a more complex FastDDS configuration - the simple UDP-only profile is sufficient.

## Implications

- **Performance**: UDP transport has slightly higher latency than shared memory, but for this application the difference is negligible.
- **Network Traffic**: All ROS 2 communication now uses UDP multicast on the host network interface.
- **Scalability**: This solution works for multiple containers communicating via ROS 2 on the same host.

## References

- [ROS 2 DDS Tuning](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html)
- [FastDDS XML Configuration](https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html)
- [ROS 2 Docker Networking](https://docs.ros.org/en/humble/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html)

## Troubleshooting

If communication still fails:

1. **Check ROS_DOMAIN_ID**: Ensure all containers use the same ROS_DOMAIN_ID (default is 0).
2. **Check ROS_LOCALHOST_ONLY**: Must be set to 0 for inter-container communication.
3. **Firewall**: Ensure the host firewall allows UDP multicast (ports 7400-7999 by default).
4. **List Topics**: Run `ros2 topic list` in both containers to verify discovery is working.
5. **Echo Topics**: Use `ros2 topic echo /odom` to verify data reception.
