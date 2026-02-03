# TurtleBot 3 Waypoint Navigator - Path Configuration

## World Geometry: turtlebot3_world

The `turtlebot3_world` environment used in the simulation has the following layout:

### Robot Spawn Position
- **Start Position**: `(-2.00, -0.50, 0.0)`
- The robot spawns on the left side of the world, west of the obstacle grid

### Obstacle Grid Layout

The world contains a **3x3 grid of cylindrical obstacles** centered around the origin:

| Position      | Obstacle Type        |
|---------------|---------------------|
| (-1.1, -1.1)  | Cylinder (r=0.15m)  |
| (-1.1,  0.0)  | Cylinder (r=0.15m)  |
| (-1.1,  1.1)  | Cylinder (r=0.15m)  |
| ( 0.0, -1.1)  | Cylinder (r=0.15m)  |
| ( 0.0,  0.0)  | Cylinder (r=0.15m)  |
| ( 0.0,  1.1)  | Cylinder (r=0.15m)  |
| ( 1.1, -1.1)  | Cylinder (r=0.15m)  |
| ( 1.1,  0.0)  | Cylinder (r=0.15m)  |
| ( 1.1,  1.1)  | Cylinder (r=0.15m)  |

### Boundary Walls
- Walls at approximately: `(-1.8, ±2.7)`, `(1.8, ±2.7)`
- Additional boundary elements at `(3.5, 0)`

### Map Information
- **Resolution**: 0.05m per pixel
- **Origin**: `(-10.0, -10.0, 0.0)`
- **Map File**: `/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.pgm`

## Waypoint Patterns

### Default Pattern (Rectangular Perimeter)

The default pattern navigates around the **outer perimeter** of the obstacle grid, staying in clear corridors:

```python
Waypoint 1: ( 0.00, -1.60)  # South escape route (get below obstacle grid)
Waypoint 2: (-1.60, -1.60)  # Southwest corner (clear corridor)
Waypoint 3: (-1.60, -2.20)  # Bottom-left corner (well below obstacles)
Waypoint 4: ( 1.60, -2.20)  # Bottom-right corner (safe southern corridor)
Waypoint 5: ( 1.60,  2.20)  # Top-right corner (safe northern corridor)
Waypoint 6: (-1.60,  2.20)  # Top-left corner (safe northern corridor)
Waypoint 7: (-1.60, -1.60)  # Return to southwest (complete loop)
```

**Path Strategy**:
1. **Escape Route**: First waypoint moves south to get below the obstacle grid row
2. **Clear Corridors**: Stays at least 0.5m away from all obstacles
3. **Large Rectangle**: Navigates around the perimeter (bottom → right → top → left)
4. **Safe Margins**: Uses ±1.6m to ±2.2m range, well outside obstacle grid (±1.1m)

### Why These Waypoints?

**Problem with Original Waypoints**:
The original waypoints `(0.5, 0.5)`, `(1.5, 0.5)`, etc. navigated **directly through the obstacle grid**, causing the robot to get stuck between cylinders.

**Solution**:
- Navigate **around** the obstacle grid, not through it
- Use clear corridors with sufficient clearance (>0.5m)
- First waypoint provides an **escape route** for robots that may have moved during testing

### Figure-Eight Pattern

Creates two circular loops using trigonometric waypoints:
- Right loop centered at `(1.0, 0.0)` with 0.5m radius
- Left loop centered at `(-1.0, 0.0)` with 0.5m radius
- 20-degree increments (18 waypoints per loop)

**Warning**: This pattern may collide with obstacles at `(±1.1, 0)` due to proximity.

### Spiral Pattern

Expands outward from the origin in a spiral:
- Starts at origin
- Each waypoint increases radius by 0.15m and rotates by 0.5 radians
- 12 waypoints total

**Warning**: This pattern **will collide** with the central obstacle at `(0, 0)`.

## Creating Custom Waypoints

### Safe Zone Guidelines

To avoid obstacles, keep waypoints in these safe zones:

**Horizontal Corridors** (east-west movement):
- **Southern corridor**: y < -1.4m (below bottom row of obstacles)
- **Northern corridor**: y > 1.4m (above top row of obstacles)

**Vertical Corridors** (north-south movement):
- **Western corridor**: x < -1.4m (left of obstacle grid)
- **Eastern corridor**: x > 1.4m (right of obstacle grid)

**Open Areas**:
- Far corners: `(±2.0, ±2.0)` and beyond

### Minimum Clearances

- **Robot radius**: ~0.178m (TurtleBot3 Waffle)
- **Obstacle radius**: 0.15m
- **Recommended clearance**: >0.3m from obstacle centers
- **Safe clearance**: >0.5m from obstacle centers

### Example: Custom Square Pattern

```python
def get_custom_square() -> List[Tuple[float, float, float]]:
    """Safe square pattern in southwestern quadrant."""
    return [
        (-2.0, -1.5, 0.0),      # Southwest starting point
        (-2.0, -2.5, -math.pi/2), # Move south
        (-0.5, -2.5, 0.0),       # Move east
        (-0.5, -1.5, math.pi/2), # Move north
        (-2.0, -1.5, math.pi),   # Return west (close square)
    ]
```

## Running with Custom Parameters

### Speed and Tolerance

```bash
# Slow and precise (tight spaces)
./docker/run-waypoint-navigator.sh --speed 0.1 --position-tolerance 0.05 --angle-tolerance 0.05

# Normal speed (recommended)
./docker/run-waypoint-navigator.sh --speed 0.2 --position-tolerance 0.15 --angle-tolerance 0.15

# Fast exploration (open areas only)
./docker/run-waypoint-navigator.sh --speed 0.4 --position-tolerance 0.2 --angle-tolerance 0.2
```

### Pattern Selection

```bash
# Default rectangular perimeter (recommended)
./docker/run-waypoint-navigator.sh --pattern default

# Figure-eight (may hit obstacles)
./docker/run-waypoint-navigator.sh --pattern figure_eight

# Spiral (will hit obstacles)
./docker/run-waypoint-navigator.sh --pattern spiral
```

### Continuous Patrol

```bash
# Run indefinitely (use Ctrl+C to stop)
./docker/run-waypoint-navigator.sh --repetitions 0

# Run 5 times
./docker/run-waypoint-navigator.sh --repetitions 5
```

## Troubleshooting

### Robot Gets Stuck

**Symptoms**: Distance to waypoint stays constant, robot oscillates in place

**Causes**:
1. Waypoint is inside or too close to an obstacle
2. Robot is trapped between obstacles
3. Position tolerance is too tight for the space

**Solutions**:
1. Increase position tolerance: `--position-tolerance 0.2`
2. Restart simulation to reset robot to spawn point
3. Use default pattern which has verified clear paths

### Robot Doesn't Move

**Symptoms**: No odometry updates, robot stationary

**Solutions**:
1. Check Docker container communication (see `DOCKER_COMMUNICATION_FIX.md`)
2. Verify Gazebo is running: `docker ps | grep simdeploynvidiatb3`
3. Check ROS topics: `ros2 topic list` should show `/cmd_vel` and `/odom`

### Waypoint Never Reached

**Symptoms**: Robot approaches but never completes waypoint

**Causes**:
1. Position tolerance too tight
2. Obstacle blocking final approach
3. Robot oscillating around target

**Solutions**:
1. Increase position tolerance: `--position-tolerance 0.15` or `0.2`
2. Move waypoint to more open area
3. Reduce speed for better control: `--speed 0.15`

## Visualization

To visualize the waypoints and robot path:

1. Open RViz (already running with the simulation)
2. The robot's path and obstacles should be visible
3. Nav2 visualizations show costmaps and planned paths

## Technical Details

### Coordinate System
- **Origin**: Center of obstacle grid
- **X-axis**: Positive = East (right)
- **Y-axis**: Positive = North (forward/up)
- **Yaw**: Counter-clockwise from East (0 = facing East)

### Waypoint Format
```python
(x, y, theta)
# x: meters from origin (East/West)
# y: meters from origin (North/South)  
# theta: target heading in radians
```

### Common Angles
- `0.0` = Facing East (→)
- `math.pi/2` = Facing North (↑)
- `math.pi` or `-math.pi` = Facing West (←)
- `-math.pi/2` = Facing South (↓)

## Performance Notes

### Typical Completion Times (default pattern)

With `--speed 0.2 --position-tolerance 0.15`:
- Single loop: ~3-4 minutes
- Per waypoint: ~20-40 seconds

### Optimization Tips

1. **Increase speed** in open areas (southern/northern corridors)
2. **Larger tolerances** reduce precision oscillation
3. **Fewer waypoints** complete faster (but less coverage)
4. **Smooth corners** by using intermediate diagonal waypoints

## References

- World file: `/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_world/`
- Map files: `/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.*`
- Navigator code: `turtlebotrossim/src/turtlebot3_waypoint_navigator/`
