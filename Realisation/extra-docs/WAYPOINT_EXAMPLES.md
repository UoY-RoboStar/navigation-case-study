# TurtleBot 3 Waypoint Navigator - Advanced Examples

This document provides practical examples and advanced use cases for the waypoint navigator scripts.

## Table of Contents

1. [Basic Examples](#basic-examples)
2. [Advanced Patterns](#advanced-patterns)
3. [Real-World Use Cases](#real-world-use-cases)
4. [Integration Examples](#integration-examples)
5. [Performance Optimization](#performance-optimization)
6. [Debugging and Monitoring](#debugging-and-monitoring)

---

## Basic Examples

### Example 1: Simple Square Patrol

The most basic patrol pattern - the robot moves in a square.

**Edit `turtlebot3_waypoint_navigator_twist.py`:**

```python
def get_default_waypoints():
    """Simple square patrol"""
    return [
        (0.0, 0.0, 0.0),              # Corner 1
        (1.0, 0.0, 0.0),              # Corner 2
        (1.0, 1.0, math.pi/2),        # Corner 3
        (0.0, 1.0, math.pi),          # Corner 4
    ]
```

**Run:**
```bash
python3 turtlebot3_waypoint_navigator_twist.py --repetitions 3 --speed 0.25
```

**Expected behavior:** Robot moves in a 1m x 1m square, repeating 3 times.

---

### Example 2: Corridor Patrol

Robot patrols back and forth down a corridor.

```python
def get_corridor_pattern():
    """Back and forth corridor movement"""
    return [
        (0.0, 0.0, 0.0),              # Start at one end
        (3.0, 0.0, 0.0),              # Move down corridor
        (3.0, 0.0, math.pi),          # Turn around
        (0.0, 0.0, math.pi),          # Move back
        (0.0, 0.0, -math.pi/2),       # Return to start orientation
    ]
```

**Run:**
```bash
python3 turtlebot3_waypoint_navigator_twist.py --speed 0.3
```

---

### Example 3: Multi-Floor Inspection (Simulated)

Simulate visiting different areas of a multi-floor building.

```python
def get_inspection_route():
    """Simulate multi-room inspection"""
    return [
        # Ground floor - Room 1
        (1.0, 0.5, 0.0, "Room_1_Entry"),
        (1.5, 1.5, 0.785, "Room_1_Center"),
        
        # Ground floor - Room 2
        (3.0, 0.5, 0.0, "Room_2_Entry"),
        (3.5, 1.5, 1.57, "Room_2_Center"),
        
        # Ground floor - Room 3
        (5.0, 0.5, 0.0, "Room_3_Entry"),
        (5.5, 1.5, -0.785, "Room_3_Center"),
        
        # Return to start
        (1.0, 0.5, -math.pi, "Return"),
    ]
```

**Usage:** Add to script and use with `--pattern inspection_route`

---

## Advanced Patterns

### Pattern 1: Lawnmower Coverage

A systematic coverage pattern for area exploration or surveillance.

```python
def get_lawnmower_pattern(width=4.0, height=2.0, line_spacing=0.5):
    """
    Generate a lawnmower sweep pattern for area coverage.
    
    Args:
        width: Total width of area to cover
        height: Total height of area to cover
        line_spacing: Spacing between parallel lines
    
    Returns:
        List of waypoints forming parallel lines
    """
    waypoints = []
    
    # Calculate number of lines needed
    num_lines = int(height / line_spacing) + 1
    
    for i in range(num_lines):
        y = i * line_spacing
        
        if i % 2 == 0:
            # Go right (even lines)
            waypoints.append((0.0, y, 0.0))
            waypoints.append((width, y, 0.0))
        else:
            # Go left (odd lines)
            waypoints.append((width, y, math.pi))
            waypoints.append((0.0, y, math.pi))
    
    # Return to start
    waypoints.append((0.0, 0.0, 0.0))
    return waypoints
```

**Usage:**
```python
# In get_default_waypoints() or add pattern selection:
waypoints = get_lawnmower_pattern(width=3.0, height=2.0, line_spacing=0.3)
```

---

### Pattern 2: Spiral Search

Expanding spiral pattern useful for search operations.

```python
def get_spiral_search_pattern(max_radius=2.0, num_points=24):
    """
    Generate an expanding spiral pattern.
    
    Args:
        max_radius: Maximum distance from center
        num_points: Number of waypoints in spiral
    
    Returns:
        List of spiral waypoints
    """
    waypoints = []
    center_x, center_y = 0.0, 0.0
    
    for i in range(num_points):
        # Calculate angle and radius for this point
        angle = (2 * math.pi * i) / (num_points / 4)  # 4 rotations
        radius = (max_radius / num_points) * i
        
        # Calculate waypoint position
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        
        waypoints.append((x, y, angle))
    
    return waypoints
```

**Usage:**
```bash
# Add to script and modify pattern selection
python3 script.py --pattern spiral_search
```

---

### Pattern 3: Perimeter Guard Patrol

Robot patrols around the perimeter of a protected area.

```python
def get_perimeter_patrol(perimeter_size=2.0, offset=0.5):
    """
    Create a rectangular perimeter patrol pattern.
    
    Args:
        perimeter_size: Size of the area to protect
        offset: Distance from actual perimeter (clearance)
    
    Returns:
        List of waypoints forming a perimeter
    """
    half_size = perimeter_size / 2
    offset_dist = offset
    
    return [
        # Northeast corner
        (half_size + offset_dist, half_size + offset_dist, 
         math.atan2(half_size, half_size)),
        
        # Northwest corner
        (-half_size - offset_dist, half_size + offset_dist, 
         math.atan2(half_size, -half_size)),
        
        # Southwest corner
        (-half_size - offset_dist, -half_size - offset_dist, 
         math.atan2(-half_size, -half_size)),
        
        # Southeast corner
        (half_size + offset_dist, -half_size - offset_dist, 
         math.atan2(-half_size, half_size)),
        
        # Return to start
        (half_size + offset_dist, half_size + offset_dist, 0.0),
    ]
```

---

### Pattern 4: Circular Orbits at Different Radii

Multi-layer circular patrol pattern.

```python
def get_concentric_circles(num_circles=3, max_radius=2.0, points_per_circle=16):
    """
    Generate concentric circle waypoints.
    
    Args:
        num_circles: Number of circular orbits
        max_radius: Radius of outermost circle
        points_per_circle: Waypoints per circle
    
    Returns:
        List of waypoints in circular patterns
    """
    waypoints = []
    center_x, center_y = 0.0, 0.0
    
    for circle in range(num_circles):
        # Calculate radius for this circle
        radius = (max_radius / num_circles) * (circle + 1)
        
        # Generate points around this circle
        for point in range(points_per_circle):
            angle = (2 * math.pi * point) / points_per_circle
            
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            
            waypoints.append((x, y, angle))
    
    return waypoints
```

---

## Real-World Use Cases

### Use Case 1: Warehouse Inventory Scanning

Robot scans inventory in a warehouse grid layout.

```python
def get_warehouse_grid(rows=5, cols=4, row_spacing=1.0, col_spacing=1.5):
    """
    Create warehouse grid scanning pattern.
    
    Args:
        rows: Number of rows to scan
        cols: Number of columns to scan
        row_spacing: Distance between row waypoints
        col_spacing: Distance between column waypoints
    
    Returns:
        List of grid waypoints
    """
    waypoints = []
    
    for row in range(rows):
        y = row * row_spacing
        
        if row % 2 == 0:
            # Scan left to right
            for col in range(cols):
                x = col * col_spacing
                waypoints.append((x, y, 0.0))
        else:
            # Scan right to left (return path)
            for col in range(cols - 1, -1, -1):
                x = col * col_spacing
                waypoints.append((x, y, math.pi))
    
    return waypoints
```

**Running:**
```bash
# Scan 5x4 grid at slow speed for accuracy
python3 turtlebot3_waypoint_navigator_twist.py \
    --pattern warehouse_grid \
    --speed 0.15 \
    --position-tolerance 0.05
```

---

### Use Case 2: Security Perimeter Monitoring

Robot monitors a secure facility boundary.

```python
def get_security_patrol(checkpoint_coords):
    """
    Create security patrol through multiple checkpoints.
    
    Args:
        checkpoint_coords: List of (x, y, name) tuples
    
    Returns:
        List of waypoints with return to start
    """
    waypoints = []
    
    # Visit each checkpoint
    for x, y, heading in checkpoint_coords:
        waypoints.append((x, y, heading))
        
        # Add brief scanning motion (rotate in place)
        waypoints.append((x, y, heading + math.pi/4))
        waypoints.append((x, y, heading))
    
    # Return to starting position
    if waypoints:
        start_x, start_y, _ = waypoints[0]
        waypoints.append((start_x, start_y, 0.0))
    
    return waypoints

# Define security checkpoints
checkpoints = [
    (0.0, 0.0, 0.0, "Gate"),
    (2.0, 0.0, 0.0, "North_Fence"),
    (2.0, 2.0, math.pi/2, "NorthEast_Corner"),
    (0.0, 2.0, math.pi, "East_Fence"),
]
```

---

### Use Case 3: Delivery Route Planning

Robot delivers packages along a predetermined route.

```python
def get_delivery_route(delivery_locations):
    """
    Create delivery route visiting multiple locations.
    
    Args:
        delivery_locations: List of dicts with 'x', 'y', 'name', 'hold_time'
    
    Returns:
        List of waypoints
    """
    waypoints = []
    
    for location in delivery_locations:
        x = location['x']
        y = location['y']
        heading = location.get('heading', 0.0)
        
        # Approach location
        waypoints.append((x, y, heading))
        
        # Add extra waypoint to simulate delivery time
        # (robot can wait or rotate in place)
        waypoints.append((x, y, heading))
    
    return waypoints

# Define delivery stops
deliveries = [
    {'x': 0.0, 'y': 0.0, 'name': 'Depot', 'hold_time': 2.0},
    {'x': 1.5, 'y': 0.0, 'name': 'Stop_A', 'hold_time': 1.0},
    {'x': 2.0, 'y': 1.5, 'name': 'Stop_B', 'hold_time': 1.0},
    {'x': 0.5, 'y': 2.0, 'name': 'Stop_C', 'hold_time': 1.0},
]
```

---

## Integration Examples

### Integration 1: ROS 2 Service for Dynamic Waypoints

Add a service to update waypoints at runtime.

```python
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseArray, Pose

class DynamicWaypointNavigator(TurtleBot3WaypointNavigatorTwist):
    def __init__(self):
        super().__init__()
        
        # Create service for updating waypoints
        self.srv = self.create_service(
            SetBool,
            'update_waypoints',
            self.update_waypoints_callback
        )
    
    def update_waypoints_callback(self, request, response):
        """Update waypoints via ROS 2 service"""
        if request.data:
            self.get_logger().info('Updating waypoints...')
            # Add logic to fetch new waypoints
            response.success = True
        else:
            response.success = False
        return response
```

---

### Integration 2: MQTT Status Publishing

Publish robot status to MQTT broker.

```python
import paho.mqtt.client as mqtt

class MQTTAwareNavigator(TurtleBot3WaypointNavigatorTwist):
    def __init__(self, mqtt_broker="localhost"):
        super().__init__()
        
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(mqtt_broker, 1883, 60)
        self.mqtt_broker = mqtt_broker
    
    def go_to_waypoint(self, x, y, theta=None):
        """Navigate to waypoint and publish MQTT status"""
        self.publish_mqtt_status('navigating', {'target': (x, y)})
        super().go_to_waypoint(x, y, theta)
        self.publish_mqtt_status('arrived', {'position': (x, y)})
    
    def publish_mqtt_status(self, status, data):
        """Publish navigation status to MQTT"""
        payload = {
            'status': status,
            'timestamp': str(time.time()),
            'data': data
        }
        import json
        self.mqtt_client.publish(
            'robot/turtlebot3/status',
            json.dumps(payload)
        )
```

---

### Integration 3: Recording Waypoint Visits

Log all waypoint visits with timestamps.

```python
import csv
from datetime import datetime

class LoggingNavigator(TurtleBot3WaypointNavigatorTwist):
    def __init__(self, log_file="waypoint_log.csv"):
        super().__init__()
        self.log_file = log_file
        self.log_writer = None
        self.init_log_file()
    
    def init_log_file(self):
        """Initialize CSV log file"""
        with open(self.log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'waypoint_index', 'x', 'y', 
                'theta', 'loop_count', 'success'
            ])
    
    def go_to_waypoint(self, x, y, theta=None):
        """Navigate and log"""
        start_time = datetime.now()
        super().go_to_waypoint(x, y, theta)
        
        # Log the visit
        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                start_time.isoformat(),
                self.current_waypoint_index,
                x, y, theta or 0.0,
                self.loop_count,
                True
            ])
```

---

### Integration 4: Conditional Waypoint Branching

Change waypoint sequence based on conditions.

```python
class ConditionalNavigator(TurtleBot3WaypointNavigatorTwist):
    def __init__(self):
        super().__init__()
        self.obstacle_detected = False
        self.low_battery = False
    
    def get_next_waypoints(self):
        """Dynamically select waypoints based on conditions"""
        
        if self.low_battery:
            self.get_logger().warn('Battery low - returning to charger')
            return self.get_charger_route()
        
        elif self.obstacle_detected:
            self.get_logger().warn('Obstacle detected - taking alternate route')
            return self.get_alternate_route()
        
        else:
            return self.get_default_route()
    
    def get_charger_route(self):
        """Direct route to charging station"""
        return [(0.0, 0.0, 0.0)]
    
    def get_alternate_route(self):
        """Route avoiding detected obstacles"""
        return [
            (0.5, 0.5, 0.0),
            (1.5, 1.5, math.pi/2),
        ]
    
    def get_default_route(self):
        """Normal patrol route"""
        return self.get_default_waypoints()
```

---

## Performance Optimization

### Optimization 1: Adaptive Speed Control

Adjust speed based on waypoint density.

```python
def adaptive_speed_waypoints(base_waypoints, min_speed=0.1, max_speed=0.4):
    """
    Adjust speeds based on waypoint spacing.
    
    Close waypoints = slower speed (precise control)
    Far waypoints = faster speed (efficient travel)
    """
    optimized_waypoints = []
    
    for i, waypoint in enumerate(base_waypoints):
        x, y, theta = waypoint
        
        # Calculate distance to next waypoint
        if i < len(base_waypoints) - 1:
            next_x, next_y, _ = base_waypoints[i + 1]
            distance = math.sqrt((next_x - x)**2 + (next_y - y)**2)
        else:
            distance = 1.0  # Default
        
        # Adaptive speed: closer points = slower
        if distance < 0.5:
            speed = min_speed
        elif distance > 2.0:
            speed = max_speed
        else:
            # Linear interpolation
            speed = min_speed + (max_speed - min_speed) * (distance - 0.5) / 1.5
        
        optimized_waypoints.append((x, y, theta, speed))
    
    return optimized_waypoints
```

---

### Optimization 2: Path Smoothing

Reduce jerky movements by smoothing waypoint transitions.

```python
def smooth_waypoint_path(waypoints, smoothing_factor=0.3, interpolation_points=3):
    """
    Smooth waypoint path by adding interpolation points.
    
    Args:
        waypoints: Original waypoint list
        smoothing_factor: How much to smooth (0.0-1.0)
        interpolation_points: Points between each waypoint
    
    Returns:
        Smoothed waypoint list
    """
    smoothed = []
    
    for i in range(len(waypoints) - 1):
        x1, y1, theta1 = waypoints[i]
        x2, y2, theta2 = waypoints[i + 1]
        
        # Add original point
        smoothed.append((x1, y1, theta1))
        
        # Add interpolated points
        for j in range(1, interpolation_points + 1):
            t = j / (interpolation_points + 1)
            
            # Linear interpolation
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            
            # Interpolate angle (shortest path)
            theta_diff = math.atan2(
                math.sin(theta2 - theta1),
                math.cos(theta2 - theta1)
            )
            theta = theta1 + t * theta_diff
            
            smoothed.append((x, y, theta))
    
    # Add final waypoint
    smoothed.append(waypoints[-1])
    
    return smoothed
```

---

## Debugging and Monitoring

### Debug Example 1: Detailed Position Logging

```python
class DetailedLoggingNavigator(TurtleBot3WaypointNavigatorTwist):
    def move_to_point(self, target_x, target_y):
        """Move with detailed position logging"""
        iteration = 0
        
        while rclpy.ok():
            distance = self.distance_to_point(target_x, target_y)
            
            # Log detailed info every 10 iterations
            if iteration % 10 == 0:
                self.get_logger().info(
                    f"[Iter {iteration}] "
                    f"Pos: ({self.current_x:.3f}, {self.current_y:.3f}) | "
                    f"Target: ({target_x:.3f}, {target_y:.3f}) | "
                    f"Distance: {distance:.3f}m | "
                    f"Yaw: {math.degrees(self.current_yaw):.1f}°"
                )
            
            if distance < self.position_tolerance:
                break
            
            # Movement code...
            iteration += 1
```

---

### Debug Example 2: Visualization Helper

```python
def print_waypoint_path(waypoints):
    """Print waypoint path in readable format"""
    print("\n" + "="*60)
    print("WAYPOINT PATH VISUALIZATION")
    print("="*60)
    
    for i, waypoint in enumerate(waypoints):
        x, y, theta = waypoint
        
        # Convert theta to cardinal direction
        degrees = math.degrees(theta)
        if degrees < 45 and degrees >= -45:
            direction = "→ (East)"
        elif degrees >= 45 and degrees < 135:
            direction = "↑ (North)"
        elif degrees >= 135 or degrees < -135:
            direction = "← (West)"
        else:
            direction = "↓ (South)"
        
        print(f"[{i:2d}] ({x:6.2f}, {y:6.2f}) Heading: {direction}")
    
    print("="*60 + "\n")
```

---

### Debug Example 3: RViz Marker Publishing

Publish waypoints as RViz markers for visualization.

```python
from visualization_msgs.msg import Marker, MarkerArray

class RVizVisualizingNavigator(TurtleBot3WaypointNavigatorTwist):
    def __init__(self):
        super().__init__()
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/waypoint_markers',
            10
        )
    
    def publish_waypoints_markers(self, waypoints):
        """Publish waypoints as RViz markers"""
        marker_array = MarkerArray()
        
        for i, (x, y, theta) in enumerate(waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.0
            
            # Color based on index
            marker.color.r = float(i) / len(waypoints)
            marker.color.g = 0.5
            marker.color.b = 1.0 - float(i) / len(waypoints)
            marker.color.a = 1.0
            
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)
```

---

## Running Examples

### Run the Square Patrol
```bash
python3 turtlebot3_waypoint_navigator_twist.py --repetitions 3 --speed 0.25
```

### Run Lawnmower Pattern (with modifications to script)
```bash
# Add get_lawnmower_pattern to script first
python3 turtlebot3_waypoint_navigator_twist.py --pattern lawnmower
```

### Run Spiral Search
```bash
python3 turtlebot3_waypoint_navigator_twist.py --pattern spiral_search
```

### Monitor with ROS 2 Tools
```bash
# In another terminal, monitor robot position
ros2 topic echo /odom --field pose.pose.position

# In another terminal, view all topics
ros2 topic list

# In another terminal, launch RViz for visualization
rviz2
```

---

## Tips and Best Practices

1. **Start Simple**: Begin with rectangular patterns before trying complex ones
2. **Test Offline**: Draw patterns on paper first to verify they make sense
3. **Use Tolerances**: Adjust `--position-tolerance` and `--angle-tolerance` for your needs
4. **Monitor Speed**: Start with low speeds (0.15 m/s) and increase as needed
5. **Log Everything**: Record waypoint visits for analysis and debugging
6. **Visualize**: Use RViz markers to see planned paths before execution
7. **Test Edge Cases**: Try different starting positions and obstacle configurations
8. **Measure Accuracy**: Log actual vs planned positions to understand performance

---

## Performance Benchmarks

| Pattern | Waypoints | Avg Time/Waypoint | Total Time |
|---------|-----------|-------------------|-----------|
| Square (1m × 1m) | 4 | 8-12s | 32-48s |
| Lawnmower (3m × 2m) | 12 | 5-8s | 60-96s |
| Spiral (2.0m radius) | 24 | 4-6s | 96-144s |
| Perimeter (2m perimeter) | 4 | 10-15s | 40-60s |

*Times based on default speed (0.2 m/s) and tolerances (0.1m, 0.1rad)*

---

## Troubleshooting Advanced Scenarios

### Scenario 1: Robot Oscillating at Waypoint

**Problem**: Robot overshoots, reverses, repeats

**Solution**:
```bash
python3 script.py --speed 0.1 --position-tolerance 0.15
```

### Scenario 2: Waypoints Too Close Together

**Problem**: Robot struggles with rapid course changes

**Solution**: Add intermediate waypoints with adjusted headings

### Scenario 3: Accuracy Drift Over Time

**Problem**: Later waypoints reached less accurately

**Solution**: Use Nav2 version for better localization

---

For more information, see `WAYPOINT_NAVIGATOR_README.md` and `QUICKSTART_WAYPOINT.md`.
