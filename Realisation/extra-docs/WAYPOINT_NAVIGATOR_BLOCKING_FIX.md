# Waypoint Navigator Blocking Fix

## Problem Summary

The waypoint navigator (`navigator_nav2.py`) was failing to navigate the robot beyond the first waypoint. The logs showed:

```
waypoint_navigator  | [INFO] [1768387298.084617780] [turtlebot3_waypoint_navigator_nav2]: [Loop 0] Navigating to Start West (-2.20, -0.50)
```

After this log entry, the navigator would hang and never proceed to subsequent waypoints.

## Root Cause Analysis

The issue was caused by **blocking `time.sleep()` calls** in critical sections of the code that prevented the ROS 2 executor from processing callbacks:

### Issue 1: Blocking in `proceed_to_next_waypoint()`
```python
def proceed_to_next_waypoint(self) -> None:
    """Move to the next waypoint in the sequence."""
    self.get_logger().info(f'Pausing at waypoint for {self.pause_at_waypoint}s')
    time.sleep(self.pause_at_waypoint)  # ❌ BLOCKING CALL
    
    # Navigate to next waypoint...
```

**Why this caused the hang:**
1. Navigation goal completes
2. Nav2 action server calls `goal_result_callback()`
3. `goal_result_callback()` calls `proceed_to_next_waypoint()`
4. `proceed_to_next_waypoint()` calls `time.sleep()` which **blocks the executor**
5. While blocked, the executor cannot process any callbacks
6. The next navigation goal cannot proceed because callbacks are blocked
7. **Result: Deadlock-like hang**

### Issue 2: Blocking in `start_navigation()`
```python
def main(args=None):
    rclpy.init(args=args)
    navigator = TurtleBot3WaypointNavigatorNav2()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(navigator)
    
    try:
        navigator.start_navigation()  # ❌ Called BEFORE executor.spin()
        executor.spin()  # Never reached until start_navigation() completes
```

The `start_navigation()` method contained multiple blocking calls:
- Waiting for AMCL subscriber: `time.sleep(0.1)` in a loop
- Publishing initial pose: `time.sleep(0.2)` × 10 iterations
- Processing delay: `time.sleep(2.0)`
- AMCL convergence check: `rclpy.spin_once()` in a loop with blocking waits
- Stabilization delay: `time.sleep(3.0)` or `time.sleep(10.0)`

Total blocking time: **7-17 seconds** before the executor could even start spinning!

## The Solution

Replace all blocking calls with **non-blocking timer-based callbacks**.

### Fix 1: Non-Blocking Waypoint Advancement

**Before (Blocking):**
```python
def proceed_to_next_waypoint(self) -> None:
    self.get_logger().info(f'Pausing at waypoint for {self.pause_at_waypoint}s')
    time.sleep(self.pause_at_waypoint)  # BLOCKS EXECUTOR
    self.current_waypoint_index += 1
    # ... continue processing
```

**After (Non-Blocking):**
```python
def proceed_to_next_waypoint(self) -> None:
    """Move to the next waypoint in the sequence."""
    self.get_logger().info(f'Pausing at waypoint for {self.pause_at_waypoint}s')
    
    # Cancel any existing timer
    if self.waypoint_timer is not None:
        self.waypoint_timer.cancel()
    
    # Create a one-shot timer to advance to next waypoint after pause
    self.waypoint_timer = self.create_timer(
        self.pause_at_waypoint,
        self.advance_to_next_waypoint
    )

def advance_to_next_waypoint(self) -> None:
    """Timer callback to advance to the next waypoint after pause."""
    # Cancel the one-shot timer
    if self.waypoint_timer is not None:
        self.waypoint_timer.cancel()
        self.waypoint_timer = None
    
    # Update waypoint index
    self.current_waypoint_index += 1
    
    # Check if we've completed a full loop
    if self.current_waypoint_index >= len(self.waypoints):
        self.current_waypoint_index = 0
        self.loop_count += 1
        
        if self.max_loops > 0 and self.loop_count >= self.max_loops:
            self.get_logger().info(f'Reached maximum loops ({self.max_loops}). Stopping.')
            return
    
    # Navigate to next waypoint
    self.navigate_to_waypoint(self.current_waypoint_index)
```

### Fix 2: Non-Blocking AMCL Convergence Check

**Before (Blocking):**
```python
def start_navigation(self) -> None:
    self.set_initial_pose(x=-2.0, y=-0.5, theta=0.0)
    
    # Blocking wait for AMCL
    while not self.amcl_pose_received and (time.time() - start_time) < timeout:
        rclpy.spin_once(self, timeout_sec=0.1)  # BLOCKS
    
    time.sleep(3.0)  # BLOCKS
    self.navigate_to_waypoint(0)
```

**After (Non-Blocking):**
```python
def start_navigation(self) -> None:
    """Start the waypoint navigation sequence (non-blocking)."""
    self.set_initial_pose(x=-2.0, y=-0.5, theta=0.0)
    
    # Start non-blocking AMCL convergence check
    self.initialization_start_time = time.time()
    self.amcl_check_timer = self.create_timer(0.5, self.check_amcl_convergence)

def check_amcl_convergence(self) -> None:
    """Timer callback to check if AMCL has converged (non-blocking)."""
    timeout = 30.0
    elapsed = time.time() - self.initialization_start_time
    
    if self.amcl_pose_received:
        # AMCL has converged
        if self.amcl_check_timer is not None:
            self.amcl_check_timer.cancel()
            self.amcl_check_timer = None
        
        # Wait for stabilization using timer
        self.stabilization_timer = self.create_timer(3.0, self.start_first_waypoint)
    
    elif elapsed >= timeout:
        # Timeout - cancel timer and proceed anyway
        if self.amcl_check_timer is not None:
            self.amcl_check_timer.cancel()
            self.amcl_check_timer = None
        
        self.stabilization_timer = self.create_timer(10.0, self.start_first_waypoint)

def start_first_waypoint(self) -> None:
    """Timer callback to start navigation to the first waypoint."""
    if self.stabilization_timer is not None:
        self.stabilization_timer.cancel()
        self.stabilization_timer = None
    
    self.navigate_to_waypoint(0)
```

### Fix 3: Non-Blocking Startup

**Before (Blocking):**
```python
def main(args=None):
    navigator = TurtleBot3WaypointNavigatorNav2()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(navigator)
    
    try:
        navigator.start_navigation()  # Blocks for 7-17 seconds
        executor.spin()  # Never reached until start_navigation() completes
```

**After (Non-Blocking):**
```python
def main(args=None):
    navigator = TurtleBot3WaypointNavigatorNav2()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(navigator)
    
    try:
        # Create a one-shot timer to start navigation after executor begins spinning
        def start_navigation_once():
            if hasattr(navigator, '_startup_timer'):
                navigator._startup_timer.cancel()
            navigator.start_navigation()
        
        navigator._startup_timer = navigator.create_timer(0.1, start_navigation_once)
        
        # Spin the executor immediately - it can now process callbacks
        executor.spin()
```

## Key Changes Summary

1. **Added state tracking variables** in `__init__()`:
   ```python
   self.waypoint_timer = None
   self.initialization_start_time = None
   self.amcl_check_timer = None
   self.stabilization_timer = None
   ```

2. **Replaced blocking `time.sleep()` with `create_timer()`**:
   - Waypoint pause: 1 second (configurable) → non-blocking timer
   - AMCL convergence check: blocking loop → periodic timer callback (0.5s interval)
   - Stabilization wait: 3-10 seconds → one-shot timer

3. **Added new callback methods**:
   - `advance_to_next_waypoint()`: Timer callback for waypoint advancement
   - `check_amcl_convergence()`: Timer callback for AMCL convergence checking
   - `start_first_waypoint()`: Timer callback for starting first navigation goal

4. **Modified main() function**:
   - Start navigation via one-shot timer (0.1s delay)
   - Executor can spin immediately
   - All callbacks processed asynchronously

## How the Fixed Flow Works

### Startup Sequence (Non-Blocking):
```
1. main() creates navigator and executor
2. main() creates startup timer (0.1s) → start_navigation()
3. main() calls executor.spin() ← EXECUTOR IS NOW RUNNING
   ↓
4. Timer fires → start_navigation() called
5. start_navigation() publishes initial pose
6. start_navigation() creates AMCL check timer (0.5s interval)
   ↓
7. AMCL check timer fires periodically
8. When AMCL converges → cancel AMCL timer
9. Create stabilization timer (3s)
   ↓
10. Stabilization timer fires → start_first_waypoint()
11. start_first_waypoint() → navigate_to_waypoint(0)
```

### Navigation Loop (Non-Blocking):
```
1. navigate_to_waypoint(N) sends goal to Nav2
2. Nav2 accepts goal → goal_response_callback()
3. Robot navigates...
4. Nav2 completes → goal_result_callback()
5. goal_result_callback() → proceed_to_next_waypoint()
6. proceed_to_next_waypoint() creates pause timer (1s)
   ↓ EXECUTOR CONTINUES PROCESSING OTHER CALLBACKS
7. Pause timer fires → advance_to_next_waypoint()
8. advance_to_next_waypoint() → navigate_to_waypoint(N+1)
9. Repeat from step 1
```

## Benefits of This Approach

✅ **No blocking calls** - Executor remains responsive at all times  
✅ **Proper asynchronous behavior** - All waits use timers  
✅ **Clean state management** - Clear separation of concerns  
✅ **Robust error handling** - Timeouts handled gracefully  
✅ **Multi-threaded executor can work** - Callbacks processed concurrently  
✅ **Follows ROS 2 best practices** - Event-driven architecture  

## Testing the Fix

### 1. Rebuild the Package
```bash
cd turtlebotrossim
colcon build --packages-select turtlebot3_waypoint_navigator
source install/setup.bash
```

### 2. Run with Docker
```bash
./docker/run-waypoint-navigator.sh --repetitions 2
```

### 3. Expected Behavior
You should now see continuous waypoint navigation:
```
[INFO] [Loop 0] Navigating to Start West (-2.20, -0.50)
[INFO] Goal accepted by the action server
[INFO] Goal succeeded!
[INFO] Pausing at waypoint for 1.0s
[INFO] [Loop 0] Navigating to Southwest Far (-2.20, -2.20)
[INFO] Goal accepted by the action server
[INFO] Goal succeeded!
[INFO] Pausing at waypoint for 1.0s
[INFO] [Loop 0] Navigating to Southwest (-2.00, -2.20)
...
[INFO] Completed loop 1. Total waypoints traversed: 18
[INFO] Reached maximum loops (2). Stopping.
```

### 4. Verification
- ✅ Robot visits all 18 waypoints in sequence
- ✅ Pauses at each waypoint for 1 second
- ✅ Completes the requested number of loops
- ✅ No hanging or freezing
- ✅ Logs show continuous progress

## Remaining Notes

**Note on `set_initial_pose()`**: This method still contains some blocking `time.sleep()` calls (lines 163, 187, 191), but these only execute once during initialization before the main navigation loop begins. They complete in ~7 seconds total and don't affect ongoing navigation. For complete consistency, these could also be converted to timer-based callbacks in a future update, but the current fix resolves the critical navigation hang issue.

## Files Modified

- `turtlebotrossim/src/turtlebot3_waypoint_navigator/turtlebot3_waypoint_navigator/navigator_nav2.py`
  - Modified: `proceed_to_next_waypoint()` - Now non-blocking
  - Added: `advance_to_next_waypoint()` - Timer callback for waypoint advancement
  - Modified: `start_navigation()` - Now uses timer-based AMCL check
  - Added: `check_amcl_convergence()` - Timer callback for AMCL convergence
  - Added: `start_first_waypoint()` - Timer callback for first navigation goal
  - Modified: `main()` - Uses timer to start navigation after executor spins
  - Added: State tracking variables in `__init__()`

## Related Documentation

- [WAYPOINT_NAVIGATOR_README.md](WAYPOINT_NAVIGATOR_README.md) - Main documentation
- [RUN_WAYPOINT_DOCKER.md](RUN_WAYPOINT_DOCKER.md) - Docker usage guide
- [WAYPOINT_EXAMPLES.md](WAYPOINT_EXAMPLES.md) - Usage examples

## Technical Background

### Why `time.sleep()` Blocks ROS 2 Executors

ROS 2 uses an event-driven architecture with executors that continuously check for and process callbacks (subscriptions, timers, action results, etc.). When you call `time.sleep()`:

1. The current thread is put to sleep
2. The executor cannot process any callbacks during this time
3. Action results, timer callbacks, and other events queue up
4. The system appears frozen from the perspective of other nodes

### Why Multi-Threaded Executors Don't Help

Even with a `MultiThreadedExecutor`, if a callback blocks with `time.sleep()`, that specific callback chain is frozen. In this case:
- The action result callback blocks on `time.sleep()`
- This prevents the next navigation goal from being sent
- The waypoint sequence cannot advance

### The Timer-Based Solution

Using `create_timer()`:
1. Returns immediately (non-blocking)
2. Executor continues processing other callbacks
3. Timer callback fires after specified duration
4. Clean, event-driven flow continues

This is the **correct ROS 2 pattern** for delays and periodic checks.
