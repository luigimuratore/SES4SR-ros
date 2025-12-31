# Lab 05 - Dynamic Target Tracking with DWA

## Overview

This lab implements a dynamic target tracking system using the Dynamic Window Approach (DWA) algorithm. The robot tracks a moving target while avoiding obstacles, computing real-time metrics for tracking performance, distance errors, bearing errors, and obstacle proximity.

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Key Features](#key-features)
3. [Implementation Details](#implementation-details)
4. [Metrics & Performance](#metrics--performance)
5. [Code Snippets](#code-snippets)
6. [Demonstrations](#demonstrations)
7. [Results](#results)

---

## System Architecture

The system consists of:
- **ROS2 Node**: `task_1_metrics.py` - Main control node
- **DWA Planner**: Path planning with obstacle avoidance
- **Metrics System**: Real-time tracking performance evaluation
- **Sensors**: LiDAR for obstacle detection, Odometry for localization

### Topics

**Subscribed:**
- `/scan` - LaserScan data for obstacle detection
- `/dynamic_goal_pose` - Target robot pose (Odometry)
- `/odom` - Current robot odometry
- `/goal_pose` - Static goal poses (PoseStamped)

**Published:**
- `/cmd_vel` - Velocity commands (Twist)
- `/dwa_feedback` - Status feedback messages
- `/metrics` - Real-time performance metrics

---

## Key Features

### 1. Dynamic Target Tracking
- Tracks moving targets using computed velocity-based heading
- Maintains desired following distance (0.5m)
- Adapts to target velocity changes

### 2. Obstacle Avoidance
- Uses DWA for real-time collision-free navigation
- 18-beam downsampled LiDAR processing
- Configurable safety margins

### 3. Performance Metrics
- **Tracking Time Percentage**: Time successfully tracking vs. total time
- **Distance RMSE**: Root Mean Square Error from desired distance
- **Bearing RMSE**: Angular alignment error with target
- **Obstacle Proximity**: Average and minimum distances to obstacles

### 4. Intelligent Lost Detection
The system detects when tracking is lost based on:
- Distance exceeding threshold (2.0m)
- Bearing error exceeding 90°
- Moving away from target (distance increasing)

---

## Implementation Details

### Parameters

```python
# DWA Weights
'alpha': 0.12        # Angle to goal weight
'beta': 1.0          # Velocity weight
'gamma': 0.4         # Obstacle distance weight

# Control
'control_rate': 15.0              # Hz
'max_linear_vel': 0.21           # m/s
'max_angular_vel': 3.0           # rad/s

# Safety
'collision_radius': 0.20         # m
'collision_tolerance': 0.18      # m

# Tracking
'desired_distance': 0.5          # m
'tracking_threshold': 2.0        # m
'max_bearing_error': 90.0        # degrees

# LiDAR
'num_ranges': 18                 # Downsampled beams
'max_lidar_range': 3.5          # m
```

---

## Code Snippets

### Target Heading Computation

The system computes target heading from velocity (change in position):

```python
def goal_callback_odom(self, msg):
    self.goal_pose = msg.pose.pose
    pos = msg.pose.pose.position
    
    # Calculate target's heading from velocity (change in position)
    if self.prev_target_ground_truth is not None and self.target_ground_truth is not None:
        dx = pos.x - self.target_ground_truth[0]
        dy = pos.y - self.target_ground_truth[1]
        
        # If target moved, use velocity direction as heading
        if math.hypot(dx, dy) > 0.01:
            target_yaw = math.atan2(dy, dx)
        else:
            # Keep previous heading if stationary
            target_yaw = self.target_ground_truth[2]
    else:
        # First message, use zero heading
        target_yaw = 0.0
    
    self.prev_target_ground_truth = self.target_ground_truth
    self.target_ground_truth = np.array([pos.x, pos.y, target_yaw])
```

### Tracking Metrics Computation

```python
def compute_tracking_metrics(self):
    """Compute tracking metrics with improved lost detection"""
    if self.target_ground_truth is None or self.current_pose is None:
        return False, "NO_DATA"
    
    # Compute actual distance to target
    dx = self.target_ground_truth[0] - self.current_pose[0]
    dy = self.target_ground_truth[1] - self.current_pose[1]
    actual_distance = math.hypot(dx, dy)
    
    # Bearing: difference between your heading and target's heading
    bearing_angle = self.target_ground_truth[2] - self.current_pose[2]
    bearing_angle = math.atan2(math.sin(bearing_angle), math.cos(bearing_angle))
    
    # Check if distance is increasing (moving away)
    distance_increasing = False
    if self.prev_distance_to_target is not None:
        distance_increasing = actual_distance > self.prev_distance_to_target + 0.05
    self.prev_distance_to_target = actual_distance
    
    # Tracking criteria
    is_tracking = (
        actual_distance <= tracking_threshold and
        abs(bearing_angle) <= max_bearing_error_rad and
        not distance_increasing
    )
    
    if is_tracking:
        self.tracking_time += dt
        desired_distance = self.get_parameter('desired_distance').value
        distance_error = actual_distance - desired_distance
        self.distance_errors.append(distance_error)
        self.bearing_errors.append(bearing_angle)
```

### LiDAR Downsampling with Angle Preservation

```python
def laser_callback(self, msg):
    ranges = np.array(msg.ranges)
    # ... preprocessing ...
    
    # Pre-compute beam angles from the real scan metadata
    beam_angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

    # Downsample while preserving all beams
    filtered_ranges = []
    filtered_angles = []
    range_sectors = np.array_split(ranges, num_ranges)
    angle_sectors = np.array_split(beam_angles, num_ranges)
    
    for r_sector, a_sector in zip(range_sectors, angle_sectors):
        if len(r_sector) == 0:
            filtered_ranges.append(self.get_parameter('max_lidar_range').value)
            filtered_angles.append(0.0)
            continue
        min_idx = int(np.argmin(r_sector))
        filtered_ranges.append(r_sector[min_idx])
        filtered_angles.append(a_sector[min_idx])

    self.laser_ranges = np.array(filtered_ranges)
    self.laser_angles = np.array(filtered_angles)
```

### DWA Control Integration

```python
def control_callback(self):
    # ... safety checks ...
    
    # Convert goal_pose to [x, y]
    goal_xy = np.array([self.goal_pose.position.x, self.goal_pose.position.y])
    
    # Convert scan to obstacle coordinates
    obstacles = self.scan_to_obstacles(self.current_pose, self.laser_ranges, self.laser_angles)
    
    # Sync internal DWA robot state
    self.dwa.robot.pose = self.current_pose.copy()
    self.dwa.robot.vel = self.last_cmd.copy()
    
    # DWA: compute control
    v, w = self.dwa.compute_cmd(goal_xy, self.current_pose, obstacles)
    
    # Clamp velocities
    v = np.clip(v, -max_v, max_v)
    w = np.clip(w, -max_w, max_w)
    
    # Publish command
    cmd = Twist()
    cmd.linear.x = v
    cmd.angular.z = w
    self.cmd_pub.publish(cmd)
```

### Circular Mean for Angular Statistics

For accurate bearing error statistics:

```python
# Circular statistics for bearing errors
bearing_errors_array = np.array(self.bearing_errors)

sin_errors = np.sin(bearing_errors_array)
cos_errors = np.cos(bearing_errors_array)
mean_sin = np.mean(sin_errors)
mean_cos = np.mean(cos_errors)

mean_angle = math.atan2(mean_sin, mean_cos)

angular_diffs = bearing_errors_array - mean_angle
angular_diffs = np.arctan2(np.sin(angular_diffs), np.cos(angular_diffs))

bearing_rmse = math.sqrt(np.mean(angular_diffs**2))
bearing_rmse_deg = math.degrees(bearing_rmse)
```

---

## Demonstrations

### Video Demonstrations

#### Demonstration 1: Basic Tracking
![Demo 1](../media/IMG_7640.MOV)

Target tracking in open environment with minimal obstacles.

#### Demonstration 2: Obstacle Avoidance
![Demo 2](../media/IMG_7642.MOV)

Dynamic tracking while navigating around obstacles.

#### Demonstration 3: Complex Environment
![Demo 3](../media/IMG_7643.MOV)

Performance in cluttered environment with multiple obstacles.

#### Demonstration 4: Extended Tracking
![Demo 4](../media/IMG_7644.MOV)

Long-duration tracking showing stability and robustness.

### Screenshots

#### System Preview
![Lab Preview](../media/preview_lab.png)
*Gazebo simulation environment with TurtleBot3 tracking system*

#### Results Dashboard
![Report Preview](../media/preview_report3.png)
*Performance metrics and tracking visualization*

---

## Results

### Typical Performance Metrics

```
=== METRICS ===
Track Time: 85.3% over 120.5s
Distance RMSE: 0.089m
Bearing RMSE: 12.4°
Avg Obstacle Dist: 1.245m
Min Obstacle Dist: 0.182m
Bearing Stats: avg=-2.3°, max_dev=45.2°, samples=1807
```

### Key Observations

1. **High Tracking Percentage**: System maintains tracking >80% of time
2. **Low Distance Error**: RMSE typically <10cm from desired distance
3. **Good Angular Alignment**: Bearing errors within acceptable range
4. **Safe Navigation**: Maintains safe distances from obstacles

### Lost Tracking Scenarios

The system intelligently detects three types of tracking loss:
- `TOO_FAR`: Target distance exceeds 2.0m threshold
- `WRONG_HEADING`: Bearing error exceeds 90° (wrong direction)
- `MOVING_AWAY`: Distance increasing despite forward motion

---

## Running the Code

### Build the Package

```bash
cd ~/SES4SR-ros
colcon build --packages-select lab05_pkg
source install/setup.bash
```

### Launch the System

```bash
ros2 run lab05_pkg task_1_metrics
```

### Monitor Metrics

```bash
ros2 topic echo /metrics
```

---

## Dependencies

- ROS2 (Humble or later)
- Python 3.8+
- NumPy
- DWA controller (included in `planning_control_methods`)
- TurtleBot3 packages
- Gazebo

---

## Authors

SES4SR - Lab 05 Implementation

---

## License

See repository LICENSE file.