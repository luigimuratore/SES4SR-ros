## Lab 05 - Dynamic Window Approach DWA

### Overview

This lab implements a dynamic window approach DWA system to track a moving target while avoiding obstacles, computing real-time metrics for tracking performance, distance errors, bearing errors, and obstacle proximity.

---

### System Architecture

The system consists of:
- **ROS2 Node**: Main control node
- **DWA Planner**: Path planning with obstacle avoidance with 
- **Metrics System**: Real-time tracking performance evaluation
- **Sensors**: LiDAR for obstacle detection, Odometry for localization

---
### Topics

**Subscriber:**
- `/scan` - LaserScan data for obstacle detection
- `/dynamic_goal_pose` - Target pose updated (Simulation)
- `/camera/landmarks` - Target pose updated (Real-World)
- `/odom` - Current robot odometry

**Publisher:**
- `/cmd_vel` - Velocity commands
- `/dwa_feedback` - Feedback about robot phase and current distance to goal
- `/metrics` - Real-time performance metrics

---

### DWA Implementation

The Dynamic Window Approach uses a cost function to evaluate potential trajectories:

**Standard DWA Cost Function:**

$$J = \alpha \cdot heading + \beta \cdot vel + \gamma \cdot dist$$

Where:
- $\alpha$ = heading term weight (0.12)
- $\beta$ = velocity term weight (1.0)
- $\gamma$ = obstacle distance term weight (0.4)

**Extended DWA with Target Tracking:**

$$J = \alpha \cdot heading + \beta \cdot vel' + \gamma \cdot dist_{obst} + \delta \cdot dist_{target}$$

Where:
- $vel'$ = velocity reduction term when approaching target
- $dist_{obst}$ = distance to nearest obstacle
- $dist_{target}$ = distance to target (used for velocity modulation)
- $\delta$ = target distance weight (0.5)

The velocity reduction component ensures the robot slows down as it approaches the target, preventing overshoot and maintaining stable tracking at the desired following distance.

---


## Parameters

```python
'alpha': 0.12        # Heading term
'beta': 1.0          # Velocity term
'gamma': 0.4         # Obstacle term

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
```

---

## Demonstrations

### Video Demonstrations

#### Demonstration 1

https://github.com/user-attachments/assets/4e507982-ced5-4998-9c22-597da9e6cfda

*Scrivere qualcosa.*

#### Demonstration 2

https://github.com/user-attachments/assets/47603892-ea69-48b0-9695-75380d90791e

*Scrivere qualcosa.*


#### Demonstration 3

https://github.com/user-attachments/assets/feff8794-6225-4bff-a3a4-d4e3eea646cb

*Scrivere qualcosa.*

#### Demonstration 4

https://github.com/user-attachments/assets/9846eefc-f319-4134-98a5-dbd0dcc499c2

*Scrivere qualcosa.*

---

## Results

### Typical Bad Performance Metrics

```
=== METRICS ===
Track Time: 85.3% over 120.5s
Distance RMSE: 0.089m
Bearing RMSE: 12.4°
Avg Obstacle Dist: 1.245m
Min Obstacle Dist: 0.182m
Bearing Stats: avg=-2.3°, max_dev=45.2°, samples=1807
```

### Typical Good Performance Metrics

```
=== METRICS ===
Track Time: 100% over 120.5s
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

The system detects three types of tracking loss:
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


### Key Features

#### 1. Dynamic Target Tracking
#### 2. Obstacle Avoidance
#### 3. Performance Metrics
- **Tracking Time Percentage**: Time successfully tracking vs. total time
- **Distance RMSE**: Root Mean Square Error from desired distance
- **Bearing RMSE**: Angular alignment error with target
- **Obstacle Proximity**: Average and minimum distances to obstacles

#### 4. Lost Detection
The system detects when tracking is lost based on:
- Distance exceeding threshold (2.0m)
- Bearing error exceeding 90°
- Moving away from target (distance increasing)
