
# Description
Collection of packages, nodes and algorithms to perform perception tasks using LiDAR and cameras.

# Installation
1. Install dependencies with rosdep
```bash
rosdep install --from-path src --ignore-src -y -r
```
2. Build the package using colcon 

## Optional Steps
**If using Oak D Pro camera**, install udev rules with the script [./turtlebot3_perception/debian/udev/install_udev](./turtlebot3_perception/debian/udev/install_udev)
```bash 
sudo ./install_udev
```
# Usage
## Drivers
### Start Camera Driver

1. Set the environment variable CAMERA_MODEL to "realsense" or "oakd"
2. Launch the camera driver with 
```bash 
ros2 launch turtlebot3_perception camera.launch.py
```
1. Launch the AprilTag detection using
```bash 
ros2 launch turtlebot3_perception apriltag.launch.py
```
## Nodes for Features Extraction
- `detection2landmark`: a node to transform AprilTag detections in landmarks with range and bearing. See [Start AprilTag landmark detection](#start-apriltag-landmark-detection)
- `laserscan2lines`: a node to extract lines from from LaserScan messages. See [Lines from LaserScan](#lines-from-laserscan)
- `landmark_simulator`: a node that simulates the detection of landmarks from a list of their coordinates. See [Landmarks Simulator](#landmarks-simulator)

### Lines from LaserScan
Run the example node with
```bash 
ros2 run turtlebot3_perception laserscan2lines
```

You can use the function directly in your nodes putting this import in your file
```python
from turtlebot3_perception.laserscan2lines import laserscan2lines
```

### Start AprilTag landmark detection

Launch the required nodes with
```bash 
ros2 launch turtlebot3_perception apriltag.launch.py
```

### Landmarks Simulator
This node simulate the detection of landmarks from a list of their coordinates. It provides range and bearing for each detected landmark and it also publishes TF to visualize in Rviz.

**Example run**
```bash 
ros2 run turtlebot3_perception landmark_simulator --ros-args -p use_sim_time:=true
```

**Subscriber**: `"ground_truth", nav_msgs/msg/Odometry`

**Publisher**: `"landmarks", landmark_msgs/msg/LandmarkArray`

**Static parameters**:
- `landmarks_file`: absolute path to landmarks file. An example file is provided in [landmarks.yaml](./turtlebot3_perception/config/landmarks.yaml)
- `frequency_hz`: the frequency at which landmarks are published

**Dynamically configurable parameters**:
- `field_of_view_deg`
- `max_range`, `min_range`
- `range_stddev`
- `bearing_stddev_deg`


