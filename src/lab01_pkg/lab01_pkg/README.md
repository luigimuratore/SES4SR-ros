# Lab01

[01_ROS 2 Basic Concepts.pdf](Lab01%2029008a578a8080ecaf51d081d2d6123b/01_ROS_2_Basic_Concepts.pdf)

---

## Task 1 - package

```python
ros2 pkg create --build-type ament_python --license Apache-2.0 lab01_pkg
```

---

## Task 2 - controller

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10) # Publisher for velocity commands
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz
        self.phase = 0
        self.N = 1
        self.count = 0

    def timer_callback(self):
        msg = Twist()
        # Determine direction based on phase
        if self.phase == 0:
            msg.linear.x = 1.0
            msg.linear.y = 0.0
            direction = "X-axis"
        elif self.phase == 1:
            msg.linear.x = 0.0
            msg.linear.y = 1.0
            direction = "Y-axis"
        elif self.phase == 2:
            msg.linear.x = -1.0
            msg.linear.y = 0.0
            direction = "opposite X-axis"
        elif self.phase == 3:
            msg.linear.x = 0.0
            msg.linear.y = -1.0
            direction = "opposite Y-axis"

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published: linear.x={msg.linear.x}, linear.y={msg.linear.y}, phase={self.phase+1} ({direction}), N={self.N}'
        )

        self.count += 1
        if self.count >= self.N:
            self.count = 0
            self.phase += 1
            if self.phase > 3:
                self.phase = 0
                self.N += 1

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- rqt_console

```jsx
ros2 run rqt_console rqt_console
```

![image.png](Lab01%2029008a578a8080ecaf51d081d2d6123b/image.png)

- rqt_graph

```python
ros2 run rqt_graph rqt_graph
```

![image.png](Lab01%2029008a578a8080ecaf51d081d2d6123b/image%201.png)

- Terminal outcome

![Screenshot 2025-10-18 at 11.58.29.png](Lab01%2029008a578a8080ecaf51d081d2d6123b/Screenshot_2025-10-18_at_11.58.29.png)

---

## Task 3 - localization

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose 
# NON abbiamo bisogno di importare tf_transformations

class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        
        # Initial position 
        self.x = 0.0
        self.y = 0.0
        
        # Sampling period (Delta T) in seconds: 1 s
        self.DT = 1.0
        
        self.publisher_pose = self.create_publisher(Pose, '/pose', 10) # Publisher for the pose
        self.subscription = self.create_subscription(Twist,'/cmd_vel', self.listener_callback,10) # Subscriber for velocity commands

        self.get_logger().info("Localization node started. Initial position: (0.0, 0.0)")
        

    def listener_callback(self, msg: Twist):
        # linear velocity
        v_x = msg.linear.x
        v_y = msg.linear.y
        
        # new position = old position + speed * DT
        self.x += v_x * self.DT
        self.y += v_y * self.DT
        
        pose_msg = Pose() # publish the pose
        
        # position
        pose_msg.position.x = self.x
        pose_msg.position.y = self.y
        pose_msg.position.z = 0.0

        # no rotation -> (x=0, y=0, z=0, w=1)
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        pose_msg.orientation.w = 1.0 # real part

        self.publisher_pose.publish(pose_msg)

        self.get_logger().info(
            f'Received: vx={v_x:.1f}, vy={v_y:.1f}. '
            f'New Pose published: X={self.x:.2f}, Y={self.y:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)

    localization_node = Localization()

    rclpy.spin(localization_node)

    localization_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

terminal outcome

![Screenshot 2025-10-18 at 11.58.40.png](Lab01%2029008a578a8080ecaf51d081d2d6123b/Screenshot_2025-10-18_at_11.58.40.png)

---

## Task 4 - plotjuggler

Install plotjuggler

```python
sudo apt install ros-humble-plotjuggler-ros
ros2 run plotjuggler plotjuggler

#to set white bg
export LIBGL_ALWAYS_SOFTWARE=1
```

Plot

![image.png](Lab01%2029008a578a8080ecaf51d081d2d6123b/image%202.png)

---

## Task 5 - reset

```python
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool 

class Reset_Node(Node):

    def __init__(self):
        super().__init__('reset_node')
        self.subscription = self.create_subscription(Pose,'/pose',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

        self.publisher_reset = self.create_publisher(Bool, '/reset', 10)
        self.last_reset_state = None  # Track previous state

    def listener_callback(self, msg: Pose):
        current_x = msg.position.x
        current_y = msg.position.y
        distance = math.sqrt(current_x**2 + current_y**2)
        is_far = distance > 6.0 #When the distance from the origin of the reference frame is larger than 6.0 m
        
        # Only publish when state changes
        if is_far != self.last_reset_state:
            reset_msg = Bool()
            reset_msg.data = is_far
            self.publisher_reset.publish(reset_msg)
            self.get_logger().warn(f'Reset signal: {is_far}, Distance: {distance:.2f} m')
            self.last_reset_state = is_far
        else:
            self.get_logger().debug(f'Distance: {distance:.2f} m')

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Reset_Node()

    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

terminal outcome

![image.png](Lab01%2029008a578a8080ecaf51d081d2d6123b/image%203.png)

---

## Task 6 - controller_reset and localization_reset

- Controller_reset

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class ControllerReset(Node):
    def __init__(self):
        super().__init__('controller_reset')
        
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10) # Publisher for velocity commands
        self.subscription_reset = self.create_subscription(Bool,'/reset',self.reset_callback,10) # Subscription for reset signal
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Control variables 
        self.phase = 0  # 0: +X, 1: +Y, 2: -X, 3: -Y
        self.N = 1
        self.count = 0
        
        self.get_logger().info('Controller Reset Node initialized with N=1, phase=0')

    def reset_callback(self, msg: Bool): #Reset controller when receiving True on /reset topic
        if msg.data:
            self.get_logger().warn('🔄 RESET RECEIVED - Restarting control sequence with N=1, phase=0')
            
            # Stop the robot first
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.linear.y = 0.0
            self.publisher_.publish(stop_msg)
            
            # Reset control variables
            self.phase = 0
            self.N = 1
            self.count = 0

    def timer_callback(self):
        msg = Twist()
        
        # Determine direction based on phase 
        if self.phase == 0:
            msg.linear.x = 1.0
            msg.linear.y = 0.0
            direction = "X-axis"
        elif self.phase == 1:
            msg.linear.x = 0.0
            msg.linear.y = 1.0
            direction = "Y-axis"
        elif self.phase == 2:
            msg.linear.x = -1.0
            msg.linear.y = 0.0
            direction = "opposite X-axis"
        elif self.phase == 3:
            msg.linear.x = 0.0
            msg.linear.y = -1.0
            direction = "opposite Y-axis"

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published: linear.x={msg.linear.x}, linear.y={msg.linear.y}, '
            f'phase={self.phase+1} ({direction}), N={self.N}'
        )

        self.count += 1
        if self.count >= self.N:
            self.count = 0
            self.phase += 1
            if self.phase > 3:
                self.phase = 0
                self.N += 1

def main(args=None):
    rclpy.init(args=args)
    node = ControllerReset()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

![image.png](Lab01%2029008a578a8080ecaf51d081d2d6123b/image%204.png)

- Localization_reset

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool

class LocalizationReset(Node):
    def __init__(self):
        super().__init__('localization_reset')
        
        # Initial position 
        self.x = 0.0
        self.y = 0.0
        
        # Sampling period (Delta T) in seconds: 1 s
        self.DT = 1.0
        
        
        self.publisher_pose = self.create_publisher(Pose, '/pose', 10) # Publisher for position
        self.subscription_cmd = self.create_subscription(Twist,'/cmd_vel',self.listener_callback,10) # Subscription for velocity
        self.subscription_reset = self.create_subscription(Bool,'/reset',self.reset_callback,10) # Subscription for reset signal
        
        self.get_logger().info("Localization Reset node started. Initial position: (0.0, 0.0)")

    def reset_callback(self, msg: Bool): #Reset position to origin when receiving True on /reset topic
        if msg.data:
            self.get_logger().warn('🔄 RESET RECEIVED - Position reset to origin (0.0, 0.0)')
            self.x = 0.0
            self.y = 0.0

    def listener_callback(self, msg: Twist):
        # linear velocity
        v_x = msg.linear.x
        v_y = msg.linear.y
        
        # new position = old position + speed * DT
        self.x += v_x * self.DT
        self.y += v_y * self.DT
        
        pose_msg = Pose() # publish the pose
        
        # position
        pose_msg.position.x = self.x
        pose_msg.position.y = self.y
        pose_msg.position.z = 0.0

        # no rotation -> (x=0, y=0, z=0, w=1)
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        pose_msg.orientation.w = 1.0 # real part

        self.publisher_pose.publish(pose_msg)

        # Log with logger
        self.get_logger().info(
            f'Received: vx={v_x:.1f}, vy={v_y:.1f}. '
            f'New Pose published: X={self.x:.2f}, Y={self.y:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    localization_node = LocalizationReset()
    rclpy.spin(localization_node)
    localization_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

![image.png](Lab01%2029008a578a8080ecaf51d081d2d6123b/image%205.png)

---

## Task 7 - plotjuggler2

- plotjupiter

![image.png](Lab01%2029008a578a8080ecaf51d081d2d6123b/image%206.png)

- rqt_graph

![image.png](Lab01%2029008a578a8080ecaf51d081d2d6123b/image%207.png)

---

---

---

# Appendix

## Setup.py

```python
from setuptools import find_packages, setup

package_name = 'lab01_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='gigiomuratore@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = lab01_pkg.controller:main',
            'localization = lab01_pkg.localization:main',
            'reset_node = lab01_pkg.reset_node:main',
            'controller_reset = lab01_pkg.controller_reset:main',
            'localization_reset = lab01_pkg.localization_reset:main',
        ],
    },
)

```

## Package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>lab01_pkg</name>
  <version>0.0.0</version>
  <description>SES4R</description>
  <maintainer email="gigiomuratore@gmail.com">ubuntu</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>math</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

```