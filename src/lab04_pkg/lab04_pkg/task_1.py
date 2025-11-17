import rclpy
from rclpy.node import Node
import numpy as np
from numpy.linalg import inv
import yaml
import os
from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import Odometry
from landmark_msgs.msg import LandmarkArray
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from lab04_pkg.models.ekf import RobotEKF


class EKFLocalizationNode(Node):
    """
    ROS 2 Node for EKF-based robot localization using landmarks
    """
    
    def __init__(self):
        super().__init__('task_1')
        
        # Declare parameters
        self.declare_parameter('prediction_rate', 20.0)  # Hz
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('process_noise_v', 0.1)
        self.declare_parameter('process_noise_omega', 0.05)
        self.declare_parameter('measurement_noise_range', 0.1)
        self.declare_parameter('measurement_noise_bearing', 0.05)
        
        # Get parameters
        self.prediction_rate = self.get_parameter('prediction_rate').value
        initial_x = self.get_parameter('initial_x').value
        initial_y = self.get_parameter('initial_y').value
        initial_theta = self.get_parameter('initial_theta').value
        self.sigma_v = self.get_parameter('process_noise_v').value
        self.sigma_omega = self.get_parameter('process_noise_omega').value
        self.sigma_range = self.get_parameter('measurement_noise_range').value
        self.sigma_bearing = self.get_parameter('measurement_noise_bearing').value
        
        # Initialize EKF
        self._initialize_ekf(initial_x, initial_y, initial_theta)
        
        # Load landmarks from yaml file
        self.landmarks = self._load_landmarks()
        self.get_logger().info(f"Loaded {len(self.landmarks)} landmarks")
        
        # Store last velocity command
        self.last_v = 0.0
        self.last_omega = 0.0
        self.last_odom_time = self.get_clock().now()
        
        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.landmark_sub = self.create_subscription(
            LandmarkArray,
            '/landmarks',  # Use '/camera/landmarks' on real robot
            self.landmark_callback,
            10
        )
        
        # Create publisher
        self.ekf_pub = self.create_publisher(Odometry, '/ekf', 10)
        
        # Create timer for prediction at 20 Hz
        timer_period = 1.0 / self.prediction_rate
        self.timer = self.create_timer(timer_period, self.prediction_callback)
        
        self.get_logger().info('EKF Localization Node initialized')
        self.get_logger().info(f'Prediction rate: {self.prediction_rate} Hz')
    
    def _initialize_ekf(self, x, y, theta):
        """Initialize the EKF with motion and measurement models"""
        
        # Motion model: g(mu, u) for differential drive
        def eval_gux(mu, u, sigma_u, dt):
            x, y, theta = mu
            v, omega = u
            
            x_new = x + v * np.cos(theta) * dt
            y_new = y + v * np.sin(theta) * dt
            theta_new = theta + omega * dt
            
            # Normalize angle to [-pi, pi]
            theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))
            
            return np.array([x_new, y_new, theta_new])
        
        # Jacobian G_t: derivative of motion model w.r.t. state
        def eval_Gt(x, y, theta, v, omega, dt):
            return np.array([
                [1, 0, -v * np.sin(theta) * dt],
                [0, 1,  v * np.cos(theta) * dt],
                [0, 0,  1]
            ])
        
        # Jacobian V_t: derivative of motion model w.r.t. control
        def eval_Vt(x, y, theta, v, omega, dt):
            return np.array([
                [np.cos(theta) * dt, 0],
                [np.sin(theta) * dt, 0],
                [0, dt]
            ])
        
        # Create EKF instance
        self.ekf = RobotEKF(
            dim_x=3,  # [x, y, theta]
            dim_u=2,  # [v, omega]
            eval_gux=eval_gux,
            eval_Gt=eval_Gt,
            eval_Vt=eval_Vt
        )
        
        # Set initial state
        self.ekf.mu = np.array([x, y, theta])
        
        # Set initial covariance (small uncertainty at start)
        self.ekf.Sigma = np.diag([0.01, 0.01, 0.01])
        
        # Set process noise covariance M_t
        self.ekf.Mt = np.diag([self.sigma_v**2, self.sigma_omega**2])
        
        self.get_logger().info(f'EKF initialized at [{x:.2f}, {y:.2f}, {theta:.2f}]')
    
    def _load_landmarks(self):
        """Load landmark positions from yaml file"""
        try:
            # Try to load from lab04_pkg first
            package_share = get_package_share_directory('lab04_pkg')
            yaml_file = os.path.join(package_share, 'config', 'landmarks.yaml')
            
            # If not found, try turtlebot3_perception
            if not os.path.exists(yaml_file):
                package_share = get_package_share_directory('turtlebot3_perception')
                yaml_file = os.path.join(package_share, 'config', 'landmarks.yaml')
            
            self.get_logger().info(f'Loading landmarks from: {yaml_file}')
            
            with open(yaml_file, 'r') as f:
                data = yaml.safe_load(f)
                
                # Parse the landmarks (format: lists of id, x, y, z)
                landmark_data = data['landmarks']
                ids = landmark_data['id']
                xs = landmark_data['x']
                ys = landmark_data['y']
                
                landmarks = {}
                for i in range(len(ids)):
                    landmarks[ids[i]] = np.array([xs[i], ys[i]])
                    self.get_logger().info(
                        f'  Landmark {ids[i]}: ({xs[i]:.2f}, {ys[i]:.2f})'
                    )
                
                return landmarks
                
        except Exception as e:
            self.get_logger().error(f'Could not load landmarks from yaml: {e}')
            self.get_logger().warn('Using default landmark positions')
            
            # Default landmarks matching your YAML
            return {
                11: np.array([-1.1, -1.1]),
                12: np.array([-1.1, 0.0]),
                13: np.array([-1.1, 1.1]),
                21: np.array([0.0, -1.1]),
                22: np.array([0.0, 0.0]),
                23: np.array([0.0, 1.1]),
                31: np.array([1.1, -1.1]),
                32: np.array([1.1, 0.0]),
                33: np.array([1.1, 1.1]),
            }
    
    def odom_callback(self, msg):
        """
        Callback for odometry messages
        Extract linear and angular velocities
        """
        # Extract velocities from odometry
        self.last_v = msg.twist.twist.linear.x
        self.last_omega = msg.twist.twist.angular.z
        self.last_odom_time = self.get_clock().now()
    
    def prediction_callback(self):
        """
        Timer callback for EKF prediction step (20 Hz)
        """
        # Compute dt
        current_time = self.get_clock().now()
        dt = 1.0 / self.prediction_rate
        
        # Control input
        u = np.array([self.last_v, self.last_omega])
        sigma_u = np.sqrt(np.diag(self.ekf.Mt))
        
        # Store state before prediction
        state_before = self.ekf.mu.copy()
        
        # Perform prediction
        self.ekf.predict(u=u, sigma_u=sigma_u, g_extra_args=(dt,))
        
        # Log state every 1 second (20 predictions)
        if not hasattr(self, '_prediction_count'):
            self._prediction_count = 0
        
        self._prediction_count += 1
        
        if self._prediction_count % 20 == 0:
            self.get_logger().info(
                f'PREDICTION #{self._prediction_count}: '
                f'State: [{self.ekf.mu[0]:.3f}, {self.ekf.mu[1]:.3f}, {np.degrees(self.ekf.mu[2]):.1f}°] | '
                f'Uncertainty: σ_x={np.sqrt(self.ekf.Sigma[0,0]):.3f}, '
                f'σ_y={np.sqrt(self.ekf.Sigma[1,1]):.3f}, '
                f'σ_θ={np.degrees(np.sqrt(self.ekf.Sigma[2,2])):.1f}° | '
                f'Control: v={self.last_v:.2f}, ω={self.last_omega:.2f}'
            )
        
        # Publish estimated state
        self._publish_state()
    
    def landmark_callback(self, msg):
        """
        Callback for landmark measurements
        Perform EKF update for each detected landmark
        """
        if len(msg.landmarks) == 0:
            return
        
        self.get_logger().info(
            f'LANDMARK MEASUREMENT: Received {len(msg.landmarks)} landmarks'
        )
        
        num_successful_updates = 0
        
        # Process each landmark measurement
        for landmark in msg.landmarks:
            landmark_id = landmark.id
            
            # Check if we know this landmark
            if landmark_id not in self.landmarks:
                self.get_logger().warn(f'Unknown landmark ID: {landmark_id}')
                continue
            
            # Get landmark position
            lm_x, lm_y = self.landmarks[landmark_id]
            
            # Measurement: [range, bearing]
            z = np.array([landmark.range, landmark.bearing])
            
            # Store state before update
            state_before = self.ekf.mu.copy()
            uncertainty_before = np.sqrt(np.diag(self.ekf.Sigma))
            
            # Measurement noise covariance
            Qt = np.diag([self.sigma_range**2, self.sigma_bearing**2])
            
            # Measurement model: h(x) = [range, bearing] from robot to landmark
            def eval_hx(x, y, theta, lm_x, lm_y):
                dx = lm_x - x
                dy = lm_y - y
                q = dx**2 + dy**2
                
                range_pred = np.sqrt(q)
                bearing_pred = np.arctan2(dy, dx) - theta
                
                # Normalize bearing to [-pi, pi]
                bearing_pred = np.arctan2(np.sin(bearing_pred), np.cos(bearing_pred))
                
                return np.array([range_pred, bearing_pred])
            
            # Jacobian H_t: derivative of measurement model w.r.t. state
            def eval_Ht(x, y, theta, lm_x, lm_y):
                dx = lm_x - x
                dy = lm_y - y
                q = dx**2 + dy**2
                sqrt_q = np.sqrt(q)
                
                return np.array([
                    [-dx/sqrt_q, -dy/sqrt_q, 0],
                    [dy/q, -dx/q, -1]
                ])
            
            # Residual function for angle normalization
            def residual(z, z_pred, angle_idx=1):
                res = z - z_pred
                # Normalize bearing difference to [-pi, pi]
                res[angle_idx] = np.arctan2(np.sin(res[angle_idx]), np.cos(res[angle_idx]))
                return res
            
            # Perform update
            try:
                self.ekf.update(
                    z=z,
                    eval_hx=eval_hx,
                    eval_Ht=eval_Ht,
                    Qt=Qt,
                    hx_args=(*self.ekf.mu, lm_x, lm_y),
                    Ht_args=(*self.ekf.mu, lm_x, lm_y),
                    residual=residual,
                    angle_idx=1
                )
                
                # Calculate change
                state_change = self.ekf.mu - state_before
                uncertainty_after = np.sqrt(np.diag(self.ekf.Sigma))
                uncertainty_reduction = uncertainty_before - uncertainty_after
                
                num_successful_updates += 1
                
                self.get_logger().info(
                    f'  └─ UPDATE with landmark {landmark_id} at ({lm_x:.1f}, {lm_y:.1f}): '
                    f'Measured [r={z[0]:.2f}m, θ={np.degrees(z[1]):.1f}°] | '
                    f'State correction: Δx={state_change[0]:.3f}, Δy={state_change[1]:.3f}, '
                    f'Δθ={np.degrees(state_change[2]):.1f}° | '
                    f'Uncertainty reduced by: {uncertainty_reduction[0]:.3f}, {uncertainty_reduction[1]:.3f}'
                )
                
            except Exception as e:
                self.get_logger().error(f'Update failed for landmark {landmark_id}: {e}')
        
        if num_successful_updates > 0:
            self.get_logger().info(
                f'AFTER UPDATES: State: [{self.ekf.mu[0]:.3f}, {self.ekf.mu[1]:.3f}, {np.degrees(self.ekf.mu[2]):.1f}°] | '
                f'Uncertainty: σ_x={np.sqrt(self.ekf.Sigma[0,0]):.3f}, '
                f'σ_y={np.sqrt(self.ekf.Sigma[1,1]):.3f}'
            )
        
        # Publish updated state
        self._publish_state()
    
    def _publish_state(self):
        """Publish current EKF state as Odometry message"""
        msg = Odometry()
        
        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        
        # Position
        msg.pose.pose.position.x = float(self.ekf.mu[0])
        msg.pose.pose.position.y = float(self.ekf.mu[1])
        msg.pose.pose.position.z = 0.0
        
        # Orientation (convert theta to quaternion)
        theta = float(self.ekf.mu[2])
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = np.sin(theta / 2.0)
        msg.pose.pose.orientation.w = np.cos(theta / 2.0)
        
        # Covariance (6x6 matrix, we only have x, y, theta)
        covariance = np.zeros((6, 6))
        covariance[0:2, 0:2] = self.ekf.Sigma[0:2, 0:2]  # x, y
        covariance[5, 5] = self.ekf.Sigma[2, 2]  # theta
        msg.pose.covariance = covariance.flatten().tolist()
        
        # Velocity (use last command)
        msg.twist.twist.linear.x = float(self.last_v)
        msg.twist.twist.angular.z = float(self.last_omega)
        
        # Publish
        self.ekf_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()