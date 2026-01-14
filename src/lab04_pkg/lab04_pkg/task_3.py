import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from landmark_msgs.msg import LandmarkArray

from lab04_pkg.models.ekf import RobotEKF
from lab04_pkg.models.utils import (normalize_angle, residual, eval_gux, eval_Gt, eval_Vt, eval_gux_5d, eval_Gt_5d, eval_Vt_5d)
from lab04_pkg.models.probabilistic_models import landmark_range_bearing_model
from lab04_pkg.task_0_b import compute_jacobians


class Task_3(Node): #Node for EKF-based robot localization using landmarks
    def __init__(self, mode):
        super().__init__('task_3')

        # Parameters
        self.declare_parameter('prediction_rate', 20.0)  # Hz
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.77)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('process_noise_v', 0.0)
        self.declare_parameter('process_noise_omega', 0.0)
        self.declare_parameter('measurement_noise_range', 1.0)
        self.declare_parameter('measurement_noise_bearing', 1.00)
        self.declare_parameter('encoder_noise_v', 1.00)
        self.declare_parameter('encoder_noise_omega', 1.00)
        self.declare_parameter('imu_noise_omega', 1.00)

        self.prediction_rate = self.get_parameter('prediction_rate').value
        initial_x = self.get_parameter('initial_x').value
        initial_y = self.get_parameter('initial_y').value
        initial_theta = self.get_parameter('initial_theta').value
        self.sigma_v = self.get_parameter('process_noise_v').value
        self.sigma_omega = self.get_parameter('process_noise_omega').value
        self.sigma_range = self.get_parameter('measurement_noise_range').value
        self.sigma_bearing = self.get_parameter('measurement_noise_bearing').value
        self.sigma_v_enc = self.get_parameter('encoder_noise_v').value
        self.sigma_w_enc = self.get_parameter('encoder_noise_omega').value
        self.sigma_w_imu = self.get_parameter('imu_noise_omega').value

        self.Qt_landmark = np.diag([self.sigma_range ** 2, self.sigma_bearing ** 2])
        self.Qt_encoder = np.diag([self.sigma_v_enc ** 2, self.sigma_w_enc ** 2])
        self.Qt_imu = np.array([[self.sigma_w_imu ** 2]])

        # Load landmarks
        self.landmarks = self._load_landmarks()
        self.get_logger().info(f"Loaded {len(self.landmarks)} landmarks")

        # EKF selection
        if mode == 1:
            # Task 1: pose-only EKF
            self.ekf = RobotEKF(
                dim_x=3,
                dim_u=2,
                eval_gux=eval_gux,
                eval_Gt=eval_Gt,
                eval_Vt=eval_Vt)
            self.ekf.mu = np.array([initial_x, initial_y, initial_theta])
            self.ekf.Sigma = np.diag([0.01, 0.01, 0.01])
            self.ekf.Mt = np.diag([self.sigma_v ** 2, self.sigma_omega ** 2])
            self.get_logger().info("Initialized EKF for Task 1 (pose only)")
        else:
            # Task 2: pose+velocity EKF
            self.ekf = RobotEKF(
                dim_x=5,
                dim_u=2,
                eval_gux=eval_gux_5d,
                eval_Gt=eval_Gt_5d,
                eval_Vt=eval_Vt_5d
            )
            self.ekf.mu = np.array([initial_x, initial_y, initial_theta, 0.0, 0.0])
            self.ekf.Sigma = np.diag([0.01, 0.01, 0.01, 0.5, 0.5])
            self.ekf.Mt = np.diag([self.sigma_v ** 2, self.sigma_omega ** 2])
            self.get_logger().info("Initialized EKF for Task 2 (pose + velocity)")

        # Last velocity from odom
        self.last_v = 0.0
        self.last_omega = 0.0

        # Time bookkeeping for dt
        self.last_prediction_time = self.get_clock().now()

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.landmark_sub = self.create_subscription(LandmarkArray,'/landmarks', self.landmark_callback,10) # '/camera/landmarks' on real robot

        # Publisher
        self.ekf_pub = self.create_publisher(Odometry, '/ekf', 10)

        # Prediction timer
        timer_period = 1.0 / self.prediction_rate
        self.timer = self.create_timer(timer_period, self.prediction_callback)

        self._prediction_count = 0
        self.get_logger().info('EKF Localization Node initialized')
        self.get_logger().info(f'Prediction rate: {self.prediction_rate} Hz')


    def _load_landmarks(self): #Load landmark positions from YAML
        package_share = get_package_share_directory('lab04_pkg')
        yaml_file = os.path.join(package_share, 'config', 'landmarks_lab.yaml')
        self.get_logger().info(f'Loading landmarks from: {yaml_file}')

        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)

        lm_data = data['landmarks']
        ids = lm_data['id']
        xs = lm_data['x']
        ys = lm_data['y']

        landmarks = {}
        for i in range(len(ids)):
            landmarks[ids[i]] = np.array([xs[i], ys[i]])
            self.get_logger().info(f'  Landmark {ids[i]}: ({xs[i]:.2f}, {ys[i]:.2f})')
        return landmarks

     
    # CALLBACKS
    def odom_callback(self, msg: Odometry): #Store latest linear and angular velocity from /odom
        self.last_v = msg.twist.twist.linear.x
        self.last_omega = msg.twist.twist.angular.z

    def prediction_callback(self): #EKF prediction step, called at fixed rate
        now = self.get_clock().now()
        dt = (now - self.last_prediction_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1.0 / self.prediction_rate  # fallback
        self.last_prediction_time = now

        u = np.array([self.last_v, self.last_omega])

        # EKF prediction
        self.ekf.predict(u=u, sigma_u=np.array([self.sigma_v, self.sigma_omega]), g_extra_args=(dt,))

        # Some periodic logging
        self._prediction_count += 1
        if self._prediction_count % int(self.prediction_rate) == 0:
            mu = self.ekf.mu
            if len(mu) == 3:
                self.get_logger().info(
                    f'PRED #{self._prediction_count}: μ = [{mu[0]:.3f}, {mu[1]:.3f}, {np.degrees(mu[2]):.1f}°], '
                    f'σx={np.sqrt(self.ekf.Sigma[0, 0]):.3f}, '
                    f'σy={np.sqrt(self.ekf.Sigma[1, 1]):.3f}, '
                    f'σθ={np.degrees(np.sqrt(self.ekf.Sigma[2, 2])):.1f}°, '
                    f'v={self.last_v:.2f}, ω={self.last_omega:.2f}')
            else:
                self.get_logger().info(
                    f'PRED #{self._prediction_count}: μ = [{mu[0]:.3f}, {mu[1]:.3f}, {np.degrees(mu[2]):.1f}°, v={mu[3]:.2f}, w={mu[4]:.2f}], '
                    f'σx={np.sqrt(self.ekf.Sigma[0, 0]):.3f}, '
                    f'σy={np.sqrt(self.ekf.Sigma[1, 1]):.3f}, '
                    f'σθ={np.degrees(np.sqrt(self.ekf.Sigma[2, 2])):.1f}°')
        self._publish_state()

    def landmark_callback(self, msg: LandmarkArray): # EKF update for each landmark measurement
        if not msg.landmarks:
            return

        mu_before = self.ekf.mu.copy()
        for lm in msg.landmarks:
            lm_id = lm.id
            if lm_id not in self.landmarks:
                self.get_logger().warn(f'Unknown landmark ID: {lm_id}')
                continue
            lm_x, lm_y = self.landmarks[lm_id]
            z = np.array([lm.range, lm.bearing])

            if len(self.ekf.mu) == 3:
                # Task 1: pose-only
                self.ekf.update(
                    z=z,
                    eval_hx=lambda x, y, th, lx, ly: landmark_range_bearing_model(np.array([x, y, th]), np.array([lx, ly]), sigma=[0.0, 0.0]),
                    eval_Ht=lambda x, y, th, lx, ly: compute_jacobians(np.array([x, y, th]), np.array([lx, ly])),
                    Qt=self.Qt_landmark,
                    hx_args=(*self.ekf.mu, lm_x, lm_y),
                    Ht_args=(*self.ekf.mu, lm_x, lm_y),
                    residual=residual,
                    angle_idx=1)
            else:
                # Task 2: pose+velocity
                def hx_landmark(x, y, th, v, w, lx, ly):
                    dx = lx - x
                    dy = ly - y
                    r = np.sqrt(dx*dx + dy*dy)
                    b = np.arctan2(dy, dx) - th
                    return np.array([r, normalize_angle(b)])
                def H_landmark(x, y, th, v, w, lx, ly):
                    dx = lx - x
                    dy = ly - y
                    q = dx*dx + dy*dy
                    sq = np.sqrt(q)
                    dr_dx = -dx / sq
                    dr_dy = -dy / sq
                    db_dx = dy / q
                    db_dy = -dx / q
                    db_dtheta = -1.0
                    return np.array([
                        [dr_dx, dr_dy, 0.0, 0.0, 0.0],
                        [db_dx, db_dy, db_dtheta, 0.0, 0.0],])
                def residual_landmark(z, z_hat, angle_idx=1):
                    res = z - z_hat
                    res[angle_idx] = normalize_angle(res[angle_idx])
                    return res
                self.ekf.update(
                    z=z,
                    eval_hx=hx_landmark,
                    eval_Ht=H_landmark,
                    Qt=self.Qt_landmark,
                    hx_args=(*self.ekf.mu, lm_x, lm_y),
                    Ht_args=(*self.ekf.mu, lm_x, lm_y),
                    residual=residual_landmark,
                    angle_idx=1)
        mu = self.ekf.mu
        if len(mu) == 3:
            self.get_logger().info(f'LANDMARK UPDATE RESULT: x={mu[0]:.3f}, y={mu[1]:.3f}, θ={np.degrees(mu[2]):.1f}°')
        else:
            self.get_logger().info(f'LANDMARK UPDATE RESULT: x={mu[0]:.3f}, y={mu[1]:.3f}, v={mu[3]:.3f}, w={mu[4]:.3f}')
        self._publish_state()

    # PUBLISHING 
    def _publish_state(self): #Publish EKF state as Odometry on /ekf
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'

        mu = self.ekf.mu
        msg.pose.pose.position.x = float(mu[0])
        msg.pose.pose.position.y = float(mu[1])
        msg.pose.pose.position.z = 0.0

        theta = float(mu[2])
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = np.sin(theta / 2.0)
        msg.pose.pose.orientation.w = np.cos(theta / 2.0)

        # 6x6 covariance: fill x, y, yaw
        cov = np.zeros((6, 6))
        cov[0:2, 0:2] = self.ekf.Sigma[0:2, 0:2]
        cov[5, 5] = self.ekf.Sigma[2, 2]
        msg.pose.covariance = cov.flatten().tolist()

        if len(mu) > 3:
            msg.twist.twist.linear.x = float(mu[3])
            msg.twist.twist.angular.z = float(mu[4])
        else:
            msg.twist.twist.linear.x = float(self.last_v)
            msg.twist.twist.angular.z = float(self.last_omega)
        self.ekf_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    # Ask for task
    mode = None
    while mode not in [1, 2]:
        try:
            mode = int(input("Select EKF mode: [1] Task 1 (pose only), [2] Task 2 (pose+velocity): "))
        except Exception:
            mode = None
    node = Task_3(mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()