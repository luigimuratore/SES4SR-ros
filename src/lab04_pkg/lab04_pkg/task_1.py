import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from landmark_msgs.msg import LandmarkArray

from lab04_pkg.models.ekf import RobotEKF
from lab04_pkg.models.utils import normalize_angle, residual
from lab04_pkg.models.probabilistic_models import landmark_range_bearing_model
from lab04_pkg.task_0_b import compute_jacobians


class Task_1(Node): #Node for EKF-based robot localization using landmarks
    def __init__(self):
        super().__init__('task_1')

        # Parameters
        self.declare_parameter('prediction_rate', 20.0)  # Hz
        self.declare_parameter('initial_x', -2.5)
        self.declare_parameter('initial_y', -0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('process_noise_v', 0.1)
        self.declare_parameter('process_noise_omega', 0.05)
        self.declare_parameter('measurement_noise_range', 0.1)
        self.declare_parameter('measurement_noise_bearing', 0.05)

        self.prediction_rate = self.get_parameter('prediction_rate').value
        initial_x = self.get_parameter('initial_x').value
        initial_y = self.get_parameter('initial_y').value
        initial_theta = self.get_parameter('initial_theta').value
        self.sigma_v = self.get_parameter('process_noise_v').value
        self.sigma_omega = self.get_parameter('process_noise_omega').value
        self.sigma_range = self.get_parameter('measurement_noise_range').value
        self.sigma_bearing = self.get_parameter('measurement_noise_bearing').value

        # Precompute control noise vector and measurement covariance
        self.sigma_u_vec = np.array([self.sigma_v, self.sigma_omega])
        self.Qt = np.diag([self.sigma_range ** 2, self.sigma_bearing ** 2])

        # Initialize EKF and landmarks
        self._initialize_ekf(initial_x, initial_y, initial_theta)
        self.landmarks = self._load_landmarks()
        self.get_logger().info(f"Loaded {len(self.landmarks)} landmarks")

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


    # EKF
    def _initialize_ekf(self, x, y, theta): #EKF with motion model and Jacobians

        def eval_gux(mu, u, sigma_u, dt): # Motion model g(mu, u)
            x, y, th = mu
            v, w = u

            # Use exact unicycle model when |w| is not tiny, else straight-line
            eps = 1e-5
            if abs(w) > eps:
                r = v / w
                th_new = th + w * dt
                x_new = x + r * (np.sin(th_new) - np.sin(th))
                y_new = y - r * (np.cos(th_new) - np.cos(th))
            else:
                th_new = th + w * dt
                x_new = x + v * np.cos(th) * dt
                y_new = y + v * np.sin(th) * dt

            th_new = normalize_angle(th_new)
            return np.array([x_new, y_new, th_new])

        # Jacobian wrt state
        def eval_Gt(x, y, th, v, w, dt):
            eps = 1e-5
            if abs(w) > eps:
                r = v / w
                th_new = th + w * dt
                s_th, c_th = np.sin(th), np.cos(th)
                s_th_new, c_th_new = np.sin(th_new), np.cos(th_new)

                dxdth = r * (c_th_new - c_th)
                dydth = r * (s_th_new - s_th)
            else:
                dxdth = -v * np.sin(th) * dt
                dydth = v * np.cos(th) * dt

            return np.array([
                [1.0, 0.0, dxdth],
                [0.0, 1.0, dydth],
                [0.0, 0.0, 1.0]
            ])

        # Jacobian wrt control
        def eval_Vt(x, y, th, v, w, dt):
            eps = 1e-5
            if abs(w) > eps:
                th_new = th + w * dt
                s_th, c_th = np.sin(th), np.cos(th)
                s_th_new, c_th_new = np.sin(th_new), np.cos(th_new)
                w2 = w * w

                dvx = (w * (s_th_new - s_th) - v * (c_th_new - c_th)) / w2
                dvy = (w * (-c_th_new + c_th) - v * (s_th_new - s_th)) / w2

                dwx = v * (c_th_new * dt * w - (s_th_new - s_th)) / w2
                dwy = v * (s_th_new * dt * w - (-c_th_new + c_th)) / w2
            else:
                dvx = np.cos(th) * dt
                dvy = np.sin(th) * dt
                dwx = -0.5 * v * dt * dt * np.sin(th)
                dwy = 0.5 * v * dt * dt * np.cos(th)

            return np.array([
                [dvx, dwx],
                [dvy, dwy],
                [0.0, dt]
            ])

        self.ekf = RobotEKF(
            dim_x=3,
            dim_u=2,
            eval_gux=eval_gux,
            eval_Gt=eval_Gt,
            eval_Vt=eval_Vt
        )

        self.ekf.mu = np.array([x, y, theta])
        self.ekf.Sigma = np.diag([0.01, 0.01, 0.01])
        self.ekf.Mt = np.diag([self.sigma_v ** 2, self.sigma_omega ** 2])

        self.get_logger().info(
            f'EKF initialized at [{x:.2f}, {y:.2f}, {theta:.2f}]')

    def _load_landmarks(self): #Load landmark positions from YAML
        package_share = get_package_share_directory('lab04_pkg')
        yaml_file = os.path.join(package_share, 'config', 'landmarks.yaml')
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
            self.get_logger().info(
                f'  Landmark {ids[i]}: ({xs[i]:.2f}, {ys[i]:.2f})'
            )

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
        self.ekf.predict(u=u, sigma_u=self.sigma_u_vec, g_extra_args=(dt,))

        # Some periodic logging
        self._prediction_count += 1
        if self._prediction_count % int(self.prediction_rate) == 0:
            self.get_logger().info(
                f'PRED #{self._prediction_count}: '
                f'μ = [{self.ekf.mu[0]:.3f}, {self.ekf.mu[1]:.3f}, '
                f'{np.degrees(self.ekf.mu[2]):.1f}°], '
                f'σx={np.sqrt(self.ekf.Sigma[0, 0]):.3f}, '
                f'σy={np.sqrt(self.ekf.Sigma[1, 1]):.3f}, '
                f'σθ={np.degrees(np.sqrt(self.ekf.Sigma[2, 2])):.1f}°, '
                f'v={self.last_v:.2f}, ω={self.last_omega:.2f}'
            )

        self._publish_state()

    def landmark_callback(self, msg: LandmarkArray): # EKF update for each landmark measurement
        if not msg.landmarks:
            return

        num_updates = 0
        for lm in msg.landmarks:
            lm_id = lm.id

            if lm_id not in self.landmarks:
                self.get_logger().warn(f'Unknown landmark ID: {lm_id}')
                continue

            lm_x, lm_y = self.landmarks[lm_id]
            z = np.array([lm.range, lm.bearing])

            mu_before = self.ekf.mu.copy()
            sigma_before = np.sqrt(np.diag(self.ekf.Sigma))

            try:
                # Use compute_jacobians from task_0_b
                self.ekf.update(
                    z=z,
                    eval_hx=lambda x, y, th, lx, ly: landmark_range_bearing_model(np.array([x, y, th]), np.array([lx, ly]), sigma=[0.0, 0.0]),
                    eval_Ht=lambda x, y, th, lx, ly: compute_jacobians(np.array([x, y, th]), np.array([lx, ly])),
                    Qt=self.Qt,
                    hx_args=(*self.ekf.mu, lm_x, lm_y),
                    Ht_args=(*self.ekf.mu, lm_x, lm_y),
                    residual=residual,
                    angle_idx=1
                )
            except Exception as e:
                self.get_logger().error(f'Update failed for landmark {lm_id}: {e}')
                continue

            num_updates += 1
            dmu = self.ekf.mu - mu_before
            sigma_after = np.sqrt(np.diag(self.ekf.Sigma))
            sigma_red = sigma_before - sigma_after

            self.get_logger().info(
                f'  UPDATE with lm {lm_id} at ({lm_x:.1f}, {lm_y:.1f}): '
                f'z = [r={z[0]:.2f}, θ={np.degrees(z[1]):.1f}°], '
                f'Δμ = [{dmu[0]:.3f}, {dmu[1]:.3f}, {np.degrees(dmu[2]):.1f}°], '
                f'σ red ≈ [{sigma_red[0]:.3f}, {sigma_red[1]:.3f}]'
            )

        if num_updates > 0:
            self.get_logger().info(
                f'AFTER UPDATES: μ = [{self.ekf.mu[0]:.3f}, '
                f'{self.ekf.mu[1]:.3f}, {np.degrees(self.ekf.mu[2]):.1f}° ({self.ekf.mu[2]:.3f} rad)], '
                f'σx={np.sqrt(self.ekf.Sigma[0,0]):.3f}, '
                f'σy={np.sqrt(self.ekf.Sigma[1,1]):.3f}'
            )

        self._publish_state()


    # PUBLISHING 
    def _publish_state(self):
        """Publish EKF state as Odometry on /ekf."""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'

        # Pose
        msg.pose.pose.position.x = float(self.ekf.mu[0])
        msg.pose.pose.position.y = float(self.ekf.mu[1])
        msg.pose.pose.position.z = 0.0

        theta = float(self.ekf.mu[2])
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = np.sin(theta / 2.0)
        msg.pose.pose.orientation.w = np.cos(theta / 2.0)

        # 6x6 covariance: fill x, y, yaw
        cov = np.zeros((6, 6))
        cov[0:2, 0:2] = self.ekf.Sigma[0:2, 0:2]
        cov[5, 5] = self.ekf.Sigma[2, 2]
        msg.pose.covariance = cov.flatten().tolist()

        # Twist
        msg.twist.twist.linear.x = float(self.last_v)
        msg.twist.twist.angular.z = float(self.last_omega)

        self.ekf_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Task_1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
