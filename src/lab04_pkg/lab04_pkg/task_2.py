import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import Odometry
from landmark_msgs.msg import LandmarkArray
from sensor_msgs.msg import Imu
from lab04_pkg.models.utils import normalize_angle, residual
from lab04_pkg.models.ekf import RobotEKF


class Task_2(Node):
    """ State = [x, y, theta, v, omega].
    - Motion model: constant velocity.
    - Updates: landmarks, wheel encoders (/odom), IMU (/imu)."""
    def __init__(self):
        super().__init__('task_2')

        # ---- Parameters ----
        self.declare_parameter('prediction_rate', 20.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)

        # Process noise on v, omega (drift in speed/turn rate)
        self.declare_parameter('process_noise_v', 0.05)
        self.declare_parameter('process_noise_omega', 0.05)

        # Landmark measurement noise
        self.declare_parameter('measurement_noise_range', 0.1)
        self.declare_parameter('measurement_noise_bearing', 0.05)

        # Encoder measurement noise (odom -> v, omega)
        # These should reflect the accuracy of your wheel encoders
        self.declare_parameter('encoder_noise_v', 0.05)       # ~5 cm/s uncertainty
        self.declare_parameter('encoder_noise_omega', 0.05)   # ~0.05 rad/s uncertainty

        # IMU measurement noise (gyro -> omega)
        # This should reflect IMU gyroscope accuracy
        self.declare_parameter('imu_noise_omega', 0.02)       # IMUs are often more accurate

        self.prediction_rate = self.get_parameter('prediction_rate').value
        initial_x = self.get_parameter('initial_x').value
        initial_y = self.get_parameter('initial_y').value
        initial_theta = self.get_parameter('initial_theta').value
        self.sigma_v_proc = self.get_parameter('process_noise_v').value
        self.sigma_w_proc = self.get_parameter('process_noise_omega').value
        self.sigma_range = self.get_parameter('measurement_noise_range').value
        self.sigma_bearing = self.get_parameter('measurement_noise_bearing').value
        self.sigma_v_enc = self.get_parameter('encoder_noise_v').value
        self.sigma_w_enc = self.get_parameter('encoder_noise_omega').value
        self.sigma_w_imu = self.get_parameter('imu_noise_omega').value

        # Precompute some measurement covariances
        self.Qt_landmark = np.diag([self.sigma_range**2, self.sigma_bearing**2])
        self.Qt_encoder = np.diag([self.sigma_v_enc**2, self.sigma_w_enc**2])
        self.Qt_imu = np.array([[self.sigma_w_imu**2]])

        # Init EKF and landmarks
        self._initialize_ekf(initial_x, initial_y, initial_theta)
        self.landmarks = self._load_landmarks()
        self.get_logger().info(f"Loaded {len(self.landmarks)} landmarks")

        # For dt calculation
        self.last_prediction_time = self.get_clock().now()
        self._prediction_count = 0


        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry,'/odom',self.odom_callback,10)  # Encoders: we use odometry twist as encoder measurement (v, omega)
        self.landmark_sub = self.create_subscription(LandmarkArray,'/landmarks',self.landmark_callback,10) # Landmarks: range + bearing measurements
        self.imu_sub = self.create_subscription(Imu,'/imu',self.imu_callback,10) # IMU: angular velocity around z

        # Publisher
        self.ekf_pub = self.create_publisher(Odometry, '/ekf', 10)
        self.timer = self.create_timer(1.0 / self.prediction_rate,self.prediction_callback)

        self.get_logger().info('EKF Task 2 node initialized')



    # EKF init + motion model
    def _initialize_ekf(self, x0, y0, theta0): 
        # 1. Motion model g(u,x)
        def eval_gux(mu, u, sigma_u, dt):
            x, y, th, v, w = mu
            
            # Check for small omega to avoid division by zero
            if abs(w) < 1e-5:
                # Use Euler limit when moving straight
                x_new = x + v * np.cos(th) * dt
                y_new = y + v * np.sin(th) * dt
                th_new = normalize_angle(th + w * dt)
            else:
                # Exact integration for curved motion
                x_new = x + (v/w) * (np.sin(th + w*dt) - np.sin(th))
                y_new = y + (v/w) * (-np.cos(th + w*dt) + np.cos(th))
                th_new = normalize_angle(th + w * dt)

            # v and w are state variables that follow a random walk (process noise added later)
            return np.array([x_new, y_new, th_new, v, w])

        # 2. Jacobian Gt (w.r.t State)
        def eval_Gt(x, y, th, v, w, u0, u1, dt):
            if abs(w) < 1e-5:
                # Jacobian limit for straight line (Euler)
                return np.array([
                    [1.0, 0.0, -v*np.sin(th)*dt, np.cos(th)*dt, 0.0],
                    [0.0, 1.0,  v*np.cos(th)*dt, np.sin(th)*dt, 0.0],
                    [0.0, 0.0,  1.0,             0.0,           dt ],
                    [0.0, 0.0,  0.0,             1.0,           0.0],
                    [0.0, 0.0,  0.0,             0.0,           1.0]
                ])
            else:
                sin_th = np.sin(th)
                cos_th = np.cos(th)
                sin_th_wdt = np.sin(th + w*dt)
                cos_th_wdt = np.cos(th + w*dt)
                
                # Derivatives w.r.t theta
                dx_dth = (v/w) * (cos_th_wdt - cos_th)
                dy_dth = (v/w) * (sin_th_wdt - sin_th)
                
                # Derivatives w.r.t v
                dx_dv = (1/w) * (sin_th_wdt - sin_th)
                dy_dv = (1/w) * (-cos_th_wdt + cos_th)
                
                # Derivatives w.r.t w 
                # d(v/w)/dw = -v/w^2
                term1_x = (-v / (w**2)) * (sin_th_wdt - sin_th)
                term2_x = (v / w) * (np.cos(th + w*dt) * dt)
                dx_dw = term1_x + term2_x
                
                term1_y = (-v / (w**2)) * (-cos_th_wdt + cos_th)
                term2_y = (v / w) * (np.sin(th + w*dt) * dt)
                dy_dw = term1_y + term2_y

                return np.array([
                    [1.0, 0.0, dx_dth, dx_dv, dx_dw],
                    [0.0, 1.0, dy_dth, dy_dv, dy_dw],
                    [0.0, 0.0, 1.0,    0.0,   dt   ],
                    [0.0, 0.0, 0.0,    1.0,   0.0  ],
                    [0.0, 0.0, 0.0,    0.0,   1.0  ]
                ])

        # 3. Jacobian Vt (w.r.t Control/Noise)
        def eval_Vt(x, y, th, v, w, u0, u1, dt):
            # Map process noise directly to v and w
            return np.array([
                [0.0, 0.0],
                [0.0, 0.0],
                [0.0, 0.0],
                [1.0, 0.0],
                [0.0, 1.0]
            ])

        self.ekf = RobotEKF(
            dim_x=5,
            dim_u=2,
            eval_gux=eval_gux,
            eval_Gt=eval_Gt,
            eval_Vt=eval_Vt
        )

        self.ekf.mu = np.array([x0, y0, theta0, 0.0, 0.0])
        self.ekf.Sigma = np.diag([0.01, 0.01, 0.01, 0.5, 0.5])
        self.ekf.Mt = np.diag([self.sigma_v_proc**2, self.sigma_w_proc**2])
        
        self.get_logger().info(f'EKF Task 2 initialized at [{x0:.2f}, {y0:.2f}, {theta0:.2f}]')
        

    # Landmarks 
    def _load_landmarks(self):
        pkg_share = get_package_share_directory('lab04_pkg')
        yaml_file = os.path.join(pkg_share, 'config', 'landmarks.yaml')
        if not os.path.exists(yaml_file):
            pkg_share = get_package_share_directory('turtlebot3_perception')
            yaml_file = os.path.join(pkg_share, 'config', 'landmarks.yaml')

        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)

        lm = data['landmarks']
        ids = lm['id']
        xs = lm['x']
        ys = lm['y']

        landmarks = {}
        for i in range(len(ids)):
            landmarks[ids[i]] = np.array([xs[i], ys[i]])
            self.get_logger().info(
                f"Landmark {ids[i]}: ({xs[i]:.2f}, {ys[i]:.2f})"
            )
        return landmarks



    # Prediction
    def prediction_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_prediction_time).nanoseconds * 1e-9
        self.last_prediction_time = now

        if dt <= 0.0:
            dt = 1.0 / self.prediction_rate

        # No real control: we pass zero, only noise matters via Mt and Vt
        u = np.array([0.0, 0.0])
        sigma_u = np.sqrt(np.diag(self.ekf.Mt))

        self.ekf.predict(u=u, sigma_u=sigma_u, g_extra_args=(dt,))

        self._prediction_count += 1
        if self._prediction_count % int(self.prediction_rate) == 0:
            x, y, th, v, w = self.ekf.mu
            self.get_logger().info(
                f"PRED: x={x:.2f}, y={y:.2f}, th={np.degrees(th):.1f}° ({th:.3f} rad), "
                f"v={v:.2f}, w={w:.2f}"
            )

        self._publish_state()


    # Encoders (/odom) -> v, omega measurement
    def odom_callback(self, msg: Odometry):
        """Use /odom twist as encoder-like measurement of v and omega."""
        v_meas = msg.twist.twist.linear.x
        w_meas = msg.twist.twist.angular.z
        z = np.array([v_meas, w_meas])

        def hx_enc(x, y, th, v, w):
            return np.array([v, w])

        def H_enc(x, y, th, v, w):
            return np.array([
                [0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 1.0],
            ])

        try:
            self.ekf.update(
                z=z,
                eval_hx=hx_enc,
                eval_Ht=H_enc,
                Qt=self.Qt_encoder,
                hx_args=tuple(self.ekf.mu),
                Ht_args=tuple(self.ekf.mu)
            )
        except Exception as e:
            self.get_logger().error(f"Encoder update error: {e}")

        self._publish_state()


    # IMU (/imu) -> omega measurement
    def imu_callback(self, msg: Imu):
        """Use IMU gyro z as measurement of omega."""
        w_meas = msg.angular_velocity.z
        z = np.array([w_meas])

        def hx_imu(x, y, th, v, w):
            return np.array([w])

        def H_imu(x, y, th, v, w):
            return np.array([[0.0, 0.0, 0.0, 0.0, 1.0]])

        try:
            self.ekf.update(
                z=z,
                eval_hx=hx_imu,
                eval_Ht=H_imu,
                Qt=self.Qt_imu,
                hx_args=tuple(self.ekf.mu),
                Ht_args=tuple(self.ekf.mu)
            )
        except Exception as e:
            self.get_logger().error(f"IMU update error: {e}")

        self._publish_state()



    # Landmarks update (same as Task 1, extended state)
    def landmark_callback(self, msg: LandmarkArray): #Update with range/bearing to known landmarks
        if not msg.landmarks:
            return

        for lm in msg.landmarks:
            lm_id = lm.id
            if lm_id not in self.landmarks:
                continue

            lm_x, lm_y = self.landmarks[lm_id]
            z = np.array([lm.range, lm.bearing])

            def hx_landmark(x, y, th, v, w, lm_x, lm_y):
                dx = lm_x - x
                dy = lm_y - y
                r = np.sqrt(dx*dx + dy*dy)
                b = np.arctan2(dy, dx) - th
                return np.array([r, normalize_angle(b)])

            def H_landmark(x, y, th, v, w, lm_x, lm_y):
                """ Jacobian of landmark measurement model w.r.t. state [x,y,θ,v,ω]
                h = [range, bearing] where:
                  range = sqrt((lm_x - x)^2 + (lm_y - y)^2)
                  bearing = atan2(lm_y - y, lm_x - x) - theta
                """
                dx = lm_x - x
                dy = lm_y - y
                q = dx*dx + dy*dy
                sq = np.sqrt(q)
                
                # Derivatives of range
                dr_dx = -dx / sq
                dr_dy = -dy / sq
                
                # Derivatives of bearing
                db_dx = dy / q
                db_dy = -dx / q
                db_dtheta = -1.0
                
                return np.array([
                    [dr_dx, dr_dy, 0.0,       0.0, 0.0],  # ∂range/∂[x,y,θ,v,ω]
                    [db_dx, db_dy, db_dtheta, 0.0, 0.0],  # ∂bearing/∂[x,y,θ,v,ω]
                ])

            def residual_landmark(z, z_hat, angle_idx=1):
                res = z - z_hat
                res[angle_idx] = normalize_angle(res[angle_idx])
                return res

            try:
                self.ekf.update(
                    z=z,
                    eval_hx=hx_landmark,
                    eval_Ht=H_landmark,
                    Qt=self.Qt_landmark,
                    hx_args=(*self.ekf.mu, lm_x, lm_y),
                    Ht_args=(*self.ekf.mu, lm_x, lm_y),
                    residual=residual_landmark,
                    angle_idx=1
                )
            except Exception as e:
                self.get_logger().error(f"Landmark update error (id={lm_id}): {e}")

        self._publish_state()

   
    # Publish /ekf
    def _publish_state(self): #Publish EKF state as nav_msgs/Odometry on /ekf
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'

        x, y, th, v, w = self.ekf.mu

        # Pose
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = np.sin(th / 2.0)
        msg.pose.pose.orientation.w = np.cos(th / 2.0)

        # 6x6 covariance, fill x,y,theta
        cov = np.zeros((6, 6))
        cov[0:3, 0:3] = self.ekf.Sigma[0:3, 0:3]
        cov[3, 3] = self.ekf.Sigma[3, 3]   # v
        cov[4, 4] = self.ekf.Sigma[4, 4]   # w (not really used by standard tools)
        msg.pose.covariance = cov.flatten().tolist()

        # Twist from state
        msg.twist.twist.linear.x = float(v)
        msg.twist.twist.angular.z = float(w)

        self.ekf_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Task_2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
