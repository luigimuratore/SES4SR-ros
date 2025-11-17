"""ROS 2 node running an EKF localization using landmark measurements."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Tuple

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import Odometry
from landmark_msgs.msg import LandmarkArray

from .models.ekf import RobotEKF


def _wrap_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""

    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _velocity_motion_model(mu: np.ndarray, u: np.ndarray, _sigma: np.ndarray, dt: float) -> np.ndarray:
    """Deterministic velocity motion model used by the EKF."""

    x, y, theta = mu
    v, w = u

    if abs(w) < 1e-6:
        x += v * dt * math.cos(theta)
        y += v * dt * math.sin(theta)
        theta += w * dt
    else:
        r = v / w
        theta_new = theta + w * dt
        x += -r * math.sin(theta) + r * math.sin(theta_new)
        y += r * math.cos(theta) - r * math.cos(theta_new)
        theta = theta_new

    return np.array([x, y, _wrap_angle(theta)], dtype=float)


def _velocity_jacobian_state(x: float, y: float, theta: float, v: float, w: float, dt: float) -> np.ndarray:
    """Jacobian of the velocity motion model with respect to the state."""

    if abs(w) < 1e-6:
        return np.array(
            [
                [1.0, 0.0, -v * dt * math.sin(theta)],
                [0.0, 1.0, v * dt * math.cos(theta)],
                [0.0, 0.0, 1.0],
            ]
        )

    r = v / w
    theta_new = theta + w * dt
    return np.array(
        [
            [1.0, 0.0, -r * math.cos(theta) + r * math.cos(theta_new)],
            [0.0, 1.0, -r * math.sin(theta) + r * math.sin(theta_new)],
            [0.0, 0.0, 1.0],
        ]
    )


def _velocity_jacobian_command(x: float, y: float, theta: float, v: float, w: float, dt: float) -> np.ndarray:
    """Jacobian of the velocity motion model with respect to the control input."""

    if abs(w) < 1e-6:
        return np.array(
            [
                [dt * math.cos(theta), 0.0],
                [dt * math.sin(theta), 0.0],
                [0.0, dt],
            ]
        )

    theta_new = theta + w * dt
    return np.array(
        [
            [
                -math.sin(theta) / w + math.sin(theta_new) / w,
                dt * v * math.cos(theta_new) / w
                + (v * math.sin(theta) - v * math.sin(theta_new)) / (w * w),
            ],
            [
                -math.cos(theta) / w - math.cos(theta_new) / w,
                dt * v * math.sin(theta_new) / w
                - (v * math.cos(theta) - v * math.cos(theta_new)) / (w * w),
            ],
            [0.0, dt],
        ]
    )


def _landmark_measurement_model(x: float, y: float, theta: float, mx: float, my: float) -> np.ndarray:
    """Range-bearing measurement model."""

    dx = mx - x
    dy = my - y
    rng = math.sqrt(dx * dx + dy * dy)
    bearing = math.atan2(dy, dx) - theta
    return np.array([rng, _wrap_angle(bearing)], dtype=float)


def _landmark_measurement_jacobian(x: float, y: float, theta: float, mx: float, my: float) -> np.ndarray:
    """Jacobian of the landmark measurement model."""

    dx = mx - x
    dy = my - y
    q = dx * dx + dy * dy

    if q < 1e-9:
        q = 1e-9

    sqrt_q = math.sqrt(q)
    return np.array(
        [
            [-dx / sqrt_q, -dy / sqrt_q, 0.0],
            [dy / q, -dx / q, -1.0],
        ]
    )


def _residual(z: np.ndarray, z_hat: np.ndarray, *, angle_idx: int) -> np.ndarray:
    """Residual used in the EKF update normalizing the bearing component."""

    y = z - z_hat
    y[angle_idx] = _wrap_angle(y[angle_idx])
    return y


class LandmarkEkfNode(Node):
    """ROS 2 node that fuses odometry and landmark measurements via an EKF."""

    def __init__(self) -> None:
        super().__init__("landmark_ekf")

        self.declare_parameter("prediction_rate", 20.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("initial_state", [0.0, 0.0, 0.0])
        self.declare_parameter("initial_covariance", [0.5, 0.5, math.radians(10.0)])
        self.declare_parameter("motion_noise", [0.05, math.radians(2.0)])
        self.declare_parameter("measurement_noise", [0.2, math.radians(5.0)])
        self.declare_parameter("landmark_file", "")

        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._child_frame_id = self.get_parameter("child_frame_id").get_parameter_value().string_value
        self._timer_period = 1.0 / self.get_parameter("prediction_rate").value

        initial_state = np.array(self.get_parameter("initial_state").value, dtype=float)
        initial_cov = np.array(self.get_parameter("initial_covariance").value, dtype=float)
        motion_noise = np.array(self.get_parameter("motion_noise").value, dtype=float)
        measurement_noise = np.array(self.get_parameter("measurement_noise").value, dtype=float)

        landmark_path = self._resolve_landmark_file(
            self.get_parameter("landmark_file").get_parameter_value().string_value
        )
        self._landmarks = self._load_landmarks(landmark_path)

        self._ekf = RobotEKF(
            dim_x=3,
            dim_u=2,
            eval_gux=_velocity_motion_model,
            eval_Gt=_velocity_jacobian_state,
            eval_Vt=_velocity_jacobian_command,
        )
        self._ekf.mu = initial_state
        self._ekf.Sigma = np.diag(initial_cov)
        self._ekf.Mt = np.diag(motion_noise ** 2)

        self._sigma_u = motion_noise
        self._Q = np.diag(measurement_noise ** 2)

        self._last_twist = np.zeros(2, dtype=float)
        self._have_odom = False
        self._last_predict_time = None

        self._odom_sub = self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self._landmark_sub = self.create_subscription(LandmarkArray, "/landmarks", self._landmarks_callback, 10)
        self._ekf_pub = self.create_publisher(Odometry, "/ekf", 10)
        self._timer = self.create_timer(self._timer_period, self._prediction_timer)

        self.get_logger().info(
            f"EKF node initialized with {len(self._landmarks)} landmarks loaded from {landmark_path}"
        )

    def _resolve_landmark_file(self, configured_path: str) -> Path:
        if configured_path:
            return Path(configured_path)

        share_dir = Path(get_package_share_directory("turtlebot3_perception"))
        return share_dir / "config" / "landmarks.yaml"

    def _load_landmarks(self, path: Path) -> Dict[int, Tuple[float, float]]:
        if not path.exists():
            raise FileNotFoundError(f"Unable to find landmarks file at {path}")

        with path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        ids = data["landmarks"]["id"]
        xs = data["landmarks"]["x"]
        ys = data["landmarks"]["y"]

        return {int(lm_id): (float(x), float(y)) for lm_id, x, y in zip(ids, xs, ys)}

    def _odom_callback(self, msg: Odometry) -> None:
        self._last_twist = np.array(
            [msg.twist.twist.linear.x, msg.twist.twist.angular.z],
            dtype=float,
        )
        self._have_odom = True

    def _prediction_timer(self) -> None:
        if not self._have_odom:
            return

        now = self.get_clock().now()
        if self._last_predict_time is None:
            self._last_predict_time = now
            return

        dt = (now - self._last_predict_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        self._ekf.predict(u=self._last_twist, sigma_u=self._sigma_u, g_extra_args=(dt,))
        self._ekf.mu[2] = _wrap_angle(self._ekf.mu[2])
        self._last_predict_time = now

        self._publish_estimate(now.to_msg())

    def _landmarks_callback(self, msg: LandmarkArray) -> None:
        if not msg.landmarks:
            return

        updated = False
        for lm in msg.landmarks:
            landmark_pose = self._landmarks.get(lm.id)
            if landmark_pose is None:
                self.get_logger().debug(f"Received measurement for unknown landmark id {lm.id}")
                continue

            z = np.array([lm.range, lm.bearing], dtype=float)
            args = (*self._ekf.mu, *landmark_pose)
            self._ekf.update(
                z,
                eval_hx=_landmark_measurement_model,
                eval_Ht=_landmark_measurement_jacobian,
                Qt=self._Q,
                Ht_args=args,
                hx_args=args,
                residual=_residual,
                angle_idx=1,
            )
            self._ekf.mu[2] = _wrap_angle(self._ekf.mu[2])
            updated = True

        if updated:
            stamp = msg.header.stamp if msg.header.stamp.sec or msg.header.stamp.nanosec else None
            self._publish_estimate(stamp)

    def _publish_estimate(self, stamp) -> None:
        if stamp is None:
            stamp = self.get_clock().now().to_msg()

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.child_frame_id = self._child_frame_id

        msg.pose.pose.position.x = float(self._ekf.mu[0])
        msg.pose.pose.position.y = float(self._ekf.mu[1])
        msg.pose.pose.position.z = 0.0

        half_yaw = 0.5 * self._ekf.mu[2]
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        pose_cov = np.zeros((6, 6))
        pose_cov[0, 0] = self._ekf.Sigma[0, 0]
        pose_cov[1, 1] = self._ekf.Sigma[1, 1]
        pose_cov[5, 5] = self._ekf.Sigma[2, 2]
        pose_cov[2, 2] = 1e3
        pose_cov[3, 3] = 1e3
        pose_cov[4, 4] = 1e3
        msg.pose.covariance = pose_cov.flatten().tolist()

        msg.twist.twist.linear.x = float(self._last_twist[0])
        msg.twist.twist.angular.z = float(self._last_twist[1])

        self._ekf_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LandmarkEkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
