import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

# --- Motor pin setup (example, adapt to your setup) ---
M1_IN1 = 9
M1_IN2 = 11
M2_IN1 = 13
M2_IN2 = 26
M1_PWM = 10
M2_PWM = 19

GPIO.setmode(GPIO.BCM)
GPIO.setup([M1_IN1, M1_IN2, M2_IN1, M2_IN2], GPIO.OUT)
GPIO.setup([M1_PWM, M2_PWM], GPIO.OUT)
pwm1 = GPIO.PWM(M1_PWM, 1000)
pwm2 = GPIO.PWM(M2_PWM, 1000)
pwm1.start(0)
pwm2.start(0)

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        self.subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info('Motor driver node started.')

    def cmd_vel_callback(self, msg):
        # Linear velocity (m/s) and angular velocity (rad/s)
        linear = msg.linear.x
        angular = msg.angular.z

        # Convert to left and right wheel speeds (simple differential model)
        wheel_base = 0.2  # distance between wheels [m]
        left_speed = linear - angular * wheel_base / 2.0
        right_speed = linear + angular * wheel_base / 2.0

        # Map to PWM duty cycle (assuming ±0.22 m/s max)
        max_speed = 0.22
        left_duty = max(min(abs(left_speed / max_speed) * 100, 100), 0)
        right_duty = max(min(abs(right_speed / max_speed) * 100, 100), 0)

        # Set directions
        self.set_motor(M1_IN1, M1_IN2, pwm1, left_speed)
        self.set_motor(M2_IN1, M2_IN2, pwm2, right_speed)

    def set_motor(self, in1, in2, pwm, speed):
        if speed > 0:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        elif speed < 0:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(min(abs(speed) / 0.22 * 100, 100))

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    rclpy.spin(node)
    node.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()