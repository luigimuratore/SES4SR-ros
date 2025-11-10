// Motor pin definitions
#define MOTOR1_ENA 13
#define MOTOR1_IN1 51
#define MOTOR1_IN2 53

#define MOTOR2_ENA 11
#define MOTOR2_IN1 45
#define MOTOR2_IN2 43

#define MOTOR3_ENA 12
#define MOTOR3_IN1 49
#define MOTOR3_IN2 47

#define MOTOR4_ENA 10
#define MOTOR4_IN1 41
#define MOTOR4_IN2 39

const int MAX_SPEED = 255;
const float DEAD_ZONE = 0.01;  // Ignore very small commands

void setup() {
  Serial.begin(115200);  // Match arduino_bridge.py baud rate
  
  // Set motor control pins as outputs
  pinMode(MOTOR1_ENA, OUTPUT);
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  
  pinMode(MOTOR2_ENA, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  
  pinMode(MOTOR3_ENA, OUTPUT);
  pinMode(MOTOR3_IN1, OUTPUT);
  pinMode(MOTOR3_IN2, OUTPUT);
  
  pinMode(MOTOR4_ENA, OUTPUT);
  pinMode(MOTOR4_IN1, OUTPUT);
  pinMode(MOTOR4_IN2, OUTPUT);

  stopMotors();
  Serial.println("Arduino Differential Drive Ready");
  Serial.println("Expecting: 'linear_x angular_z' (e.g., '0.200 0.000')");
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    
    // Parse "linear_x angular_z"
    int spaceIndex = line.indexOf(' ');
    if (spaceIndex == -1) {
      Serial.println("ERROR: Invalid format. Expected 'linear_x angular_z'");
      return;
    }
    
    float linear_x = line.substring(0, spaceIndex).toFloat();
    float angular_z = line.substring(spaceIndex + 1).toFloat();
    
    // Convert Twist to differential drive commands
    processTwist(linear_x, angular_z);
  }
}

void processTwist(float linear_x, float angular_z) {
  // Differential drive: left_speed = linear - angular, right_speed = linear + angular
  // Motors 1 & 3 are LEFT side, Motors 2 & 4 are RIGHT side
  
  float left_speed = linear_x - angular_z;
  float right_speed = linear_x + angular_z;
  
  // Apply dead zone
  if (abs(left_speed) < DEAD_ZONE) left_speed = 0.0;
  if (abs(right_speed) < DEAD_ZONE) right_speed = 0.0;
  
  // Clamp speeds to [-1.0, 1.0]
  left_speed = constrain(left_speed, -1.0, 1.0);
  right_speed = constrain(right_speed, -1.0, 1.0);
  
  // Convert to PWM (0-255)
  int left_pwm = abs(left_speed) * MAX_SPEED;
  int right_pwm = abs(right_speed) * MAX_SPEED;
  
  // Set motor directions and speeds
  setMotorSpeed(1, left_speed, left_pwm);   // Front-Left
  setMotorSpeed(3, left_speed, left_pwm);   // Rear-Left
  setMotorSpeed(2, right_speed, right_pwm); // Front-Right
  setMotorSpeed(4, right_speed, right_pwm); // Rear-Right
}

void setMotorSpeed(int motor, float speed, int pwm) {
  int ena_pin, in1_pin, in2_pin;
  
  switch(motor) {
    case 1:
      ena_pin = MOTOR1_ENA; in1_pin = MOTOR1_IN1; in2_pin = MOTOR1_IN2;
      break;
    case 2:
      ena_pin = MOTOR2_ENA; in1_pin = MOTOR2_IN1; in2_pin = MOTOR2_IN2;
      break;
    case 3:
      ena_pin = MOTOR3_ENA; in1_pin = MOTOR3_IN1; in2_pin = MOTOR3_IN2;
      break;
    case 4:
      ena_pin = MOTOR4_ENA; in1_pin = MOTOR4_IN1; in2_pin = MOTOR4_IN2;
      break;
    default:
      return;
  }
  
  if (speed > 0.0) {
    // Forward
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
    analogWrite(ena_pin, pwm);
  } else if (speed < 0.0) {
    // Backward
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
    analogWrite(ena_pin, pwm);
  } else {
    // Stop
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, LOW);
    analogWrite(ena_pin, 0);
  }
}

void stopMotors() {
  setMotorSpeed(1, 0, 0);
  setMotorSpeed(2, 0, 0);
  setMotorSpeed(3, 0, 0);
  setMotorSpeed(4, 0, 0);
}