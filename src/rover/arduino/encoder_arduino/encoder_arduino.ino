// Mega interrupt pins: 2, 3, 18, 19, 20, 21
#define ENCODER1_A 2  // Left wheel
#define ENCODER1_B 36

#define ENCODER2_A 19  // Right wheel
#define ENCODER2_B 28

// Motor control pins
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

volatile long encoder1_count = 0;  // Left side
volatile long encoder2_count = 0;  // Right side

void setup() {
  Serial.begin(115200);
  
  // Set encoder pins as inputs
  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER1_B, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);
  pinMode(ENCODER2_B, INPUT_PULLUP);
  
  // Setup all motor control pins
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
  
  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), updateEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), updateEncoder2, RISING);
  
  Serial.println("ENCODER_READY");
  
  delay(100);
}

void loop() {
  // Read commands from ROS: "linear,angular\n"
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    int commaIndex = cmd.indexOf(',');
    
    if (commaIndex > 0) {
      float linear = cmd.substring(0, commaIndex).toFloat();
      float angular = cmd.substring(commaIndex + 1).toFloat();
      
      // Convert to left/right wheel speeds
      float wheel_base = 0.30;  // meters
      float left_speed = linear - angular * wheel_base / 2.0;
      float right_speed = linear + angular * wheel_base / 2.0;
      
      // Apply to motors
      setMotorSpeed(left_speed, right_speed);
    }
  }
  
  // Send encoder data every 10ms (100Hz)
  static unsigned long last_send = 0;
  if (millis() - last_send >= 10) {
    // Format: "E,encoder1,encoder2\n"
    Serial.print("E,");
    Serial.print(encoder1_count);
    Serial.print(",");
    Serial.println(encoder2_count);
    
    last_send = millis();
  }
}

void setMotorSpeed(float left_speed, float right_speed) {
  float max_speed = 0.22;  // m/s
  
  // Left side (Motor 1 and 3)
  int left_pwm = constrain(abs(left_speed / max_speed) * 255, 0, 255);
  if (left_speed > 0) {
    // Forward
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR3_IN1, HIGH);
    digitalWrite(MOTOR3_IN2, LOW);
  } else {
    // Backward
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, HIGH);
    digitalWrite(MOTOR3_IN1, LOW);
    digitalWrite(MOTOR3_IN2, HIGH);
  }
  analogWrite(MOTOR1_ENA, left_pwm);
  analogWrite(MOTOR3_ENA, left_pwm);
  
  // Right side (Motor 2 and 4)
  int right_pwm = constrain(abs(right_speed / max_speed) * 255, 0, 255);
  if (right_speed > 0) {
    // Forward
    digitalWrite(MOTOR2_IN1, HIGH);
    digitalWrite(MOTOR2_IN2, LOW);
    digitalWrite(MOTOR4_IN1, HIGH);
    digitalWrite(MOTOR4_IN2, LOW);
  } else {
    // Backward
    digitalWrite(MOTOR2_IN1, LOW);
    digitalWrite(MOTOR2_IN2, HIGH);
    digitalWrite(MOTOR4_IN1, LOW);
    digitalWrite(MOTOR4_IN2, HIGH);
  }
  analogWrite(MOTOR2_ENA, right_pwm);
  analogWrite(MOTOR4_ENA, right_pwm);
}

// Left side encoder (Motor 1)
void updateEncoder1() {
  if (digitalRead(ENCODER1_A) == HIGH) {
    if (digitalRead(ENCODER1_B) == LOW) {
      encoder1_count++;
    } else {
      encoder1_count--;
    }
  }
}

// Right side encoder (Motor 2)
void updateEncoder2() {
  if (digitalRead(ENCODER2_A) == HIGH) {
    if (digitalRead(ENCODER2_B) == LOW) {
      encoder2_count--;
    } else {
      encoder2_count++;
    }
  }
}