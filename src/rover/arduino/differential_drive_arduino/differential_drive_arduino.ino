// Motor encoder pins - MUST USE INTERRUPT PINS ON MEGA!
#define ENCODER1_A 21
#define ENCODER1_B 36

#define ENCODER2_A 20
#define ENCODER2_B 28

// Encoder 3 is broken - skip it
#define ENCODER3_A 19
#define ENCODER3_B 34

#define ENCODER4_A 18
#define ENCODER4_B 24

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

volatile long encoder1_count = 0;
volatile long encoder2_count = 0;
volatile long encoder4_count = 0;

unsigned long last_encoder_time = 0;
const unsigned long encoder_publish_interval = 50; // 20Hz

unsigned long last_cmd_time = 0;
const unsigned long cmd_timeout = 500; // Stop if no command for 500ms

void setup() {
  Serial.begin(115200);
  
  // Set encoder pins as inputs
  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER1_B, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);
  pinMode(ENCODER2_B, INPUT_PULLUP);
  pinMode(ENCODER4_A, INPUT_PULLUP);
  pinMode(ENCODER4_B, INPUT_PULLUP);
  
  // Setup motor control pins
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
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), updateEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), updateEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER4_A), updateEncoder4, RISING);
  
  // Stop motors initially
  stopMotors();
  
  Serial.println("Arduino Differential Drive Ready");
  Serial.println("Format: linear,angular (e.g., 0.5,0.0)");
}

void loop() {
  // Handle serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      Serial.print("RX: ");
      Serial.println(command);
      processCommand(command);
      last_cmd_time = millis();
    }
  }
  
  // Safety: stop motors if no command received for timeout period
  if (millis() - last_cmd_time > cmd_timeout && last_cmd_time > 0) {
    stopMotors();
    last_cmd_time = 0; // Reset to avoid repeated stops
  }
  
  // Publish encoder data at regular intervals
  if (millis() - last_encoder_time >= encoder_publish_interval) {
    publishEncoderData();
    last_encoder_time = millis();
  }
}

void processCommand(String command) {
  int commaIndex = command.indexOf(',');
  if (commaIndex == -1) {
    Serial.println("ERR: Invalid format");
    stopMotors();
    return;
  }
  
  float linear = command.substring(0, commaIndex).toFloat();
  float angular = command.substring(commaIndex + 1).toFloat();
  
  Serial.print("CMD: lin=");
  Serial.print(linear);
  Serial.print(" ang=");
  Serial.println(angular);
  
  // Differential drive: left and right wheel speeds
  // wheel_base = 0.30m, so radius = 0.15m
  float left_speed = linear - (angular * 0.15);
  float right_speed = linear + (angular * 0.15);
  
  // Scale to max speed (assume max = 0.5 m/s corresponds to PWM 255)
  float max_speed = 0.5; // m/s
  int left_pwm = constrain(abs(left_speed / max_speed) * 255, 0, 255);
  int right_pwm = constrain(abs(right_speed / max_speed) * 255, 0, 255);
  
  Serial.print("PWM: L=");
  Serial.print(left_pwm);
  Serial.print(" R=");
  Serial.println(right_pwm);
  
  // Motor 1 (left front)
  if (left_speed > 0.01) {
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
  } else if (left_speed < -0.01) {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, HIGH);
  } else {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, LOW);
  }
  analogWrite(MOTOR1_ENA, left_pwm);
  
  // Motor 2 (right front)
  if (right_speed > 0.01) {
    digitalWrite(MOTOR2_IN1, HIGH);
    digitalWrite(MOTOR2_IN2, LOW);
  } else if (right_speed < -0.01) {
    digitalWrite(MOTOR2_IN1, LOW);
    digitalWrite(MOTOR2_IN2, HIGH);
  } else {
    digitalWrite(MOTOR2_IN1, LOW);
    digitalWrite(MOTOR2_IN2, LOW);
  }
  analogWrite(MOTOR2_ENA, right_pwm);
  
  // Motor 3 (left rear) - same as Motor 1
  if (left_speed > 0.01) {
    digitalWrite(MOTOR3_IN1, HIGH);
    digitalWrite(MOTOR3_IN2, LOW);
  } else if (left_speed < -0.01) {
    digitalWrite(MOTOR3_IN1, LOW);
    digitalWrite(MOTOR3_IN2, HIGH);
  } else {
    digitalWrite(MOTOR3_IN1, LOW);
    digitalWrite(MOTOR3_IN2, LOW);
  }
  analogWrite(MOTOR3_ENA, left_pwm);
  
  // Motor 4 (right rear) - same as Motor 2
  if (right_speed > 0.01) {
    digitalWrite(MOTOR4_IN1, HIGH);
    digitalWrite(MOTOR4_IN2, LOW);
  } else if (right_speed < -0.01) {
    digitalWrite(MOTOR4_IN1, LOW);
    digitalWrite(MOTOR4_IN2, HIGH);
  } else {
    digitalWrite(MOTOR4_IN1, LOW);
    digitalWrite(MOTOR4_IN2, LOW);
  }
  analogWrite(MOTOR4_ENA, right_pwm);
}

void stopMotors() {
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  analogWrite(MOTOR1_ENA, 0);
  
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);
  analogWrite(MOTOR2_ENA, 0);
  
  digitalWrite(MOTOR3_IN1, LOW);
  digitalWrite(MOTOR3_IN2, LOW);
  analogWrite(MOTOR3_ENA, 0);
  
  digitalWrite(MOTOR4_IN1, LOW);
  digitalWrite(MOTOR4_IN2, LOW);
  analogWrite(MOTOR4_ENA, 0);
}

void publishEncoderData() {
  // Format: "E,encoder1,encoder2,encoder4,timestamp"
  Serial.print("E,");
  Serial.print(encoder1_count);
  Serial.print(",");
  Serial.print(encoder2_count);
  Serial.print(",");
  Serial.print(encoder4_count);
  Serial.print(",");
  Serial.println(millis());
}

// Encoder interrupt handlers (reversed sign for Motor 1)
void updateEncoder1() {
  if (digitalRead(ENCODER1_A) == HIGH) {
    if (digitalRead(ENCODER1_B) == LOW) {
      encoder1_count++;
    } else {
      encoder1_count--;
    }
  }
}

void updateEncoder2() {
  if (digitalRead(ENCODER2_A) == HIGH) {
    if (digitalRead(ENCODER2_B) == LOW) {
      encoder2_count--;
    } else {
      encoder2_count++;
    }
  }
}

void updateEncoder4() {
  if (digitalRead(ENCODER4_A) == HIGH) {
    if (digitalRead(ENCODER4_B) == LOW) {
      encoder4_count--;
    } else {
      encoder4_count++;
    }
  }
}