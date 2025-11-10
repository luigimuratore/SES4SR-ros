// Motor encoder pins
#define ENCODER1_A 38
#define ENCODER1_B 36
#define ENCODER2_A 30
#define ENCODER2_B 28
#define ENCODER3_A 34
#define ENCODER3_B 32
#define ENCODER4_A 26
#define ENCODER4_B 24

// Motor control pins (from your previous code)
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

const int motorSpeed = 150;

long encoder1_count = 0;
long encoder2_count = 0;
long encoder3_count = 0;
long encoder4_count = 0;

int last1A = 0, last2A = 0, last3A = 0, last4A = 0;

void setup() {
  Serial.begin(115200);
  
  // Set encoder pins as inputs
  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER1_B, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);
  pinMode(ENCODER2_B, INPUT_PULLUP);
  pinMode(ENCODER3_A, INPUT_PULLUP);
  pinMode(ENCODER3_B, INPUT_PULLUP);
  pinMode(ENCODER4_A, INPUT_PULLUP);
  pinMode(ENCODER4_B, INPUT_PULLUP);
  
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
  
  // Read initial encoder states
  last1A = digitalRead(ENCODER1_A);
  last2A = digitalRead(ENCODER2_A);
  last3A = digitalRead(ENCODER3_A);
  last4A = digitalRead(ENCODER4_A);
  
  Serial.println("=== ALL 4 Motors Encoder Test ===");
  Serial.println("Testing each motor for 2 seconds:");
  Serial.println("Motor 1 -> Motor 2 -> Motor 3 -> Motor 4");
  Serial.println();
  
  delay(1000);
}

void loop() {
  static unsigned long phase_start = 0;
  static int test_phase = 0;
  static unsigned long last_print = 0;
  
  // Poll encoders continuously
  updateEncoder1();
  updateEncoder2();
  updateEncoder3();
  updateEncoder4();
  
  // Motor test sequence (2 seconds each)
  if (test_phase < 4) {
    if (millis() - phase_start >= 2000) {
      stopAllMotors();
      test_phase++;
      phase_start = millis();
      
      if (test_phase < 4) {
        Serial.print("\n>>> Starting Motor ");
        Serial.println(test_phase + 1);
        startMotor(test_phase + 1);
      } else {
        Serial.println("\n=== Test Complete ===");
        Serial.println("Final encoder counts:");
      }
    }
  }
  
  // Print status every 200ms
  if (millis() - last_print >= 200) {
    Serial.print("Counts - M1:");
    Serial.print(encoder1_count);
    Serial.print(" M2:");
    Serial.print(encoder2_count);
    Serial.print(" M3:");
    Serial.print(encoder3_count);
    Serial.print(" M4:");
    Serial.println(encoder4_count);
    
    // Print raw pin states to check for signals
    Serial.print("Pin States - ");
    Serial.print("M1A:");
    Serial.print(digitalRead(ENCODER1_A));
    Serial.print(" M1B:");
    Serial.print(digitalRead(ENCODER1_B));
    Serial.print(" | M2A:");
    Serial.print(digitalRead(ENCODER2_A));
    Serial.print(" M2B:");
    Serial.print(digitalRead(ENCODER2_B));
    Serial.print(" | M3A:");
    Serial.print(digitalRead(ENCODER3_A));
    Serial.print(" M3B:");
    Serial.print(digitalRead(ENCODER3_B));
    Serial.print(" | M4A:");
    Serial.print(digitalRead(ENCODER4_A));
    Serial.print(" M4B:");
    Serial.println(digitalRead(ENCODER4_B));
    
    last_print = millis();
  }
}

void startMotor(int motor) {
  if (motor == 1) {
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    analogWrite(MOTOR1_ENA, motorSpeed);
  } else if (motor == 2) {
    digitalWrite(MOTOR2_IN1, HIGH);
    digitalWrite(MOTOR2_IN2, LOW);
    analogWrite(MOTOR2_ENA, motorSpeed);
  } else if (motor == 3) {
    digitalWrite(MOTOR3_IN1, HIGH);
    digitalWrite(MOTOR3_IN2, LOW);
    analogWrite(MOTOR3_ENA, motorSpeed);
  } else if (motor == 4) {
    digitalWrite(MOTOR4_IN1, HIGH);
    digitalWrite(MOTOR4_IN2, LOW);
    analogWrite(MOTOR4_ENA, motorSpeed);
  }
}

void stopAllMotors() {
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

void updateEncoder1() {
  int currentA = digitalRead(ENCODER1_A);
  if (currentA != last1A) {
    int b = digitalRead(ENCODER1_B);
    if (currentA == b) {
      encoder1_count++;
    } else {
      encoder1_count--;
    }
    last1A = currentA;
  }
}

void updateEncoder2() {
  int currentA = digitalRead(ENCODER2_A);
  if (currentA != last2A) {
    int b = digitalRead(ENCODER2_B);
    if (currentA == b) {
      encoder2_count++;
    } else {
      encoder2_count--;
    }
    last2A = currentA;
  }
}

void updateEncoder3() {
  int currentA = digitalRead(ENCODER3_A);
  if (currentA != last3A) {
    int b = digitalRead(ENCODER3_B);
    if (currentA == b) {
      encoder3_count++;
    } else {
      encoder3_count--;
    }
    last3A = currentA;
  }
}

void updateEncoder4() {
  int currentA = digitalRead(ENCODER4_A);
  if (currentA != last4A) {
    int b = digitalRead(ENCODER4_B);
    if (currentA == b) {
      encoder4_count++;
    } else {
      encoder4_count--;
    }
    last4A = currentA;
  }
}