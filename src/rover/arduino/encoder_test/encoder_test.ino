// Mega interrupt pins: 2, 3, 18, 19, 20, 21
#define ENCODER1_A 21  
#define ENCODER1_B 36

#define ENCODER2_A 20  
#define ENCODER2_B 28

#define ENCODER3_A 19  
#define ENCODER3_B 34

#define ENCODER4_A 18 
#define ENCODER4_B 24

// Motor control pins (from forward_backward.ino)
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

volatile long encoder1_count = 0;
volatile long encoder2_count = 0;
volatile long encoder3_count = 0;
volatile long encoder4_count = 0;

void setup() {
  Serial.begin(9600);
  
  // Set encoder pins as inputs
  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER1_B, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);
  pinMode(ENCODER2_B, INPUT_PULLUP);
  pinMode(ENCODER3_A, INPUT_PULLUP);
  pinMode(ENCODER3_B, INPUT_PULLUP);
  pinMode(ENCODER4_A, INPUT_PULLUP);
  pinMode(ENCODER4_B, INPUT_PULLUP);
  
  // Setup all motor control pins (same as forward_backward.ino)
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
  
  // Attach interrupts for encoders - using RISING like your working code
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), updateEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), updateEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_A), updateEncoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER4_A), updateEncoder4, RISING);
  
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
  static bool phase_initialized = false;
  
  // Initialize first phase
  if (test_phase == 0 && !phase_initialized) {
    Serial.println("\n>>> Starting Motor 1");
    startMotor(1);
    phase_start = millis();
    phase_initialized = true;
  }
  
  // Motor test sequence (2 seconds each)
  if (test_phase < 4) {
    if (millis() - phase_start >= 2000) {
      stopAllMotors();
      test_phase++;
      
      if (test_phase < 4) {
        Serial.print("\n>>> Starting Motor ");
        Serial.println(test_phase + 1);
        startMotor(test_phase + 1);
        phase_start = millis();
      } else {
        Serial.println("\n=== Test Complete ===");
        Serial.println("Final encoder counts:");
        Serial.print("M1: "); Serial.println(encoder1_count);
        Serial.print("M2: "); Serial.println(encoder2_count);
        Serial.print("M3: "); Serial.println(encoder3_count);
        Serial.print("M4: "); Serial.println(encoder4_count);
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

// Motor 1: reversed sign (was backwards)
void updateEncoder1() {
  if (digitalRead(ENCODER1_A) == HIGH) {
    if (digitalRead(ENCODER1_B) == LOW) {
      encoder1_count++;  // Flipped: was --, now ++
    } else {
      encoder1_count--;  // Flipped: was ++, now --
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

// Motor 3: Check your wiring! 
// Make sure ENCODER3_A is connected to pin 19 and ENCODER3_B to pin 32
void updateEncoder3() {
  if (digitalRead(ENCODER3_A) == HIGH) {
    if (digitalRead(ENCODER3_B) == LOW) {
      encoder3_count--;
    } else {
      encoder3_count++;
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