#include <Wire.h>
#include <ICM_20948.h>

ICM_20948_I2C imu;

// Orientations
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

// Gyro bias
float gx_bias = 0.0f;
float gy_bias = 0.0f;
float gz_bias = 0.0f;

unsigned long lastTime = 0;

void calibrateGyro() {
  const int N = 400;
  long sum_gx = 0, sum_gy = 0, sum_gz = 0;

  Serial.println("CALIBRATING_GYRO_KEEP_STILL");

  for (int i = 0; i < N; i++) {
    imu.getAGMT();
    sum_gx += imu.gyrX();
    sum_gy += imu.gyrY();
    sum_gz += imu.gyrZ();
    delay(5);
  }

  gx_bias = sum_gx / (float)N;
  gy_bias = sum_gy / (float)N;
  gz_bias = sum_gz / (float)N;
  
  // Reset orientation to zero AFTER calibration
  roll = 0.0f;
  pitch = 0.0f;
  yaw = 0.0f;
  
  Serial.println("CALIBRATION_COMPLETE");
}

void initIMU() {
  Wire.begin();
  delay(200);

  const uint8_t AD0_VAL = 0; // ADO = GND → address 0x68

  bool ok = false;
  while (!ok) {
    imu.begin(Wire, AD0_VAL);
    if (imu.status == ICM_20948_Stat_Ok) {
      ok = true;
      Serial.println("IMU_READY");
    } else {
      Serial.println("IMU_INIT_FAIL");
      delay(300);
    }
  }

  delay(500);
  calibrateGyro();
  
  // lastTime reset is now done after calibration
  lastTime = millis();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  initIMU();
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "reset") {
      Serial.println("RESET_COMMAND_RECEIVED");
      initIMU();
      return;
    }
  }

  imu.getAGMT();

  // dt in seconds
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;

  // accel mg → g
  float ax = imu.accX() / 1000.0f;
  float ay = imu.accY() / 1000.0f;
  float az = imu.accZ() / 1000.0f;

  // accel-only tilt
  float roll_acc  = atan2(ay, az) * 180.0f / PI;
  float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;

  // gyro deg/s minus bias
  float gx = imu.gyrX() - gx_bias;
  float gy = imu.gyrY() - gy_bias;
  float gz = imu.gyrZ() - gz_bias;

  // complementary filter (98% gyro, 2% accel)
  const float alpha = 0.98f;

  roll  = alpha * (roll  + gx * dt) + (1.0f - alpha) * roll_acc;
  pitch = alpha * (pitch + gy * dt) + (1.0f - alpha) * pitch_acc;
  yaw   = yaw + gz * dt; // unbounded, natural drift

  // send to Processing
  Serial.print(roll, 2); Serial.print(" ");
  Serial.print(pitch, 2); Serial.print(" ");
  Serial.println(yaw, 2);

  delay(10); // 100 Hz
}