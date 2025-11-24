#include <Wire.h>
#include <ICM_20948.h>

ICM_20948_I2C imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  delay(200);

  const uint8_t AD0_VAL = 0; // ADO = GND -> I2C address 0x68

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
}

void loop() {
  imu.getAGMT();  // update accel + gyro + mag + temp

  // Raw accel in mg
  float ax = imu.accX();
  float ay = imu.accY();
  float az = imu.accZ();

  // Raw gyro in dps
  float gx = imu.gyrX();
  float gy = imu.gyrY();
  float gz = imu.gyrZ();

  // Single clean line: ax ay az gx gy gz
  Serial.print(ax, 3); Serial.print(' ');
  Serial.print(ay, 3); Serial.print(' ');
  Serial.print(az, 3); Serial.print(' ');
  Serial.print(gx, 3); Serial.print(' ');
  Serial.print(gy, 3); Serial.print(' ');
  Serial.println(gz, 3);

  delay(10); // ~100 Hz
}
