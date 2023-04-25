#include "driver/ledc.h"

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void beep() {
    SerialBT.println("Beep!");
    digitalWrite(BUZZER, HIGH);
    delay(70);
    digitalWrite(BUZZER, LOW);
    delay(80);
}

void save() {
    EEPROM.put(0, offsets);
    EEPROM.commit();
    EEPROM.get(0, offsets);
    if (offsets.ID1 == 99 && offsets.ID2 == 99 && offsets.ID3 == 99 && offsets.ID4 == 99) calibrated = true;
    calibrating = false;
    Serial.println("Calibration stopped. Enter \"cs\" again to calibrate other edges.");
    beep();
}

void angle_setup() {
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);

  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyZ_offset_sum += GyZ;
    delay(3);
  }
  GyZ_offset = GyZ_offset_sum >> 10;
  Serial.print("GyZ offset: "); Serial.println(GyZ_offset);
  SerialBT.print("GyZ offset: "); SerialBT.println(GyZ_offset);
  beep();
  
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyY_offset_sum += GyY;
    delay(3);
  }
  GyY_offset = GyY_offset_sum >> 10;
  Serial.print("GyY offset: "); Serial.println(GyY_offset);
  SerialBT.print("GyY offset: "); SerialBT.println(GyY_offset);
  beep();
  
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyX_offset_sum += GyX;
    delay(3);
  }
  GyX_offset = GyX_offset_sum >> 10;
  Serial.print("GyX offset:"); Serial.println(GyX_offset);
  SerialBT.print("GyX offset: "); SerialBT.println(GyX_offset);
  beep();
  beep();
}


void angle_calc() {
  
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);  
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true); 
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  // add mpu6050 offset values
  GyZ -= GyZ_offset;
  GyY -= GyY_offset;
  GyX -= GyX_offset;

  robot_angleX += GyZ * loop_time / 1000 / 65.536;   
  Acc_angleX = atan2(AcY, -AcX) * 57.2958;               // angle from acc. values * 57.2958 (deg/rad)
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);

  robot_angleY += GyY * loop_time / 1000 / 65.536; 
  Acc_angleY = -atan2(AcZ, -AcX) * 57.2958;              // angle from acc. values * 57.2958 (deg/rad)
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);

  angleX = robot_angleX;
  angleY = robot_angleY;
  //SerialBT.print("AngleX: "); SerialBT.print(angleX); SerialBT.print(" AngleY: "); SerialBT.println(angleY); 
  
  if (abs(angleX - offsets.X2) < 2 && abs(angleY - offsets.Y2) < 0.6) {
    balancing_point = 2;
    if (!vertical) beep();
    vertical = true;
  } else if (abs(angleX - offsets.X3) < 2 && abs(angleY - offsets.Y3) < 0.6) {
    balancing_point = 3;
    if (!vertical) beep();
    vertical = true;
  } else if (abs(angleX - offsets.X4) < 0.6 && abs(angleY - offsets.Y4) < 2) {
    balancing_point = 4;
    if (!vertical) beep();
    vertical = true;
  } else if (abs(angleX - offsets.X1) < 0.4 && abs(angleY - offsets.Y1) < 0.4) {
    balancing_point = 1;
    if (!vertical) beep();
    vertical = true;
  } 
}

// Calculate angle differently if the cube is balanced on the vertex
void XY_to_threeWay(float pwm_X, float pwm_Y) {
  int16_t m1 = round(0.5 * pwm_X - 0.75 * pwm_Y);  
  int16_t m2 = round(0.5 * pwm_X + 0.75 * pwm_Y);
  int16_t m3 = -pwm_X;  
  m1 = constrain(m1, -maxspeed, maxspeed);
  m2 = constrain(m2, -maxspeed, maxspeed);
  m3 = constrain(m3, -maxspeed, maxspeed);
  Motor1_control(m1);
  Motor2_control(m2);
  Motor3_control(m3);
}

// Change the frequency of the ESP32's PWM channels based on the speed value. Actual duty cycle of the PWM is locked at 50%
// The built-in ledc library was used for driving the PWM channels.
void pwmSet(uint8_t channel, uint32_t freq) {
  if (freq == 0) {
    ledcWrite(channel, 0);
  } else {
    ledcChangeFrequency(channel, 100+freq, TIMER_BIT);
    ledcWrite(channel, BASE_DUTY);
  }
}


// Receives speed which can be between -maxspeed and maxspeed
// If speed is negative then the direction pin of the motor is activated to reverse the rotation
void Motor1_control(int speed) {
  if (speed < 0) {
    digitalWrite(DIR1, HIGH);
    speed = -speed;
  } else {
    digitalWrite(DIR1, LOW);
  }
  pwmSet(PWM1_CH, speed > maxspeed ? maxspeed : speed);
}

void Motor2_control(int speed) {
  if (speed < 0) {
    digitalWrite(DIR2, HIGH);
    speed = -speed;
  } else {
    digitalWrite(DIR2, LOW);
  }
  pwmSet(PWM2_CH, speed > maxspeed ? maxspeed : speed);
}

void Motor3_control(int speed) {
  if (speed < 0) {
    digitalWrite(DIR3, HIGH);
    speed = -speed;
  } else {
    digitalWrite(DIR3, LOW);
  }
  pwmSet(PWM3_CH, speed > maxspeed ? maxspeed : speed);
}

void printValues() {
  SerialBT.print("P: "); SerialBT.print(K1);
  SerialBT.print(" I: "); SerialBT.print(K2);
  SerialBT.print(" D: "); SerialBT.println(K3,4);
}

/*
cs = calibration start/stop
cn = calibrate next (three edges and vertex)

p+ or p- = increase/decrease the P value of the PID controller
p 123.45 = set P value of the PID controller to 123.45
same for i and d
*/

int BTTuning() {
  if (!SerialBT.available())  return 0;
  //delay(1);
  char param = SerialBT.read();               // get parameter byte
  if (!SerialBT.available()) return 0;
  char cmd = SerialBT.read();                 // get command byte
  //SerialBT.flush();
  switch (param) {
    case 'p':
      if (cmd == '+')    K1 += 1;
      if (cmd == '-')    K1 -= 1;
      if (cmd == ' ') {
        K1 = SerialBT.parseFloat();
      }
      printValues();
      break;
    case 'i':
      if (cmd == '+')    K2 += 0.05;
      if (cmd == '-')    K2 -= 0.05;
      if (cmd == ' ') {
        K2 = SerialBT.parseFloat();
      }
      printValues();
      break;
    case 'd':
      if (cmd == '+')    K3 += 0.005;
      if (cmd == '-')    K3 -= 0.005;
      if (cmd == ' ') {
        K3 = SerialBT.parseFloat();
      }
      printValues();
      break; 
    case 'c':

      if (cmd == 's') {
        if (calibrating) {
          calibrating = false;
          SerialBT.println("Calibration mode stopped.");
        } else {
          calibrating = true;
          SerialBT.println("Calibration mode started. Balance the cube on any of it's three edges or vertex.");
          SerialBT.println("Input \"cn\" when the cube is balanced.");
        }
      }

      if (cmd == 'n' && calibrating)  {
        SerialBT.print("X: "); SerialBT.print(robot_angleX); SerialBT.print(" Y: "); SerialBT.println(robot_angleY);
        if (abs(robot_angleX) < 10 && abs(robot_angleY) < 10) {
          offsets.ID1 = 99;
          offsets.X1 = robot_angleX;
          offsets.Y1 = robot_angleY;
          SerialBT.println("Vertex OK. ");
          save();
        } else if (robot_angleX > -45 && robot_angleX < -25 && robot_angleY > -30 && robot_angleY < -10) {
          offsets.ID2 = 99;
          offsets.X2 = robot_angleX;
          offsets.Y2 = robot_angleY;
          SerialBT.println("First edge OK.");
          save();
        } else if (robot_angleX > 20 && robot_angleX < 40 && robot_angleY > -30 && robot_angleY < -10) {
          offsets.ID3 = 99;
          offsets.X3 = robot_angleX;
          offsets.Y3 = robot_angleY;
          SerialBT.println("Second edge OK.");
          save();
        } else if (abs(robot_angleX) < 15 && robot_angleY > 30 && robot_angleY < 50) {
          offsets.ID4 = 99;
          offsets.X4 = robot_angleX;
          offsets.Y4 = robot_angleY;
          SerialBT.println("Third edge OK.");
          save();
        } else {
          SerialBT.println("Angles out of range! Are you sure this is the right edge?");
          beep();
          beep();
        }
      }
      break;              
   }
   return 1;
}