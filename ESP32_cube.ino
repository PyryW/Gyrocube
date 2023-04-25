#include "ESP32.h"
#include <Wire.h>
#include <EEPROM.h>
#include "BluetoothSerial.h"
#include "driver/ledc.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Gyrocube");
  EEPROM.begin(EEPROM_SIZE);

  pinMode(BUZZER, OUTPUT);
  
  pinMode(BRAKE, OUTPUT);
  pinMode(ENA, OUTPUT);
  digitalWrite(BRAKE, HIGH);
  digitalWrite(ENA, HIGH);
  
  pinMode(DIR1, OUTPUT);
  ledcSetup(PWM1_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM1, PWM1_CH);
  Motor1_control(0);
  
  pinMode(DIR2, OUTPUT);
  ledcSetup(PWM2_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM2, PWM2_CH);
  Motor2_control(0);
  
  pinMode(DIR3, OUTPUT);
  ledcSetup(PWM3_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM3, PWM3_CH);
  Motor3_control(0);

  // Check if the balancing points have been previously calibrated
  EEPROM.get(0, offsets);
  if (offsets.ID1 == 99 && offsets.ID2 == 99 && offsets.ID3 == 99 && offsets.ID4 == 99) {
    calibrated = true;
  } else {
    calibrated = false;
  }
    
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);

  angle_setup();
}

void loop() {

  currentT = millis();

  // Run the loop with a period of loop_time, which is set to 10ms in ESP32.h 
  if (currentT - previousT_1 >= loop_time) {

    // Check if commands have been received over the Bluetooth serial console
    BTTuning();
    
    // Poll gyroscope for the current angle, and calculate the rotation of the cube
    angle_calc();
    // angle_calc sets balancing_point based on how the cube is oriented. There's four possibilities, as the cube can be balanced on
    // three different edges, or the vertex
    if (balancing_point == 1) {
      angleX -= offsets.X1;
      angleY -= offsets.Y1;
      if (abs(angleX) > 8 || abs(angleY) > 8) { //If the cube deviates from the balancing point too much, it's considered unrecoverable and motors are stopped.
        if (vertical) SerialBT.println("Lost balance!");
        vertical = false; // Used for keeping track of whether the cube is in balancing mode or has fallen over. Motors are deactivated if vertical==false
      }
    } else if (balancing_point == 2) {
      angleX -= offsets.X2;
      angleY -= offsets.Y2;
      if (abs(angleY) > 5) {
        if (vertical) SerialBT.println("Lost balance!");
        vertical = false;
      }
    } else if (balancing_point == 3) {
      angleX -= offsets.X3;
      angleY -= offsets.Y3;
      if (abs(angleY) > 5) {
        if (vertical) SerialBT.println("Lost balance!");
        vertical = false;
      }
    } else if (balancing_point == 4) {
      angleX -= offsets.X4;
      angleY -= offsets.Y4;
      if (abs(angleX) > 5) {
        if (vertical) SerialBT.println("Lost balance!");
        vertical = false;
      }
    }
    
    // Angle calculation is based on acceleration and actual angle (accelerometer + gyroscope)
    // Gyro_amount changes how much weight the gyroscope is given in the calculation
    // when set to nearly 1, the accelerometer is nearly ignored
    // But if the cube is far from the balancing point, but traveling fast towards it, 
    // then the accelerometer is given much more weight, as the angle will soon be correct
    // and motors should be slowed down.
    if (abs(angleX) < 8 || abs(angleY) < 8) {  // fast restore angle
      Gyro_amount = 0.996; 
    } else 
      Gyro_amount = 0.1;

    if (vertical && calibrated && !calibrating) {    
      digitalWrite(BRAKE, HIGH);
      digitalWrite(ENA, HIGH);
      gyroZ = GyZ / 131.0; // Convert to deg/s
      gyroY = GyY / 131.0; // Convert to deg/s
      gyroX = GyX / 131.0; // Convert to deg/s

      gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;
      gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;
      
      int pwm_X = constrain(K1 * angleX + K2 * gyroZfilt + K3 * motor_speed_X, -maxspeed, maxspeed);
      int pwm_Y = constrain(K1 * angleY + K2 * gyroYfilt + K3 * motor_speed_Y, -maxspeed, maxspeed);
      motor_speed_X += pwm_X; 
      motor_speed_Y += pwm_Y;
      
      if (balancing_point == 1) {
        XY_to_threeWay(-pwm_X, -pwm_Y);
      } else if (balancing_point == 2) {
        Motor1_control(pwm_Y);
      } else if (balancing_point == 3) {
        Motor2_control(-pwm_Y);
      } else if (balancing_point == 4) {
        Motor3_control(pwm_X);
      }
    } else {
      //Disable motors if balance is lost
      XY_to_threeWay(0, 0);
      digitalWrite(BRAKE, LOW);
      digitalWrite(ENA, LOW);
      motor_speed_X = 0;
      motor_speed_Y = 0;
    }
    previousT_1 = currentT;
  }
  
  if (currentT - previousT_2 >= 2000) {    
    if (!calibrated && !calibrating) {
      SerialBT.println("No calibration! \"cs\" to start calibration");
    }
    previousT_2 = currentT;
  }
}
