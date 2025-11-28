/**
 * @file rplidar_test.ino
 * @brief RPLIDAR interface test.
 *
 * This sketch demonstrates how to interface with an RPLIDAR A1/A2 using the RPLidar library.
 * It connects to the LIDAR via Serial1, controls the motor speed via PWM, and prints
 * distance, angle, and quality metrics of scanned points to Serial.
 */

#include <RPLidar.h>

// You need to create an driver instance
RPLidar lidar;

#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor. 
// This pin should connected with the RPLIDAR's MOTOCTRL signal

/**
 * @brief Setup function.
 *
 * Initializes Serial (debug) and Serial1 (LIDAR).
 * Configures motor pin and starts scanning.
 */
void setup() {
  Serial.begin(9600);
  Serial.println("satart");
  // bind the RPLIDAR driver to the arduino hardware serial
  lidar.begin(Serial1);

  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  lidar.startScan();
  analogWrite(RPLIDAR_MOTOR, 0);
}

/**
 * @brief Main loop.
 *
 * Waits for a data point from the LIDAR.
 * If a valid point is received, prints its properties.
 * If not, stops the motor and attempts to reconnect/restart scanning.
 */
void loop() {
  if (IS_OK(lidar.waitPoint())) {
    Serial.println("in IS_OK");

    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle = lidar.getCurrentPoint().angle; //anglue value in degree
    bool startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte quality = lidar.getCurrentPoint().quality; //quality of the current measurement

    Serial.print("distance=");
    Serial.println(distance);
    Serial.print("angle=");
    Serial.println(angle);
    Serial.print("startBit=");
    Serial.println(startBit);
    Serial.print("quality=");
    Serial.println(quality);

    //perform data processing here...


  } else {
    Serial.println("Out");
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor

    // try to detect RPLIDAR...
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
      // detected...
      lidar.startScan();

      // start motor rotating at max allowed speed
      analogWrite(RPLIDAR_MOTOR, 255);
      delay(1000);
    }
  }
}
