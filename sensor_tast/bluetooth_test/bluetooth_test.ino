/**
 * @file bluetooth_test.ino
 * @brief Simple Bluetooth serial communication test.
 *
 * This sketch initializes serial communication on Serial (debug) and Serial1 (Bluetooth module).
 * It listens for incoming data on Serial1 and prints it to Serial.
 */

int command = 0;

/**
 * @brief Setup function.
 *
 * Initializes Serial (115200 baud) and Serial1 (9600 baud).
 */
void setup() {
 

 Serial.begin(115200);
  Serial1.begin(9600);
   Serial.print("a");
}

/**
 * @brief Main loop.
 *
 * Relays data received from Serial1 to Serial.
 */
void loop() {

  if ( Serial1.available() > 0)   {
    Serial.print(Serial1.read());
//    pidtest_time = millis();
    command = Serial1.read();// '85117/\''68100\/''76108<''82114>'

  }


}
