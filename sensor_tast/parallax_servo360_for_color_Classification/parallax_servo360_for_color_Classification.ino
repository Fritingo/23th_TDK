/**
 * @file parallax_servo360_for_color_Classification.ino
 * @brief Control sketch for Parallax 360 continuous rotation servo.
 *
 * This sketch controls a continuous rotation servo used for a color classification disk mechanism.
 * It uses a limit switch (pin 7) to control the rotation cycles.
 */

/* 用改裝後的伺服機PARALLAX servo 搭配極限開關，用以分類顏色之圓盤用*/
#include <Servo.h>

Servo myservo;

/**
 * @brief Setup function.
 *
 * Attaches servo to pin 3 with specific pulse width range.
 * Configures limit switch pin (7) as output (Wait, likely INPUT based on usage in loop, checking now...
 * The code says `pinMode(7, OUTPUT);` but reads it with `digitalRead(7)`. This is odd.
 * If it's a switch, it should be INPUT or INPUT_PULLUP. OUTPUT mode read returns the set state.
 * Assuming the code is "as is", I will document it as is, but it looks like a bug or specific wiring).
 */
void setup()
{
  Serial.begin(9600);
  myservo.attach(3, 750, 2250); // 修正脈衝寬度範圍
  pinMode(7, OUTPUT);
}

/**
 * @brief Main loop.
 *
 * Rotates the servo while pin 7 reads LOW.
 * Stops the servo (writes 1488 microseconds) when pin 7 goes HIGH.
 * Delays, then rotates again while pin 7 reads HIGH.
 *
 * Note: Since pin 7 is set to OUTPUT in setup, digitalRead will return the last written value (LOW by default).
 * Unless the hardware pulls this pin or it's wired strangely, this logic might need review.
 */
void loop()
{
  while (!digitalRead(7)) {
    myservo.write(180); // run
  }
  myservo.writeMicroseconds(1488); // stop
  delay(1000);
  while (digitalRead(7)) {
    myservo.write(180); // run
  }
}
