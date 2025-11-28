/**
 * @file relay_test.ino
 * @brief Relay control test sketch.
 *
 * This sketch cycles through activating three different pins connected to relays
 * (likely for ball collection, pulling up, and shooting mechanisms).
 */

const int collect_ball_pin = 35;
const int pullup_ball_pin = 36;
const int shot_ball_pin = 34;

/**
 * @brief Setup function.
 *
 * Configures the relay control pins as outputs.
 */
void setup() {
  pinMode(collect_ball_pin, OUTPUT);
  pinMode(pullup_ball_pin, OUTPUT);
  pinMode(shot_ball_pin, OUTPUT);
}

/**
 * @brief Main loop.
 *
 * Sequentially activates each relay for 2 seconds, then turns it off.
 */
void loop() {
  digitalWrite(collect_ball_pin, HIGH);
  delay(2000);
  digitalWrite(collect_ball_pin, LOW);
  digitalWrite(pullup_ball_pin, HIGH);
  delay(2000);
  digitalWrite(pullup_ball_pin, LOW);
  digitalWrite(shot_ball_pin, HIGH);
  delay(2000);
  digitalWrite(shot_ball_pin, LOW);
//  digitalWrite(pullup_ball_pin, HIGH);
//  delay(2000);

}
