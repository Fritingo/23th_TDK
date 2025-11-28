/**
 * @file ic595.ino
 * @brief 7-segment display counter using 74HC595 shift register.
 *
 * This sketch demonstrates how to control a 7-segment display using a 74HC595 shift register.
 * It iterates through numbers 0-9 and letters A-F.
 */

/* 
    使用 74HC595 與七段顯示器製作倒數功能
    1 = LED on, 0 = LED off, in this order:
    74HC595 pin     Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7
    Mapping to      a,b,c,d,e,f,g of Seven-Segment LED
 */

const byte dataPin  = 2;
const byte latchPin = 4;
const byte clockPin = 7;

byte seven_seg_digits[17] = { B00000001,  // = "."
                              B11111100,  // = 0
                              B01100000,  // = 1
                              B11011010,  // = 2
                              B11110010,  // = 3
                              B01100110,  // = 4
                              B10110110,  // = 5
                              B10111110,  // = 6
                              B11100100,  // = 7
                              B11111110,  // = 8
                              B11110110,  // = 9
                              B11101110,  // = A
                              B00111110,  // = b
                              B00011010,  // = c
                              B01111010,  // = d
                              B10011110,  // = E
                              B10001110  // = F
                            };

/**
 * @brief Setup function.
 *
 * Configures the control pins (data, latch, clock) as outputs.
 */
void setup() {
  // 將 latchPin, clockPin, dataPin 設置為輸出
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
}

/**
 * @brief Displays a digit/character on the 7-segment display.
 *
 * Shifts out the bit pattern corresponding to the given index in `seven_seg_digits`.
 *
 * @param digit Index of the digit/character to display (0-16).
 */
void sevenSegWrite(byte digit) {
  // 送資料前要先把 latchPin 拉成低電位
  digitalWrite(latchPin, LOW);

  // 送出數字的位元資料 (bit pattern)
  shiftOut(dataPin, clockPin, LSBFIRST, seven_seg_digits[digit]);

  // 送完資料後要把 latchPin 拉回成高電位
  digitalWrite(latchPin, HIGH);
}

/**
 * @brief Main loop.
 *
 * Cycles through the `seven_seg_digits` array, displaying each for 500ms.
 */
void loop() {
  // 依陣列順序顯示
  for (byte digit = 1; digit < 17; digit++) {
    delay(500);
    sevenSegWrite(digit);
  }
}
