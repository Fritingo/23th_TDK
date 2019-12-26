#include <Keypad.h>    // 引用Keypad程式庫

#define KEY_ROWS 4 // 按鍵模組的列數
#define KEY_COLS 4 // 按鍵模組的行數
int show_key = 17; // 讓顯示初始為seven_seg_digits[17] (不亮)
const byte dataPin  = 2;
const byte latchPin = 3;
const byte clockPin = 4;

// 依照行、列排列的按鍵字元（二維陣列）
char keymap[KEY_ROWS][KEY_COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'0', 'F', 'E', 'D'}
};

byte seven_seg_digits[18] = {
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
  B10001110,  // = F
  B00000001,  // = "."
  B00000000   // = ""
};

byte colPins[KEY_COLS] = {13, 12, 11, 10};     // 按鍵模組，行1~4接腳。
byte rowPins[KEY_ROWS] = {9, 8, 7, 6}; // 按鍵模組，列1~4接腳。

// 初始化Keypad物件。語法：Keypad(makeKeymap(按鍵字元的二維陣列), 模組列接腳, 模組行接腳, 模組列數, 模組行數)
Keypad myKeypad = Keypad(makeKeymap(keymap), rowPins, colPins, KEY_ROWS, KEY_COLS);

void setup() {
  Serial.begin(9600);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
}

void sevenSegWrite(byte digit) {
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, seven_seg_digits[digit]);
  digitalWrite(latchPin, HIGH);
}

void loop() {
  // 透過Keypad物件的getKey()方法讀取按鍵的字元
  int key = myKeypad.getKey();

  if (key) { // 若有按鍵被按下…
    Serial.println(char(key));  // 顯示按鍵的字元
    if (key <= 58)  //將ascii code 轉成對應數字陣列索引
      key -= 48;
    else if (key <= 70)
      key -= 55;
    show_key = key;
  }
  sevenSegWrite(show_key);
}
