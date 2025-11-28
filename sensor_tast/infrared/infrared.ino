/**
 * @file infrared.ino
 * @brief Basic IR remote signal decoder.
 *
 * This sketch receives infrared signals via an IR receiver module, decodes them,
 * and prints the raw hex value, bit length, and decode type to the Serial Monitor.
 */

#include <IRremote.h>

int RECV_PIN = 12; // 使用數位腳位2接收紅外線訊號
IRrecv irrecv(RECV_PIN); // 初始化紅外線訊號輸入
decode_results results; // 儲存訊號的結構

/**
 * @brief Setup function.
 *
 * Initializes Serial and starts the IR receiver.
 */
void setup()
{
  Serial.begin(115200);
  irrecv.blink13(true); // 設為true的話，當收到訊號時，腳位13的LED便會閃爍
  irrecv.enableIRIn(); // 啟動接收
}

/**
 * @brief Main loop.
 *
 * Checks for decoded IR signals and prints their details.
 */
void loop() {
  if (irrecv.decode(&results)) { // 接收紅外線訊號並解碼
    Serial.print("results value is "); // 輸出解碼後的資料
    Serial.print(results.value, HEX);//0:FF6897/1:FF6897/2:FF18E7/3:FF7A85/4:FF10EF/5:FF38C7/6:FF5AA5/7:FF42BD/8:FF4AB5/9:FF52AD
    Serial.print(", bits is ");
    Serial.print(results.bits);
    Serial.print(", decode_type is ");
    Serial.println(results.decode_type);
    irrecv.resume(); // 準備接收下一個訊號
  }
}
