#include <M5Stack.h>

#define HIGHT_MAX 240
#define WIDTH_MAX 320

unsigned int guiBtnA_Cnt;
unsigned int guiBtnB_Cnt;
unsigned int guiBtnC_Cnt;

void setup() {
  // M5Stackの初期化
  M5.begin();

  // 初期化
  guiBtnA_Cnt = 0;
  guiBtnB_Cnt = 0;
  guiBtnC_Cnt = 0;
  // 文字サイズを変更
  M5.Lcd.setTextSize(2);
  // Aボタン　カウンタ表示
  M5.Lcd.setCursor(50,50);
  M5.Lcd.printf("Button A : %d",guiBtnA_Cnt);
  // Bボタン　カウンタ表示
  M5.Lcd.setCursor(50,100);
  M5.Lcd.printf("Button B : %d",guiBtnB_Cnt);
  // Cボタン　カウンタ表示
  M5.Lcd.setCursor(50,150);
  M5.Lcd.printf("Button C : %d",guiBtnC_Cnt);
}

void loop() {

    M5.update();
    // Aボタン
    if (M5.BtnA.wasPressed()) {
      // カウンタ更新
      guiBtnA_Cnt++;
      // カウンタ表示
      M5.Lcd.setCursor(50,50);
      M5.Lcd.printf("Button A : %d",guiBtnA_Cnt);
    }
    // Bボタン
    if (M5.BtnB.wasPressed()) {
      // カウンタ更新
      guiBtnB_Cnt++;
      // カウンタ表示
      M5.Lcd.setCursor(50,100);
      M5.Lcd.printf("Button B : %d",guiBtnB_Cnt);
    }
    // Cボタン
    if (M5.BtnC.wasPressed()) {
      // カウンタ更新
      guiBtnC_Cnt++;
      // カウンタ表示
      M5.Lcd.setCursor(50,150);
      M5.Lcd.printf("Button C : %d",guiBtnC_Cnt);
    }
}
