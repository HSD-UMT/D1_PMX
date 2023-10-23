/*
  oprint -- OLED PRINT for ESP8266/u8g2

  Usage:
  oprint("Only Strings");
  oprint(String(ImAmAnINT));
  oprint("This Line won't scroll",1);
  oprint(); << prints a empty line
  
  commands:
  cls OR clear OR clr >> clear Screen (resets scrolling)

  check LOOP for demo functions!!

*/

#include <U8g2lib.h>
U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2(U8G2_R0,U8X8_PIN_NONE); // Wemos D1 Shield

#include <DET_oprint.h>

void setup(){

  u8g2.setBusClock(1000000);
  u8g2.begin();

  u8g2.enableUTF8Print();

  u8g2.setFont(u8g2_font_profont12_tf);

  // u8g2.setFlipMode(true);

}

void justCountUp(){
  static byte r=0;
  oprint(String(r++)); // bytes flip over every 255 times
  delay(500);
}

void clearScreenSometimes(){
  static byte r=0;

  if(random(10)==5){
    oprint("clr"); // clears the screen... sometimes :)
  }
  oprint(String(r++));
  delay(500);
}

void dontScroll(){
  static bool firstrun=HIGH;

  if(firstrun){
    firstrun=LOW;
    oprint("Uploading");
  }
  
  static byte r=0;

  oprint(String(r+=5),1); // don't scroll - that's what the ",1" is for

  delay(500);
  
}

void loop(){
  // justCountUp();
  // dontScroll();
  clearScreenSometimes();
}
