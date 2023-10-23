#ifndef ESP8266oprintWasHere
#define ESP8266oprintWasHere

void oprint(String OLEDStr="",bool dontScroll=LOW){

  static byte OLEDCurrentLine=0;
  static byte OLEDStartPrintAtThisLine=0;
  static bool OLEDscrollEnable=LOW;

  byte OLEDLineHeight=u8g2.getMaxCharHeight()-2;
  byte OLEDMaxLines=u8g2.getDisplayHeight()/OLEDLineHeight;

  static const byte OLEDBufSize=10;
  static String OLEDBuf[OLEDBufSize]; // not sexy - but needs a constant to init :(

  if(OLEDStr=="cls" || OLEDStr=="clear" || OLEDStr=="clr"){
    OLEDCurrentLine=0;
    OLEDStartPrintAtThisLine=0;
    OLEDscrollEnable=LOW;
    for(byte t=0;t<OLEDBufSize;t++){
      OLEDBuf[t]="";
    }
    u8g2.clearBuffer();
    u8g2.sendBuffer();
    return;
  }

  if(dontScroll){
    OLEDBuf[OLEDCurrentLine]=OLEDStr;
  }else{
    OLEDBuf[OLEDCurrentLine++]=OLEDStr;
  }

  u8g2.clearBuffer();
  for(byte t=0;t<OLEDMaxLines;t++){
    byte OLEDLineTemp=OLEDStartPrintAtThisLine+t;
    if(OLEDLineTemp>=OLEDMaxLines){
      OLEDLineTemp-=OLEDMaxLines;
    }
    u8g2.drawStr(0,OLEDLineHeight*(t+1),OLEDBuf[OLEDLineTemp].c_str());
  }
  u8g2.sendBuffer();

  if(dontScroll){
    return;
  }

  if(OLEDCurrentLine>=OLEDMaxLines){
    OLEDCurrentLine=0;
    OLEDscrollEnable=HIGH;
  }

  if(OLEDscrollEnable){
    OLEDStartPrintAtThisLine++;
  }
  
  if(OLEDStartPrintAtThisLine>=OLEDMaxLines){
    OLEDStartPrintAtThisLine=0;
  }
}

#endif
