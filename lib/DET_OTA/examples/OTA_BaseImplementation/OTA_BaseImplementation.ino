// This demo is a bit overloaded... read the .h file for instructions.

#define _SMARTDEBUG 
#include <smartdebug.h>

String wLanHostName="ESPOTA-";

#define wifiConnectTimeout 20000

ADC_MODE(ADC_VCC);

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

ESP8266WiFiMulti wifiMulti;

#define ledpin LED_BUILTIN

#include <DETOTA.h>

void setup(){
	dinit(115200);
  dprintln("\n\nESP8266/85 OTA Server");

  pinMode(ledpin,OUTPUT);
  digitalWrite(ledpin,true);

  wLanHostName+=String(ESP.getChipId(),HEX);
  WiFi.mode(WIFI_STA);

  wifiMulti.addAP("Det24","valeria2000");
  wifiMulti.addAP("FBMV UMT","JgRV2#ZsyPX3");
  wifiMulti.addAP("UMTIOT","88888888");

  WiFi.persistent(false); 

  unsigned long wifiStartConnectionAttempt=millis()+wifiConnectTimeout;

  dprint("[WiFi] Trying to connect");
  
  while(wifiMulti.run()!=WL_CONNECTED && wifiStartConnectionAttempt>millis()){
    dprint(".");
    delay(500);
  }

  dprintln("");

  if(WiFi.status()==WL_CONNECTED){
    dprint("[WiFi] Connected to ");
    dprintln(WiFi.SSID());
    WiFi.setAutoReconnect(true);
  }else{
    dprintln("[WiFi] Cound not connect to any predefined WiFi - OTA is not available!!!");
  }

  dprint("IP address: ");
  dprintln(WiFi.localIP());
  dprint("This Unit's name: ");
  dprintln(wLanHostName);

  dprint("\nMCU is running at ");
  dprint(ESP.getCpuFreqMHz());
  dprintln("MHz");

  dprint("Free Heap: ");
  dprint(ESP.getFreeHeap());
  dprint(" Bytes\nHeap Frag: ");
  dprint(ESP.getHeapFragmentation());
  
  dprint("%\n\nFlash Chip ID: ");
  dprintln(ESP.getFlashChipId());

  dprint("Flash Chip Speed: ");
  dprint(ESP.getFlashChipSpeed());

  dprint(" Hz\nFlash Chip Size, as seen by the SDK: ");
  dprint(ESP.getFlashChipSize());
  dprint(" Bytes\nFlash Chip size, as reported by the chip itself: ");
  dprint(ESP.getFlashChipRealSize());

  dprint(" Bytes\nFlash Configuration: ");

  if(ESP.getFlashChipRealSize()!=ESP.getFlashChipSize()){
    dprintln("WRONG!!!");
  }else{
    dprintln("OK :)");
  }

  dprint("Flash Mode: ");

  switch(ESP.getFlashChipMode()){
    case FM_QIO:
      dprintln("QIO");
      break;
    case FM_QOUT:
      dprintln("QOUT");
      break;
    case FM_DIO:
      dprintln("DIO");
      break;
    case FM_DOUT:
      dprintln("DOUT");
      break;
    default:
      dprintln("UNKNOWN");
  }

  dprint("\nSketch size: ");
  dprint(ESP.getSketchSize());
  dprint(" Bytes\nFree space: ");
  dprint(ESP.getFreeSketchSpace());
  
  dprint(" Bytes\n\nMCU Voltage: ");
  dprintln(String(ESP.getVcc()/1000.0));

	OTAinit("Stinky"); // leave empty for a default name

}

#define blinkHiTime 10
#define blinkLoTime 100
#define blinkPauseTime 1200
#define blinkCount 3
byte blinkCounter=0;
unsigned long blinkLastTime;
int blinkfreq=blinkHiTime;
bool blinkInvert=false;

void blinkystuff(){
  if((unsigned long)(millis()-blinkLastTime)>=blinkfreq){
    blinkLastTime=millis();
    if(digitalRead(LED_BUILTIN)){
      digitalWrite(LED_BUILTIN,LOW^blinkInvert);
      blinkfreq=blinkHiTime;
    }else{
      digitalWrite(LED_BUILTIN,HIGH^blinkInvert);
      blinkfreq=blinkLoTime;
      blinkCounter++;
    }
    if(blinkCounter>=blinkCount){
      blinkCounter=0;
      blinkfreq=blinkPauseTime;
    }
  }
}

void loop(){
  ArduinoOTA.handle(); // needed!
  blinkystuff();
}