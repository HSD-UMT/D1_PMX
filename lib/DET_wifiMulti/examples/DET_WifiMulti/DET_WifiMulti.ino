#define _SMARTDEBUG /* if _SMARTDEBUG is defined, we get debug messages */
#include <smartdebug.h>

static const String wLanHostName="ThisIsSparta";

typedef struct{
  char ssid[48];
  char pass[48]; 
} ssidAndPass;

// This is obfuscation - not encryption. but I guess it helps to keep the script kiddies away :)
// Only put in those Wifis that are needed!
// More WiFi's seems to make the connection less successful

// Check Sketch'es folder for the encryption tool

ssidAndPass wlans[]{
	{"SFW/NiR<","elGr[WKoZSJvNCB<"},																			// Det24
	{"SjKMWhCUUUR<","TleRWiJiXmO4VEh<"},																	// FBMV UMT
	{"WT2TTT:T","PCh3PCh3PCh<"},																					// UMTIOT
	{"SmKkbV[0clt<",""},																									// Freifunk
	{"[FW/dxCmZWO/","NCBvNCBvNCB<"},																			// dets gast
	{"WF:rcFWyJF6keVWyJDigeWN<","UVWoclVfeF:rcFVfSmKoeGqhc2hfNiBwOhF<"},	// Tolles neues Haus
	{"R1ixbWOlc15<","fmS3ciNwPSh<"},																			// Chrisfon
  {"[yd<","PCh3PCh3PCh<"}                                               // Det's Fon
};

#include <DET_WifiMulti.h>

#include <TZ.h> // for NTP Time

void setup(){
	dinit(115200);
  delay(200); // wait for the serial monitor...
  dprintln("\n\nWifi Multi Demo -- START --");
	
	wifiMultiInit(wLanHostName); // that's it :)

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
	
	// NTP -- check https://github.com/esp8266/Arduino/issues/7353
	configTime(TZ_Europe_Berlin,"pool.ntp.org"); // Initializing NTP Time

}

void ntpTimeDebug(){

  static time_t oldTime=0;
  
  time_t now=time(nullptr);

  if(oldTime==now){
    return;  
  }

  oldTime=now;
  
  struct tm LocTime;
  localtime_r(&now,&LocTime);

  Serial.print(LocTime.tm_hour);
  Serial.print(":");
  Serial.print(LocTime.tm_min);
  Serial.print(":");
  Serial.print(LocTime.tm_sec);
  Serial.print(" ");
  Serial.print(LocTime.tm_mday);
  Serial.print(".");
  Serial.print(LocTime.tm_mon+1);
  Serial.print(".");
  Serial.println(LocTime.tm_year+1900);
}

void blinkTheLed(uint16_t interval=250){

	static unsigned long LastTime=0;

	if((unsigned long)(millis()-LastTime)<interval){
    return;
  }

  LastTime=millis();

  digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));

}

void loop(){
	blinkTheLed(333);
  ntpTimeDebug();
}
