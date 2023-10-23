#define wlanhostname "WebDebugDemo"

#define WiFiName "Det24"
#define WiFiPass "valeria2000"

#define _SMARTDEBUG
#include <smartdebug.h>

#include <ESP8266WiFi.h>

// #define WebDebugOff /* this disables the WebDebugging - for final code :)*/
// Sketch: 304440 // Vars: 29828 -- WebDebug enabled
// Sketch: 267240 // Vars: 27336 -- WebDebug disabled

#define WebDebugHTTPServerAdress 8888 /* leave empty for 80 */
#include <webdebug.h>

void setup(){
  dinit(115200);
  delay(200);
  dprintln("\n\nESP8266 WebDebug Demo\n");

  WiFi.mode(WIFI_STA);
  WiFi.hostname(wlanhostname);
  WiFi.setOutputPower(20.5); // MAX
  WiFi.setPhyMode(WIFI_PHY_MODE_11N);
  WiFi.setAutoReconnect(true);

	WiFi.begin(WiFiName,WiFiPass);
	
	dprint("[SETUP] Connecting to WiFi");

  while(WiFi.status()!=WL_CONNECTED){
    delay(300);
		dprint(".");
  }
	
	dprintln(" done :)");
	
	String debugURL=wprintInit("dybukk"); // initialisation - with a subpage. leave empty for root.

  dprintln("[SETUP] Open this for the debug window: "+debugURL);

	dprintln("\n[SETUP] done.");
	
}

void AsyncTask(uint16_t interval=1000){
  static unsigned long AsyncTaskLastTime=0;

  if((unsigned long)(millis()-AsyncTaskLastTime)<interval){
    return;
  }

  static uint8_t countUp=0;
  
  wprintln(String(++countUp));

  AsyncTaskLastTime=millis();

}

void loop(){
  AsyncTask();  
  wprintLoop(); // services
}