#define WiFiName "Det24"
#define WiFiPass "valeria2000"

#include <ESP8266WiFi.h>

#define WebDebugHTTPServerAdress 8888 /* leave empty for 80 */

#include <webdebug.h>

void setup(){
  Serial.begin(115200);
  delay(200);
  Serial.println("\n\nESP8266 WebDebug Demo\n");

	WiFi.begin(WiFiName,WiFiPass);
	
	Serial.print("[SETUP] Connecting to WiFi");

  while(WiFi.status()!=WL_CONNECTED){
    delay(300);
		Serial.print(".");
  }
	
	Serial.println(" done :)");
	
	String debugURL=wprintInit("dybukk"); // initialisation - with a subpage. leave empty for root.

  Serial.println("\n[SETUP] Open this for the debug window: "+debugURL);
}

void loop(){
  static uint8_t countUp=0;
	
	if(millis()%2000==0){
		wprintln(String(++countUp));
	}
	
  wprintLoop(); // services
}