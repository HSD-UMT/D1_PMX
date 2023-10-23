#define wifiConnectTimeout 12e3

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

ESP8266WiFiMulti wifiMulti;

#include <rBase64.h> // Base64 support

String WifiDecoder(String encodedStuff){
  String str="";
  
  for(uint8_t t=0;t<encodedStuff.length();t++){
    uint8_t c=encodedStuff.charAt(t);
    if(t%2==0){
      c--;
    }else{
      c++;
    }
    str+=char(c);
  }
  rbase64.decode(str);
  str=rbase64.result();
  return str;
}

void wifiMultiInit(String wifiInit_hostname="ESP"){
	wifiInit_hostname+=" "+String(ESP.getChipId(),HEX);
	
	#ifdef _SMARTDEBUG
		dprintln("\n[wifiInit] -- START --");
	#endif

  WiFi.hostname(wifiInit_hostname);

  WiFi.setOutputPower(20.5); // MAX
  WiFi.setPhyMode(WIFI_PHY_MODE_11N);
	
	WiFi.mode(WIFI_STA);

  #ifdef _SMARTDEBUG
  	dprintln("\n[wifiInit] Known WLANs:");
  #endif

  uint16_t n=sizeof(wlans)/sizeof(wlans[0]);

  for(uint16_t t=0;t<n;t++){
    String s=WifiDecoder(String(wlans[t].ssid));
    String p=WifiDecoder(String(wlans[t].pass));
    #ifdef _SMARTDEBUG
      dprintln(String(t)+": SSID: ["+s+"] Pass: ["+p+"]");
    #endif
		wifiMulti.addAP(s.c_str(),p.c_str()); // not pretty, but gets the job done.
  }

  #ifdef _SMARTDEBUG
    dprintln("\nObviously, remove this debug info for production :)\n");
  #endif

	WiFi.persistent(false); // should speed up the connection ;)

	#ifdef _SMARTDEBUG
		dprint("[wifiInit] Trying to connect");
	#endif

  unsigned long wifiStartConnectionAttempt=millis();
  
  while(wifiMulti.run()!=WL_CONNECTED && wifiStartConnectionAttempt+wifiConnectTimeout>millis()){
    #ifdef _SMARTDEBUG
			dprint(".");
			delay(500);
		#endif
    yield();
  }

  unsigned long wifiConnectDuration=millis()-wifiStartConnectionAttempt;

  #ifdef _SMARTDEBUG
		dprintln("");
	#endif

  if(WiFi.status()==WL_CONNECTED){
		#ifdef _SMARTDEBUG
			dprint("[wifiInit] We're connected!\n[wifiInit] Hostname for this device: ");
			dprintln(wifiInit_hostname);
			dprint("[wifiInit] Connected to this Wifi: ");
			dprintln(WiFi.SSID());
			dprint("[wifiInit] IP: ");
			dprintln(WiFi.localIP());
      dprint("[wifiInit] Connection Attempt took ");
      dprint(String((float)wifiConnectDuration/1000.0,1));
      dprintln("s.");
		#endif
	
    WiFi.setAutoReconnect(true);
  }else{
		#ifdef _SMARTDEBUG
			dprintln("[wifiInit] Cound not connect to any predefined WiFi :( ");
		#endif
  }

  #ifdef _SMARTDEBUG
    dprintln("[wifiInit] -- DONE -- \n");
  #endif

}
