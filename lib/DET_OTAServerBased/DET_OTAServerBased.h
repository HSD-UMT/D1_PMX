#define wifiConnectTimeout 12e3 /* Freifunk needs a bit more time... */

// Decides on defines of ESP8266oprintWasHere and _SMARTDEBUG
// if any debug should be done

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

ESP8266WiFiMulti wifiMulti;

bool OTAServerBased_WeDidTheConnect=LOW;

void OTAServerBased_wifiMultiConnect(){

	if(WiFi.status()==WL_CONNECTED){ // there's already a Wifi connection - no need to set up a nother one :)
		#ifdef _SMARTDEBUG
			dprintln("[OTAServerBased_wifiMultiConnect] Already connected :) ");
		#endif
		return;
	}
	
	// starting some wifi multi connection
	#ifdef _SMARTDEBUG
		dprintln("[OTAServerBased_wifiMultiConnect] Starting Wifi...");
	#endif

  WiFi.setOutputPower(20.5); // MAX
  WiFi.setPhyMode(WIFI_PHY_MODE_11N);
	
	WiFi.mode(WIFI_STA);

  wifiMulti.addAP("dets gast","00000000"); // works with any GUEST login!
	wifiMulti.addAP("FBMV UMT","JgRV2#ZsyPX3");
  wifiMulti.addAP("UMTIOT","88888888");
  wifiMulti.addAP("Freifunk","");

	unsigned long wifiStartConnectionAttempt=millis();
  
	#ifdef _SMARTDEBUG
		dprint("[OTAServerBased_wifiMultiConnect] Trying to connect");
	#endif

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
			dprint("[OTAServerBased_wifiMultiConnect] Connected to ");
			dprintln(WiFi.SSID());
			dprint("Connection Attempt took ");
			dprint(String((float)wifiConnectDuration/1000.0,1));
			dprintln("s.");
		#endif
		OTAServerBased_WeDidTheConnect=HIGH;
	}else{
		#ifdef _SMARTDEBUG
			dprint("[OTAServerBased_wifiMultiConnect] No connection to any predefined WiFi after ");
			dprint(String((float)wifiConnectDuration/1000.0,1));
			dprintln("s - OTA is not available!!!");
		#endif
	}
}

void OTAServerBased_disconnectWifi(){
	if(OTAServerBased_WeDidTheConnect==LOW){
		#ifdef _SMARTDEBUG
			dprintln("[OTAServerBased_disconnectWifi] Not our wifi connection - leave it running.\n");
		#endif
		return;
	}
	#ifdef _SMARTDEBUG
		dprintln("[OTAServerBased_disconnectWifi] Shutting down Wifi\n");
	#endif
  WiFi.disconnect(); //won't kill the wifi complete, but disconnects from any WLAN
}

void OTAServerBased_updateStarted(){
	#ifdef _SMARTDEBUG
		dprintln("[RunServerOTA] Update started!");
	#endif
}

void OTAServerBased_updateFinished(){
	#ifdef _SMARTDEBUG
		dprintln("[RunServerOTA] Update done -- rebooting.");
	#endif
	#ifdef ESP8266oprintWasHere
		oprint("Rebooting!");
	#endif
}

void OTAServerBased_updateProcess(int cur,int total){
	byte k=cur/(total/100);
	static byte lastk=0;
	if(k % 10==0 && lastk!=k){
		#ifdef _SMARTDEBUG
			dprint("[RunServerOTA] Process: ");
			dprint(String(k));
			dprintln("%");
		#endif
		
		#ifdef ESP8266oprintWasHere
			oprint("Update "+String(k)+"%",1);  
		#endif
		lastk=k;
	}
}

void OTAServerBased_updateError(int err){
	#ifdef _SMARTDEBUG
		dprint("[RunServerOTA] Error - that didn't work.\nError Code: ");
		dprintln(String(err));
	#endif
}

void RunServerOTA(){ // Here's where the fun starts :)

  #ifdef _SMARTDEBUG
		dprintln("\n[RunServerOTA] --START --");
	#endif

  OTAServerBased_wifiMultiConnect();

  if(WiFi.status()!=WL_CONNECTED){
		#ifdef _SMARTDEBUG
			dprintln("[RunServerOTA] no wifi connection - so no update.");
		#endif
    return;
  }

	#ifdef _SMARTDEBUG
		dprint("[RunServerOTA] Connected to ");
		dprint(WiFi.SSID());
		dprint(" (");
		dprint(WiFi.localIP());
		dprintln(") - yay!");

		dprint("[RunServerOTA] Connecting to ");
		dprint(OTAVersionFile);
		dprintln(" and fetching current version serverside");
	#endif

  WiFiClient client;
  HTTPClient http;

  http.begin(client,OTAVersionFile);
  
  int httpCode=http.GET();

  #ifdef _SMARTDEBUG
		dprint("[RunServerOTA] http return code: ");
		dprintln(String(httpCode));

		dprint("[RunServerOTA] Local Firmware Version: ");
		dprintln(String(FW_VERSION));
	#endif

  if(httpCode!=200){
		#ifdef _SMARTDEBUG
			dprint("[RunServerOTA] ERROR: can't read version file. Aborting.");
		#endif
    OTAServerBased_disconnectWifi();
    return;
  }

  String newFWVersion=http.getString();

  #ifdef _SMARTDEBUG
		dprint("[RunServerOTA] FW Version serverside: ");
		dprintln(String(newFWVersion));
	#endif
	
	http.end();

  int ServerSideVersion=newFWVersion.toInt();

  if(OTAOnlyNewerVersions){
    if(ServerSideVersion<=FW_VERSION){ // only goes for newer versions
			#ifdef _SMARTDEBUG
				dprintln("[RunServerOTA] We're already running the most current firmware");
			#endif
      OTAServerBased_disconnectWifi();
      return;
    }
  }else{
    if(ServerSideVersion==FW_VERSION){ // possibility to downgrade
      #ifdef _SMARTDEBUG
				dprintln("[RunServerOTA] We're running the Server-prefered firmware.");
			#endif
      OTAServerBased_disconnectWifi();
      return;
    }
  }
	
	#ifdef ESP8266oprintWasHere
		u8g2.setFont(u8g2_font_chikita_tf);
		oprint("clr");
		oprint("FW UPDATE!");
		oprint("Local: "+String(FW_VERSION));  
		oprint("Server: "+newFWVersion);
	#endif

  #ifdef _SMARTDEBUG
		dprintln("[RunServerOTA] NEW FIRMWARE found serverside! Starting Update process.");
	#endif

  String ServerSideFirmwareURL=OTABaseURL+OTAChannel+newFWVersion+".bin";

  #ifdef _SMARTDEBUG
		dprint("[RunServerOTA] Serverside Firmware URL: ");
		dprintln(ServerSideFirmwareURL);

		dprintln("\n[RunServerOTA] UPDATING -- just a moment, please :) \n");
	#endif
	
	ESPhttpUpdate.onStart(OTAServerBased_updateStarted);
  ESPhttpUpdate.onEnd(OTAServerBased_updateFinished);
  ESPhttpUpdate.onProgress(OTAServerBased_updateProcess);
	ESPhttpUpdate.onError(OTAServerBased_updateError);

	#ifdef _SMARTDEBUG
		t_httpUpdate_return ret=ESPhttpUpdate.update(ServerSideFirmwareURL);

		switch(ret){
			case HTTP_UPDATE_FAILED:
				dprint("[RunServerOTA] HTTP Update failed Error: ");
				dprint(String(ESPhttpUpdate.getLastError()));
				dprint(": ");
				dprintln(ESPhttpUpdate.getLastErrorString());
				break;

			case HTTP_UPDATE_NO_UPDATES:
				dprint("[RunServerOTA] HTTP Update -- no updates (this should never be reached).");
				break;
		}
	#endif
}
