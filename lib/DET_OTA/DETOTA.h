// WIFI must be initialized outside this procedure, b'c for OTA Wifi must be activated all the time

// "Knows" about smartDebug & oprint :)

/* INSTALLATION

- copy ESP8266_OTA_BaseCode.h to your sketch directory

- somehwere in the globals, put 
	#include "ESP8266_OTA_BaseCode.h"
	- in yer code.

- put
	OTAinit("Stinky");
	- somewhere in SETUP AFTER Wifi is started. The name os optional, will be
	added by the last 6 digits of the MAC
	
- put
	ArduinoOTA.handle();
	- into LOOP

- done :)
	
*/

#include <WiFiUdp.h>
#include <ArduinoOTA.h>

void OTA_updateStarted(){
	#ifdef _SMARTDEBUG
		dprintln("[OTAinit] Update started!");
	#endif

	#ifdef ESP8266oprintWasHere
		u8g2.setFont(u8g2_font_chikita_tf);
		oprint("clr");
		oprint("OTA UPDATE!");
	#endif
}

void OTA_updateFinished(){
	#ifdef _SMARTDEBUG
		dprintln("[OTAinit] Update done -- rebooting.");
	#endif
	#ifdef ESP8266oprintWasHere
		oprint("Rebooting!");
	#endif
}

void OTA_updateProcess(int progress,int total){
	byte k=progress/(total/100);
	static byte lastk=0;
	if(k % 10==0 && lastk!=k){
  
		#ifdef _SMARTDEBUG
			dprint("[OTAinit] Process: ");
			dprint(String(k));
			dprintln("%");
		#endif
		
		#ifdef ESP8266oprintWasHere
			oprint("Update "+String(k)+"%",1);  
		#endif
		lastk=k;
	}
}

void OTA_updateError(ota_error_t error){
	#ifdef _SMARTDEBUG
		dprint("[OTAinit] Error: ");
		dprint(String(error));
    dprint(" >> ");
		
		if (error==OTA_AUTH_ERROR){
      dprintln("Auth Failed");
    } else if (error==OTA_BEGIN_ERROR){
      dprintln("Begin Failed");
    } else if (error==OTA_CONNECT_ERROR){
      dprintln("Connect Failed");
    } else if (error==OTA_RECEIVE_ERROR){
      dprintln("Receive Failed");
    } else if (error==OTA_END_ERROR){
      dprintln("End Failed");
    }
	#endif
}

void OTAinit(String OTAinit_hostName="ESPOTA"){
	OTAinit_hostName+="_"+String(ESP.getChipId(),HEX);
	
	#ifdef _SMARTDEBUG
		dprintln("\n[OTAinit] -- START --");
	#endif

	if(WiFi.status()!=WL_CONNECTED){ // there's already a Wifi connection - no need to set up a nother one :)
		#ifdef _SMARTDEBUG
			dprintln("[OTAinit] No Wifi around - so no OTA :( ");
		#endif
		return;
	}
	
	ArduinoOTA.setHostname(OTAinit_hostName.c_str());
	
	#ifdef _SMARTDEBUG
		dprint("[OTAinit] Wifi found!\n[OTAinit] Using Hostname ");
		dprint(OTAinit_hostName);
		dprint(" on Wifi ");
		dprint(WiFi.SSID());
		dprint(", IP ");
		dprint(WiFi.localIP());
		dprintln(" for OTA");
	#endif
	
	ArduinoOTA.onStart(OTA_updateStarted);
  ArduinoOTA.onEnd(OTA_updateFinished);
  ArduinoOTA.onProgress(OTA_updateProcess);
	ArduinoOTA.onError(OTA_updateError);
	
  ArduinoOTA.begin();

	#ifdef _SMARTDEBUG
		dprintln("[OTAinit] OTA is up and running!\n[OTAinit] -- DONE --");
	#endif

}
