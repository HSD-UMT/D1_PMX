#define _SMARTDEBUG /* if defined, debug messages will appear */
#include <smartdebug.h>

#define FW_VERSION 100

// These generate the server path - adapt to the project (and watch the slashes!)
const String OTABaseURL="http://esp.detlefamend.de/";
const String OTAChannel="demo/";
const String OTAVersionFile=OTABaseURL+OTAChannel+"version.esp";

// HIGH >> grab only a FW that is newer than the current one
// LOW  >> grab the version, the server prefers (enables downgrading)
// normally HIGH
const bool OTAOnlyNewerVersions=HIGH;

#include <DET_OTAServerBased.h>

void setup(){

  dinit(115200); // or Serial.begin(115200);

  RunServerOTA(); // yeah, that's all :)
	
}

void loop(){
}
