#include <Arduino.h>

#define FW_VERSION 111

#define gewichteteMittelBufMaxLen 64

#define debugMode false /* TRUE while programming */

#define iniFileName "pmx.ini"

// These can be changed by the ini file:
uint16_t sampleEvery=2; // sample rate. Can be somewhere between 2 and 120 seconds. Only EVEN numbers
bool adc0state=true; // If available, ADCs will always be on (unless deactivated in INI)
bool adc1state=true;
bool adc2state=true;
bool adc3state=true;
bool adc01DiffMode=false; // ADCs Diff Modes - needs to be activated in INI
bool adc23DiffMode=false;

int16_t adc0offset=0,adc1offset=0,adc2offset=0,adc3offset=0; // offsets for the ADCs

String dataDumpComment=""; // Put in Streetname or stuff
uint64_t nrfPipe=1; // NRF24l01 pipe - more user-friendly name
bool dbgDisplay=false; // debug stuff
bool adddDebugDataToSDDump=false;  // more debug stuff
uint16_t sendThisAsXtra=1; // for the display -- see following list for values... default sending ADC0 (for the lazy ppl)
bool noUpdateCheck=false;
uint8_t smoothValuesForDisplay=1; // ranges from 2-10

// correction values...
float pmaOffset=0,pmbOffset=0,pmcOffset=0;
float pmaSpan=0,pmbSpan=0,pmcSpan=0;

// ...and the correction values control bits - set in the ini processing
bool pmaOffsetIsSet=LOW,pmbOffsetIsSet=LOW,pmcOffsetIsSet=LOW;
bool pmaSpanIsSet=LOW,pmbSpanIsSet=LOW,pmcSpanIsSet=LOW;

#define xtra_none 0 /* Don't do any xtra -- ignore parameter */ 
#define xtra_adc0 1
#define xtra_adc1 2
#define xtra_adc2 3
#define xtra_adc3 4
#define xtra_dif01 5
#define xtra_dif23 6
#define xtra_co2 7 /* SCD30 -- future use ;) */
#define xtra_voc 8 /* not sure if we need that - will be one of the adcs anyway ;) */

// IMPORTANT!!!
// Do NOT USE Serial.print!!
// Debug ONLY via dprint & dprintln

#include "screens.h"
#include "index_html.h"
//#include "includes.h" /*****************************************/
#define SoftwareSerialDebugPin 0 /* D3 On WeMos D1 */

// Debugging ON
#define dprint(txt) SoftwareSerialdbg.print(txt)
#define dprintln(txt) SoftwareSerialdbg.println(txt)

// Debugging OFF
// #define dinit(speed)
// #define dprint(txt)
// #define dprintln(txt)

#define I2CAdressDisplay 0x3c
#define I2CAdressSCD30 0x61
#define I2CAdressDisplayButtons 0x31

bool displayAttached=LOW;
bool displayButtonsAttached=LOW;
bool OPCAttached=LOW;
bool GPSReady=LOW;
bool GPSValid=LOW;
bool SDCardReady=LOW;
bool SDWriteOK=LOW;
bool NRF24L01Attached=LOW;
bool SCD30Attached=LOW;

#include <Wire.h>

//GPS

String gpsTime="--";
String gpsDate="--";
String gpsFilename="--";
String gpsFilenameNOEXT="--";

// Serielles Port fürs GPS
#include <SoftwareSerial.h>
SoftwareSerial serialGPS;

#include <TinyGPS++.h>
TinyGPSPlus gps;

// Time
#include <TimeLib.h>
bool SysTimeIsValid=false;


// Debug per SoftwareSerial
SoftwareSerial SoftwareSerialdbg;


//DISPLAY
#include <U8g2lib.h>
U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

uint16_t lineHeight;


//Display Buttons -- for the LOLIN Shields
#include <LOLIN_I2C_BUTTON.h>
I2C_BUTTON I2Cbuts(I2CAdressDisplayButtons);
#define I2CButsCheckEvery 100 /* ms */

#include <SPI.h>


// NRF24L01
#include <RF24.h>

#define NRF_CE 1 /* TX -- can be clamped to 3v3! */ 
#define NRF_CSN 3 /* RX */

RF24 nrf(NRF_CE,NRF_CSN); /* CE, CSN */

struct nrfDataStruct{ // gets updated by loopGetData
  uint8_t change;
  float pm10;
  float sumBins;
  float temp;
  float altitude;
  float hum;
  float xtra;
}nrfData;


// AlphaSense OPC's
#include "opcn3m.h"
#include "opcr1m.h"

#define AlphasenseCSPin D0

OPCN3 n3(AlphasenseCSPin);
OPCR1 r1(AlphasenseCSPin);

HistogramDataN3 histN3;
PMDataN3 pmN3;
ConfigVarsN3 varsN3;

HistogramDataR1 histR1;
PMDataR1 pmR1;
ConfigVarsR1 varsR1;

String OPCInfoString;
String OPCSerialNumber;
byte OPCSensorType=0; // 1:R1 / 2:N3 / 3:R2

float PMA_corrected=0;
float PMB_corrected=0;
float PMC_corrected=0;


//BME680
bool BME680Attached=LOW;
bool BME280Attached=LOW;
bool BMP280Attached=LOW;

float bme_temp;
float bme_humidity;
float bme_pressure;
float bme_alt;
float bme_HeightOverGround;


// ADS1115
//#include <Adafruit_ADS1015.h>
#include <Adafruit_ADS1015.h>
#define I2CAdressADS1115 0x48 /* Default and PMX BaseBoard Adress */
Adafruit_ADS1115 ads(I2CAdressADS1115);

int16_t adc0,adc1,adc2,adc3,adc01Diff,adc23Diff;
bool ADS1115Attached=LOW;
const float ADCFactor=0.125/1000.0; // for V - this may needs to be move to the INI


// SCD30
#include <SparkFun_SCD30_Arduino_Library.h>
SCD30 scdx0;

float scdx0_co2, scdx0_temp, scdx0_hum;

//WIFI
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>

DNSServer dnsServer;
ESP8266WebServer server(80);


// Wifi Manager
#include <WiFiManager.h>


//SD
#include <SPI.h>
#include "SdFat.h"

using namespace sdfat;

SdFat SD;

#define sdChipSelectPin D8

// Smooth Display Buffer -- replaces GM-stuff 20-12

#define smoothDisplayBufferSize 10

float smoothDisplayBufferPMA[smoothDisplayBufferSize];
float smoothDisplayBufferPMB[smoothDisplayBufferSize];
float smoothDisplayBufferPMC[smoothDisplayBufferSize];

float smoothDisplayPMA;
float smoothDisplayPMB;
float smoothDisplayPMC; // PM10

// Arythmetische Mittel -- 20-06

double AM_GPS_lat;
double AM_GPS_lng;
double AM_GPS_kmph;
double AM_GPS_altitude;
uint16_t AM_GPS_satellites;
double AM_GPS_hdop;

float AM_bme_temp;
float AM_bme_humidity;
float AM_bme_pressure;
float AM_bme_alt;
float AM_bme_HeightOverGround;

float AM_AS_pmA;
float AM_AS_pmB;
float AM_AS_pmC;

// corrected values - 20-12
float AM_AS_pmA_CORR;
float AM_AS_pmB_CORR;
float AM_AS_pmC_CORR;

uint16_t AM_AS_temperature;
uint16_t AM_AS_humiditiy;

double AM_AS_bin0;
double AM_AS_bin1;
double AM_AS_bin2;
double AM_AS_bin3;
double AM_AS_bin4;
double AM_AS_bin5;
double AM_AS_bin6;
double AM_AS_bin7;
double AM_AS_bin8;
double AM_AS_bin9;
double AM_AS_bin10;
double AM_AS_bin11;
double AM_AS_bin12;
double AM_AS_bin13;
double AM_AS_bin14;
double AM_AS_bin15;
double AM_AS_bin16;
double AM_AS_bin17;
double AM_AS_bin18;
double AM_AS_bin19;
double AM_AS_bin20;
double AM_AS_bin21;
double AM_AS_bin22;
double AM_AS_bin23;

double AM_AS_sumAllBins;

float AM_AS_period;
float AM_AS_sfr;

int16_t AM_adc0;
int16_t AM_adc1;
int16_t AM_adc2;
int16_t AM_adc3;
int16_t AM_adc01Diff;
int16_t AM_adc23Diff;


// WifiMode - don't forget the missing resistor between A0 and 3v3
bool wifiMode=false;
/*includes.h***************************************************************************/
//#include "helpers.h"*****************************************************************/
String SysID="PMX"; // last 6 digits of the MAC adress

uint16_t compileYear=0; // gets the year from __DATE__ used in GPSPlausibilityCheck
time_t compileDateUXTime=0; // Compile Date as Unix Time Stamp

void setCompileDateUXTime(){
  String compileDaySTR="";
  String compileMonthSTR="";
  String compileYearSTR="";

  String compileHourSTR="";
  String compileMinuteSTR="";
  String compileSecondSTR="";

  compileDaySTR+=String(__DATE__[4]);
  compileDaySTR+=String(__DATE__[5]);

  compileMonthSTR+=String(__DATE__[0]);
  compileMonthSTR+=String(__DATE__[1]);
  compileMonthSTR+=String(__DATE__[2]);

  compileYearSTR+=String(__DATE__[7]);
  compileYearSTR+=String(__DATE__[8]);
  compileYearSTR+=String(__DATE__[9]);
  compileYearSTR+=String(__DATE__[10]);

  compileHourSTR+=String(__TIME__[0]);
  compileHourSTR+=String(__TIME__[1]);

  compileMinuteSTR+=String(__TIME__[3]);
  compileMinuteSTR+=String(__TIME__[4]);

  compileSecondSTR+=String(__TIME__[6]);
  compileSecondSTR+=String(__TIME__[7]);

  tmElements_t TiEl;

  TiEl.Hour=compileHourSTR.toInt();
  TiEl.Minute=compileMinuteSTR.toInt();
  TiEl.Second=compileSecondSTR.toInt();
  TiEl.Day=compileDaySTR.toInt();
  TiEl.Year=(compileYearSTR.toInt())-1970;
  
  if     ( compileMonthSTR == "Jan" ) TiEl.Month = 1;
  else if( compileMonthSTR == "Feb" ) TiEl.Month = 2;
  else if( compileMonthSTR == "Mar" ) TiEl.Month = 3;
  else if( compileMonthSTR == "Apr" ) TiEl.Month = 4;
  else if( compileMonthSTR == "May" ) TiEl.Month = 5;
  else if( compileMonthSTR == "Jun" ) TiEl.Month = 6;
  else if( compileMonthSTR == "Jul" ) TiEl.Month = 7;
  else if( compileMonthSTR == "Aug" ) TiEl.Month = 8;
  else if( compileMonthSTR == "Sep" ) TiEl.Month = 9;
  else if( compileMonthSTR == "Oct" ) TiEl.Month = 10;
  else if( compileMonthSTR == "Nov" ) TiEl.Month = 11;
  else if( compileMonthSTR == "Dec" ) TiEl.Month = 12;

  compileDateUXTime=makeTime(TiEl);

  compileDateUXTime-=7200; // 2 hrs DST - just for usability while compiling

  dprint(F("[setCompileDateUXTime] Compile Date as seconds since 01.01.1970: "));
  dprintln(String(compileDateUXTime));
}

void setCompileYear(){
  String compileY="";

  compileY+=String(__DATE__[7]);
  compileY+=String(__DATE__[8]);
  compileY+=String(__DATE__[9]);
  compileY+=String(__DATE__[10]);

  compileYear=compileY.toInt();

  dprintln("[setCompileYear] compileYear: "+String(compileYear));
}

String SetSysID(){
  String mac;
  mac=WiFi.macAddress();
  mac.replace(":","");
  SysID=SysID+mac.substring(6);
  Serial.println("[SetSysID] This System's ID: "+SysID);
}

String uint64ToString(uint64_t input){ // to make pipe printable ;)
  String result="";
  uint8_t base=10;

  do{
    char c=input%base;
    input /= base;

    if (c < 10)
      c +='0';
    else
      c += 'A' - 10;
    result = c + result;
  } while (input);
  return result;
}

byte CheckThisAdress(int adr){
  Wire.beginTransmission(adr);
  return Wire.endTransmission();
}

String withLeadingZero(String x){
  if (x.length()==1){
  return "0"+x;
  }
  return x;
}

float ConvSTtoTemperature(unsigned int ST) {
  return -45 + 175 * (float)ST / 65535;
}

float ConvSRHtoRelativeHumidity (unsigned int SRH) {
  return 100 * (float)SRH / 65535;
}

void checkDevicesConnected(){
  if(CheckThisAdress(I2CAdressDisplay)==0){
    displayAttached=HIGH;
  }else{
    displayAttached=LOW;
  }

  if(CheckThisAdress(I2CAdressDisplayButtons)==0){
    displayButtonsAttached=HIGH;
  }else{
    displayButtonsAttached=LOW;
  }

  if(CheckThisAdress(I2CAdressADS1115)==0){
    ADS1115Attached=HIGH;
  }else{
    ADS1115Attached=LOW;    
  }

  if(CheckThisAdress(I2CAdressSCD30)==0){
    SCD30Attached=HIGH;
  }else{
    SCD30Attached=LOW;    
  }

  digitalWrite(AlphasenseCSPin,HIGH); // AlphaSense Sensor definitely OFF
  digitalWrite(NRF_CSN,HIGH);
  delay(20);

  if(SD.begin(sdChipSelectPin)){
    SDCardReady=HIGH;
  }else{
    SDCardReady=LOW;    
  }
}
/*helpers.h*************************************************************************/
//#include "DEBUG.h"***************************************************************/
void debugMemState(){
	if(!dbgDisplay){
		return;
	}
	
	dprint("[debugMemState] Free Heap: ");
	dprintln(ESP.getFreeHeap());

	dprint("[debugMemState] Heap Fragmentation: ");
	dprint(ESP.getHeapFragmentation());
	dprintln("%\n");
}

void DebugSER(){
  
  dprint("\n[DebugSER] displayAttached >> ");
  if(displayAttached){
    dprintln("OK");
  }else{
    dprintln("Nope");
  }

	dprint("[DebugSER] display has I2C Buttons  >> ");
  if(displayButtonsAttached){
    dprintln("Yeah");
  }else{
    dprintln("Nope");
  }

  dprint("[DebugSER] OPCAttached >> ");
  if(OPCAttached){
    dprintln("OK");
  }else{
    dprintln("Nope");
  }

  dprint("[DebugSER] BMP280Attached >> ");
  if(BMP280Attached){
    dprintln("OK");
  }else{
    dprintln("Nope");
  }

  dprint("[DebugSER] BME280Attached >> ");
  if(BME280Attached){
    dprintln("OK");
  }else{
    dprintln("Nope");
  }

  dprint("[DebugSER] BME680Attached >> ");
  if(BME680Attached){
    dprintln("OK");
  }else{
    dprintln("Nope");
  }

  dprint("[DebugSER] SDCardReady >> ");
  if(SDCardReady){
    dprintln("OK");
  }else{
    dprintln("Nope");
  }
  
  if(GPSReady){
    if(GPSValid){
      dprint("[DebugSER] GPSdate >> ");
      dprintln(gpsDate);
      dprint("[DebugSER] gpsTime >> ");
      dprintln(gpsTime);
      dprint("[DebugSER] gpsFilename >> ");
      dprintln(gpsFilename);
    }else{
      dprintln("[DebugSER] NO valid GPS Data available");
    }
  }else{
    dprintln("[DebugSER] NO GPS found");
  }

  if(BME680Attached){
    dprint("[DebugSER] BME680 Temperature >> ");
    dprintln(bme_temp);
    dprint("[DebugSER] BME680 humidity >> ");
    dprintln(bme_humidity);
    dprint("[DebugSER] BME680 Pressure >> ");
    dprintln(bme_pressure);
  }

  if(OPCAttached){
	
	if(OPCSensorType==1 || OPCSensorType==3){
		dprintln("[DebugSER] OPC is an R1 or R2, so BINs 16-23 are invalid!");
	}

  // Constructing Header
  
    String d="";
    d+="bin0";d+="\t";
    d+="bin1";d+="\t";
    d+="bin2";d+="\t";
    d+="bin3";d+="\t";
    d+="bin4";d+="\t";
    d+="bin5";d+="\t";
    d+="bin6";d+="\t";
    d+="bin7";d+="\t";
    d+="bin8";d+="\t";
    d+="bin9";d+="\t";
    d+="bin10";d+="\t";
    d+="bin11";d+="\t";
    d+="bin12";d+="\t";
    d+="bin13";d+="\t";
    d+="bin14";d+="\t";
    d+="bin15";d+="\t";
    d+="bin16";d+="\t";
    d+="bin17";d+="\t";
    d+="bin18";d+="\t";
    d+="bin19";d+="\t";
    d+="bin20";d+="\t";
    d+="bin21";d+="\t";
    d+="bin22";d+="\t";
    d+="bin23";d+="\t";
		
		d+="Σ bin";d+="\t";
  
    d+="PM1";d+="\t";
    d+="PM2.5";d+="\t";
    d+="PM10";d+="\t";
  
    d+="temp";d+="\t";
    d+="hum";d+="\t";
  
    d+="RCG";d+="\t";
    d+="RCL";d+="\t";
  
    d+="Period";d+="\t";
  
    dprint("[DebugSER] ");
    dprintln(d);

    // Construct Data to write

    if(OPCSensorType==2){ // N3
    
      d=String(histN3.bin0,1);d+="\t";
      d+=String(histN3.bin1,1);d+="\t";
      d+=String(histN3.bin2,1);d+="\t";
      d+=String(histN3.bin3,1);d+="\t";
      d+=String(histN3.bin4,1);d+="\t";
      d+=String(histN3.bin5,1);d+="\t";
      d+=String(histN3.bin6,1);d+="\t";
      d+=String(histN3.bin7,1);d+="\t";
      d+=String(histN3.bin8,1);d+="\t";
      d+=String(histN3.bin9,1);d+="\t";
      d+=String(histN3.bin10,1);d+="\t";
      d+=String(histN3.bin11,1);d+="\t";
      d+=String(histN3.bin12,1);d+="\t";
      d+=String(histN3.bin13,1);d+="\t";
      d+=String(histN3.bin14,1);d+="\t";
      d+=String(histN3.bin15,1);d+="\t";
      d+=String(histN3.bin16,1);d+="\t";
      d+=String(histN3.bin17,1);d+="\t";
      d+=String(histN3.bin18,1);d+="\t";
      d+=String(histN3.bin19,1);d+="\t";
      d+=String(histN3.bin20,1);d+="\t";
      d+=String(histN3.bin21,1);d+="\t";
      d+=String(histN3.bin22,1);d+="\t";
      d+=String(histN3.bin23,1);d+="\t";
      
      d+=String(histN3.sumAllBins,1);d+="\t";
  
      d+=String(histN3.pmA,1);d+="\t";
      d+=String(histN3.pmB,1);d+="\t";
      d+=String(histN3.pmC,1);d+="\t";
  
      d+=String(ConvSTtoTemperature(histN3.temp),1);d+="\t";
      d+=String(ConvSRHtoRelativeHumidity(histN3.humidity));d+="\t";
    
      d+=String(histN3.rcg,DEC);d+="\t";
      d+=String(histN3.rcl,DEC);d+="\t";
  
      d+=String(histN3.period,2);d+="\t";
    }

    if(OPCSensorType==1 || OPCSensorType==3){ // R1/2
    
      d=String(histR1.bin0,1);d+="\t";
      d+=String(histR1.bin1,1);d+="\t";
      d+=String(histR1.bin2,1);d+="\t";
      d+=String(histR1.bin3,1);d+="\t";
      d+=String(histR1.bin4,1);d+="\t";
      d+=String(histR1.bin5,1);d+="\t";
      d+=String(histR1.bin6,1);d+="\t";
      d+=String(histR1.bin7,1);d+="\t";
      d+=String(histR1.bin8,1);d+="\t";
      d+=String(histR1.bin9,1);d+="\t";
      d+=String(histR1.bin10,1);d+="\t";
      d+=String(histR1.bin11,1);d+="\t";
      d+=String(histR1.bin12,1);d+="\t";
      d+=String(histR1.bin13,1);d+="\t";
      d+=String(histR1.bin14,1);d+="\t";
      d+=String(histR1.bin15,1);d+="\t";
      
      d+=String(histR1.sumAllBins,1);d+="\t";
  
      d+=String(histR1.pmA,1);d+="\t";
      d+=String(histR1.pmB,1);d+="\t";
      d+=String(histR1.pmC,1);d+="\t";
  
      d+=String(ConvSTtoTemperature(histR1.temp),1);d+="\t";
      d+=String(ConvSRHtoRelativeHumidity(histR1.humidity));d+="\t";
    
      d+=String(histR1.rcg,DEC);d+="\t";
      d+=String(histR1.rcl,DEC);d+="\t";
  
      d+=String(histR1.period,2);d+="\t";
    }

    d.replace('.',',');

    dprint("[DebugSER] ");
    dprintln(d);
  }
  dprintln("");
}

void debugDevicesConnected(){

  String s = WiFi.macAddress();
  dprint("\n[debugDevicesConnected] This Unit's MAC: ");
  dprintln(s);

  if(displayAttached){
    dprintln("[debugDevicesConnected] Display OK");
  }else{
    dprintln("[debugDevicesConnected] !!NO Display");
  }

  dprint("[debugDevicesConnected] display has I2C Buttons  >> ");
  if(displayButtonsAttached){
    dprintln("Yeah");
  }else{
    dprintln("Nope");
  }

  if(OPCAttached){
		if(OPCSensorType==2){ // it's an N3!
			dprintln("[debugDevicesConnected] OPC N3 OK");
		}else{
	    dprintln("[debugDevicesConnected] OPC R1 OK");
		}
  }else{
    dprintln("[debugDevicesConnected] !!NO OPC");
  }

  if(!BMP280Attached && !BME280Attached && !BME680Attached ){
    dprintln("[debugDevicesConnected] !!NO BMxy80");
  }else{
    dprintln("[debugDevicesConnected] BMxy80 OK");
  }

  if(NRF24L01Attached){
    dprintln("[debugDevicesConnected] NRF24L01 Module OK");
  }else{
    dprintln("[debugDevicesConnected] !!NO NRF24L01 Module");
  }

  if(ADS1115Attached){
    dprintln("[debugDevicesConnected] ADS1115 OK");
  }else{
    dprintln("[debugDevicesConnected] !!NO ADS1115");
  }
	
	if(SCD30Attached){
    dprintln("[debugDevicesConnected] SCD30 OK");
  }else{
    dprintln("[debugDevicesConnected] !!NO SCD30");
  }
	
  if(SDCardReady){
    dprintln("[debugDevicesConnected] SD Card Ready");
  }else{
    dprintln("[debugDevicesConnected] !!NO SD Card");
  }
}
/*debug.h***************************************************************/

//oPrint
//#include "ESP8266_oprint.h"********************************************/
#ifndef ESP8266oprintWasHere
#define ESP8266oprintWasHere

void oprint(String OLEDStr="",bool dontScroll=LOW){

  static byte OLEDCurrentLine=0;
  static byte OLEDStartPrintAtThisLine=0;
  static bool OLEDscrollEnable=LOW;

  static byte OLEDLineHeight=u8g2.getMaxCharHeight()-2;
  static byte OLEDMaxLines=u8g2.getDisplayHeight()/OLEDLineHeight;

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
/*ESP8266_oprint.h***************************************************************/

// Silent Serverbased OTA
const String OTABaseURL="http://esp.detlefamend.de/";
const String OTAChannel="pmx/";
const String OTAVersionFile=OTABaseURL+OTAChannel+"version.esp";
const bool OTAOnlyNewerVersions=HIGH;

//#include "ESP8266_OTAServerBased.h"********************************************/
#define wifiConnectTimeout 10e3

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

ESP8266WiFiMulti wifiMulti;

bool OTAServerBased_WeDidTheConnect=LOW;

void OTAServerBased_wifiMultiConnect(){

	if(WiFi.status()==WL_CONNECTED){ // there's already a Wifi connection - no need to set up a nother one :)
		dprintln("[OTAServerBased_wifiMultiConnect] Already connected :) ");
		return;
	}
	
	// starting some wifi multi connection
	dprintln("[OTAServerBased_wifiMultiConnect] Starting Wifi...");

  WiFi.setOutputPower(20.5); // MAX
  WiFi.setPhyMode(WIFI_PHY_MODE_11N);
	
	WiFi.mode(WIFI_STA);

  wifiMulti.addAP("dets gast","00000000"); // works with any GUEST login!
	wifiMulti.addAP("FBMV UMT","JgRV2#ZsyPX3");
  wifiMulti.addAP("UMTIOT","88888888");
  wifiMulti.addAP("Freifunk","");
	wifiMulti.addAP("g7","88888888"); // Det's Mobile Hotspot
	wifiMulti.addAP("Chrisfon","ztxn3198"); // Christians's Mobile Hotspot

	unsigned long wifiStartConnectionAttempt=millis()+wifiConnectTimeout;

	dprint("[OTAServerBased_wifiMultiConnect] Trying to connect");

	while(wifiMulti.run()!=WL_CONNECTED && wifiStartConnectionAttempt>millis()){
		dprint(".");
		delay(500);
	}

	dprintln("");

	if(WiFi.status()==WL_CONNECTED){
		dprint("[OTAServerBased_wifiMultiConnect] Connected to ");
		dprintln(WiFi.SSID());
		OTAServerBased_WeDidTheConnect=HIGH;
	}else{
		dprintln("[OTAServerBased_wifiMultiConnect] Cound not connect to any predefined WiFi - OTA is not available!!!");
	}
}

void OTAServerBased_disconnectWifi(){
	if(OTAServerBased_WeDidTheConnect==LOW){
		dprintln("[OTAServerBased_disconnectWifi] Not our wifi connection - leave it running.\n");
		return;
	}
  dprintln("[OTAServerBased_disconnectWifi] Shutting down Wifi\n");
  WiFi.disconnect(); //won't kill the wifi complete, but disconnects from any WLAN
}

void OTAServerBased_updateStarted(){
	dprintln("[RunServerOTA] Update started!");
}

void OTAServerBased_updateFinished(){
	dprintln("[RunServerOTA] Update done -- rebooting.");
}

void OTAServerBased_updateProcess(int cur,int total){
	byte k=cur/(total/100);
	static byte lastk=0;
	if(k % 5==0 && lastk!=k){
		dprint("[RunServerOTA] Process: ");
		dprint(String(k));
		dprintln("%");

    oprint("Update "+String(k)+"%",1);  

		lastk=k;
	}
}

void OTAServerBased_updateError(int err){
	dprint("[RunServerOTA] Error - that didn't work.\nError Code: ");
  dprintln(String(err));
}

void RunServerOTA(){

  if(debugMode){
    dprintln("[RunServerOTA] debugMode - disabeling Silent Servisided OTA");
    return;
  }
	
	if(noUpdateCheck){
		dprintln("[RunServerOTA] noUpdateCheck set - disabeling Silent Servisided OTA");
    return;
	}

	OTAServerBased_wifiMultiConnect();

  dprintln("");

  if(WiFi.status()!=WL_CONNECTED){
    dprintln("[RunServerOTA] no wifi connection - so no update.");
    return;
  }

  dprint("[RunServerOTA] Connected to ");
  dprint(WiFi.SSID());
  dprint(" (");
  dprint(WiFi.localIP());
  dprintln(")");

  dprint("\n[RunServerOTA] Connecting to ");
  dprint(OTAVersionFile);
  dprintln(" and fetching current version serverside");

  WiFiClient client;
  HTTPClient http;

  http.begin(client,OTAVersionFile);
  
  int httpCode=http.GET();

  dprint("[RunServerOTA] http return code: ");
  dprintln(String(httpCode));

  dprint("[RunServerOTA] Local Firmware Version: ");
  dprintln(String(FW_VERSION));

  if(httpCode!=200){
    dprint("[RunServerOTA] ERROR: can't read version file. Aborting.");
    OTAServerBased_disconnectWifi();
    return;
  }

  String newFWVersion=http.getString();

  dprint("[RunServerOTA] FW Version serverside: ");
  dprintln(String(newFWVersion));
	
	http.end();

  int ServerSideVersion=newFWVersion.toInt();

  if(OTAOnlyNewerVersions){
    if(ServerSideVersion<=FW_VERSION){ // only goes for newer versions
      dprintln("[RunServerOTA] We're already running the most current firmware");
      OTAServerBased_disconnectWifi();
      return;
    }
  }else{
    if(ServerSideVersion==FW_VERSION){ // possibility to downgrade
      dprintln("[RunServerOTA] We're running the Server-prefered firmware.");
      OTAServerBased_disconnectWifi();
      return;
    }
  }

  dprintln("[RunServerOTA] NEW FIRMWARE found serverside! Starting Update process.");

  oprint("clr");
  oprint("FW UPDATE!");
  oprint("Local: "+String(FW_VERSION));  
  oprint("Server: "+newFWVersion);  

  String ServerSideFirmwareURL=OTABaseURL+OTAChannel+newFWVersion+".bin";

  dprint("[RunServerOTA] Serverside Firmware URL: ");
  dprintln(ServerSideFirmwareURL);

  dprintln("\n[RunServerOTA] UPDATING -- just a moment, please :) \n");
	
	ESPhttpUpdate.onStart(OTAServerBased_updateStarted);
  ESPhttpUpdate.onEnd(OTAServerBased_updateFinished);
  ESPhttpUpdate.onProgress(OTAServerBased_updateProcess);
	ESPhttpUpdate.onError(OTAServerBased_updateError);

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
}
/*ESP8266_OTAServerBased.h***************************************************************/

// Main System Modules
//#include "Webserver.h"***************************************************************/
String response2Stage(){

  String resp="<p>SD Karte: ";
  if(SDCardReady){
    resp+="OK";
  }else{
    resp+="---";
  }
  resp+=" | GPS-Status: ";
  if(GPSReady){
    resp+="OK";
  }else{
    resp+="---";
  }
  resp+="<br><b>GPS Daten</b><br>Zeit: ";
  resp+=gpsTime;
  resp+=" | Datum: ";
  resp+=gpsDate;
  resp+="<br>Dateiname: ";
  resp+=gpsFilename;
  
  if(BMP280Attached || BME280Attached || BME680Attached ){
    resp+="<hr><b>Enviroment</b><br>";
    resp+="<table style=\"text-align:left;\"><tr><td>Temperatur</td><td><b>"+String(bme_temp)+"</b> C</td></tr>";
    if(BME280Attached || BME680Attached ){
      resp+="<tr><td>Humidity</td><td><b>"+String(bme_humidity)+"</b> %relH</td></tr>";
    }
    resp+="<tr><td>Luftdruck</td><td><b>"+String(bme_pressure)+"</b> hPa</td></tr>";
  }

  if(SCD30Attached){
    resp+="<tr><td>CO2</td><td><b>"+String(scdx0_co2)+"</b> ppm</td></tr>";
  }
  
  resp+="</table>";

  // Sensirion PM1 / PM25 / PM10

  if(OPCAttached){
  
    if(OPCSensorType==2){ // it's an N3!
      resp+="<hr><b>AlphaSense OPC N3</b><br>";
    }else if(OPCSensorType==1){
      resp+="<hr><b>AlphaSense OPC R1</b><br>";
    }else if(OPCSensorType==3){
      resp+="<hr><b>AlphaSense OPC R2</b><br>";
    }
    
    resp+="<table style=\"text-align:left;\"><tr><td>PM1</td><td><b>"+String(smoothDisplayPMA,1)+"</b> ug/m3</td></tr>";
    resp+="<tr><td>PM2,5</td><td><b>"+String(smoothDisplayPMB,1)+"</b> ug/m3</td></tr>";
    resp+="<tr><td>PM10</td><td><b>"+String(smoothDisplayPMC,1)+"</b> ug/m3</td></tr></table>";
  }

  if(dbgDisplay){
    resp+="<hr>HeapFrag: ";
    resp+=String(ESP.getHeapFragmentation());
    resp+="%";
  }

  resp+="</p>";
  
  return resp;
}

void processIndex(){

  server.sendHeader("Cache-Control","no-cache");
  server.sendHeader("Access-Control-Allow-Origin","*");
  
  if (server.args()>0){

    for (int i=0;i<server.args();i++){
      if(server.argName(i)=="status"){
        server.send(200,"text/plain",response2Stage());
      }
    }
  }else{
    server.send(200,"text/html",indexHTML);
  }
}

void handleNotFound(){
  server.send(404,"text/plain","404: Not found");
}
/*Webserver.h**********************************************************************/

//#include "WIFI.h"*****************************************************************/
#define doWifiDebug

IPAddress apIP(1,2,3,4);
IPAddress apSubnet(255,255,255,0);

#define DNS_PORT 53

WiFiManager wifiManager;

void WIFIInit_AP(){
  String SoftAPName="PMX Live Data "+SysID;

  dprint("[WIFIInit_AP] SoftAPName: ");
  dprintln(SoftAPName);

  WiFi.setOutputPower(20.5); // MAX
  WiFi.setPhyMode(WIFI_PHY_MODE_11N);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP,apIP,apSubnet);
  WiFi.softAP(SoftAPName.c_str());

  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(DNS_PORT,"*",apIP);
  
  server.on("/",processIndex);
  server.on("/generate_204", processIndex);
  server.on("/fwlink", processIndex);
  server.onNotFound(handleNotFound);
  server.begin();

  dprintln("[WIFIInit_AP] Init done - server started!");

}

void WifiManagerConfigModeCallback(WiFiManager *WiMa){
  u8g2.clearBuffer();
  u8g2.drawXBMP(0,0,64,48,connectMe2Wifi);
  u8g2.sendBuffer();
  dprintln("[WIFIInit_client] No known Wifi found - starting Wifi Manager (blocking)");
}

void WIFIInit_client(){
  WiFi.hostname(SysID.c_str());
  
  WiFi.setOutputPower(20.5); // MAX
  WiFi.setPhyMode(WIFI_PHY_MODE_11N);
  WiFi.setAutoReconnect(true);

  wifiManager.setDebugOutput(false);
  wifiManager.setAPCallback(WifiManagerConfigModeCallback);
  wifiManager.setAPStaticIPConfig(IPAddress(1,2,3,4),IPAddress(1,2,3,4),IPAddress(255,255,255,0));
  wifiManager.setConfigPortalTimeout(300); // timeout, for ex. after power failure, reconnect after 5 min
  String wLANName=SysID+" CONFIG";
  if(!wifiManager.autoConnect(wLANName.c_str())){
    ESP.reset();
    delay(1000);
  }
  #ifdef doWifiDebug
    dprint("[WIFIInit_client] Connected to ");
    dprint(String(WiFi.SSID()));
    dprint(" (");
    dprint(WiFi.localIP());
    dprintln(")");
  #endif
}

void WIFIInit(){
  dprint("[WifiInit] Starting Wifi in ");

	if(wifiMode){
    dprintln("Client Mode");
		WIFIInit_client();
	}else{
    dprintln("AP Mode");
		WIFIInit_AP();
	}
}
/*WIFI.h**************************************************************************/

//#include "DISPLAY.h"**************************************************************/
#define disableOPCPowerError

void DisplayInit(){

  if(!displayAttached){
    return;
  }
  // u8g2.setBusClock(100000); // SPS30
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.setFlipMode(false);
  u8g2.setFont(u8g2_font_chikita_tf);
  lineHeight=u8g2.getMaxCharHeight()-1;

	u8g2.clearBuffer();
	u8g2.drawXBMP(0,0,64,48,hsdlogo);
	u8g2.sendBuffer();
}

void displayBlink(){
  u8g2.setDrawColor(2);
  u8g2.drawBox(0,0,64,48);
  u8g2.updateDisplay();

  delay(10);
 
  u8g2.drawBox(0,0,64,48);
  u8g2.updateDisplay();
	//u8g2.setDrawColor(0);

}

void DisplayUpdate(){

  if(!displayAttached){
    return;
  }
	
	static bool DisplaySDWarningHasBeenDrawn=LOW;
	
	if(!SDCardReady){ // while there's no SD, display a BIIIG warning (non-blocking)
		if(!DisplaySDWarningHasBeenDrawn){
			u8g2.clearBuffer();
			u8g2.drawXBMP(0,0,64,48,NoSD);
			u8g2.sendBuffer();
			DisplaySDWarningHasBeenDrawn=HIGH;
		}else{
			u8g2.setDrawColor(2);
			u8g2.drawBox(0,0,64,48);
			u8g2.updateDisplay();
		}
		return;
	}
	
	DisplaySDWarningHasBeenDrawn=LOW;

  #ifndef disableOPCPowerError

  bool OPCPowaError=false;
  
    if(OPCSensorType==2){ // N3
      if(histN3.rcg>200 || histN3.rcl>200){
        OPCPowaError=true;
      }
    }
  
    if(OPCSensorType==1 || OPCSensorType==3){ // R1/R2
      if(histR1.rcg>200 || histR1.rcl>200){
        OPCPowaError=true;
      }
    }
  	
  	if(OPCPowaError){
      u8g2.clearBuffer();
      u8g2.drawXBMP(0,0,64,48,AlphasensePowerError);
      u8g2.sendBuffer();
      return;
    }

  #endif

  String MacID=String(WiFi.macAddress());
  
  MacID.replace(":","");
  MacID=MacID.substring(6);

  byte thisLine=1;

  u8g2.clearBuffer();

  u8g2.drawStr(0,lineHeight*thisLine,String("PMX"+MacID).c_str());
	thisLine++;
 
	String GPSline;
	String GPSline2="---";
	
	time_t sysTime=now();

  if(SysTimeIsValid){
    GPSline=withLeadingZero(String(hour(sysTime)));
    GPSline+=":";
    GPSline+=withLeadingZero(String(minute(sysTime)));
    GPSline+=":";
    GPSline+=withLeadingZero(String(second(sysTime)));
    GPSline2=gpsFilenameNOEXT;
  }else{
    GPSline="SysTime Error";
  }
	
  /*
	if(GPSReady){
		if(GPSValid){
			GPSline=withLeadingZero(String(hour(sysTime)));
			GPSline+=":";
			GPSline+=withLeadingZero(String(minute(sysTime)));
			GPSline+=":";
			GPSline+=withLeadingZero(String(second(sysTime)));
			GPSline2=gpsFilenameNOEXT;
		}else{
			GPSline="NO GPS Data";
		}
	}else{
		GPSline="NO GPS";
	}
  */

  u8g2.drawStr(0,lineHeight*thisLine,GPSline.c_str());
  thisLine++;
  u8g2.drawStr(0,lineHeight*thisLine,GPSline2.c_str());
  thisLine++;

  if(BMP280Attached || BME280Attached || BME680Attached){
    u8g2.drawStr(0,lineHeight*thisLine,String(String(bme_temp,1)+" C").c_str());
	}else{
    u8g2.drawStr(0,lineHeight*thisLine,"NO TEMP SENS");
	}

  thisLine++;
	
	if(OPCAttached){
    u8g2.drawStr(0,lineHeight*thisLine,String("PM10: "+String(smoothDisplayPMC,1)).c_str());
	}else{
    if(SCD30Attached){
      u8g2.drawStr(0,lineHeight*thisLine,String("CO2: "+String(scdx0_co2,1)).c_str());
    }else{
      u8g2.drawStr(0,lineHeight*thisLine,"OPC DOWN");
    }
	}

  thisLine++;
	
	if(SDCardReady){
    u8g2.drawStr(0,lineHeight*thisLine,"SD OK");
	}else{
    u8g2.drawStr(0,lineHeight*thisLine,"NO SD!!");
	}

	u8g2.sendBuffer();

  if(!SDCardReady || !OPCAttached || !(BME680Attached || BME280Attached || BMP280Attached) || !GPSReady){
    displayBlink();
  }
}
/*DISPLAY.h************************************************************************/

//#include "OPC.h"******************************************************************/
void OPCSetSPIMode(){ // this is more a optional thing - but it makes running the OPC more secure
    // Switch off all SPI devices
  
  digitalWrite(AlphasenseCSPin,HIGH);
  digitalWrite(NRF_CSN,HIGH);
  digitalWrite(sdChipSelectPin,HIGH);
  
  SPI.beginTransaction(SPISettings(600000UL,MSBFIRST,SPI_MODE1));
  SPI.endTransaction();
  
  delay(1);
}

void OPCinit(){

  OPCSetSPIMode();

  OPCSerialNumber=n3.read_serial_number_string();
  OPCSerialNumber=n3.read_serial_number_string(); // First one can (!) contain garbage...

  dprint("[OPCinit] OPC Serial Number String: ");
  dprintln(OPCSerialNumber);

  OPCInfoString=n3.read_information_string();

  dprint("[OPCinit] OPC info String: ");
  dprintln(OPCInfoString);
	
	digitalWrite(AlphasenseCSPin,HIGH); // AlphaSense Sensor definitely OFF
	
	// What OPC do we have?
	
	OPCAttached=HIGH;
	
	if(OPCSerialNumber.charAt(4)=='R'){ // check the 5th pos in the serial numba string...
    if(OPCSerialNumber.charAt(5)=='1'){
      dprintln("[OPCinit] Ich seh eine R1!");
      OPCSensorType=1; // b'c R1 ;)
    }else if(OPCSerialNumber.charAt(5)=='2'){
      dprintln("[OPCinit] Ich seh eine R2!");
      OPCSensorType=3; //R2
    }
  }else if(OPCSerialNumber.charAt(4)=='N'){
    dprintln("[OPCinit] Ich seh eine N..irgendwas!");
		OPCSensorType=2;
  }else{
    dprintln("[OPCinit] Ohoh - kein Sensor angeklemmt?");
		OPCAttached=LOW;
		return;
  }
	
	if(OPCSensorType==1 || OPCSensorType==3){ // R1/R2
		dprintln("[OPCinit] PowerCycling the R1/R2 (4s)...");
		r1.off();
		delay(2000);
		r1.on();
		delay(2000);
		dprintln("[OPCinit] Initial Sensor Read for the R1/R2");
		varsR1=r1.read_configuration_variables();
		histR1=r1.read_histogram(true);
	}

	if(OPCSensorType==2){ // N3
		n3.on();
		delay(2000);
		dprintln("[OPCinit] Initial Sensor Read for the N3");
		varsN3=n3.read_configuration_variables();
		histN3=n3.read_histogram(true);
	}
}

void OPCGetData(){
	
  if(!OPCAttached){
    return;
  }

  OPCSetSPIMode();
	
	float pmaTmp;
	float pmbTmp;
	float pmcTmp;
	
	if(OPCSensorType==2){ // N3
		histN3=n3.read_histogram(true);
	  if(!histN3.DataValid){
			dprintln("[OPCGetData] OPC N3 Com Error :(");
		}else{
			pmaTmp=histN3.pmA;
			pmbTmp=histN3.pmB;
			pmcTmp=histN3.pmC;
		}
	}
	
	if(OPCSensorType==1 || OPCSensorType==3){ // R1/2
		histR1=r1.read_histogram(true);
	  if(!histR1.DataValid){
			dprintln("[OPCGetData] OPC R1/2 Com Error :(");
		}else{
			pmaTmp=histR1.pmA;
			pmbTmp=histR1.pmB;
			pmcTmp=histR1.pmC;
		}
	}

	digitalWrite(AlphasenseCSPin,HIGH); // AlphaSense Sensor definitely OFF

	if(pmaOffsetIsSet && pmaSpanIsSet){
		PMA_corrected=(pmaTmp+pmaOffset)/pmaSpan;
	}else{
		PMA_corrected=pmaTmp;
	}

	if(pmbOffsetIsSet && pmbSpanIsSet){
		PMB_corrected=(pmbTmp+pmbOffset)/pmbSpan;
	}else{
		PMB_corrected=pmbTmp;
	}
	
	if(pmcOffsetIsSet && pmcSpanIsSet){
		PMC_corrected=(pmcTmp+pmcOffset)/pmcSpan;
	}else{
		PMC_corrected=pmcTmp;
	}
	
	// PMx_corrected now holds either the corrected of the
	// sensor or the raw sensor data - that's a good base 
	// for anything display.
	

	static bool DisplaySmoothing_FirstRun=HIGH;
	static byte pmXGMPointer=0;

  // Gleitender Mittel
  if(DisplaySmoothing_FirstRun){
    // Init Gleitender Mittel Array...
    DisplaySmoothing_FirstRun=LOW; // Nie wieder Init

    //Es gibt noch keinen Mittel...
    smoothDisplayPMA=PMA_corrected;
    smoothDisplayPMB=PMB_corrected;
    smoothDisplayPMC=PMC_corrected;

    //Array mit initialen Daten füllen...
    for(byte t=0;t<smoothValuesForDisplay;t++){
      smoothDisplayBufferPMA[t]=PMA_corrected;
      smoothDisplayBufferPMB[t]=PMB_corrected;
      smoothDisplayBufferPMC[t]=PMC_corrected;
    }
  }else{
    //Runs on every not first runs ;)
    if(++pmXGMPointer>=smoothValuesForDisplay){
      pmXGMPointer=0;
    }
    smoothDisplayBufferPMA[pmXGMPointer]=PMA_corrected;
    smoothDisplayBufferPMB[pmXGMPointer]=PMB_corrected;
    smoothDisplayBufferPMC[pmXGMPointer]=PMC_corrected;

    smoothDisplayPMA=0;
    smoothDisplayPMB=0;
    smoothDisplayPMC=0;

    for(byte t=0;t<smoothValuesForDisplay;t++){
      smoothDisplayPMA+=smoothDisplayBufferPMA[t];
      smoothDisplayPMB+=smoothDisplayBufferPMB[t];
      smoothDisplayPMC+=smoothDisplayBufferPMC[t];
    }

    smoothDisplayPMA=smoothDisplayPMA/smoothValuesForDisplay;
    smoothDisplayPMB=smoothDisplayPMB/smoothValuesForDisplay;
    smoothDisplayPMC=smoothDisplayPMC/smoothValuesForDisplay;

  }
}
/*OPC.h***************************************************************************/
//#include "BMXY80.h"**************************************************************/
#include <BMx280I2C.h>

BMx280I2C bmx76(0x76);
BMx280I2C bmx77(0x77);

//BME680
#include <Zanshin_BME680.h>
BME680_Class BME680;

uint8_t BMXYAdress=0;

float BMx_pres2alt(float Luftdruck){
  return (((float)pow((Luftdruck/1013.25F), 0.190294957F) - 1.0F) * 288150) / -6.5;
}

uint8_t BMXY80_SensorID(uint8_t adr=0x76){
  Wire.beginTransmission(adr);
  Wire.write(0xD0); // Sensor ID
  Wire.endTransmission();

  Wire.requestFrom(adr,(uint8_t)1);
  uint8_t sensID=Wire.read();
  return sensID;
}

void BME280_init(){
  dprintln("[BME280_init] --START--");
  bool bmxInitOK=false;

  dprintln("[BME280_init] Initializing Sensor for Adress 0x"+String(BMXYAdress,HEX));

  if(BMXYAdress==0x76){
    bmxInitOK=bmx76.begin();
  }
  if(BMXYAdress==0x77){
    bmxInitOK=bmx77.begin();
  }
  
  if(!bmxInitOK){
    dprintln(F("[BME280_init] ERROR: INIT FAILED!"));
    return;
  }

  if(BMXYAdress==0x76){
    bmx76.resetToDefaults();
    bmx76.writeOversamplingPressure(BMx280MI::OSRS_P_x04);
    bmx76.writeOversamplingTemperature(BMx280MI::OSRS_T_x04);
    if(bmx76.readID()==0x60){
      bmx76.writeOversamplingHumidity(BMx280MI::OSRS_H_x02);
    }
    while(!bmx76.measure()){
      delay(20);
    }
  }

  if(BMXYAdress==0x77){
    bmx77.resetToDefaults();
    bmx77.writeOversamplingPressure(BMx280MI::OSRS_P_x04);
    bmx77.writeOversamplingTemperature(BMx280MI::OSRS_T_x04);
    if(bmx77.readID()==0x60){
      bmx77.writeOversamplingHumidity(BMx280MI::OSRS_H_x02);
    }
    while(!bmx77.measure()){
      delay(20);
    }
  }

  dprintln("[BME280_init] Sensor Initialized");
}

bool BME680_init(){
  dprintln("[BME680_init] --START--");

  if(!BME680.begin(I2C_STANDARD_MODE)){
    BME680Attached=LOW;
    dprintln(F("[BME680_init] ERROR: INIT FAILED!"));
    return false;
  }

  BME680.setOversampling(TemperatureSensor,Oversample16);
  BME680.setOversampling(HumiditySensor,Oversample16);
  BME680.setOversampling(PressureSensor,Oversample16);
  BME680.setGas(0,0);

  int32_t t,h,p,g;

  for(byte q=0;q<3;q++){ // grabbing some readings to get accurate data in the loop
    BME680.getSensorData(t,h,p,g,true);
    delay(20);
  }

  dprintln("[BME680_init] Sensor Initialized");

  return true;

}

bool checkForBMXYSensors(){
  dprintln("\n[checkForBMXYSensors] -- START --");

  for(uint8_t t=0x76;t<=0x77;t++){
    uint8_t sensID=BMXY80_SensorID(t);
    dprintln("[checkForBMXYSensors] SensID for Adress 0x"+String(t,HEX)+": 0x"+String(sensID,HEX));
    switch(sensID){
      case 0x56:
        dprintln("[checkForBMXYSensors] We have a BMP280 (NO HUMIDITY - 0x56)!");
        BMXYAdress=t;
        BME280_init();
        BMP280Attached=HIGH;
        return true;
        break;
      case 0x57:
        dprintln("[checkForBMXYSensors] We have a BMP280 (NO HUMIDITY - 0x57)!");
        BMXYAdress=t;
        BME280_init();
        BMP280Attached=HIGH;
        return true;
        break;
      case 0x58:
        dprintln("[checkForBMXYSensors] We have a BMP280 (NO HUMIDITY - 0x58)!");
        BMXYAdress=t;
        BME280_init();
        BMP280Attached=HIGH;
        return true;
        break;
      case 0x60:
        dprintln("[checkForBMXYSensors] We have a BME280 (0x60)!");
        BMXYAdress=t;
        BME280_init();
        BME280Attached=HIGH;
        return true;
        break;
      case 0x61:
        dprintln("[checkForBMXYSensors] We have a BME680 (0x61)!");
        BMXYAdress=t;
        BME680_init();
        BME680Attached=HIGH;
        return true;
        break;
      default:
        break;
    }
  }

  dprintln("[checkForBMXYSensors] Could not find a valid BMXY80 sensor, check wiring, address, sensor ID!");
  dprintln("[checkForBMXYSensors] ID of 0xFF probably means a bad address, a BMP 180 or BMP 085");
  dprintln("[checkForBMXYSensors] -- END  --\n");

  return false;

}

void BMXY80_read(){

  static bool getAltBaseline=true;
  static float AltAtBoot=0;
  
  if(BME280Attached || BMP280Attached){

    if(BMXYAdress==0x76){
      while(!bmx76.hasValue()){
        delay(20);
      }

      bme_temp=bmx76.getTemperature();
      bme_pressure=(bmx76.getPressure()/100.0F);
      bme_alt=BMx_pres2alt(bme_pressure);
      if(BME280Attached){
        bme_humidity=bmx76.getHumidity();
      }
      bmx76.measure(); // re-trigger measurement
    }

    if(BMXYAdress==0x77){
      while(!bmx77.hasValue()){
        delay(20);
      }

      bme_temp=bmx77.getTemperature();
      bme_pressure=(bmx77.getPressure()/100.0F);
      bme_alt=BMx_pres2alt(bme_pressure);
      if(BME280Attached){
        bme_humidity=bmx77.getHumidity();
      }
      bmx77.measure(); // re-trigger measurement
    }
  }

  if(BME680Attached){
    int32_t t,h,p,g;
    BME680.getSensorData(t,h,p,g,true); // last parameter: wait for valid data

    bme_temp=(float)t/100.0;
    bme_humidity=(float)h/1000.0;
    bme_pressure=(float)p/100.0;
    bme_alt=BMx_pres2alt(bme_pressure);
  }

  if(getAltBaseline){ // grabbing the alt/Presssure at boot time
    getAltBaseline=false;
    AltAtBoot=bme_alt;
  }

  bme_HeightOverGround=bme_alt-AltAtBoot; // height over ground

}
/*BMXY80.h************************************************************************/

//#include "GPS.h"******************************************************************/
#define GPSdoDebug

#define syncHysteresis 5 /* max n secs difference */
#define syncHysteresisMaxErrors 10 /* guessing here */
#define GPSDataTimeout 10e3
#define GPSTimeTilWait4GPSIsShown 14000UL

// #define GPSPlausibilityCheckDebug

void blockingNoGPSHardware(){
	
  if(debugMode){ // debugging - that way, we can see stuff without SD card
    dprintln("[blockingNoGPSHardware] Systemstart without GPS (debugMode)");
    return;
  }
  
  dprintln("\n[blockingNoGPSHardware] No Data from a GPS - connect a GPS module and restart");
  
  u8g2.clearBuffer();
  u8g2.drawXBMP(0,0,64,48,errorNoGPS);
  u8g2.sendBuffer();

  for(;;){
    delay(300);
    yield();

    u8g2.setDrawColor(2);
    u8g2.drawBox(0,0,64,48);
    u8g2.updateDisplay();
  }
}

void SyncGPSAndSystemtime(uint32_t interval=10000){
	
  static uint32_t LastRunTime=interval;
  
  if((uint32_t)(millis()-LastRunTime)<interval){
    return;
  }

  LastRunTime=millis();

  if(!GPSValid){ // don't try to sync when GPS data isn't valid
    dprintln("[SyncGPSAndSystemtime] GPSValid false -- no sync");
    return;
  }

  uint8_t hr=gps.time.hour();
  uint8_t mi=gps.time.minute();
  uint8_t sc=gps.time.second();
  uint8_t dy=gps.date.day();
  uint8_t mo=gps.date.month();
  uint16_t yr=gps.date.year();
  auto dateAge=gps.date.age();
  auto timeAge=gps.time.age();

  time_t sysTime=now();
  
  tmElements_t gpsTM;
  gpsTM.Hour=hr;
  gpsTM.Minute=mi;
  gpsTM.Second=sc;
  gpsTM.Day=dy;
  gpsTM.Month=mo;
  gpsTM.Year=yr-1970;

  time_t GPSTime=makeTime(gpsTM);

  dprint("\n[SyncGPSAndSystemtime] sys: ");
  dprint(String(sysTime));
  dprint(" | GPS: ");
  dprint(String(GPSTime));
  dprint(" >> ");

  static bool ForceSync=true;

  static uint16_t syncHysteresisErrors=0;

  if(!ForceSync){ // force sync on the first run of this function
	
    // this construct is nessescary for the date.age or time.age can
    // get bigger than 500 if there's a long wait for the first sync.
    // this way, we choose to sync, no matter how old the GPS data
  
    // if((int)abs(GPSTime-sysTime)>syncHysteresis && sysTime>10000){
    if((int)abs(GPSTime-sysTime)>syncHysteresis){
      dprintln("GPS time may compromised - skipping sync");
      
      syncHysteresisErrors++;

      dprintln("syncHysteresisErrors: "+String(syncHysteresisErrors));
      
      if(syncHysteresisErrors>=(uint16_t)syncHysteresisMaxErrors){
        dprintln("Too many syncHysteresisErrors >> forcing sync on next run");
        ForceSync=true;
        syncHysteresisErrors=0;
      }
      return;
    }

    syncHysteresisErrors=0;
  
    if(dateAge>500){
      dprintln("Skipping Sync - Date Age is > 500");
      return;
    }
  
    if(timeAge>500){
      dprintln("Skipping Sync - Time Age is > 500");
      return;
    }
  }

  ForceSync=false;

  dprintln("Syncing NOW");

  setTime(hr,mi,sc,dy,mo,yr);

  // setting global - anouncing Sync to system
  SysTimeIsValid=true;
}

void GPSLifeSignDebug(){

	#ifdef GPSdoDebug

	static unsigned long GPSLifeSignLastTime=0;
		
		if((unsigned long)(millis()-GPSLifeSignLastTime)>2000){
			GPSLifeSignLastTime=millis();
			dprint("\nGPS Chars Processed: ");
			dprintln(String(gps.charsProcessed()));
			
			dprint("GPS Position LONG: ");
			dprint(String(gps.location.lng(),6));
			dprint(" LAT: ");
			dprintln(String(gps.location.lat(),6));

			dprint("GPS Sattelites: ");
			dprintln(String(gps.satellites.value()));

			dprint("GPS Time: ");
			dprint(withLeadingZero(String(gps.date.day())));
			dprint(".");
			dprint(withLeadingZero(String(gps.date.month())));
			dprint(".");
			dprint(String(gps.date.year()));
			dprint(" ");
			dprint(withLeadingZero(String(gps.time.hour())));
			dprint(":");
			dprint(withLeadingZero(String(gps.time.minute())));
			dprint(":");
			dprintln(withLeadingZero(String(gps.time.second())));

			dprint("GPS DATE isValid: ");
			dprint(String(gps.date.isValid()));
			// dprint(" isUpdated: ");
			// dprint(String(gps.date.isUpdated()));
			dprint(" age: ");
			dprintln(String(gps.date.age()));

			dprint("GPS TIME isValid: ");
			dprint(String(gps.time.isValid()));
			// dprint(" isUpdated: ");
			// dprint(String(gps.time.isUpdated()));
			dprint(" age: ");
			dprintln(String(gps.time.age()));

			// dprint("GPS Time Value: ");          // not sure what this is
			// dprintln(String(gps.time.value()));  // but it's NOT UTC

			dprint("now (SystemTime): ");
			dprint(String(now()));

			dprintln("");
		}
	
	#endif
	
}

void updateGPSSystemStrings(time_t sysTime){ // runs every second from loop_everySecond

	gpsDate=withLeadingZero(String(day(sysTime)));
  gpsDate+=".";
  gpsDate+=withLeadingZero(String(month(sysTime)));
  gpsDate+=".";
	gpsDate+=String(year(sysTime));

  gpsTime=withLeadingZero(String(hour(sysTime)));
  gpsTime+=":";
  gpsTime+=withLeadingZero(String(minute(sysTime)));
  gpsTime+=":";
  gpsTime+=withLeadingZero(String(second(sysTime)));

  gpsFilename=withLeadingZero(String(year(sysTime)-2000));
  gpsFilename+="-";
  gpsFilename+=withLeadingZero(String(month(sysTime)));
  gpsFilename+="-";
  gpsFilename+=withLeadingZero(String(day(sysTime)));

	String ext="_PMX";
	
  if(OPCSensorType==2){ // N3
    ext+="N3";
  }

  if(OPCSensorType==1){ // R1
    ext+="R1";
  }

	if(OPCSensorType==3){ // R2
    ext+="R2";
  }

	String MacID=String(WiFi.macAddress());
  
  MacID.replace(":","");
  MacID=MacID.substring(6);

	ext+="-"+MacID+".csv";

  gpsFilenameNOEXT=gpsFilename;
  gpsFilename=gpsFilenameNOEXT+ext;

}

void GPSFeedTheEncoder(){
  while(serialGPS.available()){
    if(gps.encode(serialGPS.read())){
    }
  }
}

void GPSPlausibilityCheck(){ // called in loopRunAlways

  if(!GPSReady){
    #ifdef GPSPlausibilityCheckDebug
      dprintln("[GPSPlausibilityCheck] exiting: GPSReady=LOW");
    #endif
    return;
  }

  GPSValid=HIGH;

  if(gps.date.year()<compileYear || gps.date.year()>2079){
    #ifdef GPSPlausibilityCheckDebug
      dprintln("[GPSPlausibilityCheck] exiting: gps.date.year="+String(gps.date.year()));
    #endif
    GPSValid=LOW;
    return;
  }
	
  if(gps.date.month()<1 || gps.date.month()>12){
    #ifdef GPSPlausibilityCheckDebug
      dprintln("[GPSPlausibilityCheck] exiting: gps.date.month="+String(gps.date.month()));
    #endif
    GPSValid=LOW;
    return;
  }

  if(gps.date.day()<1 || gps.date.day()>31){
    #ifdef GPSPlausibilityCheckDebug
      dprintln("[GPSPlausibilityCheck] exiting: gps.date.day="+String(gps.date.day()));
    #endif
    GPSValid=LOW;
    return;
  }

  if(gps.time.hour()>24){
    #ifdef GPSPlausibilityCheckDebug
      dprintln("[GPSPlausibilityCheck] exiting: gps.time.hour="+String(gps.time.hour()));
    #endif
    GPSValid=LOW;
    return;
  }

  if(gps.time.minute()>59){
    #ifdef GPSPlausibilityCheckDebug
      dprintln("[GPSPlausibilityCheck] exiting: gps.time.minute="+String(gps.time.minute()));
    #endif
    GPSValid=LOW;
    return;
  }

  if(gps.time.second()>59){
    #ifdef GPSPlausibilityCheckDebug
      dprintln("[GPSPlausibilityCheck] exiting: gps.time.second="+String(gps.time.second()));
    #endif
    GPSValid=LOW;
    return;
  }
	
  tmElements_t gpsTM;

	gpsTM.Hour=gps.time.hour();
  gpsTM.Minute=gps.time.minute();
  gpsTM.Second=gps.time.second();
  gpsTM.Day=gps.date.day();
  gpsTM.Month=gps.date.month();
  gpsTM.Year=gps.date.year()-1970;

  time_t GPSTime=makeTime(gpsTM);

	if(GPSTime<compileDateUXTime){
    #ifdef GPSPlausibilityCheckDebug
			dprintln(F("[GPSPlausibilityCheck] exiting: GPS-Timestamp < compile datestamp"));
		#endif
		GPSValid=LOW;
    return;
	}

  /*
  if(gps.date.year()>2020 && gps.date.year()<2070){
    GPSValid=HIGH;
  }else{
    GPSValid=LOW;
  }
  */
}

void GPSInit(){
	
	#ifdef GPSdoDebug
		dprintln("\n[InitGPSData] START");
		dprintln("[InitGPSData] gps.date.year: "+String(gps.date.year()));
		dprintln("[InitGPSData] gps.date.age: "+String(gps.date.age()));
	#endif
	
	uint32_t checkForValidDataStart=millis();
  
  // while(!gps.date.isValid() || gps.date.year()<2020 || gps.date.year()>2079){
  // while(!(gps.date.year()>2020 && gps.date.year()<2079)){
  while(!gps.date.isValid()){
    GPSFeedTheEncoder();
    if((uint32_t)(millis()-checkForValidDataStart)>GPSDataTimeout && gps.charsProcessed()<10){
			dprintln("[GPSInit] FATAL ERROR: No GPS Data - No GPS unit connected??");
      blockingNoGPSHardware();
      // this is never reached!
			GPSReady=LOW;
    }
		yield();
  }

	#ifdef GPSdoDebug
		dprintln("[InitGPSData] GPS sends data & date.valid - GPSReady=HIGH");
	#endif

  GPSReady=HIGH;
  bool wait4GPSSyncHasBeenDisplayed=false;

  // GPS is connected - lets wait for valid data...

  while(1){ // DISCUSS WITH TEAM!!!!
    GPSFeedTheEncoder();
    GPSLifeSignDebug();
    GPSPlausibilityCheck();
    yield();

    if(wait4GPSSyncHasBeenDisplayed==false && (uint32_t)(millis()-checkForValidDataStart)>GPSTimeTilWait4GPSIsShown){
      wait4GPSSyncHasBeenDisplayed=true;
			#ifdef GPSdoDebug
				dprintln("[InitGPSData] Wait4GpsSync Screen displayed");
			#endif
      u8g2.clearBuffer();
      u8g2.drawXBMP(0,0,64,48,wait4GpsSync);
      u8g2.sendBuffer();
    }

    if(GPSValid){
      break;
    }
  }

  u8g2.clearBuffer();
  u8g2.drawXBMP(0,0,64,48,hsdlogo);
  u8g2.sendBuffer();

  SyncGPSAndSystemtime(0);
	
	dprintln("[GPSInit] GPS OKAY :)\n");

}
/*GPS.h***************************************************************************/
//#include "SD.h"*******************************************************************/
void SDSetSPIMode(){

	// Switch off all SPI devices
	
  digitalWrite(AlphasenseCSPin,HIGH);
  digitalWrite(NRF_CSN,HIGH);
	digitalWrite(sdChipSelectPin,HIGH);
	
	SPI.beginTransaction(SPISettings(1000000UL,MSBFIRST,SPI_MODE0));
	SPI.endTransaction();
	
	delay(1);
}

void blockingWaitForSD(){

  if(wifiMode){ // ignore if we're in WifiMode
    return;
  }

  if(debugMode){ // debugging - that way, we can see stuff without SD card
    dprintln("[blockingWaitForSD] Systemstart without SD (debugMode)");
    return;
  }
  
  dprintln("\n[blockingWaitForSD] Insert SD Card to get the system going");
  
  u8g2.clearBuffer();
  u8g2.drawXBMP(0,0,64,48,NoSD);
  u8g2.sendBuffer();
	
	SDSetSPIMode();

  for(;;){
    if(SD.begin(sdChipSelectPin)){
      break;
    }
    delay(300);
    yield();

    u8g2.setDrawColor(2);
    u8g2.drawBox(0,0,64,48);
    u8g2.updateDisplay();
  }

  dprintln("[blockingWaitForSD] --ESP RESTART--");

  ESP.restart();
  delay(1000);
}

String BuildNewFileHeader(){
  
  String s="";
  s+="HS DUESSELDORF - Umweltmesstechnik "+SysID;

  if(OPCAttached){
    s+=" | "+OPCInfoString;
    s+=" "+OPCSerialNumber;
    s.replace(0x0a,' ');
    s.replace("  ","");
    s.replace(",,","");
  }

  s+="\n";

  if(dataDumpComment!=""){
    s+=dataDumpComment;
    s+="\n";
  }
  
  if(OPCAttached){
    s+="BIN Values are concentration (binX / Sampling Period * Sample Flow Rate) [1/cm³]\n";
  }
 
  s+="GPS date\tGPS Time\tGPS Date And Time (UTC)\tLONG\tLAT\tGPS Speed\tGPS Altitude ASL\tSatellites\tHDOP";

  if(BMP280Attached || BME280Attached || BME680Attached){
    s+="\tTemp";
    if(BME280Attached || BME680Attached){
      s+="\tHumidity";
    }
    s+="\tPressure\tAlt BME";
  }
  
  if(OPCAttached){
    s+="\tPM1\tPM2,5\tPM10";
    
    if(pmaOffsetIsSet && pmaSpanIsSet){ // PM1
      s+="\tPM1_Corrected";
    }

    if(pmbOffsetIsSet && pmbSpanIsSet){ // PM2.5
      s+="\tPM2,5_Corrected";
    }

    if(pmcOffsetIsSet && pmcSpanIsSet){ // PM10
      s+="\tPM10_Corrected";
    }
    
    s+="\tTemp OPC\tHum OPC";
    s+="\tbin0\tbin1\tbin2\tbin3\tbin4\tbin5\tbin6\tbin7\tbin8\tbin9";
    s+="\tbin10\tbin11\tbin12\tbin13\tbin14\tbin15";
    if(OPCSensorType==2){ // it's an N3!
      s+="\tbin16\tbin17\tbin18\tbin19\tbin20\tbin21\tbin22\tbin23";
    }
    s+="\tPNC";

    s+="\tSampling Period\tSample Flow Rate";

  }
 
  if(ADS1115Attached){

    if(adc01DiffMode){
      s+="\tADC01DIFF";  
    }else{
      if(adc0state){
        s+="\tADC0";
      }
      if(adc1state){
        s+="\tADC1";
      }
    }
  
    if(adc23DiffMode){
      s+="\tADC23DIFF";  
    }else{
      if(adc2state){
        s+="\tADC2";
      }
      if(adc3state){
        s+="\tADC3";
      }
    }
  }

  if(SCD30Attached){
    s+="\tCO2";
  }
  
  if(adddDebugDataToSDDump){
    s+="\tReject Count Glitch\tReject Count LONG";
  }

  s+="\n";

  s+="\t\t\t\t\t[km/h]\t\t\t";

  // BMxy80
  if(BMP280Attached || BME280Attached || BME680Attached){
    s+="\t[C]\t[%relH]";
    if(BME280Attached || BME680Attached){
      s+="\t[hPa]";
    }
    s+="\t[m]";
  }
  
  if(OPCAttached){

    s+="\t[ug/m3]\t[ug/m3]\t[ug/m3]";
    
    if(pmaOffsetIsSet && pmaSpanIsSet){
      s+="\t[ug/m3]";
    }

    if(pmbOffsetIsSet && pmbSpanIsSet){
      s+="\t[ug/m3]";
    }

    if(pmcOffsetIsSet && pmcSpanIsSet){
      s+="\t[ug/m3]";
    }

    s+="\t[C]\t[%relH]";

    if(OPCSensorType==1 || OPCSensorType==3){ // R1 or R2
      s+="\t[";s+=String(varsR1.bbd0);s+="]";
      s+="\t[";s+=String(varsR1.bbd1);s+="]";
      s+="\t[";s+=String(varsR1.bbd2);s+="]";
      s+="\t[";s+=String(varsR1.bbd3);s+="]";
      s+="\t[";s+=String(varsR1.bbd4);s+="]";
      s+="\t[";s+=String(varsR1.bbd5);s+="]";
      s+="\t[";s+=String(varsR1.bbd6);s+="]";
      s+="\t[";s+=String(varsR1.bbd7);s+="]";
      s+="\t[";s+=String(varsR1.bbd8);s+="]";
      s+="\t[";s+=String(varsR1.bbd9);s+="]";
      s+="\t[";s+=String(varsR1.bbd10);s+="]";
      s+="\t[";s+=String(varsR1.bbd11);s+="]";
      s+="\t[";s+=String(varsR1.bbd12);s+="]";
      s+="\t[";s+=String(varsR1.bbd13);s+="]";
      s+="\t[";s+=String(varsR1.bbd14);s+="]";
      s+="\t[";s+=String(varsR1.bbd15);s+="]";
    }

    if(OPCSensorType==2){ // N3
      s+="\t[";s+=String(varsN3.bbd0);s+="]";
      s+="\t[";s+=String(varsN3.bbd1);s+="]";
      s+="\t[";s+=String(varsN3.bbd2);s+="]";
      s+="\t[";s+=String(varsN3.bbd3);s+="]";
      s+="\t[";s+=String(varsN3.bbd4);s+="]";
      s+="\t[";s+=String(varsN3.bbd5);s+="]";
      s+="\t[";s+=String(varsN3.bbd6);s+="]";
      s+="\t[";s+=String(varsN3.bbd7);s+="]";
      s+="\t[";s+=String(varsN3.bbd8);s+="]";
      s+="\t[";s+=String(varsN3.bbd9);s+="]";
      s+="\t[";s+=String(varsN3.bbd10);s+="]";
      s+="\t[";s+=String(varsN3.bbd11);s+="]";
      s+="\t[";s+=String(varsN3.bbd12);s+="]";
      s+="\t[";s+=String(varsN3.bbd13);s+="]";
      s+="\t[";s+=String(varsN3.bbd14);s+="]";
      s+="\t[";s+=String(varsN3.bbd15);s+="]";
      s+="\t[";s+=String(varsN3.bbd16);s+="]";
      s+="\t[";s+=String(varsN3.bbd17);s+="]";
      s+="\t[";s+=String(varsN3.bbd18);s+="]";
      s+="\t[";s+=String(varsN3.bbd19);s+="]";
      s+="\t[";s+=String(varsN3.bbd20);s+="]";
      s+="\t[";s+=String(varsN3.bbd21);s+="]";
      s+="\t[";s+=String(varsN3.bbd22);s+="]";
      s+="\t[";s+=String(varsN3.bbd23);s+="]";
    }

    s+="\t\t[s]\t[ml/s]"; // PNC , SFR, SFC 
  }

  if(ADS1115Attached){

    if(adc01DiffMode){
      s+="\t";  
    }else{
      if(adc0state){
        s+="\t";
      }
      if(adc1state){
        s+="\t";
      }
    }
  
    if(adc23DiffMode){
      s+="\t";  
    }else{
      if(adc2state){
        s+="\t";
      }
      if(adc3state){
        s+="\t";
      }
    }
  }

  if(SCD30Attached){
    s+="\t[ppm]";
  }

  s.replace('.',',');

  return s;
}

String BuildDataString(){
  
  time_t sysTime=now();
  
  String prettyGermanTimestring=withLeadingZero(String(day(sysTime)));
  prettyGermanTimestring+=".";
  prettyGermanTimestring+=withLeadingZero(String(month(sysTime)));
  prettyGermanTimestring+=".";
  prettyGermanTimestring+=withLeadingZero(String(year(sysTime)));
  prettyGermanTimestring+=" ";
  prettyGermanTimestring+=withLeadingZero(String(hour(sysTime)));
  prettyGermanTimestring+=":";
  prettyGermanTimestring+=withLeadingZero(String(minute(sysTime)));
  prettyGermanTimestring+=":";
  prettyGermanTimestring+=withLeadingZero(String(second(sysTime)));

  String dstring=gpsDate;
  dstring+="\t";
  dstring+=gpsTime;
  dstring+="\t";
  dstring+=prettyGermanTimestring;
  dstring+="\t";

  String dataString=String(AM_GPS_lng,6);
  dataString+="\t";
  dataString+=String(AM_GPS_lat,6);
  dataString+="\t";
  dataString+=String(AM_GPS_kmph);
  dataString+="\t";
  dataString+=String(AM_GPS_altitude);
  dataString+="\t";
  dataString+=String(AM_GPS_satellites);
  dataString+="\t";
  dataString+=String(AM_GPS_hdop);
  
  if(BMP280Attached || BME280Attached || BME680Attached){
    dataString+="\t";
    dataString+=String(AM_bme_temp);
    dataString+="\t";
    if(BME280Attached || BME680Attached){
      dataString+=String(AM_bme_humidity);
      dataString+="\t";
    }
    dataString+=String(AM_bme_pressure,2);
    dataString+="\t";
    dataString+=String(AM_bme_alt,2);
  }
  
  if(OPCAttached){
    dataString+= "\t";
    
    dataString+=String(AM_AS_pmA,1);dataString+= "\t";
    dataString+=String(AM_AS_pmB,1);dataString+= "\t";
    dataString+=String(AM_AS_pmC,1);dataString+= "\t";
    
    if(pmaOffsetIsSet && pmaSpanIsSet){ // PM1
      dataString+=String(AM_AS_pmA_CORR,1);dataString+= "\t";
    }

    if(pmbOffsetIsSet && pmbSpanIsSet){ // PM2.5
      dataString+=String(AM_AS_pmB_CORR,1);dataString+= "\t";
    }

    if(pmcOffsetIsSet && pmcSpanIsSet){ // PM10
      dataString+=String(AM_AS_pmC_CORR,1);dataString+= "\t";
    }

    dataString+=String(ConvSTtoTemperature(AM_AS_temperature),1);dataString+= "\t";
    dataString+=String(ConvSRHtoRelativeHumidity(AM_AS_humiditiy));dataString+= "\t";
    
    dataString+=String(AM_AS_bin0,1);dataString+= "\t";
    dataString+=String(AM_AS_bin1,1);dataString+= "\t";
    dataString+=String(AM_AS_bin2,1);dataString+= "\t";
    dataString+=String(AM_AS_bin3,1);dataString+= "\t";
    dataString+=String(AM_AS_bin4,1);dataString+= "\t";
    dataString+=String(AM_AS_bin5,1);dataString+= "\t";
    dataString+=String(AM_AS_bin6,1);dataString+= "\t";
    dataString+=String(AM_AS_bin7,1);dataString+= "\t";
    dataString+=String(AM_AS_bin8,1);dataString+= "\t";
    dataString+=String(AM_AS_bin9,1);dataString+= "\t";
    dataString+=String(AM_AS_bin10,1);dataString+= "\t";
    dataString+=String(AM_AS_bin11,1);dataString+= "\t";
    dataString+=String(AM_AS_bin12,1);dataString+= "\t";
    dataString+=String(AM_AS_bin13,1);dataString+= "\t";
    dataString+=String(AM_AS_bin14,1);dataString+= "\t";
    dataString+=String(AM_AS_bin15,1);dataString+= "\t";
    if(OPCSensorType==2){ // it's an N3!
      dataString+=String(AM_AS_bin16,1);dataString+= "\t";
      dataString+=String(AM_AS_bin17,1);dataString+= "\t";
      dataString+=String(AM_AS_bin18,1);dataString+= "\t";
      dataString+=String(AM_AS_bin19,1);dataString+= "\t";
      dataString+=String(AM_AS_bin20,1);dataString+= "\t";
      dataString+=String(AM_AS_bin21,1);dataString+= "\t";
      dataString+=String(AM_AS_bin22,1);dataString+= "\t";
      dataString+=String(AM_AS_bin23,1);dataString+= "\t";
    }
    
    dataString+=String(AM_AS_sumAllBins);dataString+= "\t";

    dataString+=String(AM_AS_period,3);dataString+= "\t";
    dataString+=String(AM_AS_sfr,3);
  }

  // ------------------- ADC -----------------

  if(ADS1115Attached){
    
    if(adc01DiffMode){
      // dataString+="\t";dataString+=String(AM_adc01Diff*ADCFactor); // will produce VOLT
      dataString+="\t";dataString+=String(AM_adc01Diff);              // will produce RAW data
    }else{
      if(adc0state){
        // dataString+="\t";dataString+=String(AM_adc0*ADCFactor);
        dataString+="\t";dataString+=String(AM_adc0);
      }
      if(adc1state){
        // dataString+="\t";dataString+=String(AM_adc1*ADCFactor);
        dataString+="\t";dataString+=String(AM_adc1);
      }
    }
  
    if(adc23DiffMode){
      // dataString+="\t";dataString+=String(AM_adc23Diff*ADCFactor);
      dataString+="\t";dataString+=String(AM_adc23Diff);
    }else{
      if(adc2state){
        // dataString+="\t";dataString+=String(AM_adc2*ADCFactor);
        dataString+="\t";dataString+=String(AM_adc2);
      }
      if(adc3state){
        // dataString+="\t";dataString+=String(AM_adc3*ADCFactor);
        dataString+="\t";dataString+=String(AM_adc3);
      }
    }
  }

  // ------------------- SCDx0 -----------------

  if(SCD30Attached){
    dataString+="\t";dataString+=String(scdx0_co2,0);
  }

  // ------------------- DEBUG -----------------
  
  if(adddDebugDataToSDDump){
    dataString+="\t";
    if(OPCSensorType==2){ // N3
      dataString+=String(histN3.rcg,DEC);dataString+= "\t";
      dataString+=String(histN3.rcl,DEC);
    }
    if(OPCSensorType==1 || OPCSensorType==3){ // R1/2
      dataString+=String(histR1.rcg,DEC);dataString+= "\t";
      dataString+=String(histR1.rcl,DEC);
    }
  }

  dataString.replace('.',',');

  return dstring+dataString;
}

void SDReCheck(){
  
  if(SDCardReady){
    return;
  }
  
	SDSetSPIMode();
  
  if(SD.begin(sdChipSelectPin)){
    SDCardReady=HIGH;
  }else{
    SDCardReady=LOW;    
  }

	yield();

  digitalWrite(sdChipSelectPin,HIGH);
}

void SDWrite(){

	if(!SDCardReady){
    return;
  }

  // if(!GPSValid){
  if(!SysTimeIsValid){
		return;
	}

  #ifdef SD_DoDebug
    dprintln("[SDWrite] --START--");
  #endif

	using namespace sdfat;
	
	SDSetSPIMode();

	boolean doNuFile=false;

	if(!SD.exists(gpsFilename.c_str())){
		doNuFile=true;
	}

	sdfat::File dataFile=SD.open(gpsFilename,FILE_WRITE);

  #ifdef SD_DoDebug
    dprint("[SDWrite] writing to: ");
    dprintln(gpsFilename);
  #endif

	if(dataFile){
		SDWriteOK=HIGH;
		if(doNuFile){
      String newFileHeader=BuildNewFileHeader();
      #ifdef SD_DoDebug
        dprintln("[SDWrite] That's a new file!");
        dprintln("[SDWrite] newFileHeader: ");
        dprintln(newFileHeader);
      #endif
			dataFile.println(newFileHeader);
		}
    String DataString=BuildDataString();
    #ifdef SD_DoDebug
      dprintln("[SDWrite] DataString: ");
      dprintln(DataString);
    #endif
    dataFile.println(DataString);
		dataFile.close();
    #ifdef SD_DoDebug
      dprintln("[SDWrite] data written :)");
    #endif
	}else{
		// Something went wrong while writing
		SDCardReady=LOW;
		SDWriteOK=LOW;
    dprintln("[SDWrite] Error writing to SD :(");
    // blockingWaitForSD(); // 21-07 - group decision
	}
 
	digitalWrite(sdChipSelectPin,HIGH); // deactivate SD per CS
 
  #ifdef SD_DoDebug
    dprintln("[SDWrite] --END--");
  #endif
}
/*SD.h***************************************************************************/
//#include "ADS1115.h"**************************************************************/
void ADS1115init(){
  if(!ADS1115Attached){
    return;
  }
  
  ads.begin();
  ads.setGain(GAIN_TWOTHIRDS);  // 5V mode  -- 6.144V | 0.0001875
//   ads.setGain(GAIN_ONE);     // 3v3 mode -- 4.096V | 0.000125
  
  dprintln("\n[ADS1115init] Status Dump");
  dprintln("[ADS1115init] adc0state >> "+String(adc0state));
  dprintln("[ADS1115init] adc1state >> "+String(adc1state));
  dprintln("[ADS1115init] adc2state >> "+String(adc2state));
  dprintln("[ADS1115init] adc3state >> "+String(adc3state));

  dprintln("[ADS1115init] adc0offset >> "+String(adc0offset));
  dprintln("[ADS1115init] adc1offset >> "+String(adc1offset));
  dprintln("[ADS1115init] adc2offset >> "+String(adc2offset));
  dprintln("[ADS1115init] adc3offset >> "+String(adc3offset));

  dprintln("[ADS1115init] adc01DiffMode >> "+String(adc01DiffMode));
  dprintln("[ADS1115init] adc23DiffMode >> "+String(adc23DiffMode));

  dprintln("[ADS1115init] ADS1115Attached >> "+String(ADS1115Attached));

}

void ADS1115GetData(){
  if(!ADS1115Attached){
    return;
  }

  if(adc01DiffMode){
    adc01Diff=ads.readADC_Differential_0_1();  
		#ifdef ADS1115_DebugCurrentValues
			dprint("[ADS1115GetData] ADC01Diff >> ");
			dprintln(String(adc01Diff));  
		#endif
  }else{
    if(adc0state){
      adc0=ads.readADC_SingleEnded(0);
      adc0=adc0-adc0offset;
			#ifdef ADS1115_DebugCurrentValues
				dprint("[ADS1115GetData] ADC0 >> ");
				dprintln(String(adc0));
			#endif
    }
    if(adc1state){
      adc1=ads.readADC_SingleEnded(1);
      adc1=adc1-adc1offset;
			#ifdef ADS1115_DebugCurrentValues
				dprint("[ADS1115GetData] ADC1 >> ");
				dprintln(String(adc1));
			#endif
    }
  }

  if(adc23DiffMode){
    adc23Diff=ads.readADC_Differential_2_3();
		#ifdef ADS1115_DebugCurrentValues
			dprint("[ADS1115GetData] ADC23Diff >> ");
			dprintln(String(adc23Diff));  
		#endif
  }else{
    if(adc2state){
      adc2=ads.readADC_SingleEnded(2);
      adc2=adc2-adc2offset;
			#ifdef ADS1115_DebugCurrentValues
				dprint("[ADS1115GetData] ADC2 >> ");
				dprintln(String(adc2));
			#endif
    }
    if(adc3state){
      adc3=ads.readADC_SingleEnded(3);
      adc3=adc3-adc3offset;
			#ifdef ADS1115_DebugCurrentValues
				dprint("[ADS1115GetData] ADC3 >> ");
				dprintln(String(adc3));
			#endif
    }
  }
}
/*ADS1115.h************************************************************************/
//#include "iniFile.h"**************************************************************/
void iniFileValidKeyValDebug(String key,String val){
  dprint("[SD_IniFileDataProcessing] KEY: ");
  dprint(key);
  dprint("\t VAL: ");
  dprintln(val);
}

float iniFileStringToFloat(String s){
	s.trim();
	s.replace(",","."); // for us strange germany :)
	float ret=s.toFloat();
	return ret;
}

void SD_IniFileDataProcessing(String key,String val){ // this is a bit tedious
  
  // must be if-constructs, since switch can't do Strings :\
	
	if(key=="pmaOffset"){
		pmaOffset=iniFileStringToFloat(val);
		pmaOffsetIsSet=HIGH;
    iniFileValidKeyValDebug(key,val);
    return;
	}

	if(key=="pmaSpan"){
		pmaSpan=iniFileStringToFloat(val);
		pmaSpanIsSet=HIGH;
    iniFileValidKeyValDebug(key,val);
    return;
	}

	if(key=="pmbOffset"){
		pmbOffset=iniFileStringToFloat(val);
		pmbOffsetIsSet=HIGH;
    iniFileValidKeyValDebug(key,val);
    return;
	}
	
	if(key=="pmbSpan"){
		pmbSpan=iniFileStringToFloat(val);
		pmbSpanIsSet=HIGH;
    iniFileValidKeyValDebug(key,val);
    return;
	}

	if(key=="pmcOffset"){
		pmcOffset=iniFileStringToFloat(val);
		pmcOffsetIsSet=HIGH;
    iniFileValidKeyValDebug(key,val);
    return;
	}

	if(key=="pmcSpan"){
		pmcSpan=iniFileStringToFloat(val);
		pmcSpanIsSet=HIGH;
    iniFileValidKeyValDebug(key,val);
    return;
	}
  
  if(key=="channel"){
    int x=val.toInt();
    if(x>=1 && x<=5){ // channels go from 1 to 5
      nrfPipe=x;
      dprint("[SD_IniFileDataProcessing] setting nrfPipe to ");
      dprintln(String(x));
    }else{
      dprint("[SD_IniFileDataProcessing] nrfPipe can only be values from 1 to 5, not ");
      dprintln(String(x));
    }
    iniFileValidKeyValDebug(key,val);
    return;
  }

  if(key=="sampleEvery"){
    uint16_t v=val.toInt();
    if(v>=2 && v<=(gewichteteMittelBufMaxLen+gewichteteMittelBufMaxLen)){
      sampleEvery=v;
      iniFileValidKeyValDebug(key,val);
    }
    return;
  }
	
  if(key=="sendThisAsXtra"){
	
		val.toLowerCase();
	
		if(val=="adc0"){
			sendThisAsXtra=1;
		}

		if(val=="adc1"){
			sendThisAsXtra=2;
		}

		if(val=="adc2"){
			sendThisAsXtra=3;
		}

    if(val=="adc3"){
      sendThisAsXtra=4;
    }

    if(val=="adcDiff01"){
      sendThisAsXtra=5;
    }

    if(val=="adcDiff23"){
      sendThisAsXtra=6;
    }

    if(val=="co2"){
      sendThisAsXtra=7;
    }

		iniFileValidKeyValDebug(key,val);
    return;
  }

  if(key=="noUpdateCheck"){
    val.toLowerCase(); // to be on the safe side...
    if(val=="on"){
      noUpdateCheck=true;
    }else{
      noUpdateCheck=false;
    }
    iniFileValidKeyValDebug(key,val);
    return;
  }

	if(key=="smoothValuesForDisplay"){
    int x=val.toInt();
    if(x>=2 && x<=10){ // smooting factor go from 2 to 10 -- else it stays 1
      smoothValuesForDisplay=x;
		}
    iniFileValidKeyValDebug(key,val);
    return;
	}
	
  if(key=="adc0state"){
    val.toLowerCase(); // to be on the safe side...
    if(val=="on"){
      adc0state=true;
    }else{
      adc0state=false;
    }
    iniFileValidKeyValDebug(key,val);
    return;
  }
  
  if(key=="adc1state"){
    val.toLowerCase();
    if(val=="on"){
      adc1state=true;
    }else{
      adc1state=false;
    }
    iniFileValidKeyValDebug(key,val);
    return;
  }
  
  if(key=="adc2state"){
    val.toLowerCase();
    if(val=="on"){
      adc2state=true;
    }else{
      adc2state=false;
    }
    iniFileValidKeyValDebug(key,val);
    return;
  }
  
  if(key=="adc3state"){
    val.toLowerCase();
    if(val=="on"){
      adc3state=true;
    }else{
      adc3state=false;
    }
    iniFileValidKeyValDebug(key,val);
    return;
  }

  if(key=="adc01DiffMode"){
    val.toLowerCase();
    if(val=="on"){
      adc01DiffMode=true;
    }else{
      adc01DiffMode=false;
    }
    iniFileValidKeyValDebug(key,val);
    return;
  }

  if(key=="adc23DiffMode"){
    val.toLowerCase();
    if(val=="on"){
      adc23DiffMode=true;
    }else{
      adc23DiffMode=false;
    }
    iniFileValidKeyValDebug(key,val);
    return;
  }
	
  if(key=="adc0offset"){
    int x=val.toInt();
		x=abs(x); // only positive values
    if(x>=1 && x<=32767){
      adc0offset=x;
      dprint("[SD_IniFileDataProcessing] ADC0 Offset: ");
      dprintln(String(x));
    }else{
      dprint("[SD_IniFileDataProcessing] ADC0 offset out of bounds: ");
      dprint(String(x));
			dprintln(" - Offset stays at 0.");
    }
    iniFileValidKeyValDebug(key,val);
    return;
  }

  if(key=="adc1offset"){
    int x=val.toInt();
		x=abs(x); // only positive values
    if(x>=1 && x<=32767){
      adc1offset=x;
      dprint("[SD_IniFileDataProcessing] ADC1 Offset: ");
      dprintln(String(x));
    }else{
      dprint("[SD_IniFileDataProcessing] ADC1 offset out of bounds: ");
      dprint(String(x));
			dprintln(" - Offset stays at 0.");
    }
    iniFileValidKeyValDebug(key,val);
    return;
  }

  if(key=="adc2offset"){
    int x=val.toInt();
		x=abs(x); // only positive values
    if(x>=1 && x<=32767){
      adc2offset=x;
      dprint("[SD_IniFileDataProcessing] ADC2 Offset: ");
      dprintln(String(x));
    }else{
      dprint("[SD_IniFileDataProcessing] ADC2 offset out of bounds: ");
      dprint(String(x));
			dprintln(" - Offset stays at 0.");
    }
    iniFileValidKeyValDebug(key,val);
    return;
  }

  if(key=="adc3offset"){
    int x=val.toInt();
		x=abs(x); // only positive values
    if(x>=1 && x<=32767){
      adc3offset=x;
      dprint("[SD_IniFileDataProcessing] ADC3 Offset: ");
      dprintln(String(x));
    }else{
      dprint("[SD_IniFileDataProcessing] ADC3 offset out of bounds: ");
      dprint(String(x));
			dprintln(" - Offset stays at 0.");
    }
    iniFileValidKeyValDebug(key,val);
    return;
  }

  if(key=="dataDumpComment"){
    dataDumpComment=val;
    dataDumpComment.replace("_"," "); // Spaces are killed - so replace them with _
    iniFileValidKeyValDebug(key,val);
    return;
  }

  if(key=="dbgDisplay"){ // dbgDisplay -- that's an "inofficial" hack
    val.toLowerCase();
    if(val=="on"){
      dbgDisplay=true;
    }else{
      dbgDisplay=false;
    }
    iniFileValidKeyValDebug(key,val);
    return;
  }

  if(key=="adddDebugDataToSDDump"){ // adddDebugDataToSDDump -- more inofficial hacks :)
    val.toLowerCase();
    if(val=="on"){
      adddDebugDataToSDDump=true;
    }else{
      adddDebugDataToSDDump=false;
    }
    iniFileValidKeyValDebug(key,val);
    return;
  }
	
}

void CheckAndGetIniFile(){

  // SDReCheck();

  dprintln("\n[CheckAndGetIniFile] --START--");

  using namespace sdfat;

  digitalWrite(AlphasenseCSPin,HIGH); // AlphaSense Sensor definitely OFF
  digitalWrite(NRF_CSN,HIGH);
  digitalWrite(NRF_CE,HIGH);

  delay(50);

  if(!SD.begin(sdChipSelectPin)){
    dprintln("[CheckAndGetIniFile] No SD Card - using defaults.");
    dprintln("[CheckAndGetIniFile] --END--\n");
    return; // No SD - use defaults.
  }

  dprintln("[CheckAndGetIniFile] SD found - looking for the ini file...");

  if(!SD.exists(iniFileName)){
    dprintln("[CheckAndGetIniFile] No ini file - using defaults.");
    dprintln("[CheckAndGetIniFile] --END--\n");
    return;
  }

  sdfat::File inifile=SD.open(iniFileName);

  if(!inifile){
    dprintln("[CheckAndGetIniFile] can't access ini file - using defaults.");
    dprintln("[CheckAndGetIniFile] --END--\n");
    return;
  }

  dprintln("[CheckAndGetIniFile] ini file found - grabbing settings...");


  String s,key,val;
	
  while (inifile.available()){
    s=inifile.readStringUntil(13);
		
    // basic sanitize
    s.replace("\n","");
    s.replace("\r","");
    s.replace("\t","");
    s.replace(" ",""); // this although trims ;)
		
    // kill comments
    if(s.indexOf('#')>=0){
      if(s.indexOf('#')==0){
        s="";
      }else{
        s.remove(s.indexOf('#'));
      }
    }
    // separate key Value
    if(s.indexOf('=')>=0){
      key=s.substring(0,s.indexOf('='));
      val=s.substring(s.indexOf('=')+1,s.length());
			key.trim();
			val.trim();
      SD_IniFileDataProcessing(key,val);
    }
  }
  
  inifile.close();
  
  dprintln("[CheckAndGetIniFile] --END--\n");
}
/*iniFile.h************************************************************************/
//#include "NRF24L01.h"*************************************************************/
void NRFSetSPIMode(){ // not needed since NRF is always called after SD

  // Switch off all SPI devices
  
  digitalWrite(AlphasenseCSPin,HIGH);
  digitalWrite(NRF_CSN,HIGH);
  digitalWrite(sdChipSelectPin,HIGH);
  
  SPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE0));
  SPI.endTransaction();
  
  delay(1);
}

void NRFInit(){

  dprint("[NRFInit] ");

  pinMode(NRF_CSN,FUNCTION_3);  // RX
  pinMode(NRF_CE,FUNCTION_3);  // TX

  if(!nrf.begin()){
    dprintln("failed - no NRF Module found");
    return;
  }

  dprintln("Module found - initializing");

  NRF24L01Attached=HIGH;
  
  nrf.openWritingPipe(nrfPipe);
  nrf.setChannel(108);
  nrf.setDataRate(RF24_250KBPS);
  nrf.setAutoAck(false);
  nrf.setPALevel(RF24_PA_MAX);

  dprint("[NRFInit] nrfPipe: ");
  dprintln(uint64ToString(nrfPipe));

  digitalWrite(NRF_CSN,HIGH);
  
}

void NRF24L01UpdateDataStructure(){ // called from loop_ReadData - so this runs every 2 secs
  if(!NRF24L01Attached){
    return;
  }
	
	static bool nrf_toggleBit=LOW;

	nrfData.change=0;
	
	if(OPCSensorType==2){ // N3
		nrfData.sumBins=(float)histN3.sumAllBins;
	}
	if(OPCSensorType==1 || OPCSensorType==3){ // R1/R2
		nrfData.sumBins=(float)histR1.sumAllBins;
	}
	
	nrfData.pm10=smoothDisplayPMC;
	
	nrfData.temp=(float)bme_temp;

	// nrfData.altitude=(float)gps.altitude.meters();
	// nrfData.altitude=bme_alt;
	nrfData.altitude=bme_HeightOverGround;

	nrfData.hum=(float)bme_humidity;
	nrfData.change=0;

	if(ADS1115Attached){
		
		dprint("[updateNRF24L01DataConstruct] ADS1115 is attached. Selecting Data Source for Display: Xtra >> ");

    constexpr int16_t scaleFacForADC=150;
    
		if(sendThisAsXtra==1){
			nrfData.xtra=adc0/scaleFacForADC;
			nrfData.change=1;
			dprint("adc0: ");
      dprint(String(adc0));
      dprint(" (");
      dprint(String(nrfData.xtra));
      dprintln(")");
		}
		if(sendThisAsXtra==2){
			nrfData.xtra=adc1/scaleFacForADC;
			nrfData.change=2;
			dprint("adc1: ");
      dprint(String(adc1));
      dprint(" (");
      dprint(String(nrfData.xtra));
      dprintln(")");
		}
		if(sendThisAsXtra==3){
			nrfData.xtra=adc2/scaleFacForADC;
			nrfData.change=3;
			dprint("adc2: ");
      dprint(String(adc2));
      dprint(" (");
      dprint(String(nrfData.xtra));
      dprintln(")");
		}
		if(sendThisAsXtra==4){
			nrfData.xtra=adc3/scaleFacForADC;
			nrfData.change=4;
			dprint("adc3: ");
      dprint(String(adc3));
      dprint(" (");
      dprint(String(nrfData.xtra));
      dprintln(")");
		}

		if(adc01DiffMode && (sendThisAsXtra==1 || sendThisAsXtra==2)){
			nrfData.xtra=adc01Diff/scaleFacForADC;
			nrfData.change=5;
			dprint("adc01Diff: ");
      dprint(String(adc01Diff));
      dprint(" (");
      dprint(String(nrfData.xtra));
      dprintln(")");
		}

		if(adc23DiffMode && (sendThisAsXtra==3 || sendThisAsXtra==4)){
			nrfData.xtra=adc23Diff/scaleFacForADC;
			nrfData.change=6;
			dprint("adc23Diff: ");
      dprint(String(adc23Diff));
      dprint(" (");
      dprint(String(nrfData.xtra));
      dprintln(")");
		}
	}

	dprint(" (");
	dprint(String(nrfData.xtra,1));
	dprintln(")");

  if(nrf_toggleBit){
    nrfData.change=nrfData.change^0x80;
  }

  nrf_toggleBit=!nrf_toggleBit;
	
}

void NRFsendData(){ // called from loop_runAlways - this runs a lot
  if(!NRF24L01Attached){
    return;
  }

  static unsigned long nrfLastTime=0;

  if((unsigned long)(millis()-nrfLastTime)>10){ // run only every 10ms!
  
    digitalWrite(AlphasenseCSPin,HIGH);
    digitalWrite(sdChipSelectPin,HIGH);

    nrf.write(&nrfData,sizeof(nrfData)); // transmits the whole struct
  
    digitalWrite(NRF_CSN,HIGH);

    nrfLastTime=millis();
  }
}
/*NRF24L01.h***********************************************************************/
//#include "SCD30.h"*****************************************************************/
void scd30Init(){
	dprint("[scd30Init] ");

	if(!SCD30Attached){
		dprintln("No SCD30 found - no INIT");
		return;
	}
	
	dprintln("Initializing...");
	
	if(!scdx0.begin()){
		dprint("[scd30Init] Can't initialize unit - deactivating SCD30 sensor");
		SCD30Attached=LOW;
		return;
	}
}

void scd30GetData(){ // call this one as often as you want/can!

	if(!SCD30Attached){
		return;
	}
	
	if(!scdx0.dataAvailable()){
    return;
  }
	
  scdx0_co2=scdx0.getCO2();
  scdx0_temp=scdx0.getTemperature();
  scdx0_hum=scdx0.getHumidity();
}
/*SCD30.h**************************************************************************/
//#include "i2cButtons.h"************************************************************/
void i2CButtonsService(){
	if(displayButtonsAttached==LOW){
		return;
	}
	
	static unsigned long asyncLoopLastTime=0;

	if((unsigned long)(millis()-asyncLoopLastTime)<I2CButsCheckEvery){
    return;
  }

  asyncLoopLastTime=millis();

  I2Cbuts.get();

  dprintln("|--> i2CButtonsService -- A: "+String(I2Cbuts.BUTTON_A)+" - B: "+String(I2Cbuts.BUTTON_B));

  if(I2Cbuts.BUTTON_A==4 && I2Cbuts.BUTTON_B==4){
    dprintln("Eject SD :)");
		
		u8g2.clearBuffer();
    u8g2.drawXBMP(0,0,64,48,safeRemoveSD);
    u8g2.sendBuffer();
		
    do{ // wait for button release
      I2Cbuts.get();
      yield();
    }while(I2Cbuts.BUTTON_A==4 && I2Cbuts.BUTTON_B==4);

    dprintln("Hold both buttons to reset");

    do{ // wait for both buttons
      I2Cbuts.get();
      yield();
    }while(I2Cbuts.BUTTON_A!=4 && I2Cbuts.BUTTON_B!=4);

    ESP.restart();
    delay(1000);
    
  }
	
}
/*i2cButtons.h*********************************************************************/

//#include "SETUP.h"*****************************************************************/
void setup(){
  
  serialGPS.begin(9600,SWSERIAL_8N1,D4,D4);
	
	SoftwareSerialdbg.begin(19200,SWSERIAL_8N1,SoftwareSerialDebugPin,SoftwareSerialDebugPin); // << This is our DEBUG Port!!
  SoftwareSerialdbg.enableTx(true);

  dprint("\n\n*** HSD UMT PMX ***\nCompiled at: ");
  dprint(String(__DATE__));
	dprint(" | ");
	dprintln(String(__TIME__));

  // Set all SPI Devices CS to OUTPUT and HIGH

  pinMode(AlphasenseCSPin,OUTPUT);
  digitalWrite(AlphasenseCSPin,HIGH);

  pinMode(NRF_CSN,OUTPUT);
  digitalWrite(NRF_CSN,HIGH);
  pinMode(NRF_CE,OUTPUT);
  digitalWrite(NRF_CE,HIGH);

  pinMode(sdChipSelectPin,OUTPUT);
  digitalWrite(sdChipSelectPin,HIGH);

  Wire.begin();

  setCompileYear();
	setCompileDateUXTime();

  checkDevicesConnected();

  DisplayInit();

  // check WifiMode Jumper and set systemwide wifiMode flag

  #ifdef enablewifiModeCheck
    if(analogRead(A0)<50){
      wifiMode=true;
    }
    dprintln("[SETUP] WifiMode: "+String(wifiMode));
  #endif

  dprintln("[SETUP] SDCardReady: "+String(SDCardReady));

  if(SDCardReady==LOW){
    blockingWaitForSD(); // found in helpers.h - waits for SD and does a reset
  }

  CheckAndGetIniFile();

  SetSysID();

  // RunServerOTA and WifiInit need some integration...
  // when - in the distant future - there's a pure Wifi mode,
  // ServerOTA should re-use the wifi connection

  RunServerOTA(); // closes the Wifi Connection if in AP mode!!

  WIFIInit();

  GPSInit();

	checkForBMXYSensors(); 
	
	scd30Init();

  OPCinit();

  ADS1115init();

  NRFInit();

  debugDevicesConnected();

  dprint("\n[SETUP] sampleEvery is set to ");
  dprintln(String(sampleEvery));
}
/*SETUP.h**************************************************************************/
//#include "LOOP.h"******************************************************************/
void debugSystemTime(){
  dprintln("\n[debugSystemTime]"+gpsTime+" "+gpsDate);
}

void initSystemDataFields(){ // clears/Inits all system Data Fields
	AM_GPS_lat=0; // initialize after write
	AM_GPS_lng=0;
	AM_GPS_kmph=0;
	AM_GPS_altitude=0;
	AM_GPS_satellites=0;
	AM_GPS_hdop=0;

	AM_bme_temp=0;
	AM_bme_humidity=0;
	AM_bme_pressure=0;
	AM_bme_alt=0;
	AM_bme_HeightOverGround=0;

	AM_AS_pmA=0;
	AM_AS_pmB=0;
	AM_AS_pmC=0;

	AM_AS_pmA_CORR=0;
	AM_AS_pmB_CORR=0;
	AM_AS_pmC_CORR=0;

	AM_AS_temperature=0;
	AM_AS_humiditiy=0;

	AM_AS_bin0=0;
	AM_AS_bin1=0;
	AM_AS_bin2=0;
	AM_AS_bin3=0;
	AM_AS_bin4=0;
	AM_AS_bin5=0;
	AM_AS_bin6=0;
	AM_AS_bin7=0;
	AM_AS_bin8=0;
	AM_AS_bin9=0;
	AM_AS_bin10=0;
	AM_AS_bin11=0;
	AM_AS_bin12=0;
	AM_AS_bin13=0;
	AM_AS_bin14=0;
	AM_AS_bin15=0;
	AM_AS_bin16=0;
	AM_AS_bin17=0;
	AM_AS_bin18=0;
	AM_AS_bin19=0;
	AM_AS_bin20=0;
	AM_AS_bin21=0;
	AM_AS_bin22=0;
	AM_AS_bin23=0;

	AM_AS_sumAllBins=0;

	AM_AS_period=0;
	AM_AS_sfr=0;

	AM_adc0=0;
	AM_adc1=0;
	AM_adc2=0;
	AM_adc3=0;
	AM_adc01Diff=0;
	AM_adc23Diff=0;
}

void CommulateSystmDataFields(){ // adds to all system data fields current values
	AM_GPS_lat+=gps.location.lat();
	AM_GPS_lng+=gps.location.lng();
	AM_GPS_kmph+=gps.speed.kmph();
	AM_GPS_altitude+=gps.altitude.meters();
	AM_GPS_satellites+=gps.satellites.value();
	AM_GPS_hdop+=gps.hdop.hdop();

	AM_bme_temp+=bme_temp;
	AM_bme_humidity+=bme_humidity;
  AM_bme_pressure+=bme_pressure;
  AM_bme_alt+=bme_alt;
	AM_bme_HeightOverGround+=bme_HeightOverGround;
	
	AM_AS_pmA_CORR+=PMA_corrected;
	AM_AS_pmB_CORR+=PMB_corrected;
	AM_AS_pmC_CORR+=PMC_corrected;

	if(OPCSensorType==2){ // N3
	
		AM_AS_pmA+=histN3.pmA;
		AM_AS_pmB+=histN3.pmB;
		AM_AS_pmC+=histN3.pmC;
		
		AM_AS_temperature+=histN3.temp;
		AM_AS_humiditiy+=histN3.humidity;

		AM_AS_bin0+=histN3.bin0;
		AM_AS_bin1+=histN3.bin1;
		AM_AS_bin2+=histN3.bin2;
		AM_AS_bin3+=histN3.bin3;
		AM_AS_bin4+=histN3.bin4;
		AM_AS_bin5+=histN3.bin5;
		AM_AS_bin6+=histN3.bin6;
		AM_AS_bin7+=histN3.bin7;
		AM_AS_bin8+=histN3.bin8;
		AM_AS_bin9+=histN3.bin9;
		AM_AS_bin10+=histN3.bin10;
		AM_AS_bin11+=histN3.bin11;
		AM_AS_bin12+=histN3.bin12;
		AM_AS_bin13+=histN3.bin13;
		AM_AS_bin14+=histN3.bin14;
		AM_AS_bin15+=histN3.bin15; // we don't care what sensor right here
		AM_AS_bin16+=histN3.bin16; // what fields to use only affects
		AM_AS_bin17+=histN3.bin17; // the sd writing
		AM_AS_bin18+=histN3.bin18;
		AM_AS_bin19+=histN3.bin19;
		AM_AS_bin20+=histN3.bin20;
		AM_AS_bin21+=histN3.bin21;
		AM_AS_bin22+=histN3.bin22;
		AM_AS_bin23+=histN3.bin23;

		AM_AS_sumAllBins+=histN3.sumAllBins;

		AM_AS_period+=histN3.period;
		AM_AS_sfr+=histN3.sfr;
	}

	if(OPCSensorType==1 || OPCSensorType==3){ // R1/R2
	
		AM_AS_pmA+=histR1.pmA;
		AM_AS_pmB+=histR1.pmB;
		AM_AS_pmC+=histR1.pmC;
		
		AM_AS_temperature+=histR1.temp;
		AM_AS_humiditiy+=histR1.humidity;

		AM_AS_bin0+=histR1.bin0;
		AM_AS_bin1+=histR1.bin1;
		AM_AS_bin2+=histR1.bin2;
		AM_AS_bin3+=histR1.bin3;
		AM_AS_bin4+=histR1.bin4;
		AM_AS_bin5+=histR1.bin5;
		AM_AS_bin6+=histR1.bin6;
		AM_AS_bin7+=histR1.bin7;
		AM_AS_bin8+=histR1.bin8;
		AM_AS_bin9+=histR1.bin9;
		AM_AS_bin10+=histR1.bin10;
		AM_AS_bin11+=histR1.bin11;
		AM_AS_bin12+=histR1.bin12;
		AM_AS_bin13+=histR1.bin13;
		AM_AS_bin14+=histR1.bin14;
		AM_AS_bin15+=histR1.bin15;

		AM_AS_sumAllBins+=histR1.sumAllBins;

		AM_AS_period+=histR1.period;
		AM_AS_sfr+=histR1.sfr;
	}

	if(ADS1115Attached){
		
		AM_adc0+=adc0;
		AM_adc1+=adc1;
		AM_adc2+=adc2;
		AM_adc3+=adc3;
		AM_adc01Diff+=adc01Diff;
		AM_adc23Diff+=adc23Diff;
	}
}

void calcSystemDataFields(uint8_t divider){ // does all the math before writing to SD
	AM_GPS_lat=AM_GPS_lat/divider;
	AM_GPS_lng=AM_GPS_lng/divider;
	AM_GPS_kmph=AM_GPS_kmph/divider;
	AM_GPS_altitude=AM_GPS_altitude/divider;
	AM_GPS_satellites=AM_GPS_satellites/divider;
	AM_GPS_hdop=AM_GPS_hdop/divider;

	AM_bme_temp=AM_bme_temp/divider;
	AM_bme_humidity=AM_bme_humidity/divider;
	AM_bme_pressure=AM_bme_pressure/divider;
	AM_bme_alt=AM_bme_alt/divider;
	AM_bme_HeightOverGround=AM_bme_HeightOverGround/divider;

	AM_AS_pmA=AM_AS_pmA/divider;
	AM_AS_pmB=AM_AS_pmB/divider;
	AM_AS_pmC=AM_AS_pmC/divider;

	AM_AS_pmA_CORR=AM_AS_pmA_CORR/divider;
	AM_AS_pmB_CORR=AM_AS_pmB_CORR/divider;
	AM_AS_pmC_CORR=AM_AS_pmC_CORR/divider;

	AM_AS_temperature=AM_AS_temperature/divider;
	AM_AS_humiditiy=AM_AS_humiditiy/divider;

	AM_AS_bin0=AM_AS_bin0/divider;
	AM_AS_bin1=AM_AS_bin1/divider;
	AM_AS_bin2=AM_AS_bin2/divider;
	AM_AS_bin3=AM_AS_bin3/divider;
	AM_AS_bin4=AM_AS_bin4/divider;
	AM_AS_bin5=AM_AS_bin5/divider;
	AM_AS_bin6=AM_AS_bin6/divider;
	AM_AS_bin7=AM_AS_bin7/divider;
	AM_AS_bin8=AM_AS_bin8/divider;
	AM_AS_bin9=AM_AS_bin9/divider;
	AM_AS_bin10=AM_AS_bin10/divider;
	AM_AS_bin11=AM_AS_bin11/divider;
	AM_AS_bin12=AM_AS_bin12/divider;
	AM_AS_bin13=AM_AS_bin13/divider;
	AM_AS_bin14=AM_AS_bin14/divider;
	AM_AS_bin15=AM_AS_bin15/divider;
	AM_AS_bin16=AM_AS_bin16/divider;
	AM_AS_bin17=AM_AS_bin17/divider;
	AM_AS_bin18=AM_AS_bin18/divider;
	AM_AS_bin19=AM_AS_bin19/divider;
	AM_AS_bin20=AM_AS_bin20/divider;
	AM_AS_bin21=AM_AS_bin21/divider;
	AM_AS_bin22=AM_AS_bin22/divider;
	AM_AS_bin23=AM_AS_bin23/divider;

	AM_AS_sumAllBins=AM_AS_sumAllBins/divider;

	AM_AS_period=AM_AS_period/divider;
	AM_AS_sfr=AM_AS_sfr/divider;

	AM_adc0=AM_adc0/divider;
	AM_adc1=AM_adc1/divider;
	AM_adc2=AM_adc2/divider;
	AM_adc3=AM_adc3/divider;
	AM_adc01Diff=AM_adc01Diff/divider;
	AM_adc23Diff=AM_adc23Diff/divider;
}

void loop_runAlways(){
	SyncGPSAndSystemtime(10e3);
	dnsServer.processNextRequest();
  server.handleClient();
  i2CButtonsService();
	NRFsendData();
  GPSFeedTheEncoder();
  GPSPlausibilityCheck();
  yield();
}

void loop_ReadData(time_t sysTime){

  // dprintln("\n[loop_ReadData] Getting Data at ("+withLeadingZero(String(second(sysTime)))+")");

	uint32_t loopDuration=millis();
	
	OPCGetData(); loop_runAlways();
	BMXY80_read(); loop_runAlways();
	ADS1115GetData(); loop_runAlways();
  scd30GetData(); loop_runAlways();
	NRF24L01UpdateDataStructure(); loop_runAlways();
	debugMemState(); loop_runAlways();
	
	dprintln("|-> [loop_ReadData] Getting Data took "+String(millis()-loopDuration)+"ms");
}

void loop_StoreData(time_t sysTime){

  /*
	if(!GPSValid){
		dprintln("|-> [loop_StoreData] GPS not valid jet - so no data to write");
		return;
	}
  */

  if(!SysTimeIsValid){ // this should never happen here...
    dprintln("|-> [loop_StoreData] System Time is not synced jet - so no data to write");
    return;
  }
	
	static bool zeroCrossDetection=false;
	
	if(sampleEvery==2){ // don't wait for any zero crossing
		zeroCrossDetection=true;
    // dprintln("[loop_StoreData] sampleEvery is 2 - no zeroCrossing wait");
	}
	
	if(!zeroCrossDetection){
		if(second(sysTime)==0){
			zeroCrossDetection=true;
		}else{
      // dprintln("[loop_StoreData] wait for zero crossing");
			return;
		}
	}
	
	uint32_t loopDuration=millis();
	
  // dprintln("[loop_StoreData] Storing Data at ("+withLeadingZero(String(second(sysTime)))+")");

	static byte countdown=0;
	static bool skipSDWriteOnFirstRun=true;
	
	// dprintln("[loop_StoreData] countdown: "+String(countdown));
	
	if(countdown==0){
		countdown=sampleEvery/2;
		if(skipSDWriteOnFirstRun==false){
			calcSystemDataFields(countdown);
			SDReCheck();
			loop_runAlways();
			// dprintln("[loop_StoreData] Writing AMs to SD");
			SDWrite();
			loop_runAlways();
		}
		skipSDWriteOnFirstRun=false;
		initSystemDataFields();
	}
	
	CommulateSystmDataFields();
	
	countdown--;

	dprintln("|-> [loop_StoreData] Storing Data took "+String(millis()-loopDuration)+"ms");
}

void loop_everySecond(time_t sysTime){ // runs BEFORE DataGet and DataStore!!
  updateGPSSystemStrings(sysTime);
	DisplayUpdate();
  debugSystemTime();
}

void loop(){ // combined scheduler 21-07-12
	static time_t oldTime=0;
	
  loop_runAlways();

  time_t sysTime=now();

	if(sysTime==oldTime){
		return;
	}
	
	oldTime=sysTime;

  loop_everySecond(sysTime);
	
	if(sysTime % 2==0){
    uint32_t timestamp=millis();
		loop_ReadData(sysTime);
		loop_StoreData(sysTime);
    dprintln("|-> [LOOP] Data Read & Store took "+String(millis()-timestamp)+"ms");
	}
}
/*LOOP.h***************************************************************************/
