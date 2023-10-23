#pragma once
/*
		opcn3m.h Functions to operate the Alphasense OPC-N3 Particle counter..
	
    based on the work of David H Hagan and Marcelo Yungaicela
		
		Modified for N3 by Detlef Amend, August 2019
		
*/

#define _ASSpiBegin SPI.beginTransaction(SPISettings(600000UL, MSBFIRST, SPI_MODE1));
// #define _ASSpiBegin SPI.beginTransaction(); // set the SPI parameters in main program

#include <SPI.h>
#include "opcn3.h"

OPCN3::OPCN3(uint8_t chip_select){
  // Initiate an instance of the opcn3 class
  // Ex. opcn3 alpha(chip_select = A2);
  _CS = chip_select;
 
  pinMode(_CS, OUTPUT);
  SPI.begin();
  firm_ver.major = 99;
  firm_ver.minor = 99;
}

uint16_t OPCN3::_16bit_int(byte LSB, byte MSB){
  // Combine two bytes into a 16-bit unsigned int
  return ((MSB << 8) | LSB);
}

bool OPCN3::_compare_arrays(byte array1[], byte array2[], int length){
  // Compare two arrays and return a boolean
  bool result = true;

  for (int i = 0; i < length; i++){
    if (array1[i] != array2[i]){
      result = false;
    }
  }

  return result;
}

float OPCN3::_calculate_float(byte val0, byte val1, byte val2, byte val3){
  // Return an IEEE754 float from an array of 4 bytes
  union u_tag {
    byte b[4];
    float val;
  } u;

  u.b[0] = val0;
  u.b[1] = val1;
  u.b[2] = val2;
  u.b[3] = val3;

  return u.val;
}

uint32_t OPCN3::_32bit_int(byte val0, byte val1, byte val2, byte val3){
  // Return a 32-bit unsigned int from 4 bytes
  return ((val3 << 24) | (val2 << 16) | (val1 << 8) | val0);
}

bool OPCN3::_waitForGo(byte val){
	
	byte loopCounter=0;
  byte ret;

	do{ // wait for Return is 0xF3
		delay(12);
		digitalWrite(this->_CS, LOW);
		ret=SPI.transfer(val);
		digitalWrite(this->_CS, HIGH);

		if(loopCounter++>20){
			return false;
		}
   
	}while(ret!=0xF3);

	delay(12);
	
	return true;
	
}

float OPCN3::_oneDec(float a){ // One Decimal 
  return round(a*10)/10;
}

bool OPCN3::on(){
  // Turn ON the OPC and return a boolean
  // Ex.
  // $ alpha.on()
  // $ true
	
	_ASSpiBegin
	
	if(this->_waitForGo(0x03)){
		digitalWrite(this->_CS, LOW);
		SPI.transfer(0x07); // Laser ON
		digitalWrite(this->_CS, HIGH);
	}else{
		SPI.endTransaction();
		return false;
	}
	
	delay(2000);
	
	if(this->_waitForGo(0x03)){
		digitalWrite(this->_CS, LOW);
		SPI.transfer(0x03); // Fan ON
		digitalWrite(this->_CS, HIGH);
	}else{
		SPI.endTransaction();
		return false;
	}
	
	SPI.endTransaction();	
	
	delay(5000);
	
	return true;
	
}

String OPCN3::read_information_string(){
  // Read the information string and return a string
  // Ex.
  // $ alpha.read_information_string()

	String result="";
  byte ret;
	
	_ASSpiBegin
	
	if(this->_waitForGo(0x3F)){

		digitalWrite(this->_CS, LOW);

		// Iterate to read the entire string
		for (int i=0;i<60;i++){
			ret=SPI.transfer(0x3F);
			result+=String((char)ret);
			delayMicroseconds(10);
		}

		digitalWrite(this->_CS, HIGH);
		
		SPI.endTransaction();

		return result;
		
	}else{
		return "Error Reading Info String";
	}
}

String OPCN3::read_serial_number_string(){
  // Read the serial number string and return a string
  // Ex.
  // $ alpha.read_information_string()

  String result="";
  byte ret;
	
	_ASSpiBegin
  
  if(this->_waitForGo(0x10)){
    digitalWrite(this->_CS,LOW);
    delayMicroseconds(10);
    for(int i=0;i<60;i++){
      ret=SPI.transfer(0x10);
      result+=String((char)ret);
      delayMicroseconds(10);
    }
    digitalWrite(this->_CS,HIGH);

    SPI.endTransaction();

    return result;
    
  }else{
    return "Error Reading Serial Number String";
  }
}

struct ConfigVarsN3 OPCN3::read_configuration_variables(){
  // Read the configuration variables and return the structure
  // Ex.
  // $ alpha.read_configuration_variables();
  ConfigVarsN3 results;       // empty structure for the data
  byte vals[168];
	
	byte ret=0;
	byte loopCounter=0;
	
	results.DataValid=false;
	
	_ASSpiBegin

	do{ // wait for Return is 0xF3
		delay(10);
		digitalWrite(this->_CS, LOW);
		ret=SPI.transfer(0x3c);
		digitalWrite(this->_CS, HIGH);

		if(loopCounter++>20){
			return results;
		}
		
	}while(ret!=0xf3);
  
	delay(10);
	
	results.DataValid=true;

  digitalWrite(this->_CS, LOW);
  for (int i = 0; i < 169; i++){
    vals[i] = SPI.transfer(0x3c);
    delayMicroseconds(8);
  }
  digitalWrite(this->_CS, HIGH);
	SPI.endTransaction();

  // Fill in the results
  results.bb0 = this->_16bit_int(vals[0], vals[1]);
  results.bb1 = this->_16bit_int(vals[2], vals[3]);
  results.bb2 = this->_16bit_int(vals[4], vals[5]);
  results.bb3 = this->_16bit_int(vals[6], vals[7]);
  results.bb4 = this->_16bit_int(vals[8], vals[9]);
  results.bb5 = this->_16bit_int(vals[10], vals[11]);
  results.bb6 = this->_16bit_int(vals[12], vals[13]);
  results.bb7 = this->_16bit_int(vals[14], vals[15]);
  results.bb8 = this->_16bit_int(vals[16], vals[17]);
  results.bb9 = this->_16bit_int(vals[18], vals[19]);
  results.bb10 = this->_16bit_int(vals[20], vals[21]);
  results.bb11 = this->_16bit_int(vals[22], vals[23]);
  results.bb12 = this->_16bit_int(vals[24], vals[25]);
  results.bb13 = this->_16bit_int(vals[26], vals[27]);
  results.bb14 = this->_16bit_int(vals[28], vals[29]);
  results.bb15 = this->_16bit_int(vals[30], vals[31]);
  results.bb16 = this->_16bit_int(vals[32], vals[33]);
  results.bb17 = this->_16bit_int(vals[34], vals[35]);
  results.bb18 = this->_16bit_int(vals[36], vals[37]);
  results.bb19 = this->_16bit_int(vals[38], vals[39]);
  results.bb20 = this->_16bit_int(vals[40], vals[41]);
  results.bb21 = this->_16bit_int(vals[42], vals[43]);
  results.bb22 = this->_16bit_int(vals[44], vals[45]);
  results.bb23 = this->_16bit_int(vals[46], vals[47]);
  results.bb24 = this->_16bit_int(vals[48], vals[49]);
	
  results.bbd0 = (double)this->_16bit_int(vals[50], vals[51])/100;
  results.bbd1 = (double)this->_16bit_int(vals[52], vals[53])/100;
  results.bbd2 = (double)this->_16bit_int(vals[54], vals[55])/100;
  results.bbd3 = (double)this->_16bit_int(vals[56], vals[57])/100;
  results.bbd4 = (double)this->_16bit_int(vals[58], vals[59])/100;
  results.bbd5 = (double)this->_16bit_int(vals[60], vals[61])/100;
  results.bbd6 = (double)this->_16bit_int(vals[62], vals[63])/100;
  results.bbd7 = (double)this->_16bit_int(vals[64], vals[65])/100;
  results.bbd8 = (double)this->_16bit_int(vals[66], vals[67])/100;
  results.bbd9 = (double)this->_16bit_int(vals[68], vals[69])/100;
  results.bbd10 = (double)this->_16bit_int(vals[70], vals[71])/100;
  results.bbd11 = (double)this->_16bit_int(vals[72], vals[73])/100;
  results.bbd12 = (double)this->_16bit_int(vals[74], vals[75])/100;
  results.bbd13 = (double)this->_16bit_int(vals[76], vals[77])/100;
  results.bbd14 = (double)this->_16bit_int(vals[78], vals[79])/100;
  results.bbd15 = (double)this->_16bit_int(vals[80], vals[81])/100;
  results.bbd16 = (double)this->_16bit_int(vals[82], vals[83])/100;
  results.bbd17 = (double)this->_16bit_int(vals[84], vals[85])/100;
  results.bbd18 = (double)this->_16bit_int(vals[86], vals[87])/100;
  results.bbd19 = (double)this->_16bit_int(vals[88], vals[89])/100;
  results.bbd20 = (double)this->_16bit_int(vals[90], vals[91])/100;
  results.bbd21 = (double)this->_16bit_int(vals[92], vals[93])/100;
  results.bbd22 = (double)this->_16bit_int(vals[94], vals[95])/100;
  results.bbd23 = (double)this->_16bit_int(vals[96], vals[97])/100;
  results.bbd24 = (double)this->_16bit_int(vals[98], vals[99])/100;
  
  // Bin Weightings (floats) (100-147)
  results.bw0 = (double)this->_16bit_int(vals[100], vals[101])/100;
  results.bw1 = (double)this->_16bit_int(vals[102], vals[103])/100;
  results.bw2 = (double)this->_16bit_int(vals[104], vals[105])/100;
  results.bw3 = (double)this->_16bit_int(vals[106], vals[107])/100;
  results.bw4 = (double)this->_16bit_int(vals[108], vals[109])/100;
  results.bw5 = (double)this->_16bit_int(vals[110], vals[111])/100;
  results.bw6 = (double)this->_16bit_int(vals[112], vals[113])/100;
  results.bw7 = (double)this->_16bit_int(vals[114], vals[115])/100;
  results.bw8 = (double)this->_16bit_int(vals[116], vals[117])/100;
  results.bw9 = (double)this->_16bit_int(vals[118], vals[119])/100;
  results.bw10 = (double)this->_16bit_int(vals[120], vals[121])/100;
  results.bw11 = (double)this->_16bit_int(vals[122], vals[123])/100;
  results.bw12 = (double)this->_16bit_int(vals[124], vals[125])/100;
  results.bw13 = (double)this->_16bit_int(vals[126], vals[127])/100;
  results.bw14 = (double)this->_16bit_int(vals[128], vals[129])/100;
  results.bw15 = (double)this->_16bit_int(vals[130], vals[131])/100;
  results.bw16 = (double)this->_16bit_int(vals[132], vals[133])/100;
  results.bw17 = (double)this->_16bit_int(vals[134], vals[135])/100;
  results.bw18 = (double)this->_16bit_int(vals[136], vals[137])/100;
  results.bw19 = (double)this->_16bit_int(vals[138], vals[139])/100;
  results.bw20 = (double)this->_16bit_int(vals[140], vals[141])/100;
  results.bw21 = (double)this->_16bit_int(vals[142], vals[143])/100;
  results.bw22 = (double)this->_16bit_int(vals[144], vals[145])/100;
  results.bw23 = (double)this->_16bit_int(vals[146], vals[147])/100;
 
  // Particle mass Concentration (floats) (148-153)
	results.M_A = (float)this->_16bit_int(vals[148], vals[149])/100;
  results.M_B = (float)this->_16bit_int(vals[150], vals[151])/100;
  results.M_C = (float)this->_16bit_int(vals[152], vals[153])/100;
	
	// Max Time of Flight (154-155)
	results.maxTOF=(double)this->_16bit_int(vals[154], vals[155])/48; // 48 >> see AlphaSense Sample Code
	
	// AM Sampling Interval Count (156-157)
	results.AMSamplingIntervalCount = (unsigned int)this->_16bit_int(vals[156], vals[157]);

	// AM Idle Interval Count (158-159)
	results.AMIdleIntervalCount = (unsigned int)this->_16bit_int(vals[158], vals[159]);

	// AM Max Data Arrays In File (160-161)
	results.AMMaxDataArraysInFile = (unsigned int)this->_16bit_int(vals[160], vals[161]);

	// AM Only Save PM Data (162)
	results.AMOnlySavePMData =byte(vals[162]);

	// AM Fan On In Idle (163)
	results.AMFanOnInIdle = byte(vals[163]);

	// AM Laser On In Idle (164)
	results.AMLaserOnInIdle =byte(vals[164]);

	// Time of Flight to Sample Flow Rate conversion factor (165)
	results.tof_sfr =byte(vals[165]);

	// Particle Validation Period (166)
	results.pvp = float(vals[166]/48);

	//BinWeightingIndex (167)
	results.bwi = byte(vals[167]);

  return results;
}

struct PMDataN3 OPCN3::read_pm_data(){
  // Read the PM Data and reset the histogram, return the struct
  // Only available on opcn3's with firmware >v18
  // Ex.
  // $ alpha.read_pm_data();
  PMDataN3 data;
  byte vals[14];
	
	byte ret=0;
	byte loopCounter=0;
	data.DataValid=false;
	
	_ASSpiBegin
	
  // Read the data and clear the local memory
  do{ // wait for Return is 0xF3
		delay(12);
		digitalWrite(this->_CS, LOW);
		ret=SPI.transfer(0x32);
		digitalWrite(this->_CS, HIGH);
		if(loopCounter++>20){
			return data;
		}
	}while(ret!=0xF3);
	
	data.DataValid=true;

  // Send commands and build array of data
  digitalWrite(this->_CS, LOW);

  for (int i = 0; i < 14; i++){
      vals[i] = SPI.transfer(0x32);
      delayMicroseconds(10);
  }

  digitalWrite(this->_CS, HIGH);

	SPI.endTransaction();

  data.pm1  = this->_calculate_float(vals[0], vals[1], vals[2], vals[3]);
  data.pm25 = this->_calculate_float(vals[4], vals[5], vals[6], vals[7]);
  data.pm10 = this->_calculate_float(vals[8], vals[9], vals[10], vals[11]);

  return data;
}

struct HistogramDataN3 OPCN3::read_histogram(bool convert_to_conc){
  // Read the Histogram Data and reset the histogram, return the struct
  // convert_to_conc can be set to true if you would like the result
  // returned as concentrations (rather than raw counts) with units of
  // particles per cubic centimeter [#/cc]
  // Ex.
  // $ alpha.read_histogram(true)
	
  HistogramDataN3 data;
  byte vals[86];
	
	byte ret=0;
	byte loopCounter=0;
	data.DataValid=false;
	
	_ASSpiBegin
	
	do{
		delay(10);
		digitalWrite(this->_CS, LOW);
		ret=SPI.transfer(0x30);
		digitalWrite(this->_CS, HIGH);
		if(loopCounter++>20){
			return data;
		}
	}while(ret!=0xF3);
	
	data.DataValid=true;

	delayMicroseconds(8);

  // Send commands and build array of data
  digitalWrite(this->_CS, LOW);
  for (int i = 0; i < 86; i++){
      vals[i] = SPI.transfer(0x30);
      delayMicroseconds(8);
  }
  digitalWrite(this->_CS, HIGH);
	
	SPI.endTransaction();

	// Sampling Period
	data.period=(float)this->_16bit_int(vals[52], vals[53]) / 100;
	
	// Sample Flow Rate
	data.sfr=(double)this->_16bit_int(vals[54], vals[55]) / 100;

  // If convert_to_conc = True, convert from raw data to concentration
  double conv;

  if ( convert_to_conc != true ) {
      conv = 1.0;
  }
  else {
      conv = data.sfr * data.period;
  }

  // Calculate all of the values!
	data.bin0   = this->_oneDec(((float)this->_16bit_int(vals[0],vals[1])/conv));
  data.bin1   = this->_oneDec(((float)this->_16bit_int(vals[2],vals[3])/conv));
  data.bin2   = this->_oneDec(((float)this->_16bit_int(vals[4],vals[5])/conv));
  data.bin3   = this->_oneDec(((float)this->_16bit_int(vals[6],vals[7])/conv));
  data.bin4   = this->_oneDec(((float)this->_16bit_int(vals[8],vals[9])/conv));
  data.bin5   = this->_oneDec(((float)this->_16bit_int(vals[10],vals[11])/conv));
  data.bin6   = this->_oneDec(((float)this->_16bit_int(vals[12],vals[13])/conv));
  data.bin7   = this->_oneDec(((float)this->_16bit_int(vals[14],vals[15])/conv));
  data.bin8   = this->_oneDec(((float)this->_16bit_int(vals[16],vals[17])/conv));
  data.bin9   = this->_oneDec(((float)this->_16bit_int(vals[18],vals[19])/conv));
  data.bin10  = this->_oneDec(((float)this->_16bit_int(vals[20],vals[21])/conv));
  data.bin11  = this->_oneDec(((float)this->_16bit_int(vals[22],vals[23])/conv));
  data.bin12  = this->_oneDec(((float)this->_16bit_int(vals[24],vals[25])/conv));
  data.bin13  = this->_oneDec(((float)this->_16bit_int(vals[26],vals[27])/conv));
  data.bin14  = this->_oneDec(((float)this->_16bit_int(vals[28],vals[29])/conv));
  data.bin15  = this->_oneDec(((float)this->_16bit_int(vals[30],vals[31])/conv));
	data.bin16  = this->_oneDec(((float)this->_16bit_int(vals[32],vals[33])/conv));
  data.bin17  = this->_oneDec(((float)this->_16bit_int(vals[34],vals[35])/conv));
  data.bin18  = this->_oneDec(((float)this->_16bit_int(vals[36],vals[37])/conv));
  data.bin19  = this->_oneDec(((float)this->_16bit_int(vals[38],vals[39])/conv));
	data.bin20  = this->_oneDec(((float)this->_16bit_int(vals[40],vals[41])/conv));
  data.bin21  = this->_oneDec(((float)this->_16bit_int(vals[42],vals[43])/conv));
  data.bin22  = this->_oneDec(((float)this->_16bit_int(vals[44],vals[45])/conv));
  data.bin23  = this->_oneDec(((float)this->_16bit_int(vals[46],vals[47])/conv));

	int sumupInt=round(data.bin0*10.0);
  sumupInt+=round(data.bin1*10.0);
  sumupInt+=round(data.bin2*10.0);
  sumupInt+=round(data.bin3*10.0);
  sumupInt+=round(data.bin4*10.0);
  sumupInt+=round(data.bin5*10.0);
  sumupInt+=round(data.bin6*10.0);
  sumupInt+=round(data.bin7*10.0);
  sumupInt+=round(data.bin8*10.0);
  sumupInt+=round(data.bin9*10.0);
  sumupInt+=round(data.bin10*10.0);
  sumupInt+=round(data.bin11*10.0);
  sumupInt+=round(data.bin12*10.0);
  sumupInt+=round(data.bin13*10.0);
  sumupInt+=round(data.bin14*10.0);
  sumupInt+=round(data.bin15*10.0);
  sumupInt+=round(data.bin16*10.0);
  sumupInt+=round(data.bin17*10.0);
  sumupInt+=round(data.bin18*10.0);
  sumupInt+=round(data.bin19*10.0);
	sumupInt+=round(data.bin20*10.0);
	sumupInt+=round(data.bin21*10.0);
  sumupInt+=round(data.bin22*10.0);
  sumupInt+=round(data.bin23*10.0);

  data.sumAllBins=sumupInt/10.0;

	// Mass Time-of-Flight
	data.bin1MToF = float(vals[48]) / 3.0;
  data.bin3MToF = float(vals[49]) / 3.0;
  data.bin5MToF = float(vals[50]) / 3.0;
  data.bin7MToF = float(vals[51]) / 3.0;
	
	// Temperature
	data.temp=this->_16bit_int(vals[56], vals[57]);
	
	// Humidity
	data.humidity=this->_16bit_int(vals[58], vals[59]);
	
	// PM's
	data.pmA = this->_calculate_float(vals[60], vals[61], vals[62], vals[63]);
  data.pmB = this->_calculate_float(vals[64], vals[65], vals[66], vals[67]);
  data.pmC = this->_calculate_float(vals[68], vals[69], vals[70], vals[71]);

	// Reject Count Glitch
	data.rcg=(unsigned int)this->_16bit_int(vals[72], vals[73]);
	
	//Reject count LongTOF
	data.rcl=(unsigned int)this->_16bit_int(vals[74], vals[75]);
	
	//Reject Count Ratio
	data.rcr=(unsigned int)this->_16bit_int(vals[76], vals[77]);
	
	//Reject Count OutOfRange
	data.rcoor=(unsigned int)this->_16bit_int(vals[78], vals[79]);
	
	//Fan Rev Count
	data.fanr=(unsigned int)this->_16bit_int(vals[80], vals[81]);
	
	//Laser Status
	data.laser=(unsigned int)this->_16bit_int(vals[82], vals[83]);
	
	// Checksum
	data.checksum=(unsigned int)this->_16bit_int(vals[84], vals[85]);
	
  return data;
}
