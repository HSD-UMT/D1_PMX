/*
    OPCN3.h - Library for operating the Alphasense OPC-N3 Particle counter.
    Created by David H Hagan, March 2016.
    Modified by Marcelo Yungaicela, May 2017
		Modified from N2 to N3 by Detlef Amend, August 2019
    Released with an MIT license.
*/

#ifndef Opcn3_h
#define Opcn3_h

// Includes

struct FirmwareN3{
    int major;
    int minor;
};

struct HistogramDataN3{
		bool DataValid; // set TRUE on succesful initiating Data Transfer
    
		double bin0;
    double bin1;
    double bin2;
    double bin3;
    double bin4;
    double bin5;
    double bin6;
    double bin7;
    double bin8;
    double bin9;
    double bin10;
    double bin11;
    double bin12;
    double bin13;
    double bin14;
    double bin15;
    double bin16;
    double bin17;
    double bin18;
    double bin19;
    double bin20;
    double bin21;
    double bin22;
    double bin23;
		
		double sumAllBins; // Sum of all BINs

    // Mass Time-of-Flight
    float bin1MToF;
    float bin3MToF;
    float bin5MToF;
    float bin7MToF;

		// Sample Flow Rate
    float sfr;

		// Temperature
		unsigned int temp;
		
		// Humidity
		unsigned int humidity;
		
		// Sampling Period
    float period;

		// Reject Count Glitch
		unsigned int rcg;
		
		//Reject Count LongTOF
		unsigned int rcl;

		// PM's
    float pmA;
    float pmB;
    float pmC;

		// Checksum
    unsigned int checksum;

		// From here on, the vals are N3 only - we never use them anyway :)
		
		//Reject Count Ratio
		unsigned int rcr;

		//Reject Count OutOfRange
		unsigned int rcoor;

		//Fan Rev Count
		unsigned int fanr;

		//Laser Status
		unsigned int laser;
};

struct PMDataN3{
	bool DataValid; // set TRUE on succesful initiating Data Transfer
  float pm1;
  float pm25;
  float pm10;
};

struct ConfigVarsN3{
		
		boolean DataValid; // set TRUE on succesful initiating Data Transfer
		
    // Bin Boundaries (0-49)
    unsigned int bb0;
    unsigned int bb1;
    unsigned int bb2;
    unsigned int bb3;
    unsigned int bb4;
    unsigned int bb5;
    unsigned int bb6;
    unsigned int bb7;
    unsigned int bb8;
    unsigned int bb9;
    unsigned int bb10;
    unsigned int bb11;
    unsigned int bb12;
    unsigned int bb13;
    unsigned int bb14;
    unsigned int bb15;
    unsigned int bb16;
    unsigned int bb17;
    unsigned int bb18;
    unsigned int bb19;
    unsigned int bb20;
    unsigned int bb21;
    unsigned int bb22;
    unsigned int bb23;
    unsigned int bb24;

    // Bin Boundaries diameter (um / floats) (50-99)
    float bbd0;
    float bbd1;
    float bbd2;
    float bbd3;
    float bbd4;
    float bbd5;
    float bbd6;
    float bbd7;
    float bbd8;
    float bbd9;
    float bbd10;
    float bbd11;
    float bbd12;
    float bbd13;
    float bbd14;
    float bbd15;
    float bbd16;
    float bbd17;
    float bbd18;
    float bbd19;
    float bbd20;
    float bbd21;
    float bbd22;
    float bbd23;
    float bbd24;

    // Bin Weightings (floats) (100-147)
    float bw0;
    float bw1;
    float bw2;
    float bw3;
    float bw4;
    float bw5;
    float bw6;
    float bw7;
    float bw8;
    float bw9;
    float bw10;
    float bw11;
    float bw12;
    float bw13;
    float bw14;
    float bw15;
    float bw16;
    float bw17;
    float bw18;
    float bw19;
    float bw20;
    float bw21;
    float bw22;
    float bw23;

		// Particle mass Concentration (floats) (148-153)
    float M_A;
    float M_B;
    float M_C;

		// Particle Validation Period (166)
    float pvp;
		
		// Max Time of Flight (154-155)
    float maxTOF;

		// AM Sampling Interval Count (156-157)
    unsigned int AMSamplingIntervalCount;

		// AM Idle Interval Count (158-159)
    unsigned int AMIdleIntervalCount;

		// AM Max Data Arrays In File (160-161)
    unsigned int AMMaxDataArraysInFile;

		// AM Only Save PM Data (162)
    char AMOnlySavePMData;

		// AM Fan On In Idle (163)
    char AMFanOnInIdle;

		// AM Laser On In Idle (164)
    char AMLaserOnInIdle;

		// Time of Flight to Sample Flow Rate conversion factor (165)
    char tof_sfr;

    //BinWeightingIndex (167)
		char bwi;


		// R1 Only 
		
		// Gain Scaling Coefficient
    float gsc;
		
		// Sample Flow Rate (ml/s)
    float sfr;

		// Power Status
    byte powerStatus;
		
		// LaserDAC 8 bit int
    byte laser_dac;

};

class OPCN3
{
private:
    // attributes
    uint8_t _CS;
    int _fv;

    // methods
    uint16_t _16bit_int(byte MSB, byte LSB);
    bool _compare_arrays(byte array1[], byte array2[], int length);
    float _calculate_float(byte val0, byte val1, byte val2, byte val3);
    uint32_t _32bit_int(byte val0, byte val1, byte val2, byte val3);
		bool _waitForGo(byte val);
		float _oneDec(float a);

public:
    OPCN3(uint8_t chip_select);

    // attributes
    FirmwareN3 firm_ver;

    bool on();

    String read_information_string();
    String read_serial_number_string();
		/*
    FirmwareN3 read_firmware_version();
    ConfigVarsN3 read_configuration_variables();
    PMDataN3 read_pm_data();
    HistogramDataN3 read_histogram(bool convert_to_conc = true);
		*/
    // Firmware read_firmware_version();
    ConfigVarsN3 read_configuration_variables();
    PMDataN3 read_pm_data();
    HistogramDataN3 read_histogram(bool convert_to_conc = true);
};

#endif
