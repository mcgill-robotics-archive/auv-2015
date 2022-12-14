/******************************************************************************
MS5803_I2C.h
Library for MS5803 pressure sensors.
Casey Kuhns @ SparkFun Electronics
6/26/2014
https://github.com/sparkfun/MS5803-14BA_Breakout

The MS58XX MS57XX and MS56XX by Measurement Specialties is a low cost I2C pressure
sensor.  This sensor can be used in weather stations and for altitude
estimations. It can also be used underwater for water depth measurements. 

In this file are the function prototypes in the MS5803 class 

Resources:
This library uses the Arduino Wire.h to complete I2C transactions.

Development environment specifics:
	IDE: Arduino 1.0.5
	Hardware Platform: Arduino Pro 3.3V/8MHz
	MS5803 Breakout Version: 1.0

**Updated for Arduino 1.6.4 5/2015**

This code is beerware. If you see me (or any other SparkFun employee) at the
local pub, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef MS5803_I2C_h
#define MS5803_I2C_h

#include <Arduino.h>

// Define units for conversions. 
enum temperature_units
{
	CELSIUS,
	FAHRENHEIT,
};

// Define measurement type.
enum measurement
{	
	PRESSURE = 0x00,
	TEMPERATURE = 0x10
};

// Define constants for Conversion precision
enum precision
{
	ADC_256  = 0x00,
	ADC_512  = 0x02,
	ADC_1024 = 0x04,
	ADC_2048 = 0x06,
	ADC_4096 = 0x08
};

// Define address choices for the device (I2C mode)
enum ms5803_addr
{
	ADDRESS_HIGH = 0x76,
	ADDRESS_LOW  = 0x77
};

//Commands
#define CMD_RESET 0x1E // reset command 
#define CMD_ADC_READ 0x00 // ADC read command 
#define CMD_ADC_CONV 0x40 // ADC conversion command 

#define CMD_PROM 0xA0 // Coefficient location


class MS5803
{
	public:	
		MS5803(ms5803_addr address); 
		void reset(void);	 //Reset device
		uint8_t begin(void); // Collect coefficients from sensor
		
		// Return calculated temperature from sensor
		float getTemperature(temperature_units units);
		// Return calculated pressure from sensor
		float getPressure();
                void getMeasurements(precision _precision);
                int32_t getRawTemperature();
                int32_t getRawPressure();
                int8_t getSensorStatus();


	private:
                int8_t _sensor_status;
                int32_t _temperature_raw;
                int32_t _pressure_raw;

		float _temperature_actual;
		float _pressure_actual;
	
		ms5803_addr _address; 		// Variable used to store I2C device address.
		uint16_t coefficient[8];// Coefficients;
		
		

		void sendCommand(uint8_t command);	// General I2C send command function
		uint32_t getADCconversion(measurement _measurement, precision _precision);	// Retrieve ADC result

		void sensorWait(uint8_t time); // General delay function see: delay()
};

#endif
