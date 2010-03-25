/*
	ADXL345 Library
	
	This libary contains functions to interact with the ADXL345 Triple Axis Digital Accelerometer from Analog Devices written for the ATmega328p
	In order to use this libary, define the appropriate pins in the ADXL345.h file

	created 20 Aug 2009
	by Ryan Owens
	http://www.sparkfun.com
 
*/
#include "ADXL345.h"
#include <stdlib.h>
#include <stdio.h>
#include "LPC214x.h"
#include "spi0.h"
#include "PackageTracker.h"
#include "rprintf.h"


void initAccel(void){

	adxl345_write(DATA_FORMAT, RANGE_1);	//Configure the Accelerometer for +/-8g
	
	//Set Accel. to Interrupt.  Interrupt will occur on EINT2 pin.
	adxl345_write(THRESH_FF, 0x0E);			//Set Accelerometer Threshold to 600 mg
	//adxl345_write(THRESH_FF, 0x14);			//Set Accelerometer Threshold to 600 mg
	
	adxl345_write(TIME_FF, 0x0A);			//Free Fall will trigger after falling for a minimum of  100ms.	
	
	adxl345_write(BW_RATE, 0x07);			//Set Output Rate to 100 Hz
	adxl345_write(INT_MAP, ~FREE_FALL);		//Map the Free Fall interrupt to pin INT1; all other interrupts to INT2
	adxl345_write(INT_ENABLE, FREE_FALL);	//Activate the 'Free Fall' Interrupt
	adxl345_write(POWER_CTL, MEASURE);		//Put the Accelerometer into measurement mode	
}

int accelX(void){
	char high_byte, low_byte=0;
	int value=0;
	
	high_byte = adxl345_read(DATAX1);
	low_byte = adxl345_read(DATAX0);
	value = (high_byte << 8) | low_byte;	
	
	return value;
}	

int accelY(void){	
	char high_byte, low_byte=0;
	int value=0;
	
	high_byte = adxl345_read(DATAY1);
	low_byte = adxl345_read(DATAY0);
	value = (high_byte << 8) | low_byte;	
	
	return value;
}

int accelZ(void){	
	char high_byte, low_byte=0;
	int value=0;
	
	high_byte = adxl345_read(DATAZ1);
	low_byte = adxl345_read(DATAZ0);
	value = (high_byte << 8) | low_byte;	
	
	return value;
}

void powerdownAccel(void){
	SelectAccelerometer();
	//SPI0_send(WRITE | Ctrl_Reg1);
	//SPI0_send(~PD);
	UnselectAccelerometer();
}

char adxl345_read(char register_address){
	char read_address=0x80 | register_address;
	char register_value=0;
	int spcr_setting=0;
	
	spcr_setting = S0SPCR;	//Save the current SPI Control Register Settings
	S0SPCR  = 0x38;         // Master, no interrupt enable, 8 bits, Active Low SCK pin, CPHA=1	
		
	SelectAccelerometer();
	delay_ms(1);
	SPI0_send(read_address);
	register_value=SPI0_recv();
	delay_ms(1);
	UnselectAccelerometer();
	
	S0SPCR = spcr_setting;
	return register_value;
}

void adxl345_write(char register_address, char register_value){
	int spcr_setting=0;
	
	spcr_setting = S0SPCR;	//Save the current SPI Control Register Settings
	S0SPCR  = 0x38;         // Master, no interrupt enable, 8 bits, Active Low SCK pin, CPHA=1	
		
	SelectAccelerometer();
	delay_ms(1);
	SPI0_send(register_address);
	SPI0_send(register_value);
	delay_ms(1);
	UnselectAccelerometer();
	
	S0SPCR = spcr_setting;
}
