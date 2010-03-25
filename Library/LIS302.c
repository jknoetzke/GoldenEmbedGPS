//**********************************************************
//
//             LIS30 Accelerometer Library
//						LIS302.c
//                     Ryan Owens
//			Copyright Sparkfun Electronics
//
//**********************************************************
//NOTE: SelectAccel and UnSelectAccel macros must be externally
//		defined.  They should assert and de-assert the CS line
//		respectively.

#include "LPC214x.h"
#include "LIS302.h"
#include "spi0.h"
#include "PackageTracker.h"

void initAccel(void){
	WriteLis302Register(Ctrl_Reg1, PD | FS | XEN | YEN | ZEN);
	WriteLis302Register(Ctrl_Reg2, HP_FF_WU1 | FDS | HP_coeff1 | HP_coeff2);
	WriteLis302Register(Ctrl_Reg3, I1CFG0);

}

char accelX(void){
	return ReadLis302Register(OutX);
}	

char accelY(void){	
	return ReadLis302Register(OutY);
}

char accelZ(void){	
	return ReadLis302Register(OutZ);
}

void powerdownAccel(void){
	SelectAccelerometer();
	SPI0_send(WRITE | Ctrl_Reg1);
	SPI0_send(~PD);
	UnselectAccelerometer();
}

char ReadLis302Register(char register_name){
	char register_contents;

	SelectAccelerometer();
	SPI0_send(READ | register_name);
	register_contents=SPI0_recv();
	UnselectAccelerometer();
	return register_contents;
}

void WriteLis302Register(char register_name, char register_value){
	SelectAccelerometer();
	SPI0_send(WRITE | register_name);
	SPI0_send(register_value);
	UnselectAccelerometer();
}
