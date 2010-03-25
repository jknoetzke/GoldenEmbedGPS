//**********************************************************
//
//             SHT15 Humidity Sensor Library
//						SHT15.h
//                     Ryan Owens
//			Copyright Sparkfun Electronics
//
//**********************************************************

//This file is used to define the functions  for the SHT15 library
//as well as the register addresses.

//**********************************************************
//
//                  Command Definitions
//
//**********************************************************
#define CHECK_TEMP 0x03
#define CHECK_HUMD 0x05
#define CHECK_STAT 0x07
#define WRITE_STAT 0x06

//**********************************************************
//
//                  	Function Protocols
//
//**********************************************************
char sht15_read_byte(void);
void sht15_send_byte(char sht15_command);
void sht15_start(void);
void sht15_read(unsigned int *temperature, unsigned int *humidity);
