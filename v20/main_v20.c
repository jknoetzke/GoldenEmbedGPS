/*
 * Copyright (c) 2011 Justin F. Knotzke (jknotzke@shampoo.ca)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


//*******************************************************
//                 Package Tracker Firmware
//*******************************************************
#include <stdio.h>
#include <string.h>
#include "LPC214x.h"
#include "serial.h"
#include "rprintf.h"
#include "spi0.h"
#include "target.h"
#include "main_msc.h"
#include "PackageTracker.h"

//*******************************************************
//                  Memory Management Libraries
//*******************************************************
#include "rootdir.h"
#include "sd_raw.h"
#include "fat16.h"

//*******************************************************
//                   External Component Libs
//*******************************************************
#include "ADXL345.h"
#include "SCP1000.h"
#include "SHT15.h"
#include "gps.h"

//*******************************
//       ANT+ Defines
//*******************************
#define MESG_NETWORK_KEY_ID      0x46
#define MESG_TX_SYNC 0xA4
#define MESG_BROADCAST_DATA_ID 0x4E
#define MESG_RESPONSE_EVENT_ID 0x40
#define MESG_CHANNEL_EVENT_ERROR 0x01
#define chanType  0x00 // ChanType
#define netNum    0x00    // NetNum
#define DEVTYPE_HRM	0x78	/* ANT+ HRM */
#define DEVTYPE_BIKE	0x79	/* ANT+ Bike speed and cadence */
#define DEVTYPE_PWR	0x0b	/* ANT+ Power meter */
#define DEVPERIOD_HRM	0x86	/* ANT+ HRM */
#define DEVPERIOD_PWR	0xf6	/* ANT+ Power meter */

#define FALSE 0
#define TRUE 1

int inMsg = FALSE;
int msgN = 0;
int size = 0;
int currentChannel=0;
int isBroadCast = FALSE;

//*******************************************************
//                  Core Functions
//*******************************************************


void bootUp(void);
static void ISR_RxData0(void);
static void ISR_RxData1(void);
void createLogFile(void);
int parseRMC(const char *gps_string);
void saveData(struct fat16_file_struct **fd, const char * const buf, const int buf_size);
void itoa(int n, char s[]);
void reverse(char s[]);
void reset(void);
void initializeGps(void);

void flashBoobies(int num_of_times);
void ANTAP1_Config(void);
void ANTAP1_Reset(void);
void ANTAP1_AssignCh(unsigned char);
void ANTAP1_SetChId(unsigned char, unsigned char deviceType, unsigned char deviceNum[]);
void ANTAP1_SetChRFFreq(unsigned char);
void ANTAP1_SetChPeriod(unsigned char, unsigned char);
void ANTAP1_OpenCh(unsigned char);
void ANTAP1_AssignNetwork(unsigned char);
void ANTAP1_SetSearchTimeout(unsigned char);
void ANTAP1_RequestChanID(unsigned char);
void must_we_write(void);
char parseANT(unsigned char chr);
void printDebug(char *debug, int _size);

//*******************************************************
//                    Global Variables
//*******************************************************

//GPS variables
char gps_message_complete=0, new_gps_data=0, RTC_Set=0, alarm_set,ant_message_complete=FALSE;        //Notification Flags
char gps_message[GPS_BUFFER_SIZE]; 
unsigned char ant_message[GPS_BUFFER_SIZE];      //Buffers for holding GPS messages
int gps_message_index=0, gps_message_size=0, ant_message_index=0, ant_message_size=0;    //index for copying messages to different buffers
int final_gps_message_size=0;
GPSdata GPS, safeGPS;    //GPS Struct to hold GPS coordinates.  See PackageTracker.h for Structure definition

//Logging Parameters
char file_name[32];
int log_count;  //Keeps track of how many logs we've made since we've been awake.  Get's reset before going to sleep.

//Log Parameters for logging the Sensor Data
struct fat16_file_struct * LOG_FILE; //File structure for current log file
char log_data[1024];//log_buffer holds data before putting it into log_data
int log_data_index = 0;     //Keeps track of current position in log_data

char wroteGPS = FALSE;

char led_blink=0;

//JFK ID's
unsigned char HRM[2] = { 0x79, 0x94 };
unsigned char CINQO[2] = { 0xf1, 0x2d};

int main (void)
{
    //*******************************************************
    //                     Main Code
    //*******************************************************
    //Initialize ARM I/O
    bootUp();               //Init. I/O ports, Comm protocols and interrupts
    createLogFile();        //Create a new log file in the root of the SD card

    //Initialize the GPS
    initializeGps();           //Send the initialization strings
    enable_gps_rmc_msgs(1);

    VICIntEnable |=  UART1_INT; //Enable UART1 Interrupt
    flashBoobies(5); //We are alive !

    while(TRUE)
    {
        if(gps_message_complete==1)
        {   //If we've received a new GPS message, record it.
            //Populate GPS struct with time, position, date, speed
	    VICIntEnClr |= UART1_INT;
	    if(parseRMC(gps_message))
            {
		//printDebug("Out ParseRMC\n", 13);
		gps_message_complete = FALSE;
                
		 //Copy over into safeGPS, the data is valid.
                for(int i = 0; i < 9; i++)
                    safeGPS.Latitude.position[i] = GPS.Latitude.position[i];

                for(int i = 0; i < 9; i++)
                    safeGPS.Longitude.position[i] = GPS.Longitude.position[i];

                for(int i = 0; i < 4; i++)
                    safeGPS.Speed[i] = GPS.Speed[i];

                for(int i=0; i<6; i++)
                    safeGPS.Date[i] = GPS.Date[i];

                for(int i=0; i<10; i++)
                    safeGPS.Time[i] = GPS.Time[i];

                if(wroteGPS == FALSE)
		{
		    
                    VICIntEnable |= UART0_INT; //Enable ANT+ 
                    ANTAP1_Config();
                    wroteGPS = TRUE;
                }
            }
            VICIntEnable |= UART1_INT;
	}

        if(ant_message_complete == TRUE)
        {
	    VICIntEnClr |= UART0_INT | UART1_INT;
            for(int i=0; i< ant_message_index; i++)
            {
                log_data[log_data_index++]=ant_message[i];
                ant_message[i]='\0';
                ant_message_complete = FALSE;
            }
            ant_message_index=0;

            VICIntEnable |= UART0_INT | UART1_INT;
        }

        //Only Save Data if the buffer is full! This saves write cycles to the SD card
        if(log_data_index >= MAX_BUFFER_SIZE)
        {
            VICIntEnClr |= UART0_INT | UART1_INT;
            UnselectSCP();
            saveData(&LOG_FILE, log_data, log_data_index);
            VICIntEnable |= UART0_INT | UART1_INT;
            SelectSCP();
            unselect_card();
            SCPinit();
            delay_ms(10);
            for(int i=0; i<log_data_index; i++)log_data[i]='\0';
            log_data_index=0;
            new_gps_data=0; //We've saved the GPS coordinates, so clear the GPS data flag
        }
    }
    return 0;
}


//Usage: delay_ms(1000);
//Inputs: int count: Number of milliseconds to delay
//The function will cause the firmware to delay for "count" milleseconds.
void delay_ms(int count)
{
    int i;
    count *= 10000;
    for (i = 0; i < count; i++)
        asm volatile ("nop");
}

//Usage: bootUp();
//Inputs: None
//This function initializes the serial port, the SD card, the I/O pins and the interrupts
void bootUp(void)
{
    //Initialize UART for RPRINTF
    rprintf_devopen(putc_serial1); //Init rprintf
    init_serial1(4800);

    init_serial0(4800);
    delay_ms(100); //Delay for power to stablize

    //Bring up SD and FAT
    if(!sd_raw_init())
    {
        reset();
    }
    if(openroot())
    {
        reset();
    }
    
    //Initialize I/O Ports and Peripherals
    IODIR0 = SCLK | MOSI | SD_CS | ACCEL_CS | GPS_EN | I2C_SCL | LED;
    IODIR0 &= ~(MISO | SCP_DRDY | ACCEL_INT2 | ACCEL_INT1 | BATT_MEAS);

    IODIR1 = SCP_EN | SCP_CS;

    //Make sure peripheral devices are not selected
    UnselectAccelerometer();
    UnselectSCP();

    //Initialize the SPI bus
    SPI0_Init();                    //Select pin functions for SPI signals.
    S0SPCCR = 64;           // SCK = 1 MHz (60MHz / 64 ~= 1Mhz)
    S0SPCR  = 0x20;         // Master, no interrupt enable, 8 bits

    //Setup the Interrupts
    //Enable Interrupts
    VPBDIV=1;                                                                               // Set PCLK equal to the System Clock
    VICIntSelect = ~(UART0_INT | UART1_INT);
    VICVectCntl0 = 0x20 | 6;                                                //Set up the UART1 interrupt
    VICVectAddr0 = (unsigned int)ISR_RxData0;
    VICVectCntl4 = 0x20 | 7;                                                //Set up the UART0 interrupt
    VICVectAddr4 = (unsigned int)ISR_RxData1;

    //Setup the UART1 Interrupt
    U1IER = 0x01;                           //Enable FIFO on UART with RDA interrupt (Receive Data Available)
    U1FCR &= 0x3F;                          //Enable FIFO, set RDA interrupt for 1 character

    //Setup the UART0 Interrupt
    U0IER = 0x01;                           //Enable FIFO on UART with RDA interrupt (Receive Data Available)
    U0FCR &= 0x3F;                          //Enable FIFO, set RDA interrupt for 1 character

    //Set up the RTC so it can be used for sleeping
    CCR = ~(1<<0);                          //use the system clock, and disable RTC for now
    CIIR = 0;                                       //Don't allow any increment interrupts
    AMR = ~0;                          //Don't check the minutes value of the alarm
    //Set up prescaler so RTC runs at 32.768 Khz
    PREINT = 1830;                          //Prescale Integer = (60MHz/32768)-1
    PREFRAC = 1792;                         //Prescale Fraction = 60MHz - ((PREINT+1)*32768)
}

//Usage: None (Automatically Called by FW)
//Inputs: None
//Description: Called when a character is received on UART0.
static void ISR_RxData0(void)
{
    unsigned char val = (unsigned char)U0RBR;
    ant_message[ant_message_index++] = val;
    ant_message_complete = parseANT(val); //Only throw to log_data if the message is complete
    if(ant_message_complete == TRUE)
    { 
        for(int i = 0; i < 9; i++)
            ant_message[ant_message_index++]=safeGPS.Latitude.position[i];

        for(int i = 0; i < 9; i++)
            ant_message[ant_message_index++]=safeGPS.Longitude.position[i];

        for(int i = 0; i < 4; i++)
            ant_message[ant_message_index++]=safeGPS.Speed[i];

        for(int i=0; i<6; i++)
            ant_message[ant_message_index++]=safeGPS.Date[i];

        for(int i=0; i<10; i++)
            ant_message[ant_message_index++]=safeGPS.Time[i];

        ant_message[ant_message_index++] = 0x0A;
    }

    VICVectAddr =0;                                         //Update the VIC priorities
}

//Usage: None (Automatically Called by FW)
//Inputs: None
//Description: Called when a character is received on UART1.
static void ISR_RxData1(void)
{
    char val = (char)U1RBR;

    //When we get a character on UART1, save it to the GPS message buffer
    if(val=='\n'){  //Newline means the current message is complete
        gps_message[gps_message_index]= val;
        gps_message_complete=1;                                 //Set a flag for the main FW
        gps_message_size=gps_message_index+1;
        gps_message_index=0;
    }
    else{
        //If we get the start character, reset the index
        gps_message_complete=0;
        if(val == '$')gps_message_index=0;
        gps_message[gps_message_index++]= val;
    }

    VICVectAddr =0;                                         //Update the VIC priorities
}

//Usage: createLogFile();
//Inputs: None
//Outputs: None
//Description: Creates a log file in the root directory of the SD card with the name
//                              PackageTrackerXX.csv.  XX increments to the next available number each
//                              time the function is called.
void createLogFile(void){
    int file_number = 0;

    //Create the Sensor Data Log File
    //Set an initial file name
    sprintf(file_name, "ANTGPS%03d.gep", file_number);
    //Check to see if the file already exists in the root directory.
    while(root_file_exists(file_name))
    {
        file_number++;  //If the file already exists, increment the file number and check again.
        if(file_number == 25)
        {
	    reset();
        }
        sprintf(file_name, "ANTGPS%03d.gep", file_number);
    }
    //Get the file handle of the new file.  We will log the data to this file
    LOG_FILE = root_open_new(file_name);
    sd_raw_sync();
}

//Usage: parseRMC(final_message);
//Inputs: const char *gps_string - RMC NMEA string
//This functions splits a GGA message into the
//portions and assigns them to components of
//a GPS structure
int parseRMC(const char *gps_string){
    int i=0;
    int comma_count=0, character_count=0;

    for(int j=0; gps_string[j]!= '\n'; j++){
        if(gps_string[j] == ',')comma_count+=1;
    }

    //If we didn't receive all of the RMC fields, then return an error
    if(comma_count != 11)return 0;
    //If we didn't receive the correct SiRF header, the return an error
    if(gps_string[0] != '$' || gps_string[1] != 'G' || gps_string[2] != 'P')return 0;

    //Only RMC type messages please.
    if(gps_string[0] == '$' && gps_string[1] == 'G' && gps_string[2] == 'P' && gps_string[2] == 'G')return 0;

    //Parse the GGA Message.  1st portion dismissed
    while(gps_string[i] != ',')i++;
    i++;
    //Second portion is UTC timestamp
    for(int j=0;gps_string[i] != ','; j++){
        GPS.Time[j]=gps_string[i];
        i++;
        character_count+=1;
    }
    //Make sure we received 10 character for Time
    if(character_count != 10)return 0;
    character_count=0;

    i++;
    //Third portion is fix
    while(gps_string[i] != ','){
        GPS.Fix=gps_string[i];
        i++;
    }
    i++;
    if(GPS.Fix != 'A')return 0;

    //Fourth portion is Latitude
    for(int j=0;gps_string[i] != ',';j++){
        GPS.Latitude.position[j]=gps_string[i];
        i++;
        character_count +=1;
    }
    //Make sure we received 9 characters for the Latitude
    if(character_count != 9)return 0;
    character_count=0;
    i++;
    //Fifth portion is Latitude direction
    for(int j=0;gps_string[i] != ','; j++){
        GPS.Latitude.direction=gps_string[i];
        i++;
    }
    i++;
    //Sixth portion is Long.
    for(int j=0;gps_string[i] != ','; j++){
        GPS.Longitude.position[j]=gps_string[i];
        i++;
        character_count++;
    }
    //Make sure we received 10 characters for longitude
    if(character_count != 10)return 0;
    character_count=0;

    i++;
    //Seventh portion is Long direction
    while(gps_string[i] != ','){
        GPS.Longitude.direction=gps_string[i];
        i++;
    }
    i++;
    //8th portion Speed.
    for(int j=0;gps_string[i] != ','; j++)
    {
        GPS.Speed[j]=gps_string[i];
        i++;
    }
    i++;
    //9th portion dismissed
    while(gps_string[i] != ',')i++;
    i++;
    //10th portion is Date
    for(int j=0;gps_string[i] != ','; j++){
        GPS.Date[j]=gps_string[i];
        i++;
    }

    return 1;
}

//Usage: saveData(log_data, log_data_index);
//Inputs: char *buf - character array to be saved
//                int buf_size - size of character array
//Output: buffer array is saved to LOG_FILE
//Description: Saves the buf character array to the SD card.
//CONDITIONS: LOG_FILE must be initialized to the handle of an open file.
void saveData(struct fat16_file_struct **fd, const char * const buf, const int buf_size)
{
    int error=0;

    if((buf_size > 0) && (*fd != NULL))
    {
        //Try writing the data to the card up to 10 times.
        while(error<10)
        {
            if(fat16_write_file(*fd, (const unsigned char*)buf, buf_size) < 0)
                error++;
            else
                break;
            delay_ms(100);
        }
        //If we've tried writing the data 10 times and still haven't succeeded, reset the device.
        if(error==10)
        {
            reset();
        }
        error=0;
        //Try syncing the card up to 10 times
        while(error<10){
            if(!sd_raw_sync())error+=1;
            else break;
            delay_ms(100);
        }
        //If we've tried syncing 10 times and still haven't succeeded, reset the device
        if(error==10)
        {
            reset();
        }
        flashBoobies(1);
    }
}

//Usage: itoa(batt_level, log_buffer)
//Inputs: int n - integer to convert
//Outputs:  char s[]-contains ascii rerpresentation of 'n'
/* itoa:  convert n to characters in s */
void itoa(int n, char s[])
{
    int i=0;

    do {       /* generate digits in reverse order */
        s[i++] = n % 10 + '0';   /* get next digit */
    } while ((n /= 10) > 0);     /* delete it */

    s[i] = '\0';
    reverse(s);
}

//Usage: reverse(s);
//Inputs: char s[] - contains a character string
//Outputs: char s[] - Reversed the order of original characters
/* reverse:  reverse string s in place */
void reverse(char s[])
{
    int c, i, j;

    for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
        c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}

//Usage: reset();
//Inputs: None
//Description: Resets the LPC2148
void reset(void)
{
    while(1)
    {
        flashBoobies(1);
        delay_ms(100);
    }
}

void initializeGps(void){
    //Initialize the GPS receiver
    GPSon();
    delay_ms(200);

    disable_all_gps_msgs();
    delay_ms(200);

    enable_waas();
    delay_ms(200);

}

void ANTAP1_Config (void)
{
    flashBoobies(2);
    ANTAP1_Reset();
    delay_ms(50);

    //HR
    ANTAP1_AssignCh(0x00);
    delay_ms(50);
    ANTAP1_SetChId(0x00,DEVTYPE_HRM, HRM);
    delay_ms(50);
    ANTAP1_AssignNetwork(0x00);
    delay_ms(50);
    ANTAP1_SetSearchTimeout(0x00);
    delay_ms(50);
    ANTAP1_SetChRFFreq(0x00);
    delay_ms(50);
    ANTAP1_SetChPeriod(0x00, DEVPERIOD_HRM);
    delay_ms(50);
    ANTAP1_OpenCh(0x00);
    delay_ms(50);

    //Power
    ANTAP1_AssignCh(0x01);
    delay_ms(50);
    ANTAP1_SetChId(0x01,DEVTYPE_PWR, CINQO);
    delay_ms(50);
    ANTAP1_AssignNetwork(0x01);
    delay_ms(50);
    ANTAP1_SetSearchTimeout(0x01);
    delay_ms(50);
    ANTAP1_SetChRFFreq(0x01);
    delay_ms(50);
    ANTAP1_SetChPeriod(0x01, DEVPERIOD_PWR);
    delay_ms(50);
    ANTAP1_OpenCh(0x01);
    delay_ms(50);
}

void ANTAP1_SetSearchTimeout(unsigned char chan)
{
    unsigned char i;
    unsigned char setup[6];

    setup[0] = 0xa4; // SYNC Byte
    setup[1] = 0x02; // LENGTH Byte
    setup[2] = 0x44; // ID Byte
    setup[3] = chan; // Channel
    setup[4] = 0x1e;
    setup[5] = (0xa4^0x02^0x44^chan^0x1e);  // Checksum

    for(i = 0 ; i < 6 ; i++)
        putc_serial0(setup[i]);

}


// Resets module
void ANTAP1_Reset (void)
{
    unsigned char i;
    unsigned char setup[5];

    setup[0] = 0xa4; // SYNC Byte
    setup[1] = 0x01; // LENGTH Byte
    setup[2] = 0x4a; // ID Byte
    setup[3] = 0x00; // Data Byte N (N=LENGTH)
    setup[4] = 0xef; // Checksum

    for(i = 0 ; i < 5 ; i++)
        putc_serial0(setup[i]);
}

void ANTAP1_AssignNetwork(unsigned char chan)
{
    unsigned char i;
    unsigned char setup[13];

    setup[0] = 0xa4; //Sync
    setup[1] = 0x09; //Length
    setup[2] = MESG_NETWORK_KEY_ID;
    setup[3] = chan; //chan
    setup[4] = 0xb9;
    setup[5] = 0xa5;
    setup[6] = 0x21;
    setup[7] = 0xfb;
    setup[8] = 0xbd;
    setup[9] = 0x72;
    setup[10] = 0xc3;
    setup[11] = 0x45;
    setup[12] = (0xa4^0x09^MESG_NETWORK_KEY_ID^chan^0xb9^0xa5^0x21^0xfb^0xbd^0x72^0xc3^0x45);

    for(i = 0 ; i < 13 ; i++)
        putc_serial0(setup[i]);
}


// Assigns CH=0, CH Type=00(RX), Net#=0
void ANTAP1_AssignCh (unsigned char chan)
{
    unsigned char i;
    unsigned char setup[7];

    setup[0] = 0xa4;
    setup[1] = 0x03;
    setup[2] = 0x42;
    setup[3] = chan;        // ChanNum
    setup[4] = chanType;    // ChanType
    setup[5] = netNum;        // NetNum
    setup[6] = (0xa4^0x03^0x42^chan^chanType^netNum);

    for(i = 0 ; i < 7 ; i++)
        putc_serial0(setup[i]);
}

void ANTAP1_SetChRFFreq (unsigned char chan)
{
    unsigned char i;
    unsigned char setup[6];

    setup[0] = 0xa4;
    setup[1] = 0x02;
    setup[2] = 0x45;
    setup[3] = chan;        // ChanNum
    setup[4] = 0x39;        // RF Freq
    setup[5] = (0xa4^0x02^0x45^chan^0x39);

    for(i = 0 ; i < 6 ; i++)
        putc_serial0(setup[i]);

}

// Assigns CH=0, RF Freq
void ANTAP1_RequestChanID(unsigned char chan)
{
    unsigned char i;
    unsigned char setup[6];

    setup[0] = 0xa4;
    setup[1] = 0x02;
    setup[2] = 0x4d;
    setup[3] = chan;        // ChanNum
    setup[4] = 0x51;        //Extra Info
    setup[5] = (0xa4^0x02^0x4d^chan^0x51);

    for(i = 0 ; i < 6 ; i++)
        putc_serial0(setup[i]);
}

void ANTAP1_SetChPeriod (unsigned char chan, unsigned char device)
{
    unsigned char i;
    unsigned char setup[7];

    setup[0] = 0xa4;
    setup[1] = 0x03;
    setup[2] = 0x43;
    setup[3] = chan;
    setup[4] = device;
    setup[5] = 0x1f;
    setup[6] = (0xa4^0x03^0x43^chan^device^0x1f);

    for(i = 0 ; i < 7 ; i++)
        putc_serial0(setup[i]);

}

// Assigns Device#=0000 (wildcard), Device Type ID=00 (wildcard), Trans Type=00 (wildcard)
void ANTAP1_SetChId (unsigned char chan, unsigned char deviceType, unsigned char deviceNum[])
{
    unsigned char i;
    unsigned char setup[9];

    setup[0] = 0xa4;
    setup[1] = 0x05;
    setup[2] = 0x51;
    setup[3] = chan;
    setup[4] = deviceNum[0];
    setup[5] = deviceNum[1];
    setup[6] = deviceType;
    setup[7] = 0x00;
    setup[8] = (0xa4^0x05^0x51^chan^deviceNum[0]^deviceNum[1]^deviceType^0x00);

    for(i = 0 ; i < 9 ; i++)
        putc_serial0(setup[i]);
}

// Opens CH 0
void ANTAP1_OpenCh (unsigned char chan)
{
    unsigned char i;
    unsigned char setup[5];

    setup[0] = 0xa4;
    setup[1] = 0x01;
    setup[2] = 0x4b;
    setup[3] = chan;
    setup[4] = (0xa4^0x01^0x4b^chan);

    for(i = 0 ; i < 5 ; i++)
        putc_serial0(setup[i]);

}
char parseANT(unsigned char chr)
{
    if ((chr == MESG_TX_SYNC) && (inMsg == FALSE))
    {
        msgN = 1; // Always reset msg count if we get a sync
        inMsg = TRUE;
        currentChannel=-1;
    }
    else if (msgN == 1)
    {
        msgN++;
        size = chr;
    }
    else if(msgN == 2)
    {
        if(chr == MESG_BROADCAST_DATA_ID)
        {
            isBroadCast = TRUE;
        }
        else
        {
            isBroadCast = FALSE;
        }
        msgN++;
    }
    else if (msgN == 3)
    {
        currentChannel=(int) chr; // this has to be 0x00,0x01,0x02,0x03 so okay?
        msgN++;
    }
    else if (msgN == (size + 3)) // sync, size, checksum
    {
        inMsg = FALSE;
        return TRUE; //We are at the end of the message
    }
    else if(inMsg == TRUE)
    {
        msgN++;
    }

    return FALSE;
}

void flashBoobies(int num_of_times)
{
    for(int x=0; x<num_of_times;x++)
    {
        LED_ON();
        delay_ms(50);
        LED_OFF();
        delay_ms(50);
    }
}

void printDebug(char *debug, int _size)
{
    for(int i=0; i < _size; i++)
    {
        putc_serial0(debug[i]);
    }
}

