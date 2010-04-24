//*******************************************************
//                                      Package Tracker Firmware
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

//*******************************************************
//                  Core Functions
//*******************************************************

//*******************************
//   ANT+ Defines
//*******************************
#define MESG_NETWORK_KEY_ID      0x46
#define MESG_TX_SYNC 0xa4
#define MESG_BROADCAST_DATA_ID 0x4E
#define MESG_RESPONSE_EVENT_ID 0x40
#define MESG_CHANNEL_EVENT_ERROR 0x01
#define chanType  0x00 // ChanType
#define netNum    0x00    // NetNum

#define FALSE 0
#define TRUE 1

int inMsg = FALSE;
int msgN = 0;
int size = 0;
int currentChannel=0;
int isBroadCast = FALSE;

void bootUp(void);
static void ISR_RxData0(void);
static void ISR_RxData1(void);
static void ISR_RTC(void);
static void ISR_Timer0(void);
static void ISR_EINT2(void);
void createLogFile(void);
int parseGGA(const char *gps_string);
int parseRMC(const char *gps_string);
void saveData(struct fat16_file_struct **fd, const char * const buf, const int buf_size);
void itoa(int n, char s[]);
void reverse(char s[]);
int get_adc_1(char channel);
void reset(void);
void initializeGps(void);

void flashBoobies(int num_of_times);
void ANTAP1_Config(void);
void ANTAP1_Reset(void);
void ANTAP1_AssignCh(unsigned char);
void ANTAP1_SetChId(unsigned char, unsigned char deviceType);
void ANTAP1_SetChRFFreq(unsigned char);
void ANTAP1_SetChPeriod(unsigned char, unsigned char);
void ANTAP1_OpenCh(unsigned char);
void ANTAP1_AssignNetwork(unsigned char);
void ANTAP1_SetSearchTimeout(unsigned char);
void ANTAP1_RequestChanID(unsigned char);
void must_we_write(void);
char parseANT(unsigned char chr);

//*******************************************************
//                                      Global Variables
//*******************************************************
#define NMEA_FILE_HEADER "Message ID, Time, Status, Lat., N/S, Long., E/W, Speed, Course, Date, Magnetic Var.\n"

//GPS variables
char gps_message_complete=0, new_gps_data=0, RTC_Set, alarm_set,ant_message_complete=0;        //Notification Flags
char final_message[GPS_BUFFER_SIZE], gps_message[GPS_BUFFER_SIZE], ant_message[GPS_BUFFER_SIZE];      //Buffers for holding GPS messages
int gps_message_index=0, gps_message_size=0, ant_message_index=0, ant_message_size=0;    //index for copying messages to different buffers
int final_gps_message_size=0;
GPSdata GPS;    //GPS Struct to hold GPS coordinates.  See PackageTracker.h for Structure definition

//Pressure Sensor (SCP100) Values
unsigned int scp_pressure;
int scp_temp;
char new_scp_data;

//Humidity Sensor (SHT15) Values
unsigned int sht_temp, sht_humidity;
char new_sht_data;

//Accelerometer (ADXL345) Values
signed int acceleration_x, acceleration_y, acceleration_z;

//Logging Parameters
char file_name[32];     
int battery_level; 
int log_count;  //Keeps track of how many logs we've made since we've been awake.  Get's reset before going to sleep.

//Log Parameters for logging the Sensor Data
struct fat16_file_struct * LOG_FILE; //File structure for current log file
char log_data[512], log_buffer[200];//log_buffer holds data before putting it into log_data
int log_data_index;     //Keeps track of current position in log_data

char wroteGPS = FALSE;

unsigned char ant_data[512];
int ant_data_index=0;

//Log Parameters for logging the NMEA file
//struct fat16_file_struct * NMEA_FILE; //File structure for current log file
//char nmea_data[1024];//
//int nmea_data_index=0;

char led_blink=0;

//Sleep Parameters
unsigned int power_register_values;                     //Holds the value to load to the power register after waking from sleep
char read_sensors, new_sensor_data;                     //Global flag indicating an accelerometer reading has been completed
char wake_event=0;

int main (void)
{
    //*******************************************************
    //                     Main Code
    //*******************************************************
    //Initialize ARM I/O
    bootUp();                       //Init. I/O ports, Comm protocols and interrupts

    flashBoobies(5);

    createLogFile();        //Create a new log file in the root of the SD card


    //Initialize the GPS
    initializeGps();                //Send the initialization strings
    enable_gps_rmc_msgs(1);


    //SHT15 shouldn't need to be initialized(it just needs power)

    VICIntEnable |= UART0_INT | UART1_INT | TIMER0_INT; //Enable UART1 and Timer0 Interrupts
    while(1)
    {
        if(gps_message_complete==1)
        {            //If we've received a new GPS message, record it.
            VICIntEnClr |=  UART1_INT | TIMER0_INT;          //Stop the UART1 interrupts while we read the message
            for(int i=0; i<gps_message_size; i++){ //Transfer the GPS message to the final_message buffer
                final_message[i]=gps_message[i];
                gps_message[i]='\0';
            }
            final_gps_message_size=gps_message_size;
            gps_message_complete=0;                 //Clear the message complete flag
            VICIntEnable |=  UART1_INT | TIMER0_INT;         //Re-Enable the UART0 Interrupts to get next GPS message                                

            //Populate GPS struct with time, position, fix, date
            //If we received a valid RMC message, log it to the NMEA file
            if(parseRMC(final_message))
            {
                //If we were able to parse the entire gps message, than we have new gps data to log
                new_gps_data=1;

                //Add the gps message to the nmea buffer
                CCR = (1<<1);   //Disable and Reset the RTC
                HOUR = ((GPS.Time[0]-'0')*10) + (GPS.Time[1]-'0');
                MIN = ((GPS.Time[2]-'0')*10) + (GPS.Time[3]-'0');
                SEC = ((GPS.Time[4]-'0')*10) + (GPS.Time[5] -'0');      
                RTC_Set=1;              //Set the RTC_Set flag since we have a valid time in the RTC registers                                  
                CCR = (1<<0);   //Start the RTC
            }
        }

        //int valid_gga = parseGGA(final_message);
 
        if(new_sensor_data || new_gps_data)
        {                                                    
            //Log Time
            //If there is GPS data, use this time and date
            if(new_gps_data && GPS.Fix=='A'){
                for(int i=0; i<6; i++)log_data[log_data_index++]=GPS.Date[i];
                log_data[log_data_index++]=',';
                for(int i=0; i<10; i++)log_data[log_data_index++]=GPS.Time[i];
            }
            //If there is not GPS data, use the RTC time
            else{
                //Put a place marker for the date!
                log_data[log_data_index++]=',';
                if(RTC_Set){
                    log_data[log_data_index++]=(HOUR / 10) + '0';
                    log_data[log_data_index++]=(HOUR % 10) + '0';
                    log_data[log_data_index++]=(MIN / 10) + '0';
                    log_data[log_data_index++]=(MIN % 10) + '0';
                    log_data[log_data_index++]=(SEC / 10) + '0';
                    log_data[log_data_index++]=(SEC % 10) + '0';    
                }
            }
            log_data[log_data_index++]=',';
            //If we have GPS data, add it to the log buffer
            if(new_gps_data)
            {
                //Log Fix Indicator
                log_data[log_data_index++]=GPS.Fix;
                log_data[log_data_index++]=',';

                //Log latitiude
                for(int i=0; i<9; i++)log_data[log_data_index++]=GPS.Latitude.position[i];
                log_data[log_data_index++]=',';

                log_data[log_data_index++]=GPS.Latitude.direction;
                log_data[log_data_index++]=',';

                //Log longitude
                for(int i=0; i<10; i++)log_data[log_data_index++]=GPS.Longitude.position[i];
                log_data[log_data_index++]=',';

                log_data[log_data_index++]=GPS.Longitude.direction;
//                log_data[log_data_index++]=',';

/*
                //Log Speed
                for(int i=0; i<6; i++)log_data[log_data_index++]=GPS.Speed[i];
                log_data[log_data_index++]=',';
                log_data[log_data_index++]=GPS.Speed;
                //Altitude
                if(valid_gga)
                {
                    log_data[log_data_index++]=',';
                    for(int i=0; i<10; i++)log_data[log_data_index++]=GPS.Altitude[i];
                    log_data[log_data_index++]=',';

                    log_data[log_data_index++]=GPS.Altitude;
                }
*/  
            }
          else for(int i=0; i<6; i++)log_data[log_data_index++]=',';
            log_data[log_data_index++]='\n';


            //Only Save Data if the buffer is full! This saves write cycles to the SD card
            if(log_data_index >= MAX_BUFFER_SIZE)
            {
                VICIntEnClr |= TIMER0_INT | UART1_INT;
                UnselectSCP();
                saveData(&LOG_FILE, log_data, log_data_index);
                VICIntEnable |= TIMER0_INT | UART1_INT;
                SelectSCP();
                unselect_card();
                SCPinit();
                delay_ms(10);
                for(int i=0; i<log_data_index; i++)log_data[i]='\0';
                log_data_index=0;
                if(wroteGPS == FALSE)
                {
                    ANTAP1_Config();
                    wroteGPS = TRUE; 
                }
            }
            new_gps_data=0; //We've saved the GPS coordinates, so clear the GPS data flag
            new_sensor_data=0;      //We've save the accel values, so clear the accel flag
        }

        if(ant_message_complete == TRUE)
        {
            for(int i=0; i< ant_message_index; i++)
            { 
                ant_data[ant_message_size++]=ant_message[i];
                ant_message[i]='\0';
                ant_message_complete = FALSE;
            }
            ant_message_index=0;
        }

        if(ant_message_size >= MAX_BUFFER_SIZE)
        {
            saveData(&LOG_FILE, (char *)ant_data, ant_message_size);
            for(int i=0; i<ant_message_size; i++)
                ant_data[i]='\0';
            ant_data_index = 0;
            ant_message_complete=0;
            ant_message_size=0;
        }
        //If a USB Cable gets plugged in, stop everything!
        if(IOPIN0 & (1<<23))
        {
            VICIntEnClr = UART0_INT | UART1_INT | TIMER0_INT | RTC_INT | EINT2_INT;     //Stop all running interrupts
            //Save current logged data and close the file before allowing USB communication
            if ( NULL != LOG_FILE ) {
                UnselectSCP();
                saveData(&LOG_FILE, log_data, log_data_index);
                saveData(&LOG_FILE, (char *)ant_data, ant_message_size);
                fat16_close_file(LOG_FILE);
                log_data_index=0;
            }
            main_msc();                                                             //Open the mass storage device
            reset();                                                                //Reset to check for new FW
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
    delay_ms(10); //Delay for power to stablize

    //Bring up SD and FAT
    if(!sd_raw_init())
    {
        //rprintf("SD Init Error\n");
        reset();
    }
    if(openroot())
    {
        //rprintf("SD OpenRoot Error\n");
        reset();
    }
    PINSEL0 &= ~((3<<4)|(3<<6));

    //Enable AD conversion on P0.13(AD1.4) FOR BATT_MEAS
    PINSEL0 |= (3<<26);

    //Set up the EINT2 External Interrupt Functionality
    PINSEL0 &= ~(3<<30);    //Clear P0.15 special function
    PINSEL0 |= (2<<30);     //Set P0.15 to EINT2
    VICIntEnClr |= EINT2_INT;//Make sure EINT2 interrupts are disabled
    EXTINT |= (1<<2);               //Clear the EINT2 Interrupt bit
    EXTMODE |= (1<<2);              //Set EINT2 to be edge sensitive
    EXTINT |= (1<<2);               //Clear the EINT2 Interrupt bit
    EXTPOLAR |= (1<<2);     //Set EINT2 to detect rising edges
    INTWAKE |= (1<<2);              //ARM will wake up from power down on an EINT2 interrupt
    EXTINT |= (1<<2);               //Clear the EINT2 Interrupt bit 

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
    VICIntSelect = ~(UART0_INT | UART1_INT | TIMER0_INT | RTC_INT | EINT2_INT);
    VICVectCntl0 = 0x20 | 6;                                                //Set up the UART1 interrupt
    VICVectAddr0 = (unsigned int)ISR_RxData0;
    VICVectCntl1 = 0x20 | 13;                                               //Set up the RTC interrupt
    VICVectAddr1 = (unsigned int)ISR_RTC;   
    //    VICVectCntl2 = 0x20 | 4;                                                //Timer 0 Interrupt
    //    VICVectAddr2 = (unsigned int)ISR_Timer0;
    VICVectCntl3 = 0x20 | 16;                                               //EINT2 External Interrupt 
    VICVectAddr3 = (unsigned int)ISR_EINT2;
    VICVectCntl4 = 0x20 | 7;                                                //Set up the UART0 interrupt
    VICVectAddr4 = (unsigned int)ISR_RxData1;

    //Setup the UART1 Interrupt
    U1IER = 0x01;                           //Enable FIFO on UART with RDA interrupt (Receive Data Available)
    U1FCR &= 0x3F;                          //Enable FIFO, set RDA interrupt for 1 character        

    //Setup the UART0 Interrupt
    U0IER = 0x01;                           //Enable FIFO on UART with RDA interrupt (Receive Data Available)
    U0FCR &= 0x3F;                          //Enable FIFO, set RDA interrupt for 1 character        

    //Setupt the Timer0 Interrupt
    T0PR = 1200;                            //Divide Clock(60MHz) by 1200 for 50kHz PS
    T0TCR |=0X01;                           //Enable the clock
    T0CTCR=0;                                       //Timer Mode
    T0MCR=0x0003;                           //Interrupt and Reset Timer on Match
    T0MR0=(50000/TIMER_FREQ);       //Set Interrupt frequency by dividing system clock (50KHz) by TIMER_FREQ (defined in PackageTracker.h as 10) 
    //Value will result in Timer 0 interrupts at TIMER_FREQ

    //Set up the RTC so it can be used for sleeping
    CCR = ~(1<<0);                          //use the system clock, and disable RTC for now
    CIIR = 0;                                       //Don't allow any increment interrupts
    AMR = ~(1<<1);                          //Only check the minutes value of the alarm     
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
    ant_message_complete = parseANT(val);
    if(ant_message_complete == TRUE)
        ant_message[ant_message_index++] = '\n';

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
        if(val == '$')gps_message_index=0;
        gps_message[gps_message_index++]= val;
        gps_message_complete=0;
    }
    VICVectAddr =0;                                         //Update the VIC priorities
}

//Usage: None (Automatically Called by FW)
//Inputs: None
//Description: Called when the RTC alarm goes off.  This wakes
//                              the Package Tracker from sleep mode.
static void ISR_RTC(void)
{       
    //Clear the Alarm Interrupt bit from the ILR
    ILR = ((1<<1)|(1<<0));
    wake_event=RTC_TIMEOUT_WAKE;
    VICVectAddr =0;         //Update the VIC priorities
}

//Usage: createLogFile();
//Inputs: None
//Outputs: None
//Description: Creates a log file in the root directory of the SD card with the name
//                              PackageTrackerXX.csv.  XX increments to the next available number each
//                              time the function is called.
void createLogFile(void){
    static int file_number;

    //Create the Sensor Data Log File       
    //Set an initial file name
    sprintf(file_name, "ANTGPS%03d.gep", file_number);
    //Check to see if the file already exists in the root directory.
    while(root_file_exists(file_name))
    {
        file_number++;  //If the file already exists, increment the file number and check again.
        if(file_number == 250)
        {
            //rprintf("\nToo many files in root!\n");
        }
        sprintf(file_name, "ANTGPS%03d.gep", file_number);
    }
    //Get the file handle of the new file.  We will log the data to this file
    LOG_FILE = root_open_new(file_name);
    //Now that we have the file opened, let's put a label in the first row
    //    fat16_write_file(LOG_FILE, (unsigned char*)"Date, UTC, X, Y, Z, Batt, Pres., SCP Temp., SHT Temp, Humidity, Fix, Lat., Lat. Dir., Long., Long. Dir.,\n", 105);
    sd_raw_sync();
}

//Usage: parseGGA(final_message);
//Inputs: const char *gps_string - GGA NMEA string
//This functions splits a GGA message into the
//portions and assigns them to components of
//a GPS structure
int parseGGA(const char *gps_string){
    int i=0;
    //Parse the GGA Message.  1st portion dismissed
    while(gps_string[i] != ',')i++;
    i++;

    //If we didn't receive the correct SiRF header, the return an error
    if(gps_string[0] != '$' || gps_string[1] != 'G' || gps_string[2] != 'G')return 0;

    //Second portion is UTC timestamp
    for(int j=0;gps_string[i] != ','; j++){
        GPS.Time[j]=gps_string[i];
        i++;
    }
    i++;
    //Third portion is Latitude
    for(int j=0;gps_string[i] != ',';j++){
        GPS.Latitude.position[j]=gps_string[i];
        i++;
    }
    i++;                    
    //Fourth portion is Latitude direction
    for(int j=0;gps_string[i] != ','; j++){
        GPS.Latitude.direction=gps_string[i];
        i++;
    }
    i++;    
    //Fifth portion is Long.
    for(int j=0;gps_string[i] != ','; j++){
        GPS.Longitude.position[j]=gps_string[i];
        i++;
    }
    i++;                    
    //Sixth portion is Long direction
    while(gps_string[i] != ','){
        GPS.Longitude.direction=gps_string[i];
        i++;
    }
    i++;            
    //Seventh portion is fix
    while(gps_string[i] != ','){
        GPS.Fix=gps_string[i];
        i++;
    }
    i++;
    //8th portion dismissed
    while(gps_string[i] != ',')i++;
    i++;                            
    //8th portion dismissed
    while(gps_string[i] != ',')i++;
    i++;                            
    //10th portion is Altitude
    for(int j=0;gps_string[i] != ','; j++){
        GPS.Altitude[j]=gps_string[i];
        i++;
    }       
}

//Usage: parseRMC(final_message);
//Inputs: const char *gps_string - RMC NMEA string
//This functions splits a GGA message into the
//portions and assigns them to components of
//a GPS structure
//This functions splits a RMC message into the
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
    //8th portion dismissed Justin- This should be speed.
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

    if((buf_size > 0) && (*fd != NULL)){
        //Try writing the data to the card up to 10 times.
        while(error<10){
            if(fat16_write_file(*fd, (const unsigned char*)buf, buf_size) < 0)error+=1;
            else break;
            delay_ms(100);
        }
        //If we've tried writing the data 10 times and still haven't succeeded, reset the device.
        if(error==10)reset();

        error=0;
        //Try syncing the card up to 10 times
        while(error<10){
            if(!sd_raw_sync())error+=1;
            else break;
            delay_ms(100);
        }
        //If we've tried syncing 10 times and still haven't succeeded, reset the device
        if(error==10)reset();

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

//Usage: None (Automatically Called by FW)
//Inputs: None
//This function is a global interrupt called by a match on the Timer 1 match.  The interrupt
// is responsible for determining if a button has been pressed or if the screen has been rotated
// and setting the appropriate global flag if either has occured.
static void ISR_Timer0(void)
{
    //Interrupt Code Here
    read_sensors=1;
    log_count++;

    //Update the Status LED
    led_blink++;
    if(led_blink > TIMER_FREQ)led_blink=0;
    if(led_blink > (TIMER_FREQ % 10))LED_OFF();
    else LED_ON();

    T0IR = 0xFF;                                            //Clear the timer interrupt
    VICVectAddr =0;                                         //Update the VIC priorities
}

//Usage: accel = get_adc_1(CHANNEL);
//Inputs: int channel - integer corresponding to the ADC channel to be converted
//Outputs: None
//Description: Returns the raw analog to digital conversion of the input channel.  
int get_adc_1(char channel)
{
    int val;
    AD1CR = 0;
    AD1GDR = 0;

    //AD1CR = 0x00200600 | channel;
    AD1CR = 0x00200E00 | channel;
    AD1CR |= 0x01000000;
    do
    {
        val = AD1GDR;                   // Read A/D Data Register
    }
    while ((val & 0x80000000) == 0);  //Wait for the conversion to complete
    val = ((val >> 6) & 0x03FF);  //Extract the A/D result

    return val;
}

//Usage: reset();
//Inputs: None
//Description: Resets the LPC2148
void reset(void)
{
    // Intentionally fault Watchdog to trigger a reset condition
    WDMOD |= 3;
    WDFEED = 0xAA;
    WDFEED = 0x55;
    WDFEED = 0xAA;
    WDFEED = 0x00;
}

static void ISR_EINT2(void){
    VICIntEnClr = (1<<16);                  //Temporarily disable EINT2 Interrupts
    EXTINT |= (1<<2);                               //Clear the interrupt bit in EINT2

    wake_event=ACCELEROMETER_WAKE;                  //Tell the main code that a free-fall has been detected!

    VICIntEnable = (1<<16);         //Re-enable the EINT2 Interrupts
    VICVectAddr =0;         //Update the VIC priorities
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

    ANTAP1_Reset();
    delay_ms(50);

    //HR
    ANTAP1_AssignCh(0x00);
    delay_ms(50);

    ANTAP1_SetChId(0x00,0x78); //0x78 == HR type
    delay_ms(50);
    ANTAP1_AssignNetwork(0x00);
    delay_ms(50);
    ANTAP1_SetSearchTimeout(0x00);
    delay_ms(50);
    ANTAP1_SetChRFFreq(0x00);
    delay_ms(50);
    ANTAP1_SetChPeriod(0x00, 0x86);
    delay_ms(50);
    ANTAP1_OpenCh(0x00);
    delay_ms(50);

    //Power
    ANTAP1_AssignCh(0x01);
    delay_ms(50);
    ANTAP1_SetChId(0x01,0x0B); // 0x0B == Power Type
    delay_ms(50);
    ANTAP1_AssignNetwork(0x01);
    delay_ms(50);
    ANTAP1_SetSearchTimeout(0x01);
    delay_ms(50);
    ANTAP1_SetChRFFreq(0x01);
    delay_ms(50);
    ANTAP1_SetChPeriod(0x01, 0xf6);
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
void ANTAP1_SetChId (unsigned char chan, unsigned char deviceType) 
{
    unsigned char i;
    unsigned char setup[9];

    setup[0] = 0xa4;
    setup[1] = 0x05;
    setup[2] = 0x51;
    setup[3] = chan;
    setup[4] = 0x00;
    setup[5] = 0x00;
    setup[6] = deviceType;
    setup[7] = 0x00;
    setup[8] = (0xa4^0x05^0x51^chan^0x00^0x00^deviceType^0x00);

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
        msgN = 0; // Always reset msg count if we get a sync
        inMsg = TRUE;
        currentChannel=-1;
        msgN++;
    }
    else if (msgN == 1)
    {
        msgN++;
        size = chr;
    }
    else if(msgN == 2)
    {
        if(chr == MESG_BROADCAST_DATA_ID) {
            isBroadCast = TRUE;
        } else {
            isBroadCast = FALSE;
        }
        msgN++;
    }
    else if (msgN == 3) {
        currentChannel=(int) chr; // this has to be 0x00,0x01,0x02,0x03 so okay?
        msgN++;
    }
    else if (msgN == (size + 3)) // sync, size, checksum
    {                           
        //Write the time.
        inMsg = FALSE;
        msgN = 0;
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
        delay_ms(100);
        LED_OFF();
        delay_ms(100);
    }
}
