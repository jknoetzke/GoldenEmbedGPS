//*******************************************************
//					General Definitions
//*******************************************************
#define WAKE_MINUTES	5	//Number of minutes to log data before going to sleep
#define SLEEP_MINUTES	25	//Number of minutes to stay in sleep mode

#define TIMER_FREQ		10  //Number of times per second(hz) the TIMER0 interrupt is triggered

#define SCL_DELAY	1		//Delay to prevent reading the SCL sensor to fast
#define	GPS_BUFFER_SIZE	80	//Size of buffer for GPS messages
#define MAX_BUFFER_SIZE 400	//Size of the log buffer for saving data to the SD card
//#define MAX_BUFFER_SIZE 300
#define OK	1
#define ERROR	0

#define COMMAND 124
#define CLEAR_SCREEN	0
#define TOGGLE_BACKLIGHT	2
#define DIGIT	4
#define SET_X	24
#define SET_Y	25
#define PIXEL	16
#define CIRCLE	3
#define ERASE	5
#define BOX		15
#define LINE	12
#define SPEED 	19
//*******************************************************
//					GPIO Definitions
//*******************************************************

//Port 0 Definitions
#define I2C_SCL		(1<<2)
#define	I2C_SDA		(1<<3)
#define SCLK		(1<<4)
#define MISO		(1<<5)
#define MOSI		(1<<6)
#define SD_CS		(1<<7)
#define SCP_DRDY	(1<<10)
#define ACCEL_INT2	(1<<11)
#define BATT_MEAS	(1<<13)
#define BSL			(1<<14)
#define ACCEL_INT1	(1<<15)
#define ACCEL_CS	(1<<17)
#define GPS_EN		(1<<21)
#define LED			(1<<31)
//Port 1 Definitions
#define SCP_EN		(1<<24)
#define SCP_CS		(1<<26)

//*******************************************************
//					Program Definitions
//*******************************************************
#define UART1_INT	(1<<7)
#define TIMER0_INT	(1<<4)
#define RTC_INT		(1<<13)
#define EINT2_INT	(1<<16)

#define RTC_TIMEOUT_WAKE	1
#define ACCELEROMETER_WAKE	2

//*******************************************************
//					Global Macros
//*******************************************************
#define GreenOff()	IOCLR0 = GREEN
#define GreenOn()	IOSET0 = GREEN

#define SelectAccelerometer()	IOCLR0 = ACCEL_CS
#define UnselectAccelerometer()	IOSET0 = ACCEL_CS

#define SelectSCP()		IOCLR1 = SCP_CS
#define	UnselectSCP()	IOSET1 = SCP_CS

#define SCPon()			IOCLR1 = SCP_EN
#define	SCPoff()		IOSET1 = SCP_EN

#define GPSon()			IOSET0 = GPS_EN
#define GPSoff()		IOCLR0 = GPS_EN

#define LED_ON()		IOCLR0 = LED
#define LED_OFF()		IOSET0 = LED

//*******************************************************
//					Global Functions
//*******************************************************
//Usage: delay_ms(1000);
//Inputs: int count: Number of milliseconds to delay
//The function will cause the firmware to delay for "count" milleseconds.
void delay_ms(int count);

//*******************************************************
//					Structure Definitions
//*******************************************************
typedef struct{
	char position[15];
	char direction;
} Position;

typedef struct{
	Position Latitude, Longitude;
	char Time[10], Altitude[10], Date[6], Speed[6], Heading[6];
	char Fix;
} GPSdata;
