//**********************************************************
//
//             LIS30 Accelerometer Library
//						LIS302.h
//                     Ryan Owens
//			Copyright Sparkfun Electronics
//
//**********************************************************

//This file is used to define the functions  for the LIS302 library
//as well as the register addresses for the LIS302.

//**********************************************************
//
//                  Pin Definitions
//
//**********************************************************
//Definitions for KinetaMap
//#define CHIP_SELECT	(1<<7)
//Definitions for PackageTracker
#define CHIP_SELECT	(1<<17)

//**********************************************************
//
//                  Register Addresses
//
//**********************************************************
#define	WHO_AM_I	0x0F
#define	Ctrl_Reg1	0x20
#define Ctrl_Reg2	0x21
#define	Ctrl_Reg3	0x22
#define	HP_filter_reset	0x23
#define	Status_Reg	0x27
#define OutX		0x29
#define OutY		0x2B
#define	OutZ		0x2D
#define FF_WU_CFG_1	0x30
#define FF_WU_SRC_1	0x31
#define FF_WU_THS_1	0x32
#define	FF_WU_DURATION_1	0x33
#define	FF_WU_CFG_2	0x34
#define FF_WU_SRC_2	0x35
#define FF_WU_THS_2	0x36
#define FF_WU_DURATION_2	0x37
#define CLICK_CFG	0x38
#define	CLICK_SRC	0x39
#define	CLICK_THSY_X	0x3B
#define	CLICK_THSZ	0x3C
#define CLICK_timelimit	0X3D
#define CLICK_latency	0X3E
#define CLICK_window	0x3F

//****************************
//CTRL_REG1 BITS
#define	XEN	(1<<0)
#define YEN	(1<<1)
#define ZEN	(1<<2)
#define STM (1<<3)
#define STP (1<<4)
#define FS  (1<<5)
#define PD  (1<<6)
#define DR  (1<<7)
//****************************

//****************************
//CTRL_REG2 BITS
#define	HP_coeff1	(1<<0)
#define HP_coeff2	(1<<1)
#define HP_FF_WU1	(1<<2)
#define HP_FF_WU2	(1<<3)
#define FDS			(1<<4)
#define BOOT		(1<<6)
#define SIM			(1<<7)
//****************************

//****************************
//CTRL_REG3 BITS
#define	I1CFG0	(1<<0)
#define I1CFG1	(1<<1)
#define I1CFG2	(1<<2)
#define I2CFG0 	(1<<3)
#define I2CFG1 	(1<<4)
#define I2CFG2  (1<<5)
#define PP_OD  	(1<<6)
#define IHL	  	(1<<7)
//****************************

//****************************
//FF_WU_CFG_1 BITS
#define	XLIE	(1<<0)
#define XHIE	(1<<1)
#define YLIE	(1<<2)
#define YHIE 	(1<<3)
#define ZLIE 	(1<<4)
#define ZHIE  	(1<<5)
#define LIR  	(1<<6)
#define AOI	  	(1<<7)
//****************************

//****************************
//FF_WU_SRC_1 BITS
#define	XL	(1<<0)
#define XH	(1<<1)
#define YL	(1<<2)
#define YH 	(1<<3)
#define ZL 	(1<<4)
#define ZH 	(1<<5)
#define IA 	(1<<6)
//****************************

//**********************************************************
//
//                  	Macros
//
//**********************************************************
#define SelectAccel()	IOCLR0 = CHIP_SELECT
#define UnselectAccel()	IOSET0 = CHIP_SELECT
#define READ	0x80
#define	WRITE	0x00

//**********************************************************
//
//                  Function Protocols
//
//**********************************************************
void initAccel(void);
char ReadLis302Register(char register_name);
void WriteLis302Register(char register_name, char register_value);
char accelX(void);
char accelY(void);
char accelZ(void);
void powerdownAccel(void);
