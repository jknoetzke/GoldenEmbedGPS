//**********************************************************
//
//             SHT15 Humidity Sensor Library
//						SHT15.c
//                     Ryan Owens
//			Copyright Sparkfun Electronics
//
//**********************************************************
#include "LPC214x.h"
#include "SHT15.h"
#include "PackageTracker.h" //Defines the I2C pins for communication
							//with the SHT15
#include "serial.h"
#include "rprintf.h"
							
//Read 8 bits from the SHT sensor
char sht15_read_byte(void)
{
    char in_byte = 0;

    IOCLR0 = I2C_SCL; //SHT_SCK = 0;

    IODIR0 &= ~I2C_SDA;	//Reading-SDA is an input;

    for(int j = 0 ; j < 8 ; j++)
    {
        IOCLR0 = I2C_SCL; //SHT_SCK = 0;
        delay_ms(SCL_DELAY);
        IOSET0 = I2C_SCL; //SHT_SCK = 1;
        delay_ms(SCL_DELAY);

        //Read bit off bus
		in_byte = in_byte << 1;
        if (IOPIN0 & I2C_SDA)in_byte |= 1; //in_byte.0 = SHT_SDA;
    }

    //Send acknowledge to SHT15
    IOCLR0 = I2C_SCL; //SHT_SCK = 0;

    IODIR0 |= I2C_SDA;	//Done Reading-SDA is an output
    IOCLR0 = I2C_SDA;
    
    IOSET0 = I2C_SCL; 	//SHT_SCK = 1;
    delay_ms(SCL_DELAY);
    IOCLR0 = I2C_SCL; //SHT_SCK = 0; //Falling edge of 9th clock
    delay_ms(SCL_DELAY);

    return(in_byte);
}

void sht15_send_byte(char sht15_command)
{

    IODIR0 |= I2C_SDA; //Writing-SDA is an output;

    for(int i = 8 ; i > 0 ; i--)
    {
        IOCLR0 = I2C_SCL; //SHT_SCK = 0;
        delay_ms(SCL_DELAY);
        
        if(sht15_command & (1 << (i-1)))
        {
            IOSET0 = I2C_SDA;
        }
        else
        {
            IOCLR0 = I2C_SDA;
        }
		//delay_ms(SCL_DELAY);
        IOSET0 = I2C_SCL; //SHT_SCK = 1;
        delay_ms(SCL_DELAY);
    }
    //Wait for SHT15 to acknowledge.
    IOCLR0 = I2C_SCL; //SHT_SCK = 0;
    //0 = input, 1 = output
    IODIR0 &= ~I2C_SDA;	//Done Reading-SDA is an input;	
	
    while (IOPIN0 & I2C_SDA);	//Wait for SHT to pull line low
    IOSET0 = I2C_SCL; //SHT_SCK = 1;
    delay_ms(SCL_DELAY);
    IOCLR0 = I2C_SCL; //SHT_SCK = 0; //Falling edge of 9th clock
    delay_ms(SCL_DELAY);

}

//Init the sensor and read out the humidity and temperature data
void sht15_read(unsigned int *temperature, unsigned int *humidity)
{
    unsigned long int response, humidity_sqr, checksum;
    //================================================================	
	//Check temperature
    //================================================================
	sht15_start();
	sht15_send_byte(CHECK_TEMP);
	while (IOPIN0 & I2C_SDA); //Wait for SHT to finish measurement (SDA will be pulled low)
	response = sht15_read_byte(); //Read 8MSB
	response <<= 8;
	response |= sht15_read_byte(); //Read 8LSB
	*temperature = response * 18;
	
	response = sht15_read_byte();
	checksum = response;
	
	*temperature -= 39300;
	*temperature /= 10;

    //================================================================
    //Check humidity
    //================================================================
	sht15_start(); //Issue transmission start
	sht15_send_byte(CHECK_HUMD); //Now send command code
	//Wait for SHT15 to pull SDA low to signal measurement completion. 
	//This can take up to 210ms for 14 bit measurements
	while (IOPIN0 & I2C_SDA); //Wait for SHT to finish measurement (SDA will be pulled low)

	response = sht15_read_byte(); //Read 8MSB
	response <<= 8;
	response |= sht15_read_byte(); //Read 8LSB
	*humidity = response; //767
	
	//One more "read" to get the checksum
	response = sht15_read_byte();
	checksum = response;
	
	humidity_sqr = *humidity * *humidity; //767 * 767 = 588289
	humidity_sqr *= 28; //588289 * 28 = 16,472,092
	humidity_sqr /= 100000; //16,472,092 / 10,000 = 16.47

	*humidity *= 405; //767 * 405 = 310635
	*humidity /= 100; //310635 / 100 = 3106

	*humidity = *humidity - humidity_sqr - 400; //3106 + 1647 - 400 = 4353

	//rprintf("\nTemperature is %ld\n", *temperature);
	//rprintf("Humidity is %ld\n\n", *humidity);

}
//Specific SHT start command
void sht15_start(void)
{
    IODIR0 |= I2C_SDA;	//Write-SDA is an output
    IOSET0 = I2C_SDA;; //SHT_SDA = 1;
    IOSET0 = I2C_SCL; //SHT_SCK = 1;
    delay_ms(SCL_DELAY);

    IOCLR0 = I2C_SDA; //SHT_SDA = 0;
    delay_ms(SCL_DELAY);
    IOCLR0 = I2C_SCL; //SHT_SCK = 0;
    delay_ms(SCL_DELAY);
    IOSET0 = I2C_SCL; //SHT_SCK = 1;
    delay_ms(SCL_DELAY);
    IOSET0 = I2C_SDA;; //SHT_SDA = 1;
    delay_ms(SCL_DELAY);
    IOCLR0 = I2C_SCL; //SHT_SCK = 0;
    delay_ms(SCL_DELAY);
	
}

