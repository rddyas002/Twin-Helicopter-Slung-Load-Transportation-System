#include "p18f26k22.h"
#include <stdio.h>
#include <stdlib.h>
#include "inc/setup.h"
#include "GenericTypeDefs.h"

// Use internal oscillator
#pragma config FOSC = INTIO67
#pragma config PLLCFG = ON
// Enable primary clock
#pragma config PRICLKEN = ON
// Fail safe clock monitor of
#pragma config FCMEN = OFF
// Oscillator switch-over off
#pragma config IESO = OFF
// Enable power-up timer
#pragma config PWRTEN = ON
// Enable brown-out reset
#pragma config BOREN = SBORDIS
// Set brown-out voltage to 2.5V
#pragma config BORV = 250
// Enable watchdog
#pragma config WDTEN = ON
// Set watchdog bit time
#pragma config WDTPS = 128
// Set CCP2 to be mux'd with RB3
#pragma config CCP2MX = PORTB3
// Set PORTB as digital IO upon reset
#pragma config PBADEN = OFF
// Set CCP3 tp be mux'd with RB5
#pragma config CCP3MX = PORTB5
// MCLR pin is enabled
#pragma config MCLRE = EXTMCLR
// If stack full is full, reset
#pragma config STVREN = ON
// Enable debug mode
#pragma config DEBUG = ON

volatile SETUP_i2cDecision I2C_FSM = ADDRESS_MATCH_WRITE;
volatile unsigned char percentActuation[6] = {50,50,50,50,50,50};
volatile unsigned char dataBuffer[8] = {0};
volatile bool data_updated = false;
volatile unsigned char checksum = 0;
volatile unsigned long int kickAutoCount = 0;
volatile char ESC_n = 1, AIL_n = 1, ELEV_n = 1, RUDD_n = 1, GEAR_n = 1, AUX_n = 1;
volatile unsigned int count = 0;

void i2c_process(void);

int main(void) {
    SETUP_clock();
    SETUP_io();
    SETUP_i2c();
    SETUP_PWM();
    T0CONbits.TMR0ON = 1;

    TRISAbits.TRISA1 = 0;
    INTCONbits.GIEH = 1;

    SETUP_RADIO_ON();

    while (1)
    {	
	i2c_process();
	if ((kickAutoCount < I2C_TIMEOUT) && (dataBuffer[0] && 0x01)){
            SETUP_RADIO_OFF();
        }
	else
        {
            SETUP_RADIO_ON();       
        }
	ClrWdt();
    }
    return (EXIT_SUCCESS);
}

void interrupt InterruptHandlerHigh (void)
{
    if (INTCONbits.TMR0IF)
    {
	// Clear interrupt flag
     	INTCONbits.TMR0IF = 0;

        // current value of reg indicates the time taken to enter interrupt, usually 0x11
	TMR0L = SETUP_TIMER_REG + TMR0L;

	// Set pins
	SETUP_ESC_LAT = ESC_n;
	SETUP_AIL_LAT = AIL_n;
	SETUP_RUDD_LAT = RUDD_n;
	SETUP_ELEV_LAT = ELEV_n;
        SETUP_GEAR_LAT = GEAR_n;
	SETUP_AUX_LAT = AUX_n;

	// now calculate next PWM state
	count++;
	if (count == ESC_PWM_REG)
            ESC_n = 0;
	if (count == AIL_PWM_REG)
            AIL_n = 0;
	if (count == RUDD_PWM_REG)
            RUDD_n = 0;
	if (count == ELEV_PWM_REG)
            ELEV_n = 0;
	if (count == GEAR_PWM_REG)
            GEAR_n = 0;
	if (count == AUX_PWM_REG)
            AUX_n = 0;

	if (count == SETUP_COUNT_TOP)
	{
            ESC_n = 1;
            AIL_n = 1;
            RUDD_n = 1;
            ELEV_n = 1;
            GEAR_n = 1;
            AUX_n = 1;

            // load buffer reg
            ESC_PWM_REG = ESC_PWM_BUFFER_REG;
            AIL_PWM_REG = AIL_PWM_BUFFER_REG;
            RUDD_PWM_REG = RUDD_PWM_BUFFER_REG;
            ELEV_PWM_REG = ELEV_PWM_BUFFER_REG;
            GEAR_PWM_REG = GEAR_PWM_BUFFER_REG;
            AUX_PWM_REG = AUX_PWM_BUFFER_REG;

            count = 0;
	}

	kickAutoCount++;
    }
}

volatile BOOL RECEIVING_FROM_MASTER = false;
void i2c_process(void){
    static unsigned char buffer;
    /* 	Control system design note: Communication of 1 packet takes 983.4us. (@400kHz SCL speed)
  	The packet includes the following:
	Byte 1: Address + /W
	Byte 2: CONFIG_REG
	Byte 3-9: PWM actuation 0-255 --- mapped ---> 1->2ms
	Byte 10: checksum
	Byte 11: Reply from slave, whether data corrupt or success
	*/

    // If interrupt due to I2C
    if (PIR1bits.SSPIF)
    {
	// Clear interrupt flag
	PIR1bits.SSPIF = 0;
        
	// If an address & a write command was received
	if ((!SSP1STATbits.R_NOT_W) && (!SSP1STATbits.D_NOT_A))
	{
            // Start data receiving process
            I2C_FSM = ADDRESS_MATCH_WRITE;
            buffer = SSP1BUF;
            // If a write command to the slave
            if (buffer == (DEVICE_ADDRESS << 1))
            {
            	RECEIVING_FROM_MASTER = true;
            }
	}
	// If an address & a read command was received
	else if ((SSP1STATbits.R_NOT_W) && (!SSP1STATbits.D_NOT_A) && RECEIVING_FROM_MASTER)
	{
            // Clear buffer
            buffer = SSP1BUF;
            if ((checksum == dataBuffer[7]))
            {
            	kickAutoCount = 0;
                //SETUP_TOGGLE_CH7();
            	data_updated = true;
		SSP1BUF = 'A';
            }
            else
            {
                SSP1BUF = 'N';
            }
            RECEIVING_FROM_MASTER = 0;
            I2C_FSM = NOWHERE;
	}
        else if (SSP1STATbits.D_NOT_A && RECEIVING_FROM_MASTER)
        {
            // Read buffer to extract data
            switch(I2C_FSM)
            {
		case ADDRESS_MATCH_WRITE:
                    // initialize checksum to 0
                    checksum = 0;
                    dataBuffer[0] = SSP1BUF;
                    checksum ^= dataBuffer[0];
                    I2C_FSM = GET_BYTE0;
                    break;
		case GET_BYTE0:
                    dataBuffer[1] = SSP1BUF;
                    checksum ^= dataBuffer[1];
                    I2C_FSM = GET_BYTE1;
                    break;
		case GET_BYTE1:
                    dataBuffer[2] = SSP1BUF;
                    checksum ^= dataBuffer[2];
                    I2C_FSM = GET_BYTE2;
                    break;
		case GET_BYTE2:
                    dataBuffer[3] = SSP1BUF;
                    checksum ^= dataBuffer[3];
                    I2C_FSM = GET_BYTE3;
                    break;
		case GET_BYTE3:
                    dataBuffer[4] = SSP1BUF;
                    checksum ^= dataBuffer[4];
                    I2C_FSM = GET_BYTE4;
                    break;
		case GET_BYTE4:
                    dataBuffer[5] = SSP1BUF;
                    checksum ^= dataBuffer[5];
                    I2C_FSM = GET_BYTE5;
                    break;
                case GET_BYTE5:
                    dataBuffer[6] = SSP1BUF;
                    checksum ^= dataBuffer[6];
                    I2C_FSM = GET_CHKSUM;
                    break;
		case GET_CHKSUM:
                    dataBuffer[7] = SSP1BUF;
                    I2C_FSM = NOWHERE;
                    break;
		default:
                    break;
            }
	}
	// Release clock
	SSP1CON1bits.CKP = 1;
    }

    if (data_updated)
    {
	ESC_PWM_BUFFER_REG = SETUP_getDCRegVal(dataBuffer[1]);
	AIL_PWM_BUFFER_REG = SETUP_getDCRegVal(dataBuffer[2]);
	ELEV_PWM_BUFFER_REG = SETUP_getDCRegVal(dataBuffer[3]);
	RUDD_PWM_BUFFER_REG = SETUP_getDCRegVal(dataBuffer[4]);
        GEAR_PWM_BUFFER_REG = SETUP_getDCRegVal(dataBuffer[5]);
	AUX_PWM_BUFFER_REG = SETUP_getDCRegVal(dataBuffer[6]);
	data_updated = false;
    }
}