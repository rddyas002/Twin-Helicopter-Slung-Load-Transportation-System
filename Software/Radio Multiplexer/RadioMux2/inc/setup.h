
#ifndef SETUP_H
#define	SETUP_H

#include "p18f26k22.h"
#include <stdbool.h>

#ifdef SETUP_H_IMPORT
	#define SETUP_EXTERN
#else
	#define SETUP_EXTERN extern
#endif

#define DEVICE_ADDRESS 0x4A

#define SETUP_TRIS_SELECT   TRISBbits.TRISB2
#define SETUP_RADIO_ON()    {LATBbits.LATB2 = 1;}
#define SETUP_RADIO_OFF()   {LATBbits.LATB2 = 0;}
#define SETUP_TOGGLE_CH7(); {LATAbits.LATA1 = !LATAbits.LATA1;}
#define SETUP_CH7_HIGH()    LATAbits.LATA1 = 1
#define SETUP_CH7_LOW()     LATAbits.LATA1 = 0
#define SETUP_TIMER_ON      T0CONbits.TMR0ON
#define SETUP_TIMER_REG     138
#define SETUP_COUNT_1ms     (61)//66
#define SETUP_COUNT_2ms     134
#define SETUP_COUNT_1ms_DIFF (73)//(68)//(SETUP_COUNT_2ms - SETUP_COUNT_1ms)
#define SETUP_COUNT_TOP     1325

#define SETUP_ESC_TRIS TRISBbits.TRISB0
#define	SETUP_AIL_TRIS TRISBbits.TRISB3
#define	SETUP_ELEV_TRIS TRISBbits.TRISB4
#define	SETUP_RUDD_TRIS TRISBbits.TRISB5
#define	SETUP_GEAR_TRIS TRISCbits.TRISC2
#define	SETUP_AUX_TRIS TRISAbits.TRISA4
#define SETUP_LED0_TRIS TRISCbits.TRISC5

#define SETUP_ESC_LAT LATBbits.LATB0
#define SETUP_AIL_LAT LATBbits.LATB3
#define SETUP_ELEV_LAT LATBbits.LATB4
#define SETUP_RUDD_LAT LATBbits.LATB5
#define SETUP_GEAR_LAT LATCbits.LATC2
#define SETUP_AUX_LAT LATAbits.LATA4
#define SETUP_LED0_LAT LATCbits.LATC5

#define I2C_TIMEOUT (13333) //(15us resolution)

#define SETUP_getDCRegVal(x) ((unsigned short int) (SETUP_COUNT_1ms_DIFF*x/255 + SETUP_COUNT_1ms))

typedef enum {
		ADDRESS_MATCH_WRITE,
		GET_CONFIG_REG,
		GET_BYTE0,
		GET_BYTE1,
		GET_BYTE2,
		GET_BYTE3,
		GET_BYTE4,
		GET_BYTE5,
		GET_CHKSUM,
		NOWHERE
} SETUP_i2cDecision;

volatile unsigned int ESC_PWM_REG = 0, AIL_PWM_REG = 0, ELEV_PWM_REG = 0, RUDD_PWM_REG = 0, GEAR_PWM_REG = 0, AUX_PWM_REG = 0;
volatile unsigned int ESC_PWM_BUFFER_REG = 0, AIL_PWM_BUFFER_REG = 0, ELEV_PWM_BUFFER_REG = 0, RUDD_PWM_BUFFER_REG = 0, GEAR_PWM_BUFFER_REG = 0, AUX_PWM_BUFFER_REG = 0;

SETUP_EXTERN void SETUP_clock(void);
SETUP_EXTERN void SETUP_io(void);
SETUP_EXTERN void SETUP_i2c(void);
SETUP_EXTERN void SETUP_releaseI2C(void);
SETUP_EXTERN void SETUP_PWM(void);
SETUP_EXTERN void SETUP_PWM_high(void);
SETUP_EXTERN void SETUP_swI2C_intOff(void);
SETUP_EXTERN void SETUP_swI2C_intOn(void);
SETUP_EXTERN void SETUP_T0_activate(unsigned char TIMER_MATCH);
SETUP_EXTERN void SETUP_T0_deactivate(void);

#endif	/* SETUP_H */

