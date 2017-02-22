#define SETUP_H_IMPORT
#include "../inc/setup.h"

void SETUP_clock(void)
{
    // Select primary clock
    OSCCONbits.SCS = 0;
    // Select internal frequency of 16MHz
    OSCCONbits.IRCF = 0b111;
    // Make sure PLL is enabled
    OSCTUNEbits.PLLEN = 1;

    // Generates Primary clock of 64MHz
}

void SETUP_io(void)
{
    // Enable digital input buffer for SDA and SCL
    ANSELC &= 0b11000111;
    ANSELB &= 0b11011111;

    SETUP_ESC_TRIS = 0;
    SETUP_AIL_TRIS = 0;
    SETUP_ELEV_TRIS = 0;
    SETUP_RUDD_TRIS = 0;
    SETUP_GEAR_TRIS = 0;
    SETUP_AUX_TRIS = 0;

//    SETUP_LED0_TRIS = 0;

    // Set select pin as output
    SETUP_TRIS_SELECT = 0;
}

void SETUP_i2c(void)
{
    // Set SDA and SCL as input pins, RC4 and RC3, respectively
    TRISC |= 0b00011000;

    // Set slave address
    SSP1ADD = (DEVICE_ADDRESS << 1);

    // Enable SDA and SCL
    SSP1CON1bits.SSPEN = 1;

    // Slave mode (7 bit address)
    SSP1CON1bits.SSPM = 0b0110;

    // Enable clock streching for slave
    SSP1CON2bits.SEN = 1;

    // Enable priority levels
    RCONbits.IPEN = 1;

    // Disable SSP1 interrupt
    PIE1bits.SSP1IE = 0;
    // DO NOT USE INTERRUPTS
}

void SETUP_releaseI2C(void)
{
    // Release clock
    SSP1CON1bits.CKP = 1;
    // Clear interrupt flag
    PIR1bits.SSPIF = 0;
}

void SETUP_PWM(void)
{
    // Enable Timer 0 overflow interrupt
    INTCONbits.TMR0IE = 1;
    // Enable Timer 0 interrupt as high priority
    INTCON2bits.TMR0IP = 1;
    // Setup timer 0 in 8 bit mode
    T0CON = 0b01000000;
    TMR0L = (unsigned char)(SETUP_TIMER_REG & 0x00FF);

    // Timer ticks at 64MHz/4/2 = 8MHz
    // Aim for a resolution of 5us -->
    // Requires 40 periods
    // For a PWM rate of 50Hz we need 4000 ticks of count SETUP_COUNT_TOP

    // set initial PWM duty cycle
    ESC_PWM_REG = SETUP_getDCRegVal(0);
    AIL_PWM_REG = SETUP_getDCRegVal(127);
    ELEV_PWM_REG = SETUP_getDCRegVal(127);
    RUDD_PWM_REG = SETUP_getDCRegVal(127);
    GEAR_PWM_REG = SETUP_getDCRegVal(127);
    AUX_PWM_REG = SETUP_getDCRegVal(127);
}

//unsigned short int SETUP_getDCRegVal(unsigned char percent)
//{
//    return (unsigned short int) (SETUP_COUNT_1ms_DIFF*percent/255 + SETUP_COUNT_1ms);
//}