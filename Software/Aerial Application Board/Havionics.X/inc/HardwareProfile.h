#ifndef HARDWARE_PROFILE_PIC32MX795F512L_PIM_H
#define HARDWARE_PROFILE_PIC32MX795F512L_PIM_H

    /* USB defines */
    //#define USE_USB
#if defined (USE_USB)
    #define USE_SELF_POWER_SENSE_IO
#endif

    #define CONFIG_PARAMETERS       6
    #define DEBUG
    //#define PWM_TEST              // Used for corrupting checksum for PWM comms
    #define tris_self_power         TRISDbits.TRISD5    // Input // Bit D5 is also used in the final design
    #define self_power              1

    #define USE_USB_BUS_SENSE_IO
    #define tris_usb_bus_sense      TRISEbits.TRISE9    // Input // Bit E9 is also used in the final design
    #define USB_BUS_SENSE           PORTEbits.RE9       //PORTReadBits(IOPORT_E, BIT_9)//PORTEbits.RE9//1 //was 1 mPORTEReadBits(BIT_9)//

    /* General */
    #define led0                    LATCbits.LATC1
    #define bug                     LATEbits.LATE7
    #define togglebug();            bug = !bug;
    #define toggleLED();            led0 = !led0;
    #define on                      0
    #define off                     1
    #define high                    1
    #define low                     0
    
    // Enable read of radio channel PWM
    #define SETUP_RADIO_DETECT

    /* Switch */
    #define mInitSwitch2()          TRISDbits.TRISD6=1;
    #define mInitAllSwitches()      mInitSwitch2()
    #define sw2                     PORTDbits.RD6

    #define DEMO_BOARD PIC32MX795F512L_PIM

    /** I/O pin definitions ********************************************/
    #define INPUT_PIN 1
    #define OUTPUT_PIN 0

    /* Clock */
    #define GetSystemClock()            (80000000ul)
    #define GetPeripheralClock()        (GetSystemClock())
    #define GetInstructionClock()       (GetSystemClock())

    /* SD Card */
    // Description: Macro used to enable the SD-SPI physical layer (SD-SPI.c and .h)
    #define USE_SD_INTERFACE_WITH_SPI

    // Not hard switch on board
    #define MEDIA_SOFT_DETECT

    // Registers for the SPI module you want to use
    #define MDD_USE_SPI_1

    //SPI Configuration
    #define SPI_START_CFG_1     (PRI_PRESCAL_64_1 | SEC_PRESCAL_8_1 | MASTER_ENABLE_ON | SPI_CKE_ON | SPI_SMP_ON)
    #define SPI_START_CFG_2     (SPI_ENABLE)

    // Define the SPI frequency
    #define SPI_FREQUENCY         (20000000)   // 20 MHz

    // Description: SD-SPI Chip Select Output bit
    #define SD_CS               PORTDbits.RD9
    // Description: SD-SPI Chip Select TRIS bit
    #define SD_CS_TRIS          TRISDbits.TRISD9
   
    // Description: SD-SPI Card Detect Input bit
    #define SD_CD               PORTFbits.RF0
    // Description: SD-SPI Card Detect TRIS bit
    #define SD_CD_TRIS          TRISFbits.TRISF0

    // Description: SD-SPI Write Protect Check Input bit
    #define SD_WE               0//PORTFbits.RF1
    // Description: SD-SPI Write Protect Check TRIS bit
    #define SD_WE_TRIS          TRISFbits.TRISF1
      
    // Description: The main SPI control register
    #define SPICON1             SPI1CON
    // Description: The SPI status register
    #define SPISTAT             SPI1STAT
    // Description: The SPI Buffer
    #define SPIBUF              SPI1BUF
    // Description: The receive buffer full bit in the SPI status register
    #define SPISTAT_RBF         SPI1STATbits.SPIRBF
    // Description: The bitwise define for the SPI control register (i.e. _____bits)
    #define SPICON1bits         SPI1CONbits
    // Description: The bitwise define for the SPI status register (i.e. _____bits)
    #define SPISTATbits         SPI1STATbits
    // Description: The enable bit for the SPI module
    #define SPIENABLE           SPICON1bits.ON
    // Description: The definition for the SPI baud rate generator register (PIC32)
    #define SPIBRG             SPI1BRG

    // Tris pins for SCK/SDI/SDO lines

    // Description: The TRIS bit for the SCK pin
    #define SPICLOCK            TRISDbits.TRISD10
    // Description: The TRIS bit for the SDI pin
    #define SPIIN               TRISCbits.TRISC4
    // Description: The TRIS bit for the SDO pin
    #define SPIOUT              TRISDbits.TRISD0
    //SPI library functions
    #define putcSPI             putcSPI1
    #define getcSPI             getcSPI1
    #define OpenSPI(config1, config2)   OpenSPI1(config1, config2)

	// Will generate an error if the clock speed is too low to interface to the card
	#if (GetSystemClock() < 100000)
    	#error Clock speed must exceed 100 kHz
	#endif
#endif  //HARDWARE_PROFILE_PIC32MX795F512L_PIM_H
