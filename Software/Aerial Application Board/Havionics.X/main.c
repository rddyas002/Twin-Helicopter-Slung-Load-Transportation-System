/*
 * Firmware for Aerial Avionics Boad
 * Yashren Reddi
 * Department of Electrical Engineering
 * University of Cape Town
 * June 2013
 */

#include <plib.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "./USB/usb.h"
#include "./USB/usb_function_cdc.h"

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "HardwareProfile.h"

#include "usb_callback.h"
#include "rn131.h"
#include "io.h"
#include "imu.h"
#include "ultrasonic.h"
#include "pwm.h"
#include "spektrumRX.h"
#include "MDD File System/FSIO.h"

#define FIRMWARE_VERSION "HELICOPTER_V1.0"

/* Processor configuration bits */
#pragma config UPLLEN   = ON                    // USB PLL enabled
#pragma config UPLLIDIV = DIV_2                 // USB PLL input divider
#pragma config FPLLMUL = MUL_20			// PLL multiplier
#pragma config FPLLIDIV = DIV_2			// PLL input divider
#pragma config FPLLODIV = DIV_1			// PLL output divider
#pragma config FWDTEN = OFF 			// Watchdog timer enable bit, done later in software
#pragma config WDTPS = PS1024			// Set watchdog postscaler to bit after 512ms
#pragma config POSCMOD = HS			// High speed oscillator mode
#pragma config FNOSC = PRIPLL			// Use primary oscillator with PLL
#pragma config FPBDIV = DIV_1			// Set peripheral clock divisor to 1
#pragma config ICESEL = ICS_PGx2		// Use PGD and PGC pair 2 for programming
#pragma config BWP = OFF				// Boot flash is not writable during execution
#pragma config DEBUG = OFF

unsigned short int data_pack[17] = {0};

int main(int argc, char** argv) {
    // Disable JTAG to enable corresponding IO pin functionality
    mJTAGPortEnable(0);

    // Configure MCU for maximum performance
    SYSTEMConfigWaitStatesAndPB(GetSystemClock());	// kill wait states
    CheKseg0CacheOn();                                  // Enable cache
    mCheConfigure(CHECON | 0x30);			// Enable pre-fetch module
    mBMXDisableDRMWaitState();                          // Disable RAM wait states

#if defined(USE_USB)
        InitializeUSB();
#endif

    IO_initialize_FS();
    IO_setup();
    RN131_setupDMA();
   
    INTEnableSystemMultiVectoredInt();
    INTEnableInterrupts();

#if !defined(USE_USB)
    IO_delayms(2000);

    ITG3200_setup();
    ADXL345_setup();
    ULTRASONIC_setup();
    PWM_initialize();
    SPEKTRUMRX_initialize();
    IO_LED_init(1);
    IO_changeNotificationSetup();
    IO_getRadioNominal();
    IO_LED_init(2);
    // Initialize SD file system
    IO_initialiseLogFiles();
    IO_LED_init(3);

    RN131_setDataReceived(true);

    char temp_buf[256];
    int len = sprintf(temp_buf,"%s\nDebugging log file\n", FIRMWARE_VERSION);
    IO_dmesgMessage(temp_buf, len);
    SPEKTRUM_logNominal();

    // Boot complete; Set System mode to manual
    IO_setSystemState(SYS_MANUAL);
    IO_Control_Int_Enable();
#endif

    // Enable watchdog timer
    EnableWDT();

    while(1){        
        #if defined(USE_USB)
            if(USB_BUS_SENSE && (USBGetDeviceState() == DETACHED_STATE)){
                USBDeviceAttach();
            }            
            ProcessIO();

            if (USBUSARTIsTxTrfReady() && RN131_dataAvailable()){
                putUSBUSART(RN131_getRxDataPointer(), RN131_getRxDataSize());
                CDCTxService();
                RN131_setDataAvailable(false);
                RN131_setRxMsgLenToZero();
            }
        #else

        if (IO_getSDFlush()){
            IO_flush_FS();
            IO_setSDFlush(false);
        }

        if (IO_getCloseFiles()){
            IO_terminate_FS();
            IO_setCloseFiles(false);
        }
        #endif
        // Clear watchdog timer
        ClearWDT();
    }

    return (EXIT_SUCCESS);
}

