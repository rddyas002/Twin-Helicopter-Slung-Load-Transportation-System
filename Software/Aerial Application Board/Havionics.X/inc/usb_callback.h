/* 
 * File:   usb_callback.h
 * Author: Yashren
 *
 * Created on 01 June 2013, 9:20 PM
 */

#ifndef USB_CALLBACK_H
#define	USB_CALLBACK_H

#include <plib.h>
#include <stdio.h>
#include <stdlib.h>

#include "./USB/usb.h"
#include "./USB/usb_function_cdc.h"

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "HardwareProfile.h"

#ifdef USB_CALLBACK_H_IMPORT
	#define EXTERN
#else
	#define EXTERN extern
#endif

EXTERN void InitializeUSB(void);
EXTERN void ProcessIO(void);

#endif	/* USB_CALLBACK_H */

