#ifndef PWM_H
#define	PWM_H

#ifdef PWM_H_IMPORT
    #define PWM_EXTERN
#else
    #define PWM_EXTERN extern
#endif

#include <plib.h>
#include <stdbool.h>
#include <string.h>
#include "HardwareProfile.h"
#include "GenericTypeDefs.h"
#include "../inc/common.h"

#define PWM_I2C2_FREQ       (200000)
#define PWM_I2C2_BAUD       ((GetPeripheralClock()/2/PWM_I2C2_FREQ) - 2)
// Slave MCU address
#define PWM_RADIOBOARD_WR   (0x94)
#define PWM_RADIOBOARD_RE   (0x95)
#define PWM_LENGTH_MAX      10

// Specs on PWM module in us
#define PWM_WIDTH_MAX       (1998)//(2000)
#define PWM_WIDTH_MIN       (900)//(934)
#define PWM_MAX_OUTPUT      (255)
#define PWM_INPUT_DIFF      (PWM_WIDTH_MAX - PWM_WIDTH_MIN)
#define PWM_MAP_GAIN        ((float)PWM_MAX_OUTPUT/PWM_INPUT_DIFF)

typedef struct{
	unsigned char length;
	unsigned char data[PWM_LENGTH_MAX];
	bool transmit;
}PWM_packet_struct;

PWM_EXTERN void PWM_initialize(void);
PWM_EXTERN bool PWM_sendData(void);
PWM_EXTERN bool PWM_sendPacket(unsigned char config, unsigned char data[], unsigned char len);
PWM_EXTERN UINT8 PWM_PULSEWIDTH2BYTE(int pulsewidth);
PWM_EXTERN UINT8 PWM_GAIN2BYTE(float Kp);

#endif	/* PWM_H */

