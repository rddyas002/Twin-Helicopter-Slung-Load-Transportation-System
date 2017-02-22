#define PWM_H_IMPORT

#include "../inc/pwm.h"

PWM_packet_struct PWM_packet;

void PWM_initialize(void){
    memset((PWM_packet_struct *) &PWM_packet, 0, sizeof(PWM_packet_struct));
    OpenI2C2( I2C_EN, PWM_I2C2_BAUD);
}

bool PWM_sendPacket(unsigned char config, unsigned char data[], unsigned char len){
    unsigned char k = 0;

    if (len >= PWM_LENGTH_MAX)
        return false;

    PWM_packet.data[0] = config;
    for (k = 0; k < len; k++){
        PWM_packet.data[k+1] = data[k];
    }

    PWM_packet.length = len + 1;

    return PWM_sendData();
}

bool PWM_sendData(void){
	// There is no way of improving the i2c comms with the current hardware setup.
	// The performance/speed of the i2c is limited by the performance of the slave processor
	// the master has to oprion but to wait until the slave releases the clock line
	// after a master transmission. Because this method is blocking, it is placed in the
	// main loop.

    unsigned char checksum = 0, i = 0;

    // Send start bit
    StartI2C2();
    IdleI2C2();

    // Send address + write command
    MasterWriteI2C2(PWM_RADIOBOARD_WR);
    IdleI2C2();

    // Send all data commands and compute a checksum
    for (i = 0; i < PWM_packet.length; i++)
    {
        MasterWriteI2C2(PWM_packet.data[i]);
        checksum ^= PWM_packet.data[i];
        IdleI2C2();
    }

    #if defined (PWM_TEST)
        static int count = 0;
        if (count++ < 1){
            // Write checksum
            MasterWriteI2C2(checksum);
            IdleI2C2();
        }
        else{
            // Write fake checksum
            MasterWriteI2C2(0x00);
            IdleI2C2();

            if (count == 10){
                count = 0;
            }
        }
    #else
        // Write checksum
        MasterWriteI2C2(checksum);
        IdleI2C2();
    #endif
    // End of send sequence

    // Now send restart, with read command
    RestartI2C2();
    IdleI2C2();

    MasterWriteI2C2(PWM_RADIOBOARD_RE);
    IdleI2C2();

    // If address + read was acknowledged, except data
    if(I2C2STATbits.ACKSTAT)
    {
        // Send stop bit to end transmission
        StopI2C2();
        IdleI2C2();
        return 0;
    }

    unsigned char returned_val;
    returned_val = MasterReadI2C2();
    IdleI2C2();

    // Send stop bit to end transmission
    StopI2C2();
    IdleI2C2();

    // The code for 'A' is the acknowledge byte. Return 1 if reception was successful
    if (returned_val == 'A')
    	return 1;
    else
    	return 0;
}

UINT8 PWM_GAIN2BYTE(float Kp){
    if (Kp > 70)
        Kp = 70;
    if (Kp < 30)
        Kp = 30;
    float gyrogain_us = (2000 - Kp*10);
    return PWM_PULSEWIDTH2BYTE((int)(gyrogain_us + 0.5));   // add 0.5 to round to nearest int
}

UINT8 PWM_PULSEWIDTH2BYTE(int pulsewidth){
    float temp = (((float)pulsewidth - PWM_WIDTH_MIN)*(float)PWM_MAP_GAIN);

    if (temp < 0)
        return 0;

    if (temp > PWM_MAX_OUTPUT)
        return PWM_MAX_OUTPUT;

    return (UINT8)temp;
}

#undef PWM_H_IMPORT
