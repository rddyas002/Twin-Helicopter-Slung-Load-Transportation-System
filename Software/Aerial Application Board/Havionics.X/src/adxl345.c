#define ADXL345_H_IMPORT

#include "imu.h"

void ADXL345_setup(void){
    uchar res;

    // initialize struct
    memset(&adxl345_data, 0, sizeof(struct_adxl345));

    IMU_openI2C();
    
    // setup ADXL345
    // set output rate to 50Hz, normal power
    res =  IMU_tryConfig(ADXL345_RA_BW_RATE, ADXL345_RATE_50, ADXL345_ADDRESS);
    // set power control - put in measurement mode
    res =  IMU_tryConfig(ADXL345_RA_POWER_CTL, (1 << ADXL345_PCTL_MEASURE_BIT), ADXL345_ADDRESS);
    // enable data ready interrupt
    res =  IMU_tryConfig(ADXL345_RA_INT_ENABLE, (1 << ADXL345_INT_DATA_READY_BIT), ADXL345_ADDRESS);
    // map data ready interrupt to INT1
    res =  IMU_tryConfig(ADXL345_RA_INT_MAP, (~(1 << ADXL345_INT_DATA_READY_BIT)) & 0xFF, ADXL345_ADDRESS);
    // data format
    uchar data = (1 << ADXL345_FORMAT_FULL_RES_BIT) | (ADXL345_RANGE_16G);
    res =  IMU_tryConfig(ADXL345_RA_DATA_FORMAT, data, ADXL345_ADDRESS);
    // set FIFO control - enable stream mode, set 32 data points to interrupt
    data = (ADXL345_FIFO_MODE_BYPASS << (ADXL345_FIFO_MODE_BIT + 1 - ADXL345_FIFO_MODE_LENGTH));
    data |= (0b0001 << (ADXL345_FIFO_SAMPLES_BIT + 1 - ADXL345_FIFO_SAMPLES_LENGTH));
    res =  IMU_tryConfig(ADXL345_RA_FIFO_CTL, data, ADXL345_ADDRESS);

    data = IMU_readI2C1(ADXL345_RA_DATA_FORMAT, ADXL345_ADDRESS);

    ADXL345_calibrate();
}

void ADXL345_calibrate(void){
    int i;
    float x = 0.0, y = 0.0, z = 0.0;
    short int x_int16 = 0, y_int16 = 0, z_int16 = 0;

    for (i = 0; i < ADXL345_CALIBRATE_ITERATION; i++){
        ADXL345_readData();
        wait(7937);
        x += adxl345_data.x_float;
        y += adxl345_data.y_float;

        x_int16 += adxl345_data.x_16bit;
        y_int16 += adxl345_data.y_16bit;
    }
    adxl345_data.bias_x_float = x / ADXL345_CALIBRATE_ITERATION;
    adxl345_data.bias_y_float = y / ADXL345_CALIBRATE_ITERATION;
    adxl345_data.bias_z_float = 0;

    adxl345_data.bias_x_16bit = x_int16 / ADXL345_CALIBRATE_ITERATION;
    adxl345_data.bias_y_16bit = y_int16 / ADXL345_CALIBRATE_ITERATION;
    adxl345_data.bias_z_16bit = 0;
}

void ADXL345_readData(void)
{
    // operation takes 860us at 100kHz and 246.5us at 400kHz
    uchar WRITE_ADDRESS, READ_ADDRESS;
    WRITE_ADDRESS = (ADXL345_ADDRESS << 1);
    READ_ADDRESS = ((ADXL345_ADDRESS << 1) | 0x01);

    StartI2C1();
    IdleI2C1();

    MasterWriteI2C1(WRITE_ADDRESS);
    IdleI2C1();

    MasterWriteI2C1(ADXL345_RA_DATAX0);
    IdleI2C1();

    RestartI2C1();
    IdleI2C1();

    MasterWriteI2C1(READ_ADDRESS);
    IdleI2C1();

    unsigned short int i2c1word;
    signed short int temp_data = 0;

    unsigned char counter = 0;
    for (counter = 0; counter < 3; counter++)
    {
	i2c1word = MasterReadI2C1();
	IdleI2C1();

    	AckI2C1();
	IdleI2C1();

        temp_data = (signed short int)(((ushort)MasterReadI2C1() << 8) | i2c1word);
	IdleI2C1();

	switch(counter)
	{
            case ACCEL_X:
		adxl345_data.x_16bit = temp_data;
                adxl345_data.x_float = (float)temp_data / ADXL345_TO_G_X;
		break;
            case ACCEL_Y:
		adxl345_data.y_16bit = temp_data;
                adxl345_data.y_float = (float)temp_data / ADXL345_TO_G_Y;
            	break;
            case ACCEL_Z:
                adxl345_data.z_16bit = -temp_data;
                adxl345_data.z_float = -(float)temp_data / ADXL345_TO_G_Z;
		break;
	}

	if (counter < 2)
	{
            AckI2C1();
            IdleI2C1();
	}
	else
	{
            StopI2C1();
            IdleI2C1();
	}
    }
}

// return member variables
short int ADXL345_getx(void){
    return adxl345_data.x_16bit;
}

short int ADXL345_gety(void){
    return adxl345_data.y_16bit;
}

short int ADXL345_getz(void){
    return adxl345_data.z_16bit;
}

// formatted data
float ADXL345_getxG(void){
    return (float)adxl345_data.x_16bit / ADXL345_TO_G_X;
}

float ADXL345_getyG(void){
    return (float)adxl345_data.y_16bit / ADXL345_TO_G_Y;
}

float ADXL345_getzG(void){
    return (float)adxl345_data.z_16bit / ADXL345_TO_G_Z;
}

// formatted data
float ADXL345_getxG_f(void){
    return adxl345_data.x_float;
}

float ADXL345_getyG_f(void){
    return adxl345_data.y_float;
}

float ADXL345_getzG_f(void){
    return adxl345_data.z_float;
}