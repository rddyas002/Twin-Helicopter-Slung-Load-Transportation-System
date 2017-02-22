#define ITG3200_H_IMPORT

#include "../inc/imu.h"

float ITG3200_gyroTemperature = 0;
float ITG3200_filteredTemperature = 0;
float ITG3200_biasX = 0;
float ITG3200_biasY = 0;
float ITG3200_biasZ = 0;

// Gyroscope temperature calibration parameters
float ITG3200_PX1 = (0.0338);
float ITG3200_PX2 = (0.4909);
float ITG3200_PY1 = (0.0066);
float ITG3200_PY2 = (-3.4082);
float ITG3200_PZ1 = (-0.0329);
float ITG3200_PZ2 = (0.9353);

bool ITG3200_setup(void){
    uchar res;

    // initialize struct
    memset(&itg3200_data, 0, sizeof(struct_itg3200));

    IMU_openI2C();
    
    // set bandwidth to 42 Hz and internal sampling to 1kHz
    res =  IMU_tryConfig(ITG3200_DLPF, 0b11100, ITG3200_ADDRESS);
    // for sample rate of 200Hz
    res =  IMU_tryConfig(ITG3200_SAMPLE_RATE_DIVIDER, 4, ITG3200_ADDRESS);
    // set interrupt register
    res =  IMU_tryConfig(ITG3200_INTERRUPT_CONFIGURATION, 0b00110001, ITG3200_ADDRESS);
    // set clock to PLL
    res =  IMU_tryConfig(ITG3200_POWER_MANAGEMENT, 0b00000011, ITG3200_ADDRESS);

    wait(10000);

    ITG3200_calibrate();
}

void ITG3200_setCalibrationCoefficients(float px1, float px2,
        float py1, float py2, float pz1, float pz2){
    ITG3200_PX1 = px1;
    ITG3200_PX2 = px2;
    ITG3200_PY1 = py1;
    ITG3200_PY2 = py2;
    ITG3200_PZ1 = pz1;
    ITG3200_PZ2 = pz2;
}

void ITG3200_calibrate(void){
    int i;
    float x = 0.0, y = 0.0, z = 0.0;
    int x_int32 = 0, y_int32 = 0, z_int32 = 0;

    // dummy read to initialise filter variables
    for (i = 0; i < 100; i++){
        ITG3200_readData();
        wait(100);
    }

    // read for calibration
    for (i = 0; i < ITG3200_CALIBRATE_ITERATION; i++){
        ITG3200_readData();
        wait(7937);
        x += itg3200_data.x_float;
        y += itg3200_data.y_float;
        z += itg3200_data.z_float;

        x_int32 += itg3200_data.x_16bit;
        y_int32 += itg3200_data.y_16bit;
        z_int32 += itg3200_data.z_16bit;
    }

    // store float bias for accurate on-board estimation
    itg3200_data.bias_x_float = (float)x / ITG3200_CALIBRATE_ITERATION;
    itg3200_data.bias_y_float = (float)y / ITG3200_CALIBRATE_ITERATION;
    itg3200_data.bias_z_float = (float)z / ITG3200_CALIBRATE_ITERATION;

    itg3200_data.bias_x_16bit = (short int)(x_int32 / ITG3200_CALIBRATE_ITERATION);
    itg3200_data.bias_y_16bit = (short int)(y_int32 / ITG3200_CALIBRATE_ITERATION);
    itg3200_data.bias_z_16bit = (short int)(z_int32 / ITG3200_CALIBRATE_ITERATION);
}

void ITG3200_readData(void)
{
    static int bias_compensator_counter = 0;

    // this operation takes 316us at 400kHz
    unsigned char WRITE_ADDRESS, READ_ADDRESS;
    WRITE_ADDRESS = (ITG3200_ADDRESS << 1);
    READ_ADDRESS = ((ITG3200_ADDRESS << 1) | 0x01);

    StartI2C1();
    IdleI2C1();

    MasterWriteI2C1(WRITE_ADDRESS);
    IdleI2C1();

    MasterWriteI2C1(ITG3200_TEMP_OUT_H);
    IdleI2C1();

    RestartI2C1();
    IdleI2C1();

    MasterWriteI2C1(READ_ADDRESS);
    IdleI2C1();

    //check whether ack received

    unsigned short int i2c1word;
    signed short int temp_data = 0;

    // Read out 1 16 bit temperature value + 3 Gyro rates
    // After each read the internal pointer is incremented in the device
    unsigned char counter = 0;
    for (counter = 0; counter < 4; counter++)
    {
    	i2c1word = ((unsigned short int)MasterReadI2C1() << 8);
        IdleI2C1();

	AckI2C1();
        IdleI2C1();

	temp_data = (signed short int) (i2c1word | (uchar)MasterReadI2C1());
        IdleI2C1();

	switch(counter)
	{
            case TEMPERATURE:
                // implement moving average filter on temperature
		itg3200_data.temp_16bit = temp_data;
                ITG3200_temperatureFilter(ITG3200_getTempCelcius());

                // Do bias calculation every 10 samples
                if (bias_compensator_counter++ > 10)
                {
                    ITG3200_gyroBiasComputation();
                    bias_compensator_counter = 0;
                }

		break;
            case GYRO_X:
		itg3200_data.x_16bit = temp_data - itg3200_data.bias_x_16bit;
                itg3200_data.x_float = ((float)temp_data / ITG3200_GYRO_SCALE_FACTOR) - ITG3200_biasX - itg3200_data.bias_x_float;
		break;
            case GYRO_Y:
		itg3200_data.y_16bit = temp_data - itg3200_data.bias_y_16bit;
                itg3200_data.y_float = ((float)temp_data / ITG3200_GYRO_SCALE_FACTOR) - ITG3200_biasY - itg3200_data.bias_y_float;
		break;
            case GYRO_Z:
		itg3200_data.z_16bit = -temp_data - itg3200_data.bias_z_16bit;
                itg3200_data.z_float = (-(float)temp_data / ITG3200_GYRO_SCALE_FACTOR) - ITG3200_biasZ - itg3200_data.bias_z_float;
		break;
	}

	if (counter < 3)
	{
            AckI2C1();
            IdleI2C1();
        }
	else
	{
            NotAckI2C1();
            IdleI2C1();
            StopI2C1();
            IdleI2C1();
	}
    }
}

// return member variables
short int ITG3200_getx(void){
    return itg3200_data.x_16bit;
}

short int ITG3200_gety(void){
    return itg3200_data.y_16bit;
}

short int ITG3200_getz(void){
    return itg3200_data.z_16bit;
}

// return member variables
float ITG3200_getx_float(void){
    return itg3200_data.x_float;
}

float ITG3200_gety_float(void){
    return itg3200_data.y_float;
}

float ITG3200_getz_float(void){
    return itg3200_data.z_float;
}

// formatted data
float ITG3200_getTempCelcius(void){
    ITG3200_gyroTemperature = ITG3200_TEMP_OFFSET_DEGREES + ((float)(itg3200_data.temp_16bit + ITG3200_TEMP_OFFSET))/ITG3200_TEMP_SENSITIVITY;
    return ITG3200_gyroTemperature;
}

float ITG3200_getxDPS(void){
    return itg3200_data.x_float;
}

float ITG3200_getyDPS(void){
    return itg3200_data.y_float;
}

float ITG3200_getzDPS(void){
    return itg3200_data.z_float;
}

float ITG3200_gyroXbiasCalc(void){
    ITG3200_biasX = ITG3200_filteredTemperature*ITG3200_PX1 + ITG3200_PX2;
    return ITG3200_biasX;
}

float ITG3200_gyroYbiasCalc(void){
    ITG3200_biasY = ITG3200_filteredTemperature*ITG3200_PY1 + ITG3200_PY2;
    return ITG3200_biasY;
}

float ITG3200_gyroZbiasCalc(void){
    ITG3200_biasZ = ITG3200_filteredTemperature*ITG3200_PZ1 + ITG3200_PZ2;
    return ITG3200_biasZ;
}

float ITG3200_getGyroBiasX(void){
    return ITG3200_biasX;
}

float ITG3200_getGyroBiasY(void){
    return ITG3200_biasY;
}

float ITG3200_getGyroBiasZ(void){
    return ITG3200_biasZ;
}

UINT16 ITG3200_getRawTemperature(void){
    return itg3200_data.temp_16bit;
}

float ITG3200_getFilteredTemperature(void){
    return ITG3200_filteredTemperature;
}

void ITG3200_gyroBiasComputation(void){
    ITG3200_gyroXbiasCalc();
    ITG3200_gyroYbiasCalc();
    ITG3200_gyroZbiasCalc();
}

float ITG3200_temperatureFilter(float temperature){
    static float data_buffer[ITG3200_FILTER_N + 1] = {0};
    static int current_point = ITG3200_FILTER_N;
    static float prev_output = 0;
    static bool first_enter = true;

    if (first_enter){
        int i;
        // initialise array variable
        for (i = 0; i < (ITG3200_FILTER_N + 1); i++){
            data_buffer[i] = temperature;
        }
        prev_output = temperature;
        first_enter = false;
    }

    // store current data
    data_buffer[current_point] = temperature;

    // next point to store data
    current_point = (current_point == ITG3200_FILTER_N) ? 0 : current_point + 1;

    prev_output = prev_output + (temperature - data_buffer[current_point])/ITG3200_FILTER_N;
    ITG3200_filteredTemperature = prev_output;
    return ITG3200_filteredTemperature;
}

