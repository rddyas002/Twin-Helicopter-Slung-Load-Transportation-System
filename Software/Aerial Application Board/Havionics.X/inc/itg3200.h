
#ifndef ITG3200_H
#define	ITG3200_H

#ifdef ITG3200_H_IMPORT
    #define ITG3200_EXTERN
#else
    #define ITG3200_EXTERN extern
#endif

#include "common.h"

/* ITG3200 REGISTERS */
#define ITG3200_ADDRESS (0x68)
#define ITG3200_WHO_AM_I (0x00)
#define ITG3200_SAMPLE_RATE_DIVIDER (0x15)
#define ITG3200_DLPF (0x16)
#define ITG3200_INTERRUPT_CONFIGURATION (0x17)
#define ITG3200_INTERRUPT_STATUS (0x1A)
#define ITG3200_TEMP_OUT_H (0x1B)
#define ITG3200_TEMP_OUT_L (0x1C)
#define ITG3200_GYRO_XOUT_H (0x1D)
#define ITG3200_GYRO_XOUT_L (0x1E)
#define ITG3200_GYRO_YOUT_H (0x1F)
#define ITG3200_GYRO_YOUT_L (0x20)
#define ITG3200_GYRO_ZOUT_H (0x21)
#define ITG3200_GYRO_ZOUT_L (0x22)
#define ITG3200_POWER_MANAGEMENT (0x3E)

#define ITG3200_ITG_RDY (1 << 2)
#define ITG3200_GYRO_SCALE_FACTOR (14.375)
#define ITG3200_TEMP_OFFSET (13200)
#define ITG3200_TEMP_SENSITIVITY (280)
#define ITG3200_TEMP_OFFSET_DEGREES (35)

#define ITG3200_CALIBRATE_BITSHIFT (9)
#define ITG3200_CALIBRATE_ITERATION 1000

#define ITG3200_FILTER_N    (30)

typedef struct {
    signed short int temp_16bit;

    short int x_16bit;
    short int y_16bit;
    short int z_16bit;

    float x_float;
    float y_float;
    float z_float;

    short int bias_x_16bit;
    short int bias_y_16bit;
    short int bias_z_16bit;

    float bias_x_float;
    float bias_y_float;
    float bias_z_float;

    char status;
}struct_itg3200;

typedef enum{
    TEMPERATURE,
    GYRO_Y,
    GYRO_X,
    GYRO_Z
}ITG3200_CASE;

ITG3200_EXTERN struct_itg3200 itg3200_data;

ITG3200_EXTERN bool ITG3200_setup(void);
ITG3200_EXTERN void ITG3200_readData(void);
ITG3200_EXTERN void ITG3200_calibrate(void);
ITG3200_EXTERN void ITG3200_setCalibrationCoefficients(float px1, float px2,
        float py1, float py2, float pz1, float pz2);

// return member variables
ITG3200_EXTERN short int ITG3200_getx(void);
ITG3200_EXTERN short int ITG3200_gety(void);
ITG3200_EXTERN short int ITG3200_getz(void);
ITG3200_EXTERN UINT16 ITG3200_getRawTemperature(void);

// return member variables
ITG3200_EXTERN float ITG3200_getx_float(void);
ITG3200_EXTERN float ITG3200_gety_float(void);
ITG3200_EXTERN float ITG3200_getz_float(void);

// formatted data
ITG3200_EXTERN float ITG3200_getTempCelcius(void);
ITG3200_EXTERN float ITG3200_getxDPS(void);
ITG3200_EXTERN float ITG3200_getyDPS(void);
ITG3200_EXTERN float ITG3200_getzDPS(void);

// temperature calibration
ITG3200_EXTERN float ITG3200_calibrateGyroX(float data);
ITG3200_EXTERN float ITG3200_calibrateGyroY(float data);
ITG3200_EXTERN float ITG3200_calibrateGyroZ(float data);
ITG3200_EXTERN float ITG3200_temperatureFilter(float temperature);
ITG3200_EXTERN float ITG3200_getFilteredTemperature(void);
ITG3200_EXTERN float ITG3200_getGyroBiasX(void);
ITG3200_EXTERN float ITG3200_getGyroBiasY(void);
ITG3200_EXTERN float ITG3200_getGyroBiasZ(void);
ITG3200_EXTERN void ITG3200_gyroBiasComputation(void);
#endif	/* ITG3200_H */

