/* 
 * File:   Helicopter.h
 * Author: yashren
 *
 * Created on 19 May 2014, 11:35 AM
 */

#ifndef HELICOPTER_H
#define	HELICOPTER_H

#include "UDP_socket.h"
#include <qt4/QtCore/qstring.h>
#include <time.h>
#include "Multicast.h"
#include "ekf_coder_initialize.h"
#include "projectStateAndCov.h"
#include "Reb.h"
#include "eulerAnglesFromQuaternion.h"
#include "quaternionRotation.h"

#define SERVO_LEFT_BIAS 1502
#define SERVO_RIGHT_BIAS 1508
#define SERVO_REAR_BIAS 1455
#define SERVO_AVG_HEIGHT_2_COLLECTIVE_GRAD (0.067259)
#define SERVO_AVG_HEIGHT_2_COLLECTIVE_OFFSET (-1.9331)
#define SERVO_LONG_DIFF_2_LONG_GRAD (0.014896)
#define SERVO_LONG_DIFF_2_LONG_OFFSET (1.0403)
#define SERVO_LAT_DIFF_2_LAT_GRAD (0.024593)
#define SERVO_LAT_DIFF_2_LAT_OFFSET (0.82615)

#define ACCELEROMETER_X_2_SI    (9.81/272)
#define ACCELEROMETER_Y_2_SI    (9.81/258)
#define ACCELEROMETER_Z_2_SI    (9.81/225)

#define GYROSCOPE_2_SI          ((M_PI/180)/14.375)

#define INITIAL_POSITION_X  (0)
#define INITIAL_POSITION_Y  (0)
#define INITIAL_POSITION_Z  (756E-3)

#define POSITION_STATE_ERROR_VARIANCE   (1E-6)
#define VELOCITY_STATE_ERROR_VARIANCE   (1E-4)

#define EKF_INIT_ITERATION 100

#define T0_TIME 1375679601

class Helicopter {
public:
    typedef boost::signals2::connection signal_connection;
    
    Helicopter(int id, char * ip_add, char *ip_port);
    void initializeVariables(void);
    void decodeData(char data[]);
    bool makeConnection(const char * ip_address, const char * ip_port);
    void sendPacket(char data[]);
    void sendDummy(void);
    bool getAutoMode(void);
    void enableStatePropagation(bool en);
    void initialiseEKFvariables(void);
    real_T * kalmanState(void);
    real_T * kalmanErrorCovariance(void);
    real_T * lastCorrection(void);
    real_T * lastProjection(void);
    double getEKFRate(void);
    real_T timeSinceLastCorrection(void);
    int getID(void);   
        
    real_T getx(void);
    real_T gety(void);
    real_T getz(void);
    real_T getVx(void);
    real_T getVy(void);
    real_T getVz(void);    
    real_T getRoll(void);
    real_T getPitch(void);
    real_T getYaw(void);
    real_T getOnboardRoll(void);
    real_T getOnboardPitch(void);
    real_T getOnboardYaw(void);
    real_T getOnboardTx(void);
    real_T getOnboardTy(void);
    real_T getOnboardTz(void);    
    float getVoltage(void);
    float getHeadSpeed(void);
    void setx(real_T val);
    void sety(real_T val);
    void setz(real_T val);
    void setRoll(real_T val);
    void setPitch(real_T val);
    void setYaw(real_T val);
    void setForwardQuaternionEstimate(float quat[4]);
    void runEKF(float dt, real_T u[6]);
    void float2ASCIIHex(const float val, char * return_string);
    void shortInt2ASCIIHex(const short int val, char * return_string);
    short int returnTruncINT16(float val);
    double timeSinceStart(void);
    inline float round(float in){
        return floor(in + 0.5);
    }
    
    
    // Logging helicopter received data
    void initialiseLog(void);
    std::ofstream dataLog;
    std::ofstream ekfLog;
    void openLog(char * str);
    void openEKFLog(char * str);
    void closeLog(void);
    bool writeLog(void);   
    bool writeEKFLog(void);   
    
    virtual ~Helicopter();
private:
    Multicast * broadcast;
    int helicopter_id;
    
    double start_t0;
    
    // Helicopter instrument readings
    signed short int gyroscope[3];
    signed short int gyro_temperature;
    signed short int accelerometer[3];
    signed short int magnetometer[3];
    float temperature_bmp;
    unsigned short int ultrasonic_distance;
    unsigned short int voltage;
    unsigned short int head_speed;
    unsigned short int key;
    unsigned long int pressure;
    unsigned char checksum;
    
    signed short int onboard_pitch;
    signed short int onboard_yaw;
    signed short int onboard_roll;
    signed short int tx_x_position;
    signed short int tx_y_position;
    signed short int tx_z_position;   
    bool auto_mode;
    
    float collective, lateral, longitudinal, latency;
    
    /***************************/
    UDP_socket * udp_socket;
    signal_connection data_rx_connection;
    
    /*************EKF***********/
    bool state_propagation_on;
    real_T x_ekf[16];
    real_T P_ekf[256];
    real_T Q_ekf[324];
    real_T body_acceleration[3];
    real_T static_acceleration_earth[3];
    real_T static_acceleration_body[3];   
    real_T roll_ekf, pitch_ekf, yaw_ekf;
    real_T helicopter_roll, helicopter_pitch, helicopter_yaw;
    real_T helicopter_x, helicopter_y, helicopter_z;
    real_T helicopter_vx, helicopter_vy, helicopter_vz;
    real_T quaternion_cam[4];
    real_T init_gyro_bias_est[3];
    bool ekf_initialising;
    real_T last_correction;
    real_T last_projection;
    double ekf_rate;
    char tx_data[256];
    unsigned short int sequence;
    
    char log_buffer[512];
    char ekf_buffer[512];
    
    double connect_time;
};

#endif	/* HELICOPTER_H */

