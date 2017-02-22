/* 
 * File:   Helicopter.h
 * Author: yashren
 *
 * Created on 19 May 2014, 11:35 AM
 */

#ifndef HELICOPTER_H
#define	HELICOPTER_H

#include "UDP_socket.h"
#include "Multicast.h"
#include <time.h>
#include <pthread.h>

#include "ekf_coder_initialize.h"
#include "projectStateAndCov.h"
#include "Reb.h"
#include "eulerAnglesFromQuaternion.h"
#include "quaternionRotation.h"

#define ACCMAPX11   (38.5782531832037e-003)
#define ACCMAPX12   (-1.17054651277354e-003)
#define ACCMAPX13   (-648.265926944091e-006)
#define ACCMAPX21   (723.950285718723e-006)
#define ACCMAPX22   (38.3083952068034e-003)
#define ACCMAPX23   (-2.40808381814941e-003)
#define ACCMAPX31   (827.042921335916e-006)
#define ACCMAPX32   (2.19604387212654e-003)
#define ACCMAPX33   (39.4169633189162e-003)
#define ACCMAPT1   (362.203749764242e-003)
#define ACCMAPT2   (280.687603194030e-003)
#define ACCMAPT3   (-1.66652634449654e000)

#define SERVO_LEFT_BIAS 1502
#define SERVO_RIGHT_BIAS 1508
#define SERVO_REAR_BIAS 1455
#define SERVO_AVG_HEIGHT_2_COLLECTIVE_GRAD (0.067259)
#define SERVO_AVG_HEIGHT_2_COLLECTIVE_OFFSET (-1.9331)
#define SERVO_LONG_DIFF_2_LONG_GRAD (0.014896)
#define SERVO_LONG_DIFF_2_LONG_OFFSET (1.0403)
#define SERVO_LAT_DIFF_2_LAT_GRAD (0.024593)
#define SERVO_LAT_DIFF_2_LAT_OFFSET (0.82615)

#define POSITION_STATE_ERROR_VARIANCE   (0.1)
#define VELOCITY_STATE_ERROR_VARIANCE   (100)
#define ACCELEROMETER_VARIANCE_SCALER   (1)    // misalignment error
#define GYROSCOPE_VARIANCE_SCALER       (1)    // misalignment error
#define EKF_DT                          (10e-3)
#define GYROSCOPE_BIAS_VARIANCE         (1E-15)
#define ACCELEROMETER_BIAS_VARIANCE     (1E-15)

#define ACCELEROMETER_FILTER_BW     (2*M_PI*2.5)  // rad/s
#define ACCELEROMETER_FILTER_WN     (2*M_PI*3.9)

#define LPF_KINEMATICS_WN           (2*M_PI*1)

#define EKF_INIT_ITERATION 100
#define MULTICAST_IP "225.0.0.37"

#define STATS_SAMPLE    (100)
#define POS_COVAR_LIMIT (10e-3)

#define AUTO_MODE           (1 << 0)
#define RN131_TIMEOUT       (1 << 1)
#define RN131_SYNC          (1 << 2)
#define POSITION_CNTL       (1 << 3)
#define STABLE_EKF          (1 << 4)
#define SPEKTRUM_TIMEOUT    (1 << 5)

#define TRANSMIT_SLEEP      (50000)

typedef enum uav_type{
    YASHHELI, 
    ARNOQUAD}
uavType;

typedef struct{
    // Identification
    int id;
        
    // Network details
    char ip_address[64];
    char ip_port[10];
    char pc_port[10];
    char multicast_port[10];
    bool multicast_use;
                
    // Initial parameters
    real_T start_time;
    real_T objects_points[9];
    real_T initial_pose[7];
    
    real_T accelraw1g[3];
    real_T gyro_raw2dps[3];
    
    uavType type;
    
    // lsq data computer (UDP))
    char ip_address_udp[64];
    char ip_port_udp[10];    
} Helicopter_info_struct;

typedef struct{
    // identification
    int id;
    // position [meters]
    float tx;
    float ty;
    float tz;
    // velocity [meters/second]
    float vx;
    float vy;
    float vz;    
    // quaternion
    float q0;
    float q1;
    float q2;
    float q3;
    // raw gyro and accelerometer readings
    short int gyro_x;
    short int gyro_y;
    short int gyro_z;
    short int accel_x;
    short int accel_y;
    short int accel_z;    
    // ultrasonic distance
    unsigned short int ultrasonic;
    // battery voltage
    unsigned short int voltage;
    // motor head speed
    unsigned short int head_speed;
    // Gyro temperature
    short int temperature;
    
    // when data was captured
    unsigned int timestamp;
}Helicopter_data_struct;

class Helicopter {
public:    
    typedef boost::signals2::connection signal_connection;
    
    Helicopter(Helicopter_info_struct h_info);
    void initializeVariables(void);
    double FPSCalc(void);
    double deltaTime(void);
    double dtCalc(void);
    unsigned int timeNow(void);
    void decodeData(char data[]);
    bool makeConnection(const char * ip_address, const char * ip_port, const char * rx_port);
    bool HelicopterSyncd(void);
    void sendPacket(char data[]);
    void sendDummy(void);
    bool getAutoMode(void);
    void enableStatePropagation(bool en);
    void initialiseEKFvariables(void);
    real_T * kalmanState(void);
    real_T * kalmanErrorCovariance(void);
    real_T * lastProjection(void);
    double getEKFRate(void);
    int getID(void);   
    void logCorrectionTime(void);
    unsigned int getCorrectionTime(void);
    unsigned int timeSinceCorrection(void);
    void printTiming(void);
    virtual void transmitThread(void);
    void enableTransmitThread(bool en);
    void sendData(void);
    uavType getType(void);
    Helicopter_info_struct * getHelicopterInfo(void);
    
    // Filter equations
    float notchAccel(float accel_in, float input[3], float output[3], float dt);
    void filter_2nd_order(float n0, float n1, float n2, float d0, float d1, float d2, float input[3], float output[3]);
    float filter_notch(float wn, float Q, float Ts, float input[3], float output[3]);
    void shiftData(float data[3]);
    void initialiseNotchFilter(void);
    void filter_1st_order(float n1, float n2, float d1, float d2, float input[3], float output[3]);
    float lpf_filter(float filter_in, float dt, float wn, float input[3], float output[3]);
        
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
    float getTemperature(void);
    void setx(real_T val);
    void sety(real_T val);
    void setz(real_T val);
    void setRoll(real_T val);
    void setPitch(real_T val);
    void setYaw(real_T val);
    void setForwardQuaternionEstimate(float quat[4]);
    void runEKF();
    void projectStateErrorCovarianceCurrent(void);
    void float2ASCIIHex(const float val, char * return_string);
    void u32int2ASCIIHex(const unsigned int val, char * return_string);
    void shortInt2ASCIIHex(const short int val, char * return_string);
    void calibratedAcceleration(const short int acceleration_in[3], real_T acceleration_out[3]);
    void calculateAcceleration(const real_T x[16], const real_T u[6], real_T dynamic_accel[3]);
    
    real_T * getRed3D(void);
    real_T * getGreen3D(void);
    real_T * getBlue3D(void);
    real_T * getObjectPoints(void);
    void setRed3D(real_T * blob);
    void setGreen3D(real_T * blob);
    void setBlue3D(real_T * blob);
    void updateObjectPoints(void);
    
    real_T * getInitPose(void);
    real_T * getInitCoordinates(void);
    real_T * getInitOrientation(void);
    void setInitCoordinates(real_T * coordinates);
    void setInitOrientation(real_T * orientation); 
    
    void setInitialisingPose(bool val);
    bool getInitialisingPose(void);
    void initialiseOffsetVariables(void);
    
    int * getInitCounterSum(void);
    real_T * getInitq0Sum(void);
    real_T * getInitq1Sum(void);
    real_T * getInitq2Sum(void);
    real_T * getInitq3Sum(void);
    real_T * getInitxSum(void);
    real_T * getInitySum(void);
    real_T * getInitzSum(void);
    
    // Mocap kinematic functions
    float * mocapPositionX(void);
    float * mocapPositionY(void);
    float * mocapPositionZ(void);
    float * mocapVelocityX(void);
    float * mocapVelocityY(void);
    float * mocapVelocityZ(void);
    float * mocapRoll(void);
    float * mocapPitch(void);
    float * mocapYaw(void);          
    float * mocapRollRate(void);
    float * mocapPitchRate(void);
    float * mocapYawRate(void);
    float * mocapq0(void);
    float * mocapq1(void);
    float * mocapq2(void);    
    float * mocapq3(void);        
    
    // Network info
    char * getIpAddress(void);
    char * getIpPort(void);
    char * getPCport(void);
    
    inline float round(float in){
        return floor(in + 0.5);
    }
    
    void debugMessage(const char * str);
    void errorMessage(const char * str);
    
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
    Helicopter_info_struct helicopter_info;
    Helicopter_data_struct helicopter_data;
    double helicopter_t0_us;
    
    Multicast * multicast;
    char multicast_data[256];
    
    int helicopter_id;
    char ip_address[64];
    char ip_port[10];
    char pc_port[10];
    char broadcast_port[10];
    
    // Helicopter instrument readings
    signed short int gyroscope[3];
    signed short int gyro_temperature;
    signed short int accelerometer[3];
    signed short int magnetometer[3];
    short int temperature;
    float temperature_f;
    unsigned short int ultrasonic_distance;
    unsigned short int voltage;
    unsigned short int head_speed;
    unsigned short int configuration;
    unsigned short int key;
    unsigned long int pressure;
    unsigned char checksum;
    unsigned char PWM_packet[6];
    unsigned int measurement_timestamp;
    unsigned int previous_stateupdate_timestamp;
    
    signed short int onboard_pitch;
    signed short int onboard_yaw;
    signed short int onboard_roll;
    signed short int tx_x_position;
    signed short int tx_y_position;
    signed short int tx_z_position;   
    bool auto_mode;
    
    float accelxFilterIn[3];
    float accelxFilterOut[3];
    float accelyFilterIn[3];
    float accelyFilterOut[3];
    float accelzFilterIn[3];
    float accelzFilterOut[3];
    
    float trans_x_filter_in[3], trans_x_filter_out[3];
    float trans_y_filter_in[3], trans_y_filter_out[3];
    float trans_z_filter_in[3], trans_z_filter_out[3];
    
    float vel_x_filter_in[3], vel_x_filter_out[3];
    float vel_y_filter_in[3], vel_y_filter_out[3];
    float vel_z_filter_in[3], vel_z_filter_out[3];    
    
    float collective, lateral, longitudinal, latency;
    
    // pure camera derived kinematics
    float mocap_position_x;
    float mocap_position_y;
    float mocap_position_z;
    float mocap_prev_position_x;
    float mocap_prev_position_y;
    float mocap_prev_position_z;    
    float mocap_velocity_x;
    float mocap_velocity_y;
    float mocap_velocity_z;
    float mocap_q0;
    float mocap_q1;
    float mocap_q2;    
    float mocap_q3;        
    float mocap_roll;
    float mocap_pitch;
    float mocap_yaw;
    float mocap_prev_roll;
    float mocap_prev_pitch;
    float mocap_prev_yaw;    
    float mocap_wx;
    float mocap_wy;
    float mocap_wz;    
    
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
    real_T helicopter_ax, helicopter_ay, helicopter_az;
    real_T quaternion_cam[4];
    real_T init_gyro_bias_est[3];
    bool ekf_initialising;
    unsigned int correction_time;
    real_T last_projection;
    char tx_data[256];
    real_T gyro_raw2dps[3];
    real_T accel1g[3];
    
    char log_buffer[512];
    char ekf_buffer[512];

    double fps;
    double process_time;
    
    real_T red3D[3];
    real_T green3D[3];
    real_T blue3D[3];
    real_T object_points[9];
    real_T initial_coordinates[3];
    real_T initial_orientation[4];
    real_T initial_pose[7];
    bool initialising_pose;  
    
    double q_offset[4];
    double translation_offset[3];
    int init_counter_sum; 
    
    struct timespec dt_last;
    struct timespec fps_last_t;
    struct timespec delta_last_t;
    
    // Replace static variables
    bool fps_first_enter;
    bool calc_first_enter;
    bool delta_first_enter;
    int stats_count;
    double fps_sum;
    double proc_time_sum;

    int sample;
    double accel_body_sum[3];
    double ang_rate_body_sum[3];    
    
    unsigned char helicopter_config;
    bool quit_transmit_thread;
    bool transmit_thread_active;
    pthread_t transmit_thread;
};

#endif	/* HELICOPTER_H */

