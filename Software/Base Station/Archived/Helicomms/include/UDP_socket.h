/* 
 * File:   UDP_socket.h
 * Author: yashren
 *
 * Created on 11 March 2013, 10:36 AM
 */

#ifndef UDP_SOCKET_H
#define	UDP_SOCKET_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <fstream>
#include <QString>

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

#define IMU_INIT_IER (200)

class UDP_socket {
public:
    UDP_socket(const QString& ipaddress, const QString& ipport);
    bool connect_socket(void);
    void disconnect_socket(void);
    bool isConnected(void);
    
    void close_rx(void);
    void close_tx(void);
    
    virtual void transmitThread(void);
    bool isTransmitting(void);
    void enableTransmitThread(bool en);
    void setTransmitRate(float Hz);
    void setHelicopterTxRate(char char_tx_rate);
    
    bool setupReceive(int port);
    bool isReceving(void);
    virtual void receiveThread(void);
    void enableReceiveThread(bool en);
    void decodeReceivedData(void);
        
    signed short int * getGyroData(void);
    signed short int * getAccelData(void);
    signed short int * getMagnetoData(void);
    unsigned long int * getPressureData(void);
    unsigned short int * getUltrasonicData(void);
    unsigned short int * getVoltageData(void);
    unsigned short int * getSpeedData(void);
    float * getTempBaroData(void);
    unsigned short int getESC(void);
    unsigned short int getRudder(void);
    unsigned short int getServoLeft(void);
    unsigned short int getServoRight(void);
    unsigned short int getServoRear(void);
    unsigned short int getGain(void);   
    float getLatency(void);
    void set_realTime_priority(void);
    
    float getCollective(void);
    float getLateral(void);
    float getLongitudinal(void);
    
    void setLEDS(bool rd, bool gr, bool bl);
    void setVisualData(int tx, int ty, int tz, float q0v,
        float q1v, float q2v, float q3v, int cam_q);
    void setCamQuality(int quality);
    void sendPacket(void);
    
    void setAutoMode(char c);
    void setGPSMode(char c);
    
    float limitLengthTo5(float x);
    float limitLengthTo4(float x);
           
    virtual ~UDP_socket();
    
    static std::ofstream logFile;
    static void openLogFile(void);
    static void closeLogFile(void);
    static bool writeLogFile(void);
    static char write_buffer[512];
    
    // return filter data
    double * kalmanState(void);
    double * kalmanErrorCovariance(void);
    real_T get_tx(void);
    real_T get_ty(void);
    real_T get_tz(void);
    real_T get_vx(void);
    real_T get_vy(void);
    real_T get_vz(void);
    real_T get_roll(void);
    real_T get_pitch(void);
    real_T get_yaw(void);
    real_T get_xgyrobias(void);
    real_T get_ygyrobias(void);
    real_T get_zgyrobias(void);
    real_T get_xaccelbias(void);
    real_T get_yaccelbias(void);
    real_T get_zaccelbias(void);
    void float2ASCIIHex(const float val, char * return_string);
    void shortInt2ASCIIHex(const short int val, char * return_string);
    
    double * lastCorrection(void);
    
    inline double timeSince(double start_time){
        struct timeval tv;
        gettimeofday(&tv,NULL);
        double t = (double) tv.tv_sec + (double) 1e-6*tv.tv_usec - start_time;
        return t;
    }        

private:
    int sockfd, sockrx, sockBroadcastRx;
    bool sockfd_connected;
    int portno;
    socklen_t fromlen1, fromlenBroadcastRx;
    struct sockaddr_in from1, fromBroadcastRx;
    char heli_ip_address[25];
    int broadcastServerPort;
    
    struct sockaddr_in serv_addr;
    struct hostent *server;    
    
    bool transmitting;
    bool quit_tx_thread;
    pthread_t transmit_thread;
    char tx_data[256];
    float tx_ms_delay;
    
    bool receiving;
    bool quit_rx_thread;
    pthread_t receive_thread;
    int read_timeout;
    char rx_data[256];
        
    // decoding
    signed short int gyroscope[4];
    signed short int accelerometer[3];
    signed short int magnetometer[3];
    float temperature_bmp;
    unsigned short int ultrasonic_distance;
    unsigned short int voltage;
    unsigned short int head_speed;
    unsigned short int key;
    unsigned long int pressure;
    unsigned char checksum;
    
    unsigned short int servo_left_measured;
    unsigned short int servo_right_measured;
    unsigned short int servo_rear_measured;
    unsigned short int servo_rudder_measured;
    unsigned short int esc_measured;
    unsigned short int gyro_gain;
    
    // transmit data
    char auto_mode, gps_mode, tx_rate;
    unsigned short int servo_left, servo_right, servo_front, servo_yaw, esc_thrust;
    int room_x, room_y, room_z;
    float q0, q1, q2, q3;
    int cam_quality; 
    unsigned short int sequence;
    
    // LED data
    unsigned char leds;
    char red_led,green_led,blue_led;
    
    double time_data_received;
    double latency;
    
    float collective, lateral, longitudinal;
    
    // These are the EKF variables
    real_T x[16];
    real_T P[256];
    real_T Q[324];
    real_T static_acceleration_earth[3];
    real_T static_acceleration_body[3];
    
    real_T body_accel[3];
    
    real_T roll, pitch, yaw;
    real_T init_gyro_bias_est[3];
    
    real_T last_correction;
};

#endif	/* UDP_SOCKET_H */

