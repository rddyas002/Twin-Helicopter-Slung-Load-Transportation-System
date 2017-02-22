/* 
 * File:   UAV.h
 * Author: mocap004
 *
 * Created on 17 July 2014, 3:26 PM
 */

#ifndef UAV_H
#define	UAV_H

#include <cstdlib>
#include <iostream>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <sched.h>
#include <stdlib.h>

#include "Helicopter.h"
#include "PoseEstimation.h"
#include "TCP_client.h"
#include <boost/signals2.hpp>
#include <boost/signals2/preprocessed_signal.hpp>

#define SERVER_HOST_PORT (2010)
#define SYNC_PULSES         (100)
#define MOCAP_MODE          "mocap.conf"
#define SCHWARZ_MODE        "schwarz.conf"

class UAV {
public:
    enum CONFIG_ITEM{
        ID,
        IP_ADDRESS,
        IP_PORT,
        PC_PORT,
        BROADCAST_PORT,
        INITIAL_POSE,
        RED_BLOB,
        GREEN_BLOB,
        BLUE_BLOB,
        GYRORAW2DEGREES,
        ACCELRAW2G,
        TYPE,
        UDP_IP,
        UDP_PORT
    };    
    typedef boost::signals2::connection signal_connection;
    UAV(const char * configuration_file);
    int getConfiguration(FILE * file, Helicopter_info_struct * helicopter_info);
    void requestMocapData(void);
    void connectClients(void);
    void connectClients4Mocap(void);
    void setMocamDataReceived(int val);
    void clearMocamDataReceived(void);
    unsigned int timeNow(void);
    void synchronizeClients(void);
    void TCP_Client_callback(TCP_client::callback_msg msg, int id);
    void disconnectClients(void);
    virtual ~UAV();
private:
    signal_connection tcp_callback_connection[NUM_OF_CAMERAS], data_received_connection[NUM_OF_CAMERAS];
    signal_connection correction_complete;
    timespec uav_t0;
    double uav_t0_us;

    PoseEstimation * poseEstimation;
    TCP_client * tcp_client[NUM_OF_CAMERAS];
    Helicopter * helicopter[MAX_NUM_OF_HELICOPTERS];
    Helicopter_info_struct helicopter_info[MAX_NUM_OF_HELICOPTERS];
    int active_helicopters;
    
    // for mocap only
    pthread_mutex_t uav_cam_mutex;
    bool mc_dataFromCamReceived[NUM_OF_CAMERAS];
};

#endif	/* UAV_H */

