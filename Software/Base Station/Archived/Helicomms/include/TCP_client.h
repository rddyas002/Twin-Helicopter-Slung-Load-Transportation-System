/* 
 * File:   TCP_client.h
 * Author: yashren
 *
 * Created on 05 May 2013, 1:08 PM
 */

#ifndef TCP_CLIENT_H
#define	TCP_CLIENT_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <poll.h>
#include <boost/signals2.hpp>
#include <locale.h>
#include <qt4/QtCore/qstring.h>
#include <sys/time.h>
#include "OpencvCamera.h"

#define MAX_CLIENTS 4

class TCP_client {
public:
    enum client_msg{
        GET_DATA,
        KILL_PROGRAM,
        START_AUTO,
        STOP_AUTO
    };
    
    enum callback_msg{
        TERMINATE_CLIENT
    };    
    
    typedef boost::signals2::signal<void(  callback_msg, int)> m_Signal;
    typedef boost::signals2::signal<void(  int )> dataReceived_Signal;
    TCP_client(int id, int port, const char * ipaddress, OpencvCamera * vCam);
    void send2server();
    void write2server(const char * data);
    void sendMsg(client_msg msg);
    bool tryConnect(void);
    virtual void receiveThread(void);
    void stopReceiveThread(void);
    bool newDataAvailable(void);
    void decodeData(void);
    
    std::ofstream image_data_log;
    void openLog(char * str);
    void closeLog(void);
    bool writeLog(void);    
    
    void closeTCPClient(void);
    int getClientID(void);
    void sendBlobData(char * str);
    void sendBuffer(void);
    void debugMessage(const char * str);
    void errorMessage(const char * str);
    virtual ~TCP_client();
    m_Signal* getCallbackSignal() { return &callback_signal; }
    dataReceived_Signal* getDataReceivedSignal() { return &data_received; }
    inline double timeSinceEpoch(double start_time){
        struct timeval tv;
        gettimeofday(&tv,NULL);
        double t = (double) tv.tv_sec + (double) 1e-6*tv.tv_usec - start_time;
        return t;
    }     
private:  
    m_Signal callback_signal;
    dataReceived_Signal data_received;
    int sockfd, portno;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    const char * server_ip;
    
    char tx_buffer[256];
    char rx_buffer[256];
    
    bool quit_rx_thread;
    pthread_t receive_thread;
    
    socklen_t fromlen1;
    struct sockaddr_in from1;

    bool new_rx_data_flag;
    int client_id;
    OpencvCamera * virtualCamera;
    void loadData(char * data);
    
    char log_buffer[512];
};

#endif	/* TCP_CLIENT_H */

