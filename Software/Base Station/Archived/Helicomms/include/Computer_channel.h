/* 
 * File:   Computer_channel.h
 * Author: yashren
 *
 * Created on 04 May 2013, 1:36 PM
 */

#ifndef COMPUTER_CHANNEL_H
#define	COMPUTER_CHANNEL_H

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
#include <boost/signals2.hpp>
#include <sys/time.h>


class Computer_channel {
public:
    enum client_msg{
    START_CAPTURE,
    CAPTURE_DONE,
    TERMINATE,
    TIMEOUT,
    TEST,
    CALIBRATE,
    CALIBRATE_DONE,
    ERR
    };
    
    typedef boost::signals2::signal<void( Computer_channel* )> m_Signal;
    typedef boost::signals2::signal<void( char* )> m_dataReceivedSignal;
    Computer_channel();
    void instantiate_computer(int id, int fd);
    
    void stopReceiveThread(void);
    void write2client(char * data);
    int getID(void);
    bool timeoutRead(int timeout);
    bool send2client(client_msg msg);
    double getChannelDelay(void);
    char * getDataPacket(void);
    
    virtual void receiveThread(void);
    
    m_Signal* getChannelLostSignal() { return &channelLostSignal; }
    m_dataReceivedSignal* getDataReceivedSignal() { return &dataReceived; }
    
    void errorMessage(const char * str);
    void debugMessage(const char * str);
    
    virtual ~Computer_channel();
private:
    m_Signal channelLostSignal;
    m_dataReceivedSignal dataReceived;
    int channel_id;
    int comms_fd;
    int write_retry;
    bool quit_rx_thread;
    int read_timeout;
    bool connected;
    
    char rx_data_buffer[256];
    bool new_rx_data_flag;
    socklen_t fromlen1;
    struct sockaddr_in from1;
    
    char tx_data_buffer[256];
    
    pthread_t receive_thread;
    
    // timing
    timeval start_counter, end_counter;
    double elapsed_time;
};

#endif	/* COMPUTER_CHANNEL_H */

