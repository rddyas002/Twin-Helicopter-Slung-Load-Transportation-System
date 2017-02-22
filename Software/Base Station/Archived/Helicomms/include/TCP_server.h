/* 
 * File:   TCP_server.h
 * Author: yashren
 *
 * Created on 04 May 2013, 12:15 PM
 */

#ifndef TCP_SERVER_H
#define	TCP_SERVER_H

#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <netinet/in.h>
#include <resolv.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>
#include <poll.h>
#include <iostream>
#include <sys/time.h>

#include "../include/Computer_channel.h"

#define NUM_OF_CLIENTS 4

class TCP_server {
public:
    TCP_server(int hostPort);
    
    typedef boost::signals2::connection signal_connection;
    
    typedef boost::signals2::signal<void( char* )> m_dataReceivedSignal;
    
    Computer_channel * getCommChannel(int id);
    int getNumClients(void);
    void signalLost(void);
    bool channelExists(int channel);
    bool channelConnected(int channel);
    void loadData(char * data, int camera_num);
    
    void displayReceivedData(char * data);
    
    virtual void waiting4clients(void);
    void stopReceiveThread(void);
    void write2allClients(char * str);
    
    virtual ~TCP_server();
    
    m_dataReceivedSignal* getDataReceivedSignal() { return &dataReceived; }
    
    inline double timeSinceEpoch(void){
        struct timeval tv;
        gettimeofday(&tv,NULL);
        double t = (double) tv.tv_sec + (double) 1e-6*tv.tv_usec;
        return t;
    }
private:
    int host_port;
	struct sockaddr_in my_addr;

	int hsock;
	int err;    
    
    int fd_client[NUM_OF_CLIENTS];
    int num_clients;
    pthread_t wait_4_client_thread;
    
    bool quit_wait_thread;
    
    Computer_channel * pc_file[NUM_OF_CLIENTS];
    bool pc_connected[NUM_OF_CLIENTS];
    
    signal_connection channelLostConnection[NUM_OF_CLIENTS];
    signal_connection dataReceivedConnection[NUM_OF_CLIENTS];
    
    m_dataReceivedSignal dataReceived;
};

#endif	/* TCP_SERVER_H */

