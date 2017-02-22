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
#include <boost/signals2.hpp>

class UDP_socket {
public:
    typedef boost::signals2::signal<void(char [])> dataReceived_Signal;
    dataReceived_Signal* getDataReceivedSignal() { return &data_received;}
    
    UDP_socket(const char * ip_address, const char * ip_port);
    void errorMsg(const char * message);
    void commentMsg(const char * message);
    
    bool connect_socket(void);
    
    void close_rx(void);
    void close_tx(void);
    
    bool setupReceive(int port);
    bool isReceving(void);
    virtual void receiveThread(void);
    void enableReceiveThread(bool en);
    double getLastReceiveTime(void);
    void sendPacket(char data[]);
    void set_realTime_priority(void);
           
    virtual ~UDP_socket();
    
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
    
    dataReceived_Signal data_received;
    bool receiving;
    bool quit_rx_thread;
    pthread_t receive_thread;
    int read_timeout;
    double lastReceiveTime;
    char rx_data[256];
};

#endif	/* UDP_SOCKET_H */

