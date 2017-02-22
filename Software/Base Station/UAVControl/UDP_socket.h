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
#include <boost/signals2.hpp>
#include <boost/signals2/preprocessed_signal.hpp>

#define UDP_socket_PACKETSIZE (51)

class UDP_socket {
public:
    typedef boost::signals2::signal<void(char [])> dataReceived_Signal;
    dataReceived_Signal* getDataReceivedSignal() { return &data_received;}
    
    UDP_socket(const int id, const char * ip_address, const char * ip_port, const double t0);
    void errorMsg(const char * message);
    void commentMsg(const char * message);
    
    bool connect_socket(void);
    
    void close_rx(void);
    void close_tx(void);
    
    bool setupReceive(int port);
    bool isReceving(void);
    virtual void receiveThread(void);
    void enableReceiveThread(bool en);
    unsigned int getLastReceiveTime(void);
    void sendPacket(char data[]);
    void sendPacketLen(char data[], char len);
    void set_realTime_priority(void);
    unsigned int timeNow(void);
           
    virtual ~UDP_socket();
  
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
    int timeout_count;
    unsigned int lastReceiveTime;
    char rx_data[256];
    
    int socket_id;
    
    double time_t0;
};

#endif	/* UDP_SOCKET_H */

