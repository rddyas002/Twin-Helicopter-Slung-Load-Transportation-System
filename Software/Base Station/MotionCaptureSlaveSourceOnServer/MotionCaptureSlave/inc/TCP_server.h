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

#include "Computer_channel.h"
#include "common.h"

#define NUM_OF_CLIENTS 4
#define MAX_CAMERAS 10

class TCP_server {
public:
    // define connections and callback signal
    typedef boost::signals2::connection signal_connection;
    typedef boost::signals2::signal<void( relay_callback_msg )> m_callbackSignal;	
    
    TCP_server(int hostPort);
	virtual ~TCP_server();
	// on separate thread, waiting to receive client connections
	virtual void waiting4clients(void);
	void stopReceiveThread(void);
	
	// used to get signal from parent class for binding
	m_callbackSignal* getMainCallbackSignal() { return &main_callback_signal; }
	// used to receive events from Computer_channel class
	void ComputerChannelCallback(relay_callback_msg msg, int id);
	// used to disconnect computer channel callback
	void disconnectSignals(int id);
    
	// Helping functions
	// Write str to all clients connected
	void write2clients(char * str);
    bool channelConnected(int channel);
    int getNumClients(void);

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
    
    int num_clients;
    pthread_t wait_4_client_thread;
    
    bool quit_wait_thread;
    
    int fd_client[NUM_OF_CLIENTS];
    Computer_channel * pc_file[NUM_OF_CLIENTS];
    bool pc_connected[NUM_OF_CLIENTS];
    
    signal_connection dataReceivedConnection[NUM_OF_CLIENTS];
    signal_connection callbackConnection[NUM_OF_CLIENTS];
    
    m_callbackSignal main_callback_signal;
};

#endif	/* TCP_SERVER_H */

