/* 
 * File:   UDP_socket.cpp
 * Author: yashren
 * 
 * Created on 11 March 2013, 10:36 AM
 */
#include "UDP_socket.h"
#include <locale.h>
#include <algorithm>
#include <string>
#include <cstdio>

extern "C"
{
    // this C function will be used to receive the thread and pass it back to the Thread instance
    void* thread_catch_udp_rx(void* arg)
    {
        UDP_socket* t = static_cast<UDP_socket*>(arg);
        t->receiveThread();
        return 0;
    }      
}

using namespace std;

UDP_socket::UDP_socket(const int id, const char * ip_address, const char * ip_port, const double t0) {
    setlocale(LC_ALL, "C");
    
    // initialize
    receiving = false;
    read_timeout = 5000;
    timeout_count = 0;
    time_t0 = t0;
    memset(&rx_data[0],0,sizeof(char)*256);
    socket_id = id;
    
    // set helicopter IP address and port for connecting through UDP
    portno = atoi(ip_port);
    strcpy(&heli_ip_address[0], ip_address);
}

bool UDP_socket::connect_socket(void){
    // create UDP socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    
    if (sockfd < 0){
        errorMsg("Creating transmit socket");
        return false;
    }
    
    server = gethostbyname(&heli_ip_address[0]);
    
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(portno);

    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0){
        errorMsg("Connecting to socket");
        return false;
    }
    
    sockfd_connected = true;
    
    return true;
}

bool UDP_socket::setupReceive(int port){
   int length;
   struct sockaddr_in server;

   sockrx = socket(AF_INET, SOCK_DGRAM, 0);
   if (sockrx < 0){
       errorMsg("Creating receive socket");
       return false;
   }    
   
   length = sizeof(server);
   bzero(&server,length);
   server.sin_family=AF_INET;
   server.sin_addr.s_addr=INADDR_ANY;
   server.sin_port=htons(port);   
    
  // make socket non-blocking
    int flags = fcntl(sockrx, F_GETFL);
    flags |= O_NONBLOCK;
    fcntl(sockrx, F_SETFL, flags);

   // Bind socket to port
   if (bind(sockrx,(struct sockaddr *)&server,length)<0){
       errorMsg("Binding receive socket");
       return false;
   }
    
   fromlen1 = sizeof(struct sockaddr_in); 
   
   // Now create rx thread
   enableReceiveThread(true);
   return true;
}

void UDP_socket::sendPacket(char data[]){
    send(sockfd,data,strlen(data),0); 
}

void UDP_socket::sendPacketLen(char data[], char len){
    send(sockfd,data,len,0); 
}

void UDP_socket::set_realTime_priority(void){   
    // get tid
    pthread_t this_thread = pthread_self();
    
    struct sched_param params;
    params.sched_priority = sched_get_priority_max(SCHED_FIFO);
    
    if(pthread_setschedparam(this_thread, SCHED_FIFO, &params) != 0){
        errorMsg("FIFO thread priority NOT set");
    }
    else{
        int policy = 0;
        if(pthread_getschedparam(this_thread, &policy, &params) != 0)
            errorMsg("Failed to get policy priority");
        else
            std::cout << "FIFO thread priority set to " << params.sched_priority << std::endl;
    }
}

void UDP_socket::receiveThread(void){
    commentMsg("Receive thread running");
    
    // Set priority of this thread to be highest
    set_realTime_priority();
    
    // setup nonblocking read here
    struct pollfd fds;
    fds.fd = sockrx;
    fds.events = POLLIN;
       
    while(!quit_rx_thread)
    {
        int poll_ret = poll(&fds,1,read_timeout);
        if(poll_ret > 0)
        {    
                int n = recvfrom(sockrx,rx_data,UDP_socket_PACKETSIZE,MSG_WAITALL,(struct sockaddr *)&from1,&fromlen1);
                
                if (n > 10){
                    lastReceiveTime = timeNow();
                    timeout_count = 0;
                    read_timeout = 10;  // milliseconds
                    data_received(rx_data);
                }
        }
        else if (poll_ret == 0){
            //commentMsg("Receive timeout");
            timeout_count++;
            if (timeout_count == 25){
                commentMsg("Receive timeout 250ms no data");
                read_timeout = 5000;
            }
        }
    }
    
    commentMsg("Receive thread exited");
}

unsigned int UDP_socket::getLastReceiveTime(void){
    return lastReceiveTime;
}

void UDP_socket::enableReceiveThread(bool en){
    if (en){
        quit_rx_thread = false;
                
        if (pthread_create(&receive_thread, 0, &thread_catch_udp_rx, this) != 0){
            receiving = false;
            errorMsg("Creating receive thread");
        }
        
        // Make thread priority high
        struct sched_param sp;
        sp.sched_priority = 95;
        pthread_setschedparam(receive_thread, SCHED_FIFO, &sp); 
        
        receiving = true;
    }
    else{
        quit_rx_thread = true;
        // wait for thread to end
        pthread_join(receive_thread, NULL);
        receiving = false;
    }
}

unsigned int UDP_socket::timeNow(void){
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC_RAW, &now);
    
    return (unsigned int) ((double)now.tv_sec*1e6 + (double)now.tv_nsec/1e3 - time_t0);
}

void UDP_socket::close_rx(void){
    if (sockrx > 0){    
        close(sockrx);
        commentMsg("sockrx closed.");
    }
}

void UDP_socket::close_tx(void){
    if (sockfd > 0)    {
        close(sockfd);
        commentMsg("sockfd closed.");
    }
}

void UDP_socket::errorMsg(const char * message){
    cout << "UDP_socket ERR: " << message << endl; 
}

void UDP_socket::commentMsg(const char * message){
    cout << "UDP_socket COMMENT: " << message << endl; 
}

UDP_socket::~UDP_socket() {
    if (receiving){
        enableReceiveThread(false);
    }
    close_rx();
    close_tx();
}

