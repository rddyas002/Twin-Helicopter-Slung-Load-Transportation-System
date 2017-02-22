/* 
 * File:   Multicast.cpp
 * Author: mocap004
 * 
 * Created on 18 July 2014, 6:57 AM
 */

#include "Multicast.h"

Multicast::Multicast(const char * address, const char * sport) {
    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0){
        errorMsg("Cannot open socket");
        exit(1);
    }
    
    port = atoi(sport);
    
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(address);
    addr.sin_port = htons(port);
}

void Multicast::sendPacket(char data[]){
    if(sendto(fd, data, strlen(data), 0, (struct sockaddr *) &addr, sizeof(addr)) < 0){
        errorMsg("Sending");
    }
}

void Multicast::errorMsg(const char * message){
    cout << "Multicast ERR: " << message << endl; 
}

void Multicast::commentMsg(const char * message){
    cout << "Multicast COMMENT: " << message << endl; 
}

void Multicast::closefd(void){
    if (fd > 0)
        close(fd);
}

Multicast::~Multicast() {
    closefd();
}

