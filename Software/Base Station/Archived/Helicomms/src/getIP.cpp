/* 
 * File:   getIP.cpp
 * Author: yashren
 * 
 * Created on 12 March 2013, 7:35 AM
 */

#include "include/getIP.h"

getIP::getIP() {
    
    const char* google_dns_server = "8.8.8.8";
    int dns_port = 53;

    struct sockaddr_in serv;

    int sock = socket ( AF_INET, SOCK_DGRAM, 0);
    
    strcpy(localIP, "NULL");

    //Socket could not be created
    if(sock < 0)
    {
	perror("GET_MY_IP: Socket error");
        return;
    }

    memset( &serv, 0, sizeof(serv) );
    serv.sin_family = AF_INET;
    serv.sin_addr.s_addr = inet_addr( google_dns_server );
    serv.sin_port = htons( dns_port );

    connect( sock , (const struct sockaddr*) &serv , sizeof(serv) );

    struct sockaddr_in name;
    socklen_t namelen = sizeof(name);
    getsockname(sock, (struct sockaddr*) &name, &namelen);

    char buffer[100];
    const char* p = inet_ntop(AF_INET, &name.sin_addr, buffer, 100);

    if(p != NULL)
    {
        strcpy(localIP, p);
    }

    close(sock);  
}

char * getIP::ip(void){
    return &localIP[0];
}

getIP::~getIP() {
}

