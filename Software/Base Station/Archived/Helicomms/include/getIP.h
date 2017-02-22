/* 
 * File:   getIP.h
 * Author: yashren
 *
 * Created on 12 March 2013, 7:35 AM
 */

#ifndef GETIP_H
#define	GETIP_H

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

class getIP {
public:
    getIP();
    char * ip(void);
    virtual ~getIP();
private:
        char localIP[20];
};

#endif	/* GETIP_H */

