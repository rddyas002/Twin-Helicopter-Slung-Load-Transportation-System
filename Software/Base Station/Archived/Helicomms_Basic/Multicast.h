/* 
 * File:   Multicast.h
 * Author: mocap004
 *
 * Created on 18 July 2014, 6:57 AM
 */

#ifndef MULTICAST_H
#define	MULTICAST_H

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

using namespace std;

class Multicast {
public:
    Multicast(const char * address, const char * sport);
    void sendPacket(char data[]);
    void errorMsg(const char * message);
    void commentMsg(const char * message);
    void closefd(void);
    virtual ~Multicast();
private:
    struct sockaddr_in addr;
    int fd, cnt;
    struct ip_mreq mreq;
    int port;
};

#endif	/* MULTICAST_H */

