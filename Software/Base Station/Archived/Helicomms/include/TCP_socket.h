/* 
 * File:   TCP_socket.h
 * Author: yashren
 *
 * Created on 11 March 2013, 10:24 AM
 */

#ifndef TCP_SOCKET_H
#define	TCP_SOCKET_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/time.h>

class TCP_socket {
public:
    TCP_socket();
    TCP_socket(const TCP_socket& orig);
    virtual ~TCP_socket();
private:

};

#endif	/* TCP_SOCKET_H */

