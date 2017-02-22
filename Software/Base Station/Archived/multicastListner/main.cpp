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

#define HELI_GROUP "225.0.0.37"
#define MSGBUFSIZE 256

using namespace std;

main(int argc, char *argv[])
{
     struct sockaddr_in addr;
     int fd, nbytes, addrlen;
     struct ip_mreq mreq;
     char msgbuf[MSGBUFSIZE] = {0};
     
     if (argc < 2){
         cout << "Insufficient arguments" << endl;
     }
     else{
         cout << "Multicast listening on: " << argv[1] << " | " << HELI_GROUP << endl;
     }
     
     int listening_port = atoi(argv[1]);

     /* create what looks like an ordinary UDP socket */
     if ((fd=socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
	  perror("socket");
	  exit(1);
     }

     u_int yes = 1;
    /* allow multiple sockets to use the same PORT number */
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0) {
        perror("Reusing ADDR failed");
        exit(1);
    }

     /* set up destination address */
     memset(&addr, 0, sizeof(addr));
     addr.sin_family = AF_INET;
     addr.sin_addr.s_addr = htonl(INADDR_ANY); /* N.B.: differs from sender */
     addr.sin_port = htons(listening_port);
     
     /* bind to receive address */
     if (bind(fd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
	perror("bind");
	exit(1);
     }
     
     /* use setsockopt() to request nbytesthat the kernel join a multicast group */
     mreq.imr_multiaddr.s_addr = inet_addr(HELI_GROUP);
     mreq.imr_interface.s_addr = htonl(INADDR_ANY);
     if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
	perror("setsockopt");
	exit(1);
     }

     /* now just enter a read-print loop */
     while (1) {
	  addrlen = sizeof(addr);
	  if ((nbytes = recvfrom(fd, msgbuf, MSGBUFSIZE, 0,
			       (struct sockaddr *) &addr, (socklen_t*) &addrlen)) < 0) {
	       perror("recvfrom");
	       exit(1);
	  }
          msgbuf[nbytes] = '\0';
	  printf("%s", msgbuf);
          fflush(stdout);
     }
}