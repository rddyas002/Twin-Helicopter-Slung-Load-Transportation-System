/* 
 * File:   main.cpp
 * Author: mocap004
 *
 * Created on 18 July 2014, 7:41 AM
 */

#include <cstdlib>
#include "Multicast.h"

#define HELI_MULTI_DATA "225.0.0.37"
#define HELI_MULTI_PORT "12121"

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {
    Multicast * broadcast = new Multicast(HELI_MULTI_DATA, HELI_MULTI_PORT);
    char temp[] = "hello_world\r\n\0";
    
    for (int i = 0; i < 10; i++){
        broadcast->sendPacket(temp);
        usleep(100000);
    }
    
    delete broadcast;
    return 0;
}

