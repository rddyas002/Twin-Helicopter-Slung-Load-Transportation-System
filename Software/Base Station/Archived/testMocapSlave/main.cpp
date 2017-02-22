/* 
 * File:   main.cpp
 * Author: yashren
 *
 * Created on 07 June 2014, 9:09 AM
 */

#include <cstdlib>
#include "TCP_client.h"
#include "OpencvCamera.h"

using namespace std;

double deltaTime(void){
    static struct timespec t0;
    static bool first_enter = false;
    struct timespec now;
    
    if (!first_enter){
        first_enter = true;
        clock_gettime(CLOCK_MONOTONIC_RAW, &t0);
        return 0.0;
    }
    else{
        clock_gettime(CLOCK_MONOTONIC_RAW, &now);
        double time_elaps = (now.tv_sec - t0.tv_sec)*1e6 + (now.tv_nsec - t0.tv_nsec)/1e3;
        memcpy(&t0, &now, sizeof(timespec));
        return time_elaps;
    }
}

void test_latency(TCP_client *tcp, OpencvCamera *cv){
    cv = new OpencvCamera(-1, 1);
    tcp = new TCP_client(1,2010,"192.168.1.100", cv);  
    tcp->tryConnect();
    sleep(2);
    
    for(int i = 0; i < 1000; i++){
        tcp->clrDataAvailable();
        deltaTime();
        tcp->sendMsg(TCP_client::GET_DATA);
        while(!tcp->newDataAvailable());
        printf("%.0fus\r\n",deltaTime());
    }
    
    tcp->sendMsg(TCP_client::KILL_PROGRAM);
    
    delete tcp; tcp = NULL;
    delete cv; cv = NULL;      
}

void test_slave(TCP_client *tcp, OpencvCamera *cv){
    cv = new OpencvCamera(-1, 1);
    tcp = new TCP_client(1,2010,"192.168.1.102", cv);
    tcp->tryConnect();
    usleep(1000000);
    tcp->sendMsg(TCP_client::GET_DATA);
    usleep(100000);
    tcp->sendMsg(TCP_client::GET_DATA);
    usleep(100000);
    tcp->sendMsg(TCP_client::GET_DATA);
    usleep(1000000);
    delete tcp;
    tcp = NULL;
    
    delete cv;
    cv = NULL;    
}

int main(int argc, char** argv) {
    
    TCP_client * tcp_client;
    OpencvCamera * virtual_camera;
    test_latency(tcp_client, virtual_camera);
//    for (int i = 0; i < 10; i++){
//        test_slave(tcp_client, virtual_camera);
//    }

    return 0;
}

