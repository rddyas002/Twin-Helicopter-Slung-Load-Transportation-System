/* 
 * File:   Computer_channel.cpp
 * Author: yashren
 * 
 * Created on 04 May 2013, 1:36 PM
 */

#include "../inc/Computer_channel.h"

extern "C"
{
    // this C function will be used to receive the thread and pass it back to the Thread instance
    void* thread_catch_tcp_rx2(void* arg)
    {
        Computer_channel* t = static_cast<Computer_channel*>(arg);
        t->receiveThread();
        return 0;
    }      
}

Computer_channel::Computer_channel() {
  
}

void Computer_channel::instantiate_computer(int id, int fd){
    channel_id = id;
    comms_fd = fd;
    write_retry = 0;
    connected = false;
    
    synchronizing = false;
    sync_ping = 0;
    offset_time = 0.0;
    
    quit_rx_thread = true;
    read_timeout = 10000;
    
    new_rx_data_flag = false;
    
//    if (!timeoutRead(5000)){
//        char temp_buffer[64];
//        strcpy(temp_buffer,rx_data_buffer);
//        rx_data_buffer[11] = '\0';
//        if (strcmp(rx_data_buffer,"CLIENT PC: ") == 0)
//        {
//            int ident = 0;
//            sscanf(temp_buffer,"CLIENT PC: %d\r\n",&ident);
//            if ((ident > 0) && (ident < 10))
//            {
//                channel_id = ident;
//                printf("CLIENT %d connected.\r\n",channel_id);
//                connected = true;
//            }
//            else
//                debugMessage("Connected but ID not in range.");
//        }
//        else
//            debugMessage("Opening string from client does not match expected.");
//    }
//    else
//        debugMessage("Timeout/No response from client.");

    connected = true;    
    
    quit_rx_thread = false;
    if (pthread_create(&receive_thread, 0, &thread_catch_tcp_rx2, this) != 0)
    {
        errorMessage("Receive thread creation error.");
    }      
    

}

int Computer_channel::getID(void){
    return channel_id;
}

void Computer_channel::write2client(char * data){
    if (comms_fd < 0){
        errorMessage("No socket created.");
    }
    else
    {
        int str_len = strlen(data);
        // append transmit time
        sprintf(data + str_len, "|%u#\r\n", timeNow());
        
        int n = write(comms_fd,data,strlen(data));
        if (n < 0) {
             debugMessage("Writing to socket error. Client may have disconnected.");
             if(write_retry++ > 3){
                callbackSignal(CLIENT_DISCONNECT_RELAY, channel_id);
                connected = false;                 
             }
        }
    } 
    
}

void Computer_channel::receiveThread(void){
    debugMessage("Entering receive thread");
  
    int counter = 0;
    
    // setup nonblocking read here
    struct pollfd fds;
    fds.fd = comms_fd;
    fds.events = POLLIN;
    
    num_failed_rx = 0;
           
    while((!quit_rx_thread) && connected)
    {

        int poll_ret = poll(&fds,1,read_timeout);
        if(poll_ret > 0)
        {    
            int n = read(comms_fd,rx_data_buffer,256);
            //int n = recvfrom(comms_fd,rx_data_buffer,256,MSG_DONTWAIT,(struct sockaddr *)&from1,&fromlen1);
            
            rx_data_buffer[n] = '\0';

            if (n != 0)
            {
                gettimeofday(&end_counter,NULL);
                elapsed_time = (end_counter.tv_sec - start_counter.tv_sec)*1000;
                elapsed_time += (end_counter.tv_usec - start_counter.tv_usec)/1000;
            
                decodeData(&rx_data_buffer[0]);
                memset(rx_data_buffer,0,256);
                new_rx_data_flag = true;
            }
            else
            {
                if (num_failed_rx++ > 10){
                    connected = false;
                    callbackSignal(CLIENT_DISCONNECT_RELAY, channel_id);
                    debugMessage("comms brokens");
                }
            }
        }
    }
    debugMessage("Exiting receive thread");
}

unsigned int Computer_channel::timeNow(void){
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC_RAW, &now);    
    return (unsigned int) ((double)now.tv_sec*1e6 + (double)now.tv_nsec/1e3 - offset_time);
}

void Computer_channel::decodeData(char * str){  
    
    char * pnt_token;
    pnt_token = strtok(str,"\n\t");
    
    if (strcmp(str, "GET_DATA") == 0){    
        callbackSignal(GET_DATA_RELAY, channel_id);
        return;
    }
    
    if (strcmp(str, "SYNC_TIME") == 0){
        synchronizing = true;
            sync_ping++;
            pnt_token = strtok(NULL,"\n\0");
            unsigned int basestation_time = atoi(pnt_token);
            sum_basetime += (timeNow() - (double)basestation_time);
            if (sync_ping == SYNC_SAMPLES){
                offset_time = (sum_basetime/SYNC_SAMPLES);
                synchronizing = false;
                sum_basetime = 0.0;
                sync_ping = 0;    
                std::cout << "Synchronisation complete. Offset is " << offset_time << " Current time (us): " << timeNow() << std::endl; 
            }
        return;
    }
    
    if (strcmp(str, "START_AUTO") == 0){    
        callbackSignal(START_AUTO_RELAY, channel_id);
        return;
    }    
    
    if (strcmp(str, "STOP_AUTO") == 0){    
        callbackSignal(STOP_AUTO_RELAY, channel_id);
        return;
    }    

    if (strcmp(str, "CALIBRATE") == 0){    
        callbackSignal(CALIBRATE_RELAY, channel_id);
        return;
    }        
    
    if (strcmp(str, "KILL_PROGRAM") == 0){
        quit_rx_thread = true;
        callbackSignal(KILL_PROGRAM_RELAY, channel_id);
        std::cout << "KILL_PROGRAM signal recevied from client.\r\n" << std::endl;
        return;
    }
    
    std::cout << "Unknown command: " << str << std::endl;
    
    char buffer[256];
    sprintf(buffer,"INVALID CMD ?? %s \n\
    Try:\n\
    START_AUTO: auto sample\n\
    STOP_AUTO: stop auto sample\n\
    GET_DATA: get one sample\n\
    KILL_PROGRAM: terminate server\r\n", str);
    write2client(buffer);
}

bool Computer_channel::timeoutRead(int timeout){
    // setup nonblocking read here
    struct pollfd fds;
    fds.fd = comms_fd;
    fds.events = POLLIN;
    
    int poll_ret = poll(&fds,1,timeout);
    if(poll_ret > 0)
    {    
        int n = recvfrom(comms_fd,rx_data_buffer,256,MSG_DONTWAIT,(struct sockaddr *)&from1,&fromlen1);
        if (n < 0){
            return true;
        }
        rx_data_buffer[n] = '\0';
    }
    else
        return true;
    
    return false;
}

double Computer_channel::getChannelDelay(void){
    return elapsed_time;
}

char * Computer_channel::getDataPacket(void){
    return &rx_data_buffer[0];
}

void Computer_channel::stopReceiveThread(void){
    quit_rx_thread = true;
    // wait for thread to end
    pthread_join(receive_thread, NULL);    
}

//bool Computer_channel::send2client(client_msg msg)
//{
//    int len,n;
//    // Check for valid connection
//    if (connected)
//    {
//        switch(msg)
//        {
//            case START_CAPTURE:
//                gettimeofday(&start_counter,NULL);
//                sprintf(tx_data_buffer, "START_CAPTURE\r\n");
//                len = strlen(tx_data_buffer);
//                n = write(comms_fd, tx_data_buffer, len);
//                break;
//            case TERMINATE:
//                sprintf(tx_data_buffer, "TERMINATE\r\n");
//                len = strlen(tx_data_buffer);
//                n = write(comms_fd, tx_data_buffer, len);
//                std::cout << "Terminate commands sent" << std::endl;
//                
//                break;
//            case TEST:
//                sprintf(tx_data_buffer, "TEST\r\n");
//                len = strlen(tx_data_buffer);
//                n = write(comms_fd, tx_data_buffer, len);    
//                break;                
//            default:
//                break;
//        }
//        if (len == n){
//            return true;
//        }
//        else
//        {
//            return false;
//        }
//    }
//    else
//    {
//        errorMessage("Trying to transmit with connect = false");
//        return false;
//    }
//        
//}

void Computer_channel::errorMessage(const char * str){
    printf("ERR Computer_channel %d: %s\r\n", channel_id, str);
}

void Computer_channel::debugMessage(const char * str){
    printf("DBG Computer_channel %d: %s\r\n", channel_id, str);
}


Computer_channel::Computer_channel(const Computer_channel& orig) {
}

Computer_channel::~Computer_channel() {
    stopReceiveThread();
}

