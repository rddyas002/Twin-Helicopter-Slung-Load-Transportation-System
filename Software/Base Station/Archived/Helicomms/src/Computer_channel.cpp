/* 
 * File:   Computer_channel.cpp
 * Author: yashren
 * 
 * Created on 04 May 2013, 1:36 PM
 */

#include "../include/Computer_channel.h"

extern "C"
{
    // this C function will be used to receive the thread and pass it back to the Thread instance
    void* thread_catch_tcp_rx(void* arg)
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
    
    quit_rx_thread = true;
    read_timeout = 1000;
    
    new_rx_data_flag = false;
    
//    if (!timeoutRead(5000)){
//        char temp_buffer[64];
//        strcpy(temp_buffer,rx_data_buffer);
//        rx_data_buffer[11] = '\0';
//        if (strcmp(rx_data_buffer,"CLIENT ID: ") == 0)
//        {
//            int ident = 0;
//            sscanf(temp_buffer,"CLIENT ID: %d\r\n",&ident);
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
        
    quit_rx_thread = false;
    if (pthread_create(&receive_thread, 0, &thread_catch_tcp_rx, this) != 0)
    {
        errorMessage("Receive thread creation error.");
    }      
    
    // take this out later--just for debugging
    connected = true;
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
        int n = write(comms_fd,data,strlen(data));
        if (n < 0) {
             debugMessage("Writing to socket error. Client may have disconnected.");
             write_retry++;
        }
        if (write_retry == 3){
            // means connection lost
            channelLostSignal(this);
            connected = false;
        }
    } 
    
}

void Computer_channel::receiveThread(void){
    static int read_fail = 0;
    debugMessage("Entering receive thread");
  
    // setup nonblocking read here
    struct pollfd fds;
    fds.fd = comms_fd;
    fds.events = POLLIN;
           
    while((!quit_rx_thread) && connected)
    {

        int poll_ret = poll(&fds,1,read_timeout);
        if(poll_ret > 0)
        {    
            int n = read(comms_fd,rx_data_buffer,256);
            
            rx_data_buffer[n] = '\0';

            if (n != 0)
            {
                gettimeofday(&end_counter,NULL);
                elapsed_time = (end_counter.tv_sec - start_counter.tv_sec)*1000;
                elapsed_time += (end_counter.tv_usec - start_counter.tv_usec)/1000;
            
                dataReceived(&rx_data_buffer[0]);
                new_rx_data_flag = true;
            }
            else{
                if (read_fail++ > 10){
                    connected = false;
                    debugMessage("comms broken");
                }
            }
        }
//        else if (poll_ret == 0)
//        {
//             std::cout << "TCP: RX timeout: " << channel_id << std::endl;
//        }
    }
    debugMessage("Exiting receive thread");
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

bool Computer_channel::send2client(client_msg msg)
{
    int len,n;
    // Check for valid connection
    if (connected)
    {
        switch(msg)
        {
            case START_CAPTURE:
                gettimeofday(&start_counter,NULL);
                sprintf(tx_data_buffer, "START_CAPTURE\r\n");
                len = strlen(tx_data_buffer);
                n = write(comms_fd, tx_data_buffer, len);
                break;
            case TERMINATE:
                sprintf(tx_data_buffer, "TERMINATE\r\n");
                len = strlen(tx_data_buffer);
                n = write(comms_fd, tx_data_buffer, len);
                std::cout << "Terminate commands sent" << std::endl;
                
                break;
            case TEST:
                sprintf(tx_data_buffer, "TEST\r\n");
                len = strlen(tx_data_buffer);
                n = write(comms_fd, tx_data_buffer, len);    
                break;                
            default:
                break;
        }
        if (len == n){
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        errorMessage("Trying to transmit with connect = false");
        return false;
    }
        
}

void Computer_channel::errorMessage(const char * str){
    printf("ERR Computer_channel %d: %s\r\n", channel_id, str);
}

void Computer_channel::debugMessage(const char * str){
    printf("DBG Computer_channel %d: %s\r\n", channel_id, str);
}


Computer_channel::~Computer_channel() {
    //send2client(Computer_channel::TERMINATE);
}

