/* 
 * File:   TCP_client.cpp
 * Author: yashren
 * 
 * Created on 05 May 2013, 1:08 PM
 */

#include "TCP_client.h"

extern "C"
{
    // this C function will be used to receive the thread and pass it back to the Thread instance
    void* thread_catch_tcp_rx2(void* arg)
    {
        TCP_client* t = static_cast<TCP_client*>(arg);
        t->receiveThread();
        return 0;
    }      
}

TCP_client::TCP_client(int id, int port, const char * ipaddress, OpencvCamera * vCam) {
        
    portno = port;
    sockfd = -1;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    client_id = id;
    new_rx_data_flag = false;
    quit_rx_thread = true;
    server_ip = ipaddress;
    virtualCamera = vCam;
    
    if (sockfd < 0) {
        errorMessage("Opening socket");
        exit(0);
    }
    
    initialiseLog();
    
    // add camera data
    sprintf(log_buffer,"Intrinsic parameters (fx,fy,cx,cy) [pixel/mm]: %f,%f,%f,%f\r\n",
            vCam->getfx(),vCam->getfy(),vCam->getcx(),vCam->getcy());
    writeLog();
    sprintf(log_buffer,"Rotation parameters (th_z,th_x,th_z2) [rad/s]: %f,%f,%f\r\n",
            vCam->get_theta_z(),vCam->get_theta_x(),vCam->get_theta_z2());
    writeLog();    
    sprintf(log_buffer,"Translation parameters (Tcw_x,Tcw_y,Tcw_z) [mm]: %f,%f,%f\r\n",
            vCam->get_Tcw_x(),vCam->get_Tcw_y(),vCam->get_Tcw_z());
    writeLog();        
}

void TCP_client::initialiseLog(void){
    struct tm * sTm;
    char file_buffer[128];
    char time_buffer[128];
    time_t now = time(0);
    sTm = localtime(&now);
    strftime(time_buffer,sizeof(time_buffer),"%Y-%m-%d_%H-%M", sTm);
    
    sprintf(file_buffer,"log/image_data_client%d_%s.txt", client_id, time_buffer);
    openLog(&file_buffer[0]);
}

void TCP_client::openLog(char * file_name){
    image_data_log.open(file_name);
}

void TCP_client::closeLog(void){
    if (image_data_log.is_open())
        image_data_log.close();
}

bool TCP_client::writeLog(void){
    if (image_data_log.is_open()){
        image_data_log << log_buffer;
        return true;
    }
    else
        return false;
}

bool TCP_client::tryConnect(void){
    
    server = gethostbyname(server_ip);
    
    if (server == NULL) {
        errorMessage("Invalid host");
        return false;
    }    
    
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);

    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) {
        errorMessage("Connecting socket"); 
        return false;
    }    
        
    // create a read thread
    quit_rx_thread = false;
    if (pthread_create(&receive_thread, 0, &thread_catch_tcp_rx2, this) != 0){
        errorMessage("Thread creation");
    }        
    return true;
}

int TCP_client::getClientID(void){
    return client_id;
}

void TCP_client::send2server(){
    if (sockfd < 0){
        errorMessage("No socket exists");
    }
    else
    {
        int n = write(sockfd,tx_buffer,strlen(tx_buffer));
        if (n < 0) 
             errorMessage("Writing to socket");  
    }
}

void TCP_client::sendMsg(client_msg msg){
    
    switch (msg){
        case GET_DATA:
            write2server("GET_DATA\n");
            break;
        case KILL_PROGRAM:
            write2server("KILL_PROGRAM\n");
            break;
        case START_AUTO:
            write2server("START_AUTO\n");            
            break;
        case STOP_AUTO:
            write2server("STOP_AUTO\n");            
            break;            
        default:
            debugMessage("Unknown sendMsg argument");
            break;
    }
}

void TCP_client::sendBlobData(char * str){
    write2server(str);
}

void TCP_client::write2server(const char * data){
    if (sockfd < 0){
        errorMessage("No socket exists");
    }
    else{
        int n = write(sockfd,data,strlen(data));
        if (n < 0) 
             errorMessage("Writing to socket");
    }
}

void TCP_client::sendBuffer(void){
    if (sockfd < 0){
        errorMessage("No socket exists");
    }
    else{
        int n = write(sockfd,tx_buffer,strlen(tx_buffer));
        if (n < 0) 
             errorMessage("Writing to socket");
    }
}

void TCP_client::receiveThread(void){
    int read_zero = 0;
    
    // Make this client id run on a separate core
    cpu_set_t my_set;
    CPU_ZERO(&my_set);
    CPU_SET(client_id - 1, &my_set);
//    CPU_SET(2, &my_set);
    if(pthread_setaffinity_np(receive_thread, sizeof(cpu_set_t), &my_set))
        debugMessage("Affinity setting unsuccessful");
    else
        debugMessage("Affinity setting successful");
    CPU_ZERO(&my_set);
    pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &my_set);
    
    for (int i = 0; i < 4; i++){
        if (CPU_ISSET(i, &my_set))
            printf("Core [%d] is used for tid [%u]\r\n", i, pthread_self());
    }
    
    // setup nonblocking read here
    struct pollfd fds;
    fds.fd = sockfd;
    fds.events = POLLIN;
    
    int read_timeout = 1000;
           
    while(!quit_rx_thread)
    {
        int poll_ret = poll(&fds,1,read_timeout);
        if(poll_ret > 0)
        {    
            int n = recv(sockfd,rx_buffer,256,MSG_DONTWAIT);
            rx_buffer[n] = '\0';
            
            if (n == 0){
                if (read_zero++ > 10){
                    // expect server disconnected
                    quit_rx_thread = true;
                }
            }         
            else{           
                loadData(&rx_buffer[0]);
                new_rx_data_flag = true;
            }          
            
            
        }
//        else if (poll_ret == 0)
//        {
//             std::cout << "TCP: RX timeout: " << channel_id << std::endl;
//        }
    }
    callback_signal(TERMINATE_CLIENT,client_id);
}


void TCP_client::loadData(char * data){
    char data_cpy[256]={0};
    double blobs[40]={0};       
    
    memcpy(data_cpy, &data[0], strlen(data));
    
    sprintf(log_buffer,"%s",data);
    writeLog();         
    
//    std::cout << data << std::endl;
    
    // find start character *
    int i = 0, start_char = 0, end_char = 0;
    while(i < 256){
        if (data_cpy[i] == '*')
        {
            start_char = i;
            break;
        }
        i++;
    }
    
    int rem = 255 - i;
    while(i < rem){
        if (data_cpy[i] == '#')
        {
            end_char = i;
            break;
        }
        i++;
    }
    
    int string_len = end_char - start_char;
    if (string_len < 3){
        errorMessage("Received packet doesn't meet protocol");
        return;
    }
    
    //memcpy(data_cpy, &data[start_char + 1], (end_char - start_char + 2));

    int num_red = 0, num_green = 0, num_blue = 0;
    int client_num = 0;
    char temp_buf[64];
    
    setlocale(LC_ALL,"C");
    
    // First extract num of blobs
    int scn_ret = sscanf(data_cpy, "*C%d:%d,%d,%d", &client_num, &num_red, &num_green, &num_blue);  
    if (scn_ret != 4)
        std::cout << "scan err" << std::endl;  
    
    int temp_len = sprintf(temp_buf, "*C%d:%d,%d,%d", client_num, num_red, num_green, num_blue);
    
    char * temp_char = (char *) malloc(sizeof(char) * 20);
    char * saveptr = (char *) malloc(sizeof(char) * 20);
    temp_char = strtok_r(&data_cpy[temp_len], "|", &saveptr);
    blobs[0] = atof(temp_char);
    i = 1;
    
    while((temp_char = strtok_r(NULL, "|", &saveptr))){
        blobs[i++] = atof(temp_char);
    }
            
    // now update blob structs
    virtualCamera->blob_data.red_blobs = num_red;
    virtualCamera->blob_data.green_blobs = num_green;
    virtualCamera->blob_data.blue_blobs = num_blue;

    int j = 0;
    for (i = 0; i < num_red; i++){
        virtualCamera->blob_data.red[i].x = blobs[j++];
        virtualCamera->blob_data.red[i].y = blobs[j++];
    }
    
    for (i = 0; i < num_green; i++){
        virtualCamera->blob_data.green[i].x = blobs[j++];
        virtualCamera->blob_data.green[i].y = blobs[j++];
    }    
    
    for (i = 0; i < num_blue; i++){
        virtualCamera->blob_data.blue[i].x = blobs[j++];
        virtualCamera->blob_data.blue[i].y = blobs[j++];
    }        
    
    if (num_red || num_green || num_blue)
    {
        virtualCamera->blob_data.timestamp = blobs[j];
    }
    else
    {
        virtualCamera->blob_data.timestamp = blobs[0];
    }
    
    if (virtualCamera->blob_data.timestamp == 0)
    {
        virtualCamera->blob_data.red_blobs = 0;
        virtualCamera->blob_data.green_blobs = 0;
        virtualCamera->blob_data.blue_blobs = 0;
        std::cout << "Trouble.." << data << std::endl;
    }
    
    virtualCamera->blob_data.updated = true;
        
    free(temp_char);    
    
    data_received(client_id);
}

bool TCP_client::newDataAvailable(void){
    return new_rx_data_flag;
}

void TCP_client::stopReceiveThread(void){
    if (!quit_rx_thread){
        quit_rx_thread = true;
        // wait for thread to end
        pthread_join(receive_thread, NULL);            
    }
}

void TCP_client::closeTCPClient(void){
    stopReceiveThread();
    if (!(sockfd < 0)){
        close(sockfd);
        debugMessage("Closing file descriptor");
    }    
    closeLog();
}

void TCP_client::debugMessage(const char * str){
    printf("DBG [TCP_client %d] >> %s\r\n",client_id, str);
}

void TCP_client::errorMessage(const char * str){
    printf("ERR [TCP_client %d] >> %s\r\n",client_id, str);
}

TCP_client::~TCP_client() {
    closeTCPClient();
}

