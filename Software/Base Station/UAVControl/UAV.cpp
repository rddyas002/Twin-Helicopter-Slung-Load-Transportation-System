
#include "UAV.h"

UAV::UAV(const char * configuration_file) {
    int i;
    
    // Load configuration file
    FILE * file = fopen(configuration_file, "r");
    
    for (i = 0; i < NUM_OF_CAMERAS; i++){
        tcp_client[i] = NULL;
    }            
    
    for (i = 0; i < MAX_NUM_OF_HELICOPTERS; i++){
        helicopter[i] = NULL;
    }    
    poseEstimation = NULL;    
    
    clearMocamDataReceived();
    
    // Get initial time that will be a reference for both helicopters and the systems t0
    clock_gettime(CLOCK_MONOTONIC_RAW, &uav_t0);
    uav_t0_us = (double)uav_t0.tv_sec*1e6 + (double)uav_t0.tv_nsec/1e3;    
    
    // MOCAP_MODE: captures blob information from the cameras only
    // There is no estimation of pose and translation of some pre-defined object in the flying space
    // Blobs in the scene are simply captured by the cameras at a fixed rate and
    // stored to text.
    if (strcmp(configuration_file, MOCAP_MODE) == 0){
        cout << "Mocap Mode" << endl;
        
        // Make psuedo PoseEstimation object for virtualCamera struct reference
        poseEstimation = new PoseEstimation(0, helicopter);
        
        // Connect to servers and send sync pulses
        connectClients4Mocap();        
        
        // start automatic request operation
        requestMocapData();
        
        return;
    }
 
    if (file == NULL){
        cout << "Error opening configuration file" << endl;
    }
    else{
        // Initialise struct
        memset(&helicopter_info[0], 0, sizeof(Helicopter_info_struct)*MAX_NUM_OF_HELICOPTERS);
        Helicopter_info_struct * h_pntr = &helicopter_info[0];
        active_helicopters = getConfiguration(file, h_pntr);
        fclose(file);
    }
    
    if (active_helicopters < 0){
        cout << "Error reading configuration file" << endl;
        exit(-1);
    }
      
    // Allocate new helicopters
    for (i = 0; i < active_helicopters; i++){
        helicopter_info[i].start_time = uav_t0_us;
        helicopter[i] = new Helicopter(helicopter_info[i]);
    }  
            
    // Pass helicopter reference to PoseEstimation for state correction and initialisation
    poseEstimation = new PoseEstimation(active_helicopters, helicopter);
    
    // Signal to allow auto request when all data has been received and processed
    correction_complete = poseEstimation->getSignal()->connect(boost::bind(&UAV::requestMocapData,this));
    
    // Connect to servers and send sync pulses
    connectClients();
    
    // Request data from servers to start process
    requestMocapData();
    
    // Wait here until initial estimates are done
    while(poseEstimation->getState() == PoseEstimation::PE_INITIALISING);

    switch(poseEstimation->getState()){
        case PoseEstimation::PE_RUNNING:
            for (i = 0; i < active_helicopters; i++){
                helicopter[i]->initialiseNotchFilter();
                helicopter[i]->enableStatePropagation(true);
            } 
            poseEstimation->enableEKFCorrection(true);            
            break;
        case PoseEstimation::PE_LSQ_MODE:
            cout << "LSQ mode" << endl;
            break;
        case PoseEstimation::PE_TIMEOUT:
        default:
            cout << "PoseEstimation timeout. Helicopters not detected in scene or residual too large" << endl;            
            cout << "Disconnecting from Mocap servers" << endl;            
            disconnectClients();
    }
}

unsigned int UAV::timeNow(void){
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC_RAW, &now);
    return (unsigned int) ((double)now.tv_sec*1e6 + (double)now.tv_nsec/1e3 - uav_t0_us);
}

int UAV::getConfiguration(FILE * file, Helicopter_info_struct * helicopter_info){
    int num_of_helicopters = 0;
    
    if (file != NULL){
        char line[1024];
        int i = 0;

        while(fgets(line, sizeof(line), file) != NULL){
            char * cfline;
            cfline = strstr(&line[0], "=");
            cfline += 1;
            
            if (i > UDP_PORT){
                // Another helicopter configuration exists
                helicopter_info++;
                i = ID;                
            }
            
            switch(i++){
                case ID:
                    helicopter_info->id = atoi(strtok(cfline, " \n\t"));
                    break;
                case IP_ADDRESS:
                    sprintf(&helicopter_info->ip_address[0], "%s", strtok(cfline, " \n\t"));
                    break;
                case IP_PORT:
                    sprintf(&helicopter_info->ip_port[0], "%s", strtok(cfline, " \n\t"));
                    break;
                case PC_PORT:
                    sprintf(&helicopter_info->pc_port[0], "%s", strtok(cfline, " \n\t"));
                    break;
                case BROADCAST_PORT:
                    sprintf(&helicopter_info->multicast_port[0], "%s", strtok(cfline, " \n\t"));
                    if (strcmp(&helicopter_info->multicast_port[0], "-1") == 0)
                        helicopter_info->multicast_use = false;
                    else
                        helicopter_info->multicast_use = true;
                    break;
                case INITIAL_POSE:
                    helicopter_info->initial_pose[0] = strtod(strtok(cfline, ","), NULL);
                    helicopter_info->initial_pose[1] = strtod(strtok(NULL, ","), NULL);
                    helicopter_info->initial_pose[2] = strtod(strtok(NULL, ","), NULL);                    
                    helicopter_info->initial_pose[0] /= 1000;
                    helicopter_info->initial_pose[1] /= 1000;
                    helicopter_info->initial_pose[2] /= 1000;
                    helicopter_info->initial_pose[3] = strtod(strtok(NULL, ","), NULL);
                    helicopter_info->initial_pose[4] = strtod(strtok(NULL, ","), NULL);                    
                    helicopter_info->initial_pose[5] = strtod(strtok(NULL, ","), NULL);
                    helicopter_info->initial_pose[6] = strtod(strtok(NULL, " \n\t"), NULL);                                        
                    break;
                case RED_BLOB:
                    helicopter_info->objects_points[0] = strtod(strtok(cfline, ","), NULL);
                    helicopter_info->objects_points[1] = strtod(strtok(NULL, ","), NULL);
                    helicopter_info->objects_points[2] = strtod(strtok(NULL, ","), NULL);                                        
                    break;
                case GREEN_BLOB:
                    helicopter_info->objects_points[3] = strtod(strtok(cfline, ","), NULL);
                    helicopter_info->objects_points[4] = strtod(strtok(NULL, ","), NULL);
                    helicopter_info->objects_points[5] = strtod(strtok(NULL, ","), NULL);                                        
                    break;
                case BLUE_BLOB:
                    helicopter_info->objects_points[6] = strtod(strtok(cfline, ","), NULL);
                    helicopter_info->objects_points[7] = strtod(strtok(NULL, ","), NULL);
                    helicopter_info->objects_points[8] = strtod(strtok(NULL, ","), NULL);                                        
                    break;
                case GYRORAW2DEGREES:
                    helicopter_info->gyro_raw2dps[0] = strtod(strtok(cfline, ","), NULL);
                    helicopter_info->gyro_raw2dps[1] = strtod(strtok(NULL, ","), NULL);
                    helicopter_info->gyro_raw2dps[2] = strtod(strtok(NULL, ","), NULL);                                        
                    break;
                case ACCELRAW2G:
                    helicopter_info->accelraw1g[0] = strtod(strtok(cfline, ","), NULL);
                    helicopter_info->accelraw1g[1] = strtod(strtok(NULL, ","), NULL);
                    helicopter_info->accelraw1g[2] = strtod(strtok(NULL, ","), NULL);
                    break;
                case TYPE:
                    char buffer[20];
                    sprintf(&buffer[0], "%s", strtok(cfline, " \n\t"));
                    if (strcmp(buffer, "QUAD0") == 0){
                        helicopter_info->type = ARNOQUAD;
                    }
                    else{
                        helicopter_info->type = YASHHELI;
                    }                    
                    num_of_helicopters++; // One configuration capture complete                    
                    break;
                case UDP_IP:
                    sprintf(&helicopter_info->ip_address_udp[0], "%s", strtok(cfline, " \n\t"));
                    break;
                case UDP_PORT:
                    sprintf(&helicopter_info->ip_port_udp[0], "%s", strtok(cfline, " \n\t"));
                    break;                    
                default:
                    cout << "CONFIG file error" << endl;
                    return -1;
                    break;
            }
        }
    }
    return num_of_helicopters;
}

void UAV::requestMocapData(void){
    for (int i = 0; i < NUM_OF_CAMERAS; i++){
        if (tcp_client[i] != NULL){
            tcp_client[i]->sendMsg(TCP_client::GET_DATA);
        }else
            std::cout << "Client " << i+1 << "NULL" << std::endl;
    }    
}

void UAV::synchronizeClients(void){
    for (int j = 0; j < SYNC_PULSES; j++){
        for (int i = 0; i < NUM_OF_CAMERAS; i++){
            // get current time
            unsigned int sync_time = timeNow();        
            char buf[64];
            sprintf(&buf[0],"SYNC_TIME\t%u\n",sync_time);
            if (tcp_client[i] != NULL){
                tcp_client[i]->write2server(&buf[0]);
            }else
                std::cout << "Client " << i+1 << "NULL" << std::endl;
            // wait 20 milliseconds
            usleep(20000);
        }     
    }
    cout << "Base station synchronisation pulses complete" << endl;
}

void UAV::clearMocamDataReceived(void){
    for (int i = 0; i < NUM_OF_CAMERAS; i++){
        mc_dataFromCamReceived[i] = false;
    }    
}

void UAV::setMocamDataReceived(int val){     
    int i = 0;
    pthread_mutex_lock(&uav_cam_mutex);
    if (mc_dataFromCamReceived[val - 1] == true)
        std::cout << "scan error" << std::endl;
    
    mc_dataFromCamReceived[val - 1] = true;
    
    for (i = 0; i < NUM_OF_CAMERAS; i++){
        if(!mc_dataFromCamReceived[i]){
            pthread_mutex_unlock(&uav_cam_mutex);
            return;
        }
    }
   
    pthread_mutex_unlock(&uav_cam_mutex);
    clearMocamDataReceived();
    // perform next request
    requestMocapData();    
}

void UAV::connectClients4Mocap(void){
    int i;
    char * server_ip_addresses[NUM_OF_CAMERAS];
    // extract char ip addresses
    for (i = 0; i < NUM_OF_CAMERAS; i++){
        server_ip_addresses[i] = (char*) malloc(20);
    }
    
    sprintf(server_ip_addresses[0], "192.168.1.100");
    sprintf(server_ip_addresses[1], "192.168.1.101");
    sprintf(server_ip_addresses[2], "192.168.1.102");
    sprintf(server_ip_addresses[3], "192.168.1.103");   
    
    for (i = 0; i < NUM_OF_CAMERAS; i++){
        if (tcp_client[i] == NULL){
            tcp_client[i] = new TCP_client(i+1,SERVER_HOST_PORT, server_ip_addresses[i], poseEstimation->getVirtualCamera(i), uav_t0_us);
            if (!(tcp_client[i]->tryConnect())){
                delete tcp_client[i];
                tcp_client[i] = NULL;
            }
            else{
                // Kill signal
                tcp_callback_connection[i] = tcp_client[i]->getCallbackSignal()->connect(boost::bind(&UAV::TCP_Client_callback, this,_1,_2));
                // Data received signal
                data_received_connection[i] = tcp_client[i]->getDataReceivedSignal()->connect(boost::bind(&UAV::setMocamDataReceived, this,_1));
            }
        }
    }
    sleep(3);    
    synchronizeClients();
    sleep(1);    
}

void UAV::connectClients(void){
    int i;
    char * server_ip_addresses[NUM_OF_CAMERAS];
    // extract char ip addresses
    for (i = 0; i < NUM_OF_CAMERAS; i++){
        server_ip_addresses[i] = (char*) malloc(20);
    }
    
    sprintf(server_ip_addresses[0], "192.168.1.100");
    sprintf(server_ip_addresses[1], "192.168.1.101");
    sprintf(server_ip_addresses[2], "192.168.1.102");
    sprintf(server_ip_addresses[3], "192.168.1.103");   
    
    for (i = 0; i < NUM_OF_CAMERAS; i++){
        if (tcp_client[i] == NULL){
            tcp_client[i] = new TCP_client(i+1,SERVER_HOST_PORT, server_ip_addresses[i], poseEstimation->getVirtualCamera(i), uav_t0_us);
            if (!(tcp_client[i]->tryConnect())){
                delete tcp_client[i];
                tcp_client[i] = NULL;
            }
            else{
                tcp_callback_connection[i] = tcp_client[i]->getCallbackSignal()->connect(boost::bind(&UAV::TCP_Client_callback, this,_1,_2));
                data_received_connection[i] = tcp_client[i]->getDataReceivedSignal()->connect(boost::bind(&PoseEstimation::setCamDataReceived, poseEstimation,_1));
            }
        }
    }
    sleep(3);    
    synchronizeClients();
    sleep(1);    
}

void UAV::TCP_Client_callback(TCP_client::callback_msg msg, int id){
    switch (msg){
        case TCP_client::TERMINATE_CLIENT:
            if (tcp_client[id] != NULL){
                delete tcp_client[id];
                tcp_client[id] = NULL;
            }            
            break;
        default:
            std::cout << "Unknown msg received in TCP_Client_callback" << std::endl;
            break;
    }
}

void UAV::disconnectClients(void){
    correction_complete.disconnect();
    sleep(1);
    for (int i = 0; i < NUM_OF_CAMERAS; i++){
        if (tcp_client[i] != NULL){
            tcp_callback_connection[i].disconnect();
            data_received_connection[i].disconnect();
            delete tcp_client[i];
            tcp_client[i] = NULL;
        }
    }    
    sleep(1);
}

UAV::~UAV() {
    disconnectClients();    
   
    for (int i = 0; i < MAX_NUM_OF_HELICOPTERS; i++){
        if (helicopter[i] != NULL){
            delete helicopter[i];
            helicopter[i] = NULL;
        }
    }    
    if (poseEstimation != NULL){
        delete poseEstimation;
        poseEstimation = NULL;
    }      
}

