/* 
 * File:   Helicopter.cpp
 * Author: Yashren Reddi
 * 
 * Created on 19 May 2014, 11:35 AM
 */

#include "Helicopter.h"

extern "C"
{
    pthread_mutex_t run_ekf_mutex;
    pthread_mutex_t dt_mutex;
    
    // this C function will be used to receive the thread and pass it back to the Thread instance
    void* thread_catch_heli_tx(void* arg)
    {
        Helicopter* t = static_cast<Helicopter*>(arg);
        t->transmitThread();
        return 0;
    }        
}

using namespace std;

Helicopter::Helicopter(Helicopter_info_struct h_info){
    helicopter_info = h_info;
    helicopter_id = helicopter_info.id;
    
    multicast = NULL;
    
    // copy blob positions to internal variables
    memcpy(&object_points[0], &helicopter_info.objects_points[0], sizeof(double)*9);
    memcpy(&red3D[0], &helicopter_info.objects_points[0], sizeof(double)*3);
    memcpy(&green3D[0], &helicopter_info.objects_points[3], sizeof(double)*3);
    memcpy(&blue3D[0], &helicopter_info.objects_points[6], sizeof(double)*3);
    
    // copy initial pose
    memcpy(&initial_pose[0], &helicopter_info.initial_pose[0], sizeof(double)*7);
    memcpy(&initial_coordinates[0], &helicopter_info.initial_pose[0], sizeof(double)*3);
    memcpy(&initial_orientation[0], &helicopter_info.initial_pose[3], sizeof(double)*4);
    
    memcpy(&gyro_raw2dps[0], &helicopter_info.gyro_raw2dps[0], sizeof(real_T)*3);
    memcpy(&accel1g[0], &helicopter_info.accelraw1g[0], sizeof(real_T)*3);
        
    initializeVariables();
    initialiseEKFvariables();
    initialiseLog();
    
    if (pthread_mutex_init(&run_ekf_mutex, NULL) != 0){
        std::cout << "H" << helicopter_id << ": Mutex initialisation failed" << std::endl;
    }
    
    if (pthread_mutex_init(&dt_mutex, NULL) != 0){
        std::cout << "H" << helicopter_id << ": Mutex initialisation failed" << std::endl;
    }    
    
    if (helicopter_info.multicast_use)
        multicast = new Multicast(MULTICAST_IP, &helicopter_info.multicast_port[0]);
    
    // Start time for synchronisation between helicopters and base station
    helicopter_t0_us = helicopter_info.start_time;
    
    // keep a copy of network details
    sprintf(&ip_address[0], "%s", &helicopter_info.ip_address[0]);
    sprintf(&ip_port[0], "%s", &helicopter_info.ip_port[0]);
    sprintf(&pc_port[0], "%s", &helicopter_info.pc_port[0]);
    sprintf(&broadcast_port[0], "%s", &helicopter_info.multicast_port[0]);
    
    // Create UDP connection to helicopter
    makeConnection(ip_address, ip_port, pc_port);
}

void Helicopter::initialiseLog(void){
    struct tm * sTm;
    char file_buffer[64];
    char time_buffer[64];
    time_t now = time(0);
    sTm = localtime(&now);
    strftime(time_buffer,sizeof(time_buffer),"%Y-%m-%d_%H-%M", sTm);
    
    // File for logging data received from helicopter
    sprintf(file_buffer,"log/log_Heli%d_%s.txt", helicopter_id, time_buffer);
    openLog(&file_buffer[0]);
    
    // File for logging estimation results
    sprintf(file_buffer,"log/ekf_Heli%d_%s.txt", helicopter_id, time_buffer);
    openEKFLog(&file_buffer[0]);    
}

bool Helicopter::makeConnection(const char * ip_address, const char * ip_port, const char * rx_port){
    udp_socket = new UDP_socket(helicopter_id, ip_address, ip_port, helicopter_t0_us);

    if (!udp_socket->connect_socket()){
        delete udp_socket;
        udp_socket = NULL;
        return false;
    }
    // Link data receive signal to decode function
    data_rx_connection = udp_socket->getDataReceivedSignal()->connect(boost::bind(&Helicopter::decodeData,this,_1));
    int rx_p = atoi(rx_port);
    return udp_socket->setupReceive(rx_p);
}

void Helicopter::initializeVariables(void){
    gyroscope[0] = gyroscope[1] = gyroscope[2] = 0;
    gyro_temperature = 0;
    accelerometer[0] = accelerometer[1] = accelerometer[2] = 0;
    magnetometer[0] = magnetometer[1] = magnetometer[2] = 0;            
    temperature = 0;
    temperature_f = 0;
    ultrasonic_distance = voltage = key = 0;
    pressure = 0;
    checksum = 0;
    head_speed = 0;
    configuration = 0x0000;
    
    onboard_roll = 0;
    onboard_pitch = 0;
    onboard_yaw = 0;
    tx_x_position = 0;
    tx_y_position = 0;
    tx_z_position = 0;
    auto_mode = false;
    
    helicopter_roll = helicopter_pitch = helicopter_yaw = 0;
    helicopter_x = helicopter_y = helicopter_z = 0;
    quaternion_cam[0] = quaternion_cam[2] = quaternion_cam[3] = 0;
    quaternion_cam[1] = 1;
    
    key = 0;
    process_time = 0.0;
    
    state_propagation_on = false;
    ekf_initialising = false;
    
    // 'static variables'
    fps_first_enter = false;
    calc_first_enter = false;
    delta_first_enter = false;
    
    stats_count = 0;
    fps_sum = 0.0;
    proc_time_sum = 0.0;    
    
    sample = 0;
    memset(&accel_body_sum[0], 0, sizeof(double)*3);
    memset(&ang_rate_body_sum[0], 0, sizeof(double)*3);
    
    initialiseOffsetVariables();        
    
    initialising_pose = false;
    
    memset(&helicopter_data, 0, sizeof(Helicopter_data_struct));
    helicopter_config = 0;
    
    memset(&PWM_packet[0], 0, sizeof(unsigned char)*6);
    
    accelxFilterIn[0] = accelxFilterIn[1] = accelxFilterIn[2] = 0;
    accelxFilterOut[0] = accelxFilterOut[1] = accelxFilterOut[2] = 0;
    accelyFilterIn[0] = accelyFilterIn[1] = accelyFilterIn[2] = 0;
    accelyFilterOut[0] = accelyFilterOut[1] = accelyFilterOut[2] = 0;
    accelzFilterIn[0] = accelzFilterIn[1] = accelzFilterIn[2] = 0;
    accelzFilterOut[0] = accelzFilterOut[1] = accelzFilterOut[2] = 0;
    
    memset(&trans_x_filter_in[0], 0, sizeof(float)*3);
    memset(&trans_x_filter_out[0], 0, sizeof(float)*3);
    memset(&trans_y_filter_in[0], 0, sizeof(float)*3);
    memset(&trans_y_filter_out[0], 0, sizeof(float)*3);
    memset(&trans_z_filter_in[0], 0, sizeof(float)*3);
    memset(&trans_z_filter_out[0], 0, sizeof(float)*3);

    memset(&vel_x_filter_in[0], 0, sizeof(float)*3);
    memset(&vel_x_filter_out[0], 0, sizeof(float)*3);
    memset(&vel_y_filter_in[0], 0, sizeof(float)*3);
    memset(&vel_y_filter_out[0], 0, sizeof(float)*3);
    memset(&vel_z_filter_in[0], 0, sizeof(float)*3);
    memset(&vel_z_filter_out[0], 0, sizeof(float)*3);
    
    // Mocap variable intiatialisation
    mocap_position_x = 0.0;
    mocap_position_y = 0.0;
    mocap_position_z = 0.0;  
    mocap_velocity_x = 0.0;
    mocap_velocity_y = 0.0;
    mocap_velocity_z = 0.0;
    mocap_q0 = 0.0;
    mocap_q1 = 1.0;
    mocap_q2 = 0.0;    
    mocap_q3 = 0.0;        
    mocap_roll = 0.0;
    mocap_pitch = 0.0;
    mocap_yaw = 0.0;  
    mocap_wx = 0.0;
    mocap_wy = 0.0;
    mocap_wz = 0.0;    
    
    quit_transmit_thread = false;
    transmit_thread_active = false;
}

void Helicopter::initialiseOffsetVariables(void){
    memset(&q_offset[0], 0, sizeof(double)*4);
    memset(&translation_offset[0], 0, sizeof(double)*3);
    init_counter_sum = 0;            
}

uavType Helicopter::getType(void){
    return helicopter_info.type;
}

double Helicopter::FPSCalc(void){
    pthread_mutex_lock(&dt_mutex);
    if (!fps_first_enter){
        fps_first_enter = true;
        clock_gettime(CLOCK_MONOTONIC_RAW, &fps_last_t);
        pthread_mutex_unlock(&dt_mutex);
        return 0.0;
    }
    else{
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC_RAW, &now);
        double time_elaps = (now.tv_sec - fps_last_t.tv_sec) + (now.tv_nsec - fps_last_t.tv_nsec)/1e9;
        memcpy(&fps_last_t, &now, sizeof(timespec));
        pthread_mutex_unlock(&dt_mutex);
        if (time_elaps != 0)
            return 1/time_elaps;
        else
            return 0;
    }
}

double Helicopter::dtCalc(void){
    pthread_mutex_lock(&dt_mutex);
    if (!calc_first_enter){
        calc_first_enter = true;
        clock_gettime(CLOCK_MONOTONIC_RAW, &dt_last);
        pthread_mutex_unlock(&dt_mutex);
        return -1;
    }
    else{
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC_RAW, &now);
        double time_elaps = (now.tv_sec - dt_last.tv_sec) + (now.tv_nsec - dt_last.tv_nsec)/1e9;
        memcpy(&dt_last, &now, sizeof(timespec));
        pthread_mutex_unlock(&dt_mutex);
        return time_elaps;
    }
}

double Helicopter::deltaTime(void){    
    pthread_mutex_lock(&dt_mutex);
    if (!delta_first_enter){
        delta_first_enter = true;
        clock_gettime(CLOCK_MONOTONIC_RAW, &delta_last_t);
        pthread_mutex_unlock(&dt_mutex);
        return 0.0;
    }
    else{
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC_RAW, &now);
        double time_elaps = (now.tv_sec - delta_last_t.tv_sec)*1e6 + (now.tv_nsec - delta_last_t.tv_nsec)/1e3;
        memcpy(&delta_last_t, &now, sizeof(timespec));
        pthread_mutex_unlock(&dt_mutex);
        return time_elaps;
    }
}

Helicopter_info_struct * Helicopter::getHelicopterInfo(void){
    return &helicopter_info;
}

unsigned int Helicopter::timeNow(void){
    struct timespec now;
    pthread_mutex_lock(&dt_mutex);
    clock_gettime(CLOCK_MONOTONIC_RAW, &now);
    pthread_mutex_unlock(&dt_mutex);
    
    return (unsigned int) ((double)now.tv_sec*1e6 + (double)now.tv_nsec/1e3 - helicopter_t0_us);
}

void Helicopter::calibratedAcceleration(const short int acceleration_in[3], real_T acceleration_out[3]){
    acceleration_out[0] = ACCMAPX11*(float)acceleration_in[0] + ACCMAPX12*(float)acceleration_in[1] + ACCMAPX13*(float)acceleration_in[2] + ACCMAPT1;
    acceleration_out[1] = ACCMAPX21*(float)acceleration_in[0] + ACCMAPX22*(float)acceleration_in[1] + ACCMAPX23*(float)acceleration_in[2] + ACCMAPT2;
    acceleration_out[2] = ACCMAPX31*(float)acceleration_in[0] + ACCMAPX32*(float)acceleration_in[1] + ACCMAPX33*(float)acceleration_in[2] + ACCMAPT3;
}

void Helicopter::decodeData(char rx_data[]){    
    deltaTime();
    
    char checksum = 0;
    for (int i = 2; i < 48; i++){
        checksum ^= rx_data[i];
    }    
    
    unsigned int timestamp_uint = timeNow();
    
    // check if complete packet has been received
    if ((rx_data[0] == '*') && (rx_data[1] == '#') && (rx_data[49] == '@') && (rx_data[50] == '!')
        && (checksum == rx_data[48])){     
        unsigned char i = 0, index = 2, LSB = 0;
        unsigned short int MSB = 0;        
        
        for (i = 0; i < 3; i++){
            MSB = ((unsigned short int)rx_data[index++] << 8);
            LSB = rx_data[index++];
            gyroscope[i] = (signed short int) (MSB | LSB);
        }

        for (i = 0; i < 3; i++){
            MSB = ((unsigned short int)rx_data[index++] << 8);
            LSB = rx_data[index++];
            accelerometer[i] = (signed short int) (MSB | LSB);
        }
        calibratedAcceleration(&accelerometer[0], &body_acceleration[0]);

        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        ultrasonic_distance = (unsigned short int) (MSB | LSB);

        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        voltage = (unsigned short int) (MSB | LSB);
        
        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        onboard_roll = (signed short int) (MSB | LSB);
        
        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        onboard_pitch = (signed short int) (MSB | LSB);        
            
        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        onboard_yaw = (signed short int) (MSB | LSB);        
        
        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        tx_x_position = (signed short int) (MSB | LSB);        
        
        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        tx_y_position = (signed short int) (MSB | LSB);
        
        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        tx_z_position = (unsigned short int) (MSB | LSB); 
        
        PWM_packet[0] = rx_data[index++];
        PWM_packet[1] = rx_data[index++];
        PWM_packet[2] = rx_data[index++];
        PWM_packet[3] = rx_data[index++];
        PWM_packet[4] = rx_data[index++];
        PWM_packet[5] = rx_data[index++];
                
        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        head_speed = (unsigned short int) (MSB | LSB);
        
        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        configuration = (unsigned short int) (MSB | LSB);        
        
        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        temperature = (signed short int) (MSB | LSB);
        temperature_f = 35 + ((float)temperature + 13200)/280;
        
        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        key = (unsigned short int) (MSB | LSB);
        
        unsigned char timestamp_uchar[4] = {0};
        timestamp_uchar[0] = rx_data[index++];
        timestamp_uchar[1] = rx_data[index++];
        timestamp_uchar[2] = rx_data[index++];
        timestamp_uchar[3] = rx_data[index++];
        
        measurement_timestamp = (unsigned int)((unsigned int)timestamp_uchar[2] << 24) 
                | ((unsigned int)timestamp_uchar[3] << 16) 
                | ((unsigned int)timestamp_uchar[0] << 8) 
                | (timestamp_uchar[1]);
                
        // log helicopter data
        sprintf(log_buffer,"%d,%d,%d,%d,%d,%d,%u,%.2f,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%d,%u,%u,%u,%.6G,%.6G,%.6G,%.6G\r\n",
                gyroscope[0],gyroscope[1],gyroscope[2],
                accelerometer[0],accelerometer[1],accelerometer[2],
                ultrasonic_distance,
                getVoltage(),
                onboard_roll, onboard_pitch, onboard_yaw,
                tx_x_position, tx_y_position, tx_z_position,
                PWM_packet[0], PWM_packet[1], PWM_packet[2],
                PWM_packet[3], PWM_packet[4], PWM_packet[5],
                head_speed, configuration,   
                temperature,
                key,
                measurement_timestamp,
                timestamp_uint,
                *mocapq0(),*mocapq1(),*mocapq2(),*mocapq3());
        writeLog();    
        
        // update helicopter data struct
        helicopter_data.id = helicopter_id;
        helicopter_data.gyro_x = onboard_roll;//gyroscope[0];
        helicopter_data.gyro_y = onboard_pitch;//gyroscope[1];
        helicopter_data.gyro_z = onboard_yaw;//gyroscope[2];
        helicopter_data.accel_x = accelerometer[0];
        helicopter_data.accel_y = accelerometer[1];
        helicopter_data.accel_z = accelerometer[2];
        helicopter_data.ultrasonic = ultrasonic_distance;
        helicopter_data.voltage = voltage;
        helicopter_data.head_speed = head_speed;
        helicopter_data.temperature = temperature;
        helicopter_data.timestamp = timestamp_uint;        
                                      
        if (state_propagation_on){           
            runEKF();
        }
        
        // lpf position and velocity
        lpf_filter(x_ekf[0], 0.01, LPF_KINEMATICS_WN, &trans_x_filter_in[0], &trans_x_filter_out[0]);
        lpf_filter(x_ekf[1], 0.01, LPF_KINEMATICS_WN, &trans_y_filter_in[0], &trans_y_filter_out[0]);
        lpf_filter(x_ekf[2], 0.01, LPF_KINEMATICS_WN, &trans_z_filter_in[0], &trans_z_filter_out[0]);
        lpf_filter(x_ekf[3], 0.01, LPF_KINEMATICS_WN, &vel_x_filter_in[0], &vel_x_filter_out[0]);
        lpf_filter(x_ekf[4], 0.01, LPF_KINEMATICS_WN, &vel_y_filter_in[0], &vel_y_filter_out[0]);
        lpf_filter(x_ekf[5], 0.01, LPF_KINEMATICS_WN, &vel_z_filter_in[0], &vel_z_filter_out[0]);            
                    
        // if heli has been sync'd then switch to async send
        if (HelicopterSyncd() && !transmit_thread_active){
            enableTransmitThread(true);
        }
        
        // if transmit thread is not running then send data via this path
        if (!transmit_thread_active)
            sendData();
            
        // Compute average fps for 100 samples
        ++stats_count;
        fps_sum += FPSCalc();
        proc_time_sum += deltaTime();
                    
        if (stats_count == STATS_SAMPLE){
            fps = fps_sum/STATS_SAMPLE;
            process_time = proc_time_sum/STATS_SAMPLE;
            stats_count = 0;
            fps_sum = 0;
            proc_time_sum = 0;
//          printTiming();
        }            
            
        // update mocap data in helicopter struct
        helicopter_data.tx = (float) helicopter_x;
        helicopter_data.ty = (float) helicopter_y;
        helicopter_data.tz = (float) helicopter_z;
        helicopter_data.vx = (float) helicopter_vx;
        helicopter_data.vy = (float) helicopter_vy;
        helicopter_data.vz = (float) helicopter_vz;
        helicopter_data.q0 = (float) x_ekf[6];
        helicopter_data.q1 = (float) x_ekf[7];
        helicopter_data.q2 = (float) x_ekf[8];
        helicopter_data.q3 = (float) x_ekf[9];
        
        if (helicopter_info.multicast_use){
            // Send data via multicast
            int sz = sizeof(Helicopter_data_struct);
            memcpy(&multicast_data[0], &helicopter_data, sz);
            multicast_data[sz] = multicast->computeChecksum(multicast_data, sz);
            multicast->sendBinary(multicast_data, sz + 1);        
        }
    }        
}

void Helicopter::sendData(void){
    // get current time
    unsigned int timestamp_uint = timeNow();
    
    // compute mean of covariance for position
    real_T mean_cov_pos = sqrt(pow(P_ekf[0],2)+pow(P_ekf[17],2)+pow(P_ekf[34],2));
    if ((mean_cov_pos > POS_COVAR_LIMIT) || !state_propagation_on){
        helicopter_config = 0xCA;
        if (state_propagation_on)
            cout << "POS_COVAR_LIMIT exceeded" << endl;
    }
    else{
        helicopter_config = 0x0B;
    }              
            
    short int translation_int16[3] = {0};
    short int velocity_int16[3] = {0};
    float quaternion_float[4] = {0};
    char translation_s[3][20] = {{0},{0}};
    char velocity_s[3][20] = {{0},{0}};
    char quaternion_s[4][20] = {{0},{0}};
    char sequence_s[20] = {0};
    char checksum_s[20] = {0};
    char heli_config_s[3] = {0};
    char timestamp_s[20] = {0};
            
    sprintf(&heli_config_s[0], "%02X", (unsigned char) helicopter_config);
            
    translation_int16[0] = (short int) (x_ekf[0]*1000);
    translation_int16[1] = (short int) (x_ekf[1]*1000);
    translation_int16[2] = (short int) (x_ekf[2]*1000);
//        velocity_int16[0] = (short int) (x_ekf[3]*1000);
//        velocity_int16[1] = (short int) (x_ekf[4]*1000);
//        velocity_int16[2] = (short int) (x_ekf[5]*1000);  
            
//        translation_int16[0] = (short int) (trans_x_filter_out[0]*1000);
//        translation_int16[1] = (short int) (trans_y_filter_out[0]*1000);
//        translation_int16[2] = (short int) (trans_z_filter_out[0]*1000);
    velocity_int16[0] = (short int) (vel_x_filter_out[0]*1000);
    velocity_int16[1] = (short int) (vel_y_filter_out[0]*1000);
    velocity_int16[2] = (short int) (vel_z_filter_out[0]*1000);
        
    quaternion_float[0] = (float) x_ekf[6];
    quaternion_float[1] = (float) x_ekf[7];
    quaternion_float[2] = (float) x_ekf[8];
    quaternion_float[3] = (float) x_ekf[9];        
            
    shortInt2ASCIIHex(translation_int16[0], &translation_s[0][0]);
    shortInt2ASCIIHex(translation_int16[1], &translation_s[1][0]);
    shortInt2ASCIIHex(translation_int16[2], &translation_s[2][0]);
        
    shortInt2ASCIIHex(velocity_int16[0], &velocity_s[0][0]);
    shortInt2ASCIIHex(velocity_int16[1], &velocity_s[1][0]);
    shortInt2ASCIIHex(velocity_int16[2], &velocity_s[2][0]);        
        
    float2ASCIIHex(quaternion_float[0], &quaternion_s[0][0]);
    float2ASCIIHex(quaternion_float[1], &quaternion_s[1][0]);
    float2ASCIIHex(quaternion_float[2], &quaternion_s[2][0]);
    float2ASCIIHex(quaternion_float[3], &quaternion_s[3][0]); 
            
    shortInt2ASCIIHex(key, &sequence_s[0]);
            
    u32int2ASCIIHex(timestamp_uint, &timestamp_s[0]);         
            
    int len = sprintf(tx_data,"*#%s%s%s%s%s%s%s%s%s%s%s%s%s",
        &translation_s[0][0], &translation_s[1][0], &translation_s[2][0],
        &velocity_s[0][0], &velocity_s[1][0], &velocity_s[2][0],
        &quaternion_s[0][0], &quaternion_s[1][0], &quaternion_s[2][0], &quaternion_s[3][0],
        &heli_config_s[0],
        &timestamp_s[0],
        &sequence_s[0]);
        
    unsigned short int checksum2 = 0;
    for (int i = 0; i < len - 1; i++){
        checksum2 ^= (unsigned short int) ((unsigned char)tx_data[i] << 8) | (unsigned char)tx_data[i+1];
    }        
        
    shortInt2ASCIIHex(checksum2, &checksum_s[0]);
    sprintf(&tx_data[len], "%s\r\n", &checksum_s[0]);    
    sendPacket(tx_data);
}

void Helicopter::enableTransmitThread(bool en){
    if (en){
        quit_transmit_thread = false;
        if (pthread_create(&transmit_thread, 0, &thread_catch_heli_tx, this) != 0){
            errorMessage("Creating helicopter transmit_thread thread");
        }
        transmit_thread_active = true;
        // Make thread priority high
        struct sched_param sp;
        sp.sched_priority = 95;
        pthread_setschedparam(transmit_thread, SCHED_FIFO, &sp); 
    }
    else{
        quit_transmit_thread = true;
        // wait for thread to end
        pthread_join(quit_transmit_thread, NULL);
        transmit_thread_active = false;
    }
}

bool Helicopter::HelicopterSyncd(void){
    if (configuration & RN131_SYNC){
        return true;
    }
    else
        return false;
}

void Helicopter::transmitThread(void){
    debugMessage("Transmit thread activated");
    
    while(!quit_transmit_thread){
        // send last updated packet
        sendData();
        usleep(TRANSMIT_SLEEP);
    }
    
    debugMessage("Transmit thread quiting");
}

void Helicopter::initialiseNotchFilter(void){    
    accelxFilterIn[0] = accelxFilterIn[1] = accelxFilterIn[2] = (real_T)accelerometer[0]*9.81/accel1g[0];
    accelxFilterOut[0] = accelxFilterOut[1] = accelxFilterOut[2] = (real_T)accelerometer[0]*9.81/accel1g[0];
    accelyFilterIn[0] = accelyFilterIn[1] = accelyFilterIn[2] = (real_T)accelerometer[1]*9.81/accel1g[1];
    accelyFilterOut[0] = accelyFilterOut[1] = accelyFilterOut[2] = (real_T)accelerometer[1]*9.81/accel1g[1];
    accelzFilterIn[0] = accelzFilterIn[1] = accelzFilterIn[2] = (real_T)accelerometer[2]*9.81/accel1g[2];
    accelzFilterOut[0] = accelzFilterOut[1] = accelzFilterOut[2] = (real_T)accelerometer[2]*9.81/accel1g[2];    
}

float Helicopter::notchAccel(float accel_in, float input[3], float output[3], float dt){
    // shift input and output data
    shiftData(&input[0]);
    input[0] = accel_in;
    shiftData(&output[0]);
    
    float Q = ACCELEROMETER_FILTER_WN/ACCELEROMETER_FILTER_BW;
    return filter_notch(ACCELEROMETER_FILTER_WN, Q, dt, &input[0], &output[0]);
}

void Helicopter::filter_1st_order(float n1, float n2, float d1, float d2, float input[3], float output[3]){
    output[0] = (n1*input[0] + n2*input[1] - d2*output[1])/d1;
}

float Helicopter::lpf_filter(float filter_in, float dt, float wn, float input[3], float output[3]){
    float n0 = (dt*wn);
    float n1 = (dt*wn);
    float d0 = (dt*wn + 2);
    float d1 = (dt*wn - 2);

    shiftData(&input[0]);
    input[0] = filter_in;
    shiftData(&output[0]);

    filter_1st_order(n0, n1, d0, d1, &input[0], &output[0]);
    return output[0];
}

void Helicopter::filter_2nd_order(float n0, float n1, float n2, float d0, float d1, float d2, float input[3], float output[3]){
    output[0] = (n0*input[0] + n1*input[1] + n2*input[2] - d1*output[1] - d2*output[2])/d0;
}

float Helicopter::filter_notch(float wn, float Q, float Ts, float input[3], float output[3]){
    // Compute prewarp frequency
    float wnp = (2/Ts)*tan(wn*Ts/2);
    float Ts_sqrd = pow(Ts,2);
    float wnp_sqrd = pow(wnp,2);
    float n0 = (Q*Ts_sqrd*wnp_sqrd + 4*Q);
    float n1 = (2*Q*Ts_sqrd*wnp_sqrd - 8*Q);
    float n2 = Q*Ts_sqrd*wnp_sqrd + 4*Q;
    float d0 = (Q*Ts_sqrd*wnp_sqrd + 2*Ts*wnp + 4*Q);
    float d1 = (2*Q*Ts_sqrd*wnp_sqrd - 8*Q);
    float d2 = Q*Ts_sqrd*wnp_sqrd - 2*Ts*wnp + 4*Q;

    filter_2nd_order(n0, n1, n2, d0, d1, d2, &input[0], &output[0]);
    return output[0];
}

void Helicopter::shiftData(float data[3]){
    data[2] = data[1];
    data[1] = data[0];
}

void Helicopter::printTiming(void){
    printf("[H%d] Process time: %07.0fus FPS: %04.2f\r\n", helicopter_id, process_time, fps);
}

void Helicopter::projectStateErrorCovarianceCurrent(void){
    // compute dt 
    unsigned int current_time = timeNow();
    real_T dt = (real_T)(current_time - previous_stateupdate_timestamp)/1e6;   
    if (dt < 10e-3)
        return;
        
    pthread_mutex_lock(&run_ekf_mutex);
    if (!ekf_initialising){
        real_T u_ekf[6] = {0};
        u_ekf[0] = (real_T)gyroscope[0]*(M_PI/180)/gyro_raw2dps[0];
        u_ekf[1] = (real_T)gyroscope[1]*(M_PI/180)/gyro_raw2dps[1];
        u_ekf[2] = (real_T)gyroscope[2]*(M_PI/180)/gyro_raw2dps[2];
        u_ekf[3] = accelerometer[0]*9.81/accel1g[0];
        u_ekf[4] = accelerometer[1]*9.81/accel1g[1];
        u_ekf[5] = accelerometer[2]*9.81/accel1g[2]; 
        
        real_T mean[16];
        real_T Cov[256];
        real_T dynamic_accel[3] = {0};
        
        // modify previous update time
        previous_stateupdate_timestamp = current_time;

        projectStateAndCov(x_ekf, P_ekf, u_ekf, Q_ekf, dt, static_acceleration_earth, &mean[0], &Cov[0]);
        calculateAcceleration(x_ekf, u_ekf, &dynamic_accel[0]);
        
        helicopter_ax = dynamic_accel[0];
        helicopter_ay = dynamic_accel[1];
        helicopter_az = dynamic_accel[2];
        
        memcpy(&x_ekf[0],&mean[0],sizeof(real_T)*16);
        memcpy(&P_ekf[0],&Cov[0],sizeof(real_T)*256);
        
        // Set helicopter position for display
        helicopter_x = x_ekf[0];
        helicopter_y = x_ekf[1];
        helicopter_z = x_ekf[2];     
        
        // Set helicopter velocity for display
        helicopter_vx = x_ekf[3];
        helicopter_vy = x_ekf[4];
        helicopter_vz = x_ekf[5];             
        
        // rotate quaternion by PI rad/s then store euler angles
        real_T rotated_quat[4] = {-x_ekf[7],x_ekf[6],-x_ekf[9],x_ekf[8]};
        eulerAnglesFromQuaternion(&rotated_quat[0], &roll_ekf, &pitch_ekf, &yaw_ekf); 
        int yaw_diff = (int) round((helicopter_yaw - yaw_ekf)/360);
        helicopter_yaw = yaw_ekf + yaw_diff*360;
        helicopter_roll = roll_ekf;
        helicopter_pitch = pitch_ekf;  
        
        sprintf(ekf_buffer,"%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%u\r\n",
                x_ekf[0],x_ekf[1],x_ekf[2],
                x_ekf[3],x_ekf[4],x_ekf[5],
                x_ekf[6],x_ekf[7],x_ekf[8],x_ekf[9],
                x_ekf[10],x_ekf[11],x_ekf[12],
                x_ekf[13],x_ekf[14],x_ekf[15],
                helicopter_ax, helicopter_ay, helicopter_az,
                P_ekf[0],P_ekf[17],P_ekf[34],
                P_ekf[51],P_ekf[68],P_ekf[85],
                P_ekf[102],P_ekf[119],P_ekf[136],P_ekf[153],
                timeNow());

        writeEKFLog();
    }
    pthread_mutex_unlock(&run_ekf_mutex);
}

void Helicopter::runEKF(){
    real_T angular_rate_body[3] = {0};
    real_T accel_body[3] = {0};
    
    angular_rate_body[0] = (real_T)gyroscope[0]*(M_PI/180)/gyro_raw2dps[0];
    angular_rate_body[1] = (real_T)gyroscope[1]*(M_PI/180)/gyro_raw2dps[1];
    angular_rate_body[2] = (real_T)gyroscope[2]*(M_PI/180)/gyro_raw2dps[2];
      
    accel_body[0] = body_acceleration[0];
    accel_body[1] = body_acceleration[1];
    accel_body[2] = body_acceleration[2];
    
    pthread_mutex_lock(&run_ekf_mutex);
    
    if (ekf_initialising){
        if (sample++ < EKF_INIT_ITERATION){
            accel_body_sum[0] += accel_body[0];
            accel_body_sum[1] += accel_body[1];
            accel_body_sum[2] += accel_body[2];
            
            ang_rate_body_sum[0] += angular_rate_body[0];
            ang_rate_body_sum[1] += angular_rate_body[1];
            ang_rate_body_sum[2] += angular_rate_body[2];
        }
        else{
            // Find average in the body frame
            static_acceleration_body[0] = accel_body_sum[0]/EKF_INIT_ITERATION;
            static_acceleration_body[1] = accel_body_sum[1]/EKF_INIT_ITERATION;
            static_acceleration_body[2] = accel_body_sum[2]/EKF_INIT_ITERATION;
            
            // Rotate gravity vector from body frame to inertial frame for subtraction later
            // i.e. by PI rad along x-axis approximately
            // initial quaternion already set by pose estimator
            
            Reb(&x_ekf[6],&static_acceleration_body[0],&static_acceleration_earth[0]);
            
            init_gyro_bias_est[0] = x_ekf[10] = ang_rate_body_sum[0]/EKF_INIT_ITERATION;
            init_gyro_bias_est[1] = x_ekf[11] = ang_rate_body_sum[1]/EKF_INIT_ITERATION;
            init_gyro_bias_est[2] = x_ekf[12] = ang_rate_body_sum[2]/EKF_INIT_ITERATION;
            
            // let lsqnonlin determine this
//            // Set initial position
//            x_ekf[0] = INITIAL_POSITION_X; 
//            x_ekf[1] = INITIAL_POSITION_Y; 
//            x_ekf[2] = INITIAL_POSITION_Z;          
            // Set initial velocity
            x_ekf[3] = 0; 
            x_ekf[4] = 0; 
            x_ekf[5] = 0;
            // Initial gyro biases set above
            // Initially the acceleration bias should be zero because any bias
            // should be contained in static_acceleration_earth
            x_ekf[13] = 0;
            x_ekf[14] = 0;
            x_ekf[15] = 0;
            
            // clear variables
            memset(P_ekf,0,sizeof(real_T)*256);
            memset(Q_ekf,0,sizeof(real_T)*324);
            
            real_T accel_noise_xy = 0.75;    // LSB-rms
            real_T accel_noise_z = 1.1;    // LSB-rms
            real_T gyro_noise = 0.38;    // deg/s-rms
            real_T accel_xy_noise_var = pow(((accel_noise_xy/256)*9.81),2);  // (m/s)^2
            real_T accel_z_noise_var = pow(((accel_noise_z/256)*9.81),2);  // (m/s)^2
            real_T gyro_noise_var = pow((gyro_noise*M_PI/180),2);  // (rad/s)^2
            
            // set process noise covariance
            Q_ekf[0] = Q_ekf[19] = Q_ekf[38] = POSITION_STATE_ERROR_VARIANCE;
            Q_ekf[57] = Q_ekf[76] = Q_ekf[95] = VELOCITY_STATE_ERROR_VARIANCE;
            Q_ekf[114] = accel_xy_noise_var*ACCELEROMETER_VARIANCE_SCALER;
            Q_ekf[133] = accel_xy_noise_var*ACCELEROMETER_VARIANCE_SCALER;
            Q_ekf[152] = accel_z_noise_var*ACCELEROMETER_VARIANCE_SCALER;
            Q_ekf[171] = Q_ekf[190] = Q_ekf[209] = gyro_noise_var*GYROSCOPE_VARIANCE_SCALER;
            Q_ekf[228] = Q_ekf[247] = Q_ekf[266] = GYROSCOPE_BIAS_VARIANCE;
            Q_ekf[285] = Q_ekf[304] = Q_ekf[323] = ACCELEROMETER_BIAS_VARIANCE;
    
            // set initial state-error covariance
            // Position
            P_ekf[0] = 1e-8;
            P_ekf[17] = 1e-8;
            P_ekf[34] = 1e-8;
            // Velocity
            P_ekf[51] = 1e-8;
            P_ekf[68] = 1e-8;
            P_ekf[85] = 1e-8;
            // Quaternion
            P_ekf[102] = 1e-8;
            P_ekf[119] = 1e-8;
            P_ekf[136] = 1e-8;
            P_ekf[153] = 1e-8;
            // Gyro bias eq
            P_ekf[170] = 1e-8;
            P_ekf[187] = 1e-8;
            P_ekf[204] = 1e-8;
            // Accel bias eq
            P_ekf[221] = 1e-2;
            P_ekf[238] = 1e-2;
            P_ekf[255] = 1e-2;            
            
            ekf_initialising = false;
            
            // set previous_measurement_timestamp to equal measurement_timestamp for first propagation
            previous_stateupdate_timestamp = measurement_timestamp;
        }                       
    }
    else{
        real_T u_ekf[6] = {angular_rate_body[0],angular_rate_body[1],angular_rate_body[2],
            accel_body[0], accel_body[1], accel_body[2]};
        real_T mean[16];
        real_T Cov[256];
        real_T dynamic_accel[3] = {0};
        
        // propagate state to current time
        unsigned int current_time = timeNow();
        real_T dt = (real_T)(current_time - previous_stateupdate_timestamp)/1e6;
        // shift measurement timestamp so know dt to propagate
        previous_stateupdate_timestamp = current_time;

        // if abnormal timestamp set default
        if ((dt > 1) || (dt < 0)){
            dt = 10e-3;
            debugMessage("abnormal dt");
        }

        projectStateAndCov(x_ekf, P_ekf, u_ekf, Q_ekf, dt, static_acceleration_earth, &mean[0], &Cov[0]);
        calculateAcceleration(x_ekf, u_ekf, &dynamic_accel[0]);
        
        helicopter_ax = dynamic_accel[0];
        helicopter_ay = dynamic_accel[1];
        helicopter_az = dynamic_accel[2];
        
        memcpy(&x_ekf[0],&mean[0],sizeof(real_T)*16);
        memcpy(&P_ekf[0],&Cov[0],sizeof(real_T)*256);
        
        // Set helicopter position for display
        helicopter_x = x_ekf[0];
        helicopter_y = x_ekf[1];
        helicopter_z = x_ekf[2];     
        
        // Set helicopter velocity for display
        helicopter_vx = x_ekf[3];
        helicopter_vy = x_ekf[4];
        helicopter_vz = x_ekf[5];             
        
        // rotate quaternion by PI rad/s then store euler angles
        real_T rotated_quat[4] = {-x_ekf[7],x_ekf[6],-x_ekf[9],x_ekf[8]};
        eulerAnglesFromQuaternion(&rotated_quat[0], &roll_ekf, &pitch_ekf, &yaw_ekf); 
        int yaw_diff = (int) round((helicopter_yaw - yaw_ekf)/360);
        helicopter_yaw = yaw_ekf + yaw_diff*360;
        helicopter_roll = roll_ekf;
        helicopter_pitch = pitch_ekf;
        
        // log ekf result
        sprintf(ekf_buffer,"%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%u\r\n",
                x_ekf[0],x_ekf[1],x_ekf[2],
                x_ekf[3],x_ekf[4],x_ekf[5],
                x_ekf[6],x_ekf[7],x_ekf[8],x_ekf[9],
                x_ekf[10],x_ekf[11],x_ekf[12],
                x_ekf[13],x_ekf[14],x_ekf[15],
                helicopter_ax, helicopter_ay, helicopter_az,
                P_ekf[0],P_ekf[17],P_ekf[34],
                P_ekf[51],P_ekf[68],P_ekf[85],
                P_ekf[102],P_ekf[119],P_ekf[136],P_ekf[153],
                timeNow());

        writeEKFLog();           
    }
    pthread_mutex_unlock(&run_ekf_mutex);
}

void Helicopter::calculateAcceleration(const real_T x[16], const real_T u[6], real_T dynamic_accel[3]){
    real_T corrected_accel[3] = {u[3] - x[13], u[4] - x[14],u[5] - x[15]};
    Reb(&x_ekf[6],&corrected_accel[0],&dynamic_accel[0]);
    dynamic_accel[0] -= static_acceleration_earth[0];
    dynamic_accel[1] -= static_acceleration_earth[1];
    dynamic_accel[2] -= static_acceleration_earth[2];
}

void Helicopter::sendDummy(void){
    short int translation16int_x = 0;
    short int translation16int_y = 0;
    short int translation16int_z = 0;
    char checksum = 1;
        
    int len = sprintf(tx_data,"*#%07.4f,%07.4f,%07.4f,%07.4f,%06d,%06d,%06d,%.2X\r\n",               
        x_ekf[6], x_ekf[7], x_ekf[8], x_ekf[9], translation16int_x, translation16int_y, translation16int_z, 
        (unsigned char)checksum);     
    std::cout << len << " - " << tx_data << std::endl;
    sendPacket(tx_data);   
}

void Helicopter::initialiseEKFvariables(void){
    // Initialise EKF here
    ekf_coder_initialize();
    x_ekf[0] = 0; x_ekf[1] = 0; x_ekf[2] = 756e-3;          // position
    x_ekf[3] = 0; x_ekf[4] = 0; x_ekf[5] = 0;               // velocity
    x_ekf[6] = 0; x_ekf[7] = 1; x_ekf[8] = 0; x_ekf[9] = 0; // quaternion
    x_ekf[10] = 0.002; x_ekf[11] = -0.0022; x_ekf[12] = -0.0013;        // gyro bias
    x_ekf[13] = -0.0013; x_ekf[14] = 0.0051; x_ekf[15] = 0.0040;        // accel bias
    
    // clear variables
    memset(P_ekf,0,sizeof(real_T)*256);
    memset(Q_ekf,0,sizeof(real_T)*324);
    
    // set process noise covariance
    Q_ekf[0] = Q_ekf[19] = Q_ekf[38] = 1e-2;
    Q_ekf[57] = Q_ekf[76] = Q_ekf[95] = 1e1;
    Q_ekf[114] = 900e-6;
    Q_ekf[133] = 745e-6;
    Q_ekf[152] = 2.33e-3;
    Q_ekf[171] = Q_ekf[190] = Q_ekf[209] = 0.144;
    Q_ekf[228] = Q_ekf[247] = Q_ekf[266] = 1e-10;
    Q_ekf[285] = Q_ekf[304] = Q_ekf[323] = 1e-10;
    
    // set initial state error covariance
    P_ekf[0] = P_ekf[17] = P_ekf[34] = P_ekf[51] = P_ekf[68] = P_ekf[85] = P_ekf[102] = P_ekf[119] = P_ekf[136] = P_ekf[153] = P_ekf[170] = P_ekf[187] = P_ekf[204] = P_ekf[221] = P_ekf[238] = P_ekf[255] = 1e-10;
//    P_ekf[34] = 0.5;    
    
    ekf_initialising = true;
}

real_T * Helicopter::kalmanState(void){
    return &x_ekf[0];
}

real_T * Helicopter::kalmanErrorCovariance(void){
    return &P_ekf[0];
}

void Helicopter::logCorrectionTime(void){
    correction_time = timeNow();
}

unsigned int Helicopter::getCorrectionTime(void){
    return correction_time;
}

unsigned int Helicopter::timeSinceCorrection(void){
    return (timeNow() - correction_time);
}

void Helicopter::sendPacket(char data[]){
    udp_socket->sendPacket(data);
}

int Helicopter::getID(void){
    return helicopter_id;
}

bool Helicopter::getAutoMode(void){
    return auto_mode;
}

void Helicopter::enableStatePropagation(bool en){
    state_propagation_on = en;
}

real_T Helicopter::getx(void){
    return helicopter_x*1000;
}

real_T Helicopter::gety(void){
    return helicopter_y*1000;
}

real_T Helicopter::getz(void){
    return helicopter_z*1000;
}

float Helicopter::getVoltage(void){
    return ((float)(voltage))/47.288136 + 0.33;
}

float Helicopter::getHeadSpeed(void){
    return (float)head_speed;
}

float Helicopter::getTemperature(void){
    return temperature_f;
}

real_T Helicopter::getVx(void){
    return helicopter_vx*1000;
}

real_T Helicopter::getVy(void){
    return helicopter_vy*1000;
}

real_T Helicopter::getVz(void){
    return helicopter_vz*1000;
}

real_T Helicopter::getRoll(void){
    return helicopter_roll;
}

real_T Helicopter::getPitch(void){
    return helicopter_pitch;
}

real_T Helicopter::getYaw(void){
    return helicopter_yaw;
}

real_T Helicopter::getOnboardRoll(void){
    return (real_T)onboard_roll/100;
}

real_T Helicopter::getOnboardPitch(void){
    return (real_T)onboard_pitch/100;
}

real_T Helicopter::getOnboardYaw(void){
    return (real_T)onboard_yaw/100;
}

real_T Helicopter::getOnboardTx(void){
    return (real_T) tx_x_position;
}

real_T Helicopter::getOnboardTy(void){
    return (real_T) tx_y_position;
}

real_T Helicopter::getOnboardTz(void){
    return (real_T) tx_z_position;
}

float * Helicopter::mocapPositionX(void){
    return &mocap_position_x;
}

float * Helicopter::mocapPositionY(void){
    return &mocap_position_y;
}

float * Helicopter::mocapPositionZ(void){
    return &mocap_position_z;
}

float * Helicopter::mocapVelocityX(void){
    return &mocap_velocity_x;
}

float * Helicopter::mocapVelocityY(void){
    return &mocap_velocity_y;
}

float * Helicopter::mocapVelocityZ(void){
    return &mocap_velocity_z;
}

float * Helicopter::mocapRoll(void){
    return &mocap_roll;
}

float * Helicopter::mocapPitch(void){
    return &mocap_pitch;
}

float * Helicopter::mocapYaw(void){
    return &mocap_yaw;
}

float * Helicopter::mocapRollRate(void){
    return &mocap_wx;
}

float * Helicopter::mocapPitchRate(void){
    return &mocap_wy;
}

float * Helicopter::mocapYawRate(void){
    return &mocap_wz;
}

float * Helicopter::mocapq0(void){
    return &mocap_q0;
}

float * Helicopter::mocapq1(void){
    return &mocap_q1;
}

float * Helicopter::mocapq2(void){
    return &mocap_q2;
}

float * Helicopter::mocapq3(void){
    return &mocap_q3;
}

void Helicopter::float2ASCIIHex(const float val, char * return_string){
    char temp[4] = {0};
    memcpy(&temp[0], &val, sizeof(float));
    
    sprintf(return_string, "%.2X%.2X%.2X%.2X", (unsigned char)temp[0]
            ,(unsigned char)temp[1]
            ,(unsigned char)temp[2]
            ,(unsigned char)temp[3]);
}

void Helicopter::u32int2ASCIIHex(const unsigned int val, char * return_string){
    char temp[4] = {0};
    memcpy(&temp[0], &val, sizeof(unsigned int));
    
    sprintf(return_string, "%.2X%.2X%.2X%.2X", (unsigned char)temp[0]
            ,(unsigned char)temp[1]
            ,(unsigned char)temp[2]
            ,(unsigned char)temp[3]);
}

void Helicopter::shortInt2ASCIIHex(const short int val, char * return_string){
    char temp[2] = {0};
    memcpy(&temp[0], &val, sizeof(short int));
    
    sprintf(return_string, "%.2X%.2X", (unsigned char)temp[0]
            ,(unsigned char)temp[1]);
}

void Helicopter::setx(real_T val){
    helicopter_x = val;
}

void Helicopter::sety(real_T val){
    helicopter_y = val;
}

void Helicopter::setz(real_T val){
    helicopter_z = val;
}

real_T * Helicopter::getRed3D(void){
    return &red3D[0];
}

real_T * Helicopter::getGreen3D(void){
    return &green3D[0];
}

real_T * Helicopter::getBlue3D(void){
    return &blue3D[0];
}

void Helicopter::updateObjectPoints(void){
    object_points[0] = red3D[0];
    object_points[1] = red3D[1];
    object_points[2] = red3D[2];
    object_points[3] = green3D[0];
    object_points[4] = green3D[1];
    object_points[5] = green3D[2];
    object_points[6] = blue3D[0];
    object_points[7] = blue3D[1];
    object_points[8] = blue3D[2];
}

real_T * Helicopter::getObjectPoints(void){
    return &object_points[0];
}

void Helicopter::setRed3D(real_T * blob){
    memcpy(&red3D[0], blob, sizeof(real_T)*3);
}

void Helicopter::setGreen3D(real_T * blob){
    memcpy(&green3D[0], blob, sizeof(real_T)*3);
}

void Helicopter::setBlue3D(real_T * blob){
    memcpy(&blue3D[0], blob, sizeof(real_T)*3);
}

real_T * Helicopter::getInitCoordinates(void){
    return &initial_coordinates[0];
}

real_T * Helicopter::getInitOrientation(void){
    return &initial_orientation[3];
}

real_T * Helicopter::getInitPose(void){
    return &initial_pose[0];
}

void Helicopter::setInitCoordinates(real_T * coordinates){
    memcpy(&initial_coordinates[0], coordinates, sizeof(real_T)*3);
}

void Helicopter::setInitOrientation(real_T * orientation){
    memcpy(&initial_orientation[0], orientation, sizeof(real_T)*4);
}

bool Helicopter::getInitialisingPose(void){
    return initialising_pose;
}

void Helicopter::setInitialisingPose(bool val){
    initialising_pose = val;
}

int * Helicopter::getInitCounterSum(void){
    return &init_counter_sum;
}

real_T * Helicopter::getInitq0Sum(void){
    return &q_offset[0];
}

real_T * Helicopter::getInitq1Sum(void){
    return &q_offset[1];
}

real_T * Helicopter::getInitq2Sum(void){
    return &q_offset[2];
}

real_T * Helicopter::getInitq3Sum(void){
    return &q_offset[3];
}

real_T * Helicopter::getInitxSum(void){
    return &translation_offset[0];
}

real_T * Helicopter::getInitySum(void){
    return &translation_offset[1];
}

real_T * Helicopter::getInitzSum(void){
    return &translation_offset[2];
}

char * Helicopter::getIpAddress(void){
    return &ip_address[0];
}

char * Helicopter::getIpPort(void){
    return &ip_port[0];
}

char * Helicopter::getPCport(void){
    return &pc_port[0];
}

void Helicopter::setRoll(real_T val){
    helicopter_roll = val;
}

void Helicopter::setPitch(real_T val){
    helicopter_pitch = val;
}

void Helicopter::setYaw(real_T val){
    helicopter_yaw = val;
}

void Helicopter::openLog(char * file_name){
    dataLog.open(file_name);
}

void Helicopter::openEKFLog(char * file_name){
    ekfLog.open(file_name);
}

void Helicopter::closeLog(void){
    if (dataLog.is_open())
        dataLog.close();
    if (ekfLog.is_open())
        ekfLog.close();    
}

bool Helicopter::writeLog(void){
    if (dataLog.is_open()){
        dataLog << log_buffer;
        return true;
    }
    else
        return false;
}

bool Helicopter::writeEKFLog(void){
    if (ekfLog.is_open()){
        ekfLog << ekf_buffer;
        return true;
    }
    else
        return false;
}

void Helicopter::debugMessage(const char * str){
    printf("DBG [Helicopter %d] >> %s\r\n", helicopter_id,str);
}

void Helicopter::errorMessage(const char * str){
    printf("ERR [Helicopter %d] >> %s\r\n", helicopter_id, str);
}

Helicopter::~Helicopter() {
    if (multicast != NULL){
        delete multicast;
        multicast = NULL;
    }        
    if (udp_socket != NULL){
        data_rx_connection.disconnect();
        delete udp_socket;
        udp_socket = NULL;
    }    
    if (transmit_thread_active){
        enableTransmitThread(false);
    }
    
    closeLog();
    
    pthread_mutex_destroy(&run_ekf_mutex);
    pthread_mutex_destroy(&dt_mutex);
}

