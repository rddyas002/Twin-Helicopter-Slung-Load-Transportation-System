/* 
 * File:   UDP_socket.cpp
 * Author: yashren
 * 
 * Created on 11 March 2013, 10:36 AM
 */

#include <qt4/QtCore/qstring.h>

#include "../include/UDP_socket.h"
#include <locale.h>
#include <algorithm>
#include <string>
#include <cstdio>
#include <QDateTime>

#define T0_TIME 1393861628

extern "C"
{
    // this C function will be used to receive the thread and pass it back to the Thread instance
    void* thread_catch_udp_tx(void* arg)
    {
        UDP_socket* t = static_cast<UDP_socket*>(arg);
        t->transmitThread();
        return 0;
    }

    // this C function will be used to receive the thread and pass it back to the Thread instance
    void* thread_catch_udp_rx(void* arg)
    {
        UDP_socket* t = static_cast<UDP_socket*>(arg);
        t->receiveThread();
        return 0;
    }      
}

using namespace std;
ofstream UDP_socket::logFile;
char UDP_socket::write_buffer[512] = {0};

void UDP_socket::openLogFile(void){
    logFile.open("dataLog.txt");
}

void UDP_socket::closeLogFile(void){
    if (logFile.is_open())
        logFile.close();
}

bool UDP_socket::writeLogFile(void){
    if (logFile.is_open()){
        logFile << write_buffer;
        return true;
    }
    else
        return false;
}

UDP_socket::UDP_socket(const QString& ipaddress,const  QString& ipport) {
    
    setlocale(LC_ALL, "C");
    
    // initialize
    transmitting = false;
    receiving = false;
    tx_ms_delay = 200;     //default delay set to 100ms
    read_timeout = 5000;
    
    transmit_thread = NULL;
    
    auto_mode = gps_mode = 'N';
    tx_rate = '0';      // off
    servo_left = servo_right = servo_front = servo_yaw = esc_thrust = 0;    
    
    leds = 0;
    red_led = green_led = blue_led = 'N';
    
    //initialize variables
    gyroscope[0] = gyroscope[1] = gyroscope[2] = gyroscope[3] = 0;
    accelerometer[0] = accelerometer[1] = accelerometer[2] = 0;
    magnetometer[0] = magnetometer[1] = magnetometer[2] = 0;            
    temperature_bmp = 0.0;
    ultrasonic_distance = voltage = key = 0;
    pressure = 0;
    checksum = 0;
    head_speed = 0;
    room_x = room_y = room_z = 0;
    q0 = 0; q1 = 1; q2 = 0; q3 = 0;
    cam_quality = 0;
    
    servo_left_measured = 0;
    servo_right_measured = 0;
    servo_rear_measured = 0;
    servo_rudder_measured = 0;
    esc_measured = 0;
    gyro_gain = 0;
    time_data_received = 0;
    latency = 0;
    
    QByteArray ip_add_lat = ipaddress.toLatin1();
    QByteArray port_add_lat = ipport.toLatin1();
    
    const char *c_ipadd = ip_add_lat.data();
    const char *c_port = port_add_lat.data();
    
    // set helicopter IP address and port for connecting through UDP
    portno = atoi(c_port);
    strcpy(&heli_ip_address[0], c_ipadd);
    
    openLogFile();
    
    // Initialise EKF here
    ekf_coder_initialize();
    x[0] = 0; x[1] = 0; x[2] = 756e-3;           // position
    x[3] = 0; x[4] = 0; x[5] = 0;           // velocity
    x[6] = 0; x[7] = 1; x[8] = 0; x[9] = 0; // quaternion
    x[10] = 0.002; x[11] = -0.0022; x[12] = -0.0013;        // gyro bias
    x[13] = -0.0013; x[14] = 0.0051; x[15] = 0.0040;        // accel bias
    
    // clear variables
    memset(P,0,sizeof(real_T)*256);
    memset(Q,0,sizeof(real_T)*324);
    
    // set process noise covariance
    Q[0] = Q[19] = Q[38] = 1e-2;
    Q[57] = Q[76] = Q[95] = 1e1;
    Q[114] = 900e-6;
    Q[133] = 745e-6;
    Q[152] = 2.33e-3;
    Q[171] = Q[190] = Q[209] = 0.144;
    Q[228] = Q[247] = Q[266] = 1e-10;
    Q[285] = Q[304] = Q[323] = 1e-10;
    
    // set initial state error covariance
    P[0] = P[17] = P[34] = P[51] = P[68] = P[85] = P[102] = P[119] = P[136] = P[153] = P[170] = P[187] = P[204] = P[221] = P[238] = P[255] = 1e-10;
    P[34] = 0.5;
    
    sequence = 0;
    
    memset(&rx_data[0], 0, sizeof(char)*256);
    
//    // Make this process a real-time process
//    struct sched_param sp;
//    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);    
//    if (sched_setscheduler(0, SCHED_FIFO, &sp) == -1){
//        std::cout << "UDP: Process priority set failed" << std::endl;
//    }
//    else{
//        std::cout << "Priority set to max = " << sp.sched_priority  << std::endl;
//    }
    
}

real_T * UDP_socket::kalmanState(void){
    return &x[0];
}

real_T * UDP_socket::kalmanErrorCovariance(void){
    return &P[0];
}

double * UDP_socket::lastCorrection(void){
    return &last_correction;
}

bool UDP_socket::connect_socket(void){
    // create UDP socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    
    if (sockfd < 0)
    {
        cout << "UDP: ERROR creating TX socket." << endl;
        return false;
    }
    
    server = gethostbyname(&heli_ip_address[0]);
    
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(portno);

    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
    {
        cout << "UDP: ERROR connecting socket." << endl;
        return false;
    }
    sockfd_connected = true;
    return true;
}

void UDP_socket::disconnect_socket(void){
    // stop transmitting first
    if (transmitting)
        enableTransmitThread(false);
    
    sockfd_connected = false;
    close_rx();
    close_tx();
}

bool UDP_socket::setupReceive(int port){
   int length;
   struct sockaddr_in server;

   sockrx = socket(AF_INET, SOCK_DGRAM, 0);
   if (sockrx < 0)
   {
       cout << "UDP: Error creating RX socket." << endl;
       return false;
   }    
   
   length = sizeof(server);
   bzero(&server,length);
   server.sin_family=AF_INET;
   server.sin_addr.s_addr=INADDR_ANY;
   server.sin_port=htons(port);   
    
  // make socket non-blocking
    int flags = fcntl(sockrx, F_GETFL);
    flags |= O_NONBLOCK;
    fcntl(sockrx, F_SETFL, flags);

   // Bind socket to port
   if (bind(sockrx,(struct sockaddr *)&server,length)<0)
   {
       cout << "UDP: Error binding RX socket." << endl;
       return false;
   }
    
   fromlen1 = sizeof(struct sockaddr_in); 
   
   // now create rx thread
   enableReceiveThread(true);
   return true;
}

void UDP_socket::set_realTime_priority(void){   
    // get tid
    pthread_t this_thread = pthread_self();
    
    struct sched_param params;
    params.sched_priority = sched_get_priority_max(SCHED_FIFO);
    
    if(pthread_setschedparam(this_thread, SCHED_FIFO, &params) != 0){
        cout << "FIFO thread priority NOT set" << endl;
    }
    else{
        int policy = 0;
        if(pthread_getschedparam(this_thread, &policy, &params) != 0)
            cout<< "Failed to get policy priority" << endl;
        else
            std::cout << "FIFO thread priority set to " << params.sched_priority << std::endl;
            std::cout << "TID " << this_thread << std::endl;
    }
}

void UDP_socket::transmitThread(void){
    cout << "In transmit thread" << endl;
    struct timespec delay;
    
    // Do initialization sequence
    leds = 7;
    for (int i = 0; i < 5; i++){
        leds = 7;
        sendPacket();
        usleep(150000);
        leds = 0;
        sendPacket();
        usleep(150000);
    }
    
    while(!quit_tx_thread)
    {
        //sendPacket();
        char buf[7] = {1}; 
        send(sockfd,buf,7,0);    
        delay.tv_nsec = tx_ms_delay*1000000;
        delay.tv_sec = 0;
        while(nanosleep(&delay, &delay));
    }
    cout << "Out of transmit thread" << endl;
}

void UDP_socket::sendPacket(void){
    if (isConnected()){
        sprintf(tx_data,"*#%d,%d,%d,%d,%d,%c,%c,%c,%u,%d,%d,%d,%d,%.4f*%.4f*%.4f*%.4f*",
        esc_thrust,servo_left,servo_right,servo_front,servo_yaw,auto_mode,gps_mode,
                tx_rate,leds,cam_quality,room_x,room_y,room_z,q0,q1,q2,q3);

        int packet_length = strlen(tx_data);

        int chksum = 0;
        char chksum_ascii[10] = {0};
        for (int i = 0; i < packet_length; i++)
            chksum ^= tx_data[i];

        sprintf(chksum_ascii, "%X", chksum);   
        
        if (strlen(chksum_ascii) == 1)
        {
            tx_data[packet_length++] = '0';
            tx_data[packet_length++] = chksum_ascii[0];
        }
        else
        {
            tx_data[packet_length++] = chksum_ascii[0];
            tx_data[packet_length++] = chksum_ascii[1];
        }

        tx_data[packet_length++] = '\r';
        tx_data[packet_length++] = '\n';
                    
        tx_data[packet_length++] = '\0';
        
        send(sockfd,tx_data,strlen(tx_data),0);    
    }
}

void UDP_socket::receiveThread(void){
    cout << "In receive thread" << endl;
    
    set_realTime_priority();
            
    // setup nonblocking read here
    struct pollfd fds;
    fds.fd = sockrx;
    fds.events = POLLIN;
       
    while(!quit_rx_thread)
    {
        int poll_ret = poll(&fds,1,read_timeout);
        if(poll_ret > 0)
        {    
                int n = recvfrom(sockrx,rx_data,44,MSG_WAITALL,(struct sockaddr *)&from1,&fromlen1);
//                cout << n << endl;
                if (n > 10){
                    time_data_received = timeSince(0);
                    decodeReceivedData();
                }
        }
        else if (poll_ret == 0)
        {
             cout << "UDP: RX timeout." << endl;
        }
    }
    cout << "Out of receive thread" << endl;
}


void UDP_socket::enableTransmitThread(bool en){
    if (en){
        quit_tx_thread = false;
                
        if (pthread_create(&transmit_thread, 0, &thread_catch_udp_tx, this) != 0)
        {
            transmitting = FALSE;
            cout << "UDP: TX thread creating error." << endl;
        }
        
        // make thread priority high
        struct sched_param sp;
        sp.sched_priority = 95;
        pthread_setschedparam(transmit_thread, SCHED_FIFO, &sp);        
        
        transmitting = true;
    }
    else{
        quit_tx_thread = true;
        
        if (transmit_thread != NULL){
            // wait for thread to end
            pthread_join(transmit_thread, NULL);
            transmitting = false;
            transmit_thread = NULL;
        }
    }
}

void UDP_socket::enableReceiveThread(bool en){
    if (en){
        quit_rx_thread = false;
                
        if (pthread_create(&receive_thread, 0, &thread_catch_udp_rx, this) != 0)
        {
            receiving = FALSE;
            cout << "UDP: RX thread creating error." << endl;
        }
        
        receiving = true;
    }
    else{
        quit_rx_thread = true;
        // wait for thread to end
        pthread_join(receive_thread, NULL);
        receiving = false;
    }
}

void UDP_socket::decodeReceivedData(void){
    // this function is called only when new data arrives
    static double first_time_received = 0;
    static unsigned int sample = 0;
    static bool initialise = true;
    static double prev_timestamp = 0;
    
    static float x_count = 1, y_count = 2, z_count = 3;
    static float roll_count = 4, pitch_count = 5, yaw_count = 6;
    
    if (first_time_received == 0)
    {
        first_time_received = time_data_received;
        prev_timestamp = time_data_received;
    }
    
    
    char checksum = 0;
    for (int i = 2; i < 32; i++){
        checksum ^= rx_data[i];
    }    
    
    // check if complete packet has been received
    if ((rx_data[0] == '*') && (rx_data[1] == '#') && (rx_data[33] == '@') && (rx_data[34] == '!') 
            && (checksum == rx_data[32]))
    {     
        unsigned char i = 0, index = 2, LSB = 0;
        unsigned short int MSB = 0;        

        for (i = 0; i < 3; i++)
        {
            MSB = ((unsigned short int)rx_data[index++] << 8);
            LSB = rx_data[index++];
            gyroscope[i+1] = (signed short int) (MSB | LSB);
        }

        for (i = 0; i < 3; i++)
        {
            MSB = ((unsigned short int)rx_data[index++] << 8);
            LSB = rx_data[index++];
            accelerometer[i] = (signed short int) (MSB | LSB);
        }
        body_accel[0] = (float)accelerometer[0]*9.81/272;
        body_accel[1] = (float)accelerometer[1]*9.81/258;
        body_accel[2] = (float)accelerometer[2]*9.81/225;

        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        ultrasonic_distance = (unsigned short int) (MSB | LSB);

        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        voltage = (unsigned short int) (MSB | LSB);
        
        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        servo_rudder_measured = (unsigned short int) (MSB | LSB);
        
        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        servo_left_measured = (unsigned short int) (MSB | LSB);        
            
        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        servo_right_measured = (unsigned short int) (MSB | LSB);        
        
        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        servo_rear_measured = (unsigned short int) (MSB | LSB);        
        
        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        esc_measured = (unsigned short int) (MSB | LSB);
        
        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        gyro_gain = (unsigned short int) (MSB | LSB);        
                
        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        head_speed = (unsigned short int) (MSB | LSB);    

//        unsigned int msb4 = 0, msb3 = 0, msb2 = 0, msb1 = 0;
//        msb4 = (((unsigned int)rx_data[index++]) << 24) & 0xFFFFFF;
//        msb3 = (((unsigned int)rx_data[index++]) << 16) & 0xFFFFFF;
//        msb2 = (((unsigned int)rx_data[index++]) << 8) & 0xFFFF;
//        msb1 = (rx_data[index++] & 0xFF);
        
//        msb4 = (unsigned int)((unsigned int)rx_data[index++] << 24);
//        msb3 = (unsigned int)((unsigned int)rx_data[index++] << 16);
//        msb2 = (unsigned int)((unsigned int)rx_data[index++] << 8);
//        msb1 = (unsigned int)rx_data[index++];
//        helicopter_data_timestamp = (unsigned int) (msb4|msb3|msb2|msb1);
        
        // extract time stamp
        uint32_t temp1 = 0;
        uint32_t temp2 = 0;
        for (int j = 0; j < 4; j++){
            temp1 |= (uint32_t)((unsigned char)rx_data[42 - j] << j*8);
        }
        for (int j = 0; j < 4; j++){
            temp2 |= (uint32_t)((unsigned char)rx_data[38 - j] << j*8);
        }           
        uint64_t temp =  ((uint64_t)temp2 << 32);
        temp |= temp1;        
        
        double time_sent = (double)temp/32768;

//        printf("%.3f\r\n", time_data_received - time_sent);
        
//        std::cout << (double)temp/32768 << std::endl;
        
        double data_timestamp = (double)temp/32768 - T0_TIME;
        double time_data_rx = time_data_received - T0_TIME;
        
        latency = (time_data_rx - data_timestamp)*1000;
        
        float right_delta = servo_right_measured - SERVO_RIGHT_BIAS;
        float left_delta = -(servo_left_measured - SERVO_LEFT_BIAS);
        float rear_delta = -(servo_rear_measured - SERVO_REAR_BIAS);
        
        float servos_avg_height = (right_delta+left_delta+rear_delta)/3;
        float servo_lon_diff = 2*rear_delta - (left_delta + right_delta);
        float servo_lat_diff = right_delta - left_delta;
        collective = servos_avg_height*SERVO_AVG_HEIGHT_2_COLLECTIVE_GRAD + SERVO_AVG_HEIGHT_2_COLLECTIVE_OFFSET;
        longitudinal = servo_lon_diff*SERVO_LONG_DIFF_2_LONG_GRAD + SERVO_LONG_DIFF_2_LONG_OFFSET;
        lateral = servo_lat_diff*SERVO_LAT_DIFF_2_LAT_GRAD + SERVO_LAT_DIFF_2_LAT_OFFSET;
        /*
        sprintf(write_buffer,"%d,%d,%d,%d,%d,%d,%u,%.2f,%u,%u,%u,%u,%u,%u,%u,%.3f,%.3f,%.3f\r\n",
                gyroscope[1],gyroscope[2],gyroscope[3],
                accelerometer[0],accelerometer[1],accelerometer[2],
                ultrasonic_distance,
                ((float)(voltage))/47.288136 + 0.33,
                servo_rudder_measured,
                servo_left_measured,
                servo_right_measured,
                servo_rear_measured,
                esc_measured,
                gyro_gain,
                head_speed,
                data_timestamp,
                time_data_rx,
                latency);
        writeLogFile();
        */
        real_T dt = time_data_received - prev_timestamp;
//        printf("%.3f\r\n",dt);
        prev_timestamp = time_data_received;
        real_T u[6];
        real_T mean[16];
        real_T Cov[256];
        
        if (!initialise){
            u[0] = gyroscope[1]*(M_PI/180)*(1/14.375);
            u[1] = gyroscope[2]*(M_PI/180)*(1/14.375);
            u[2] = gyroscope[3]*(M_PI/180)*(1/14.375);
            u[3] = body_accel[0];
            u[4] = body_accel[1];
            u[5] = body_accel[2];
            projectStateAndCov(x, P, u, Q, dt, &static_acceleration_earth[0], &mean[0], &Cov[0]);
            memcpy(&x[0],&mean,sizeof(real_T)*16);
            memcpy(&P[0],&Cov,sizeof(real_T)*256);
            
            real_T flip[4] = {0,1,0,0};
            real_T ret_quat[4];
            quaternionRotation(&x[6], &flip[0], &ret_quat[0]);
            eulerAnglesFromQuaternion(&ret_quat[0], &roll, &pitch, &yaw);
        }
        
        // limit ascii length so it is predictable on receive size
        float trans_x = limitLengthTo5(get_tx());
        float trans_y = limitLengthTo5(get_ty());
        float trans_z = limitLengthTo5(get_tz());
        float vel_x = limitLengthTo5(get_vx());
        float vel_y = limitLengthTo5(get_vy());
        float vel_z = limitLengthTo5(get_vz());
        float roll_e = limitLengthTo5(get_roll());
        float pitch_e = limitLengthTo5(get_pitch());
        float yaw_e = limitLengthTo5(get_yaw());
        float accel_bias_x = limitLengthTo4(x[10]*10000);
        float accel_bias_y = limitLengthTo4(x[11]*10000);
        float accel_bias_z = limitLengthTo4(x[12]*10000);
        float gyro_bias_x = limitLengthTo4(x[13]*10000);
        float gyro_bias_y = limitLengthTo4(x[14]*10000);
        float gyro_bias_z = limitLengthTo4(x[15]*10000);        
        
//        // once this estimation is done, we should send filtered data back to helicopter
//        int tx_len = sprintf(tx_data,"*#%05.0f,%05.0f,%05.0f,%05.0f,%05.0f,%05.0f,%05.1f,%05.1f,%05.1f,%05.0f,%05.0f,%05.0f,%05.0f,%05.0f,%05.0f,%.2X\r\n\0",
//                trans_x,trans_y,trans_z,
//                vel_x,vel_y,vel_z,
//                roll_e,pitch_e,yaw_e,
//                accel_bias_x,accel_bias_y,accel_bias_z,
//                gyro_bias_x,gyro_bias_y,gyro_bias_z, (unsigned char)checksum);

//        x_count = limitLengthTo5(x_count+1);
//        y_count = limitLengthTo5(y_count+1);
//        z_count = limitLengthTo5(z_count+1);
//        roll_count = limitLengthTo4(roll_count+1);
//        pitch_count = limitLengthTo4(pitch_count+1);
//        yaw_count = limitLengthTo4(yaw_count+1);
        
        if ((timeSince(0) - *lastCorrection()) > 0.1)
            checksum = 0;
        else
            checksum = 1;
        
//        // once this estimation is done, we should send filtered data back to helicopter
//        int tx_len = sprintf(tx_data,"*#%05.0f,%05.0f,%05.0f,%06.1f,%06.1f,%06.1f,%.2X\r\n\0",               
//                trans_x,trans_y,trans_z,
//                roll_e,pitch_e,yaw_e,
//                (unsigned char)checksum);
                
        short int translation_int16[3] = {0};
        short int velocity_int16[3] = {0};
        float quaternion_float[4] = {0};
        
        char translation_s[3][20] = {0};
        char velocity_s[3][20] = {0};
        char quaternion_s[4][20] = {0};
        char sequence_s[20] = {0};
        char checksum_s[20] = {0};
        
        translation_int16[0] = (short int) 1111;//x[0];
        translation_int16[1] = (short int) -2222;//x[1];
        translation_int16[2] = (short int) 3333;//x[2];
        
        velocity_int16[0] = (short int) -4444;//x[3];
        velocity_int16[1] = (short int) 5555;//x[4];
        velocity_int16[2] = (short int) -6666;//x[5];        
        
        quaternion_float[0] = (float) 123e-3;//x[6];
        quaternion_float[1] = (float) -456e-4;//x[7];
        quaternion_float[2] = (float) 789e-5;//x[8];
        quaternion_float[3] = (float) -12345e-6;//x[9];

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
        
        shortInt2ASCIIHex(sequence, &sequence_s[0]);        
        
        int len = sprintf(tx_data,"*#%s%s%s%s%s%s%s%s%s%s%s",
                &translation_s[0][0], &translation_s[1][0], &translation_s[2][0],
                &velocity_s[0][0], &velocity_s[1][0], &velocity_s[2][0],
                &quaternion_s[0][0], &quaternion_s[1][0], &quaternion_s[2][0], &quaternion_s[3][0],
                &sequence_s[0]);
        
        unsigned short int checksum2 = 0;
        for (i = 0; i < len - 1; i++){
            checksum2 ^= (unsigned short int) ((unsigned char)tx_data[i] << 8) | (unsigned char)tx_data[i+1];
        }        
        
        shortInt2ASCIIHex(checksum2, &checksum_s[0]);
        int len2 = sprintf(&tx_data[len], "%s\r\n", &checksum_s[0]);
        
        sequence++;
        send(sockfd,tx_data,len+len2,0); 
         
        sprintf(write_buffer,"%d,%d,%d,%d,%d,%d,%.3f\r\n",
                gyroscope[1],gyroscope[2],gyroscope[3],
                accelerometer[0],accelerometer[1],accelerometer[2],
                time_data_rx);
        writeLogFile();         
        
        // if still capturing first 100 samples use to estimate static_acceleration_body
        if (initialise){
            if (sample++ < IMU_INIT_IER){
                static_acceleration_body[0] += body_accel[0];
                static_acceleration_body[1] += body_accel[1];
                static_acceleration_body[2] += body_accel[2];
                
                init_gyro_bias_est[0] += gyroscope[1]*(M_PI/180)*(1/14.375);
                init_gyro_bias_est[1] += gyroscope[2]*(M_PI/180)*(1/14.375);
                init_gyro_bias_est[2] += gyroscope[3]*(M_PI/180)*(1/14.375);
            }
            else{
                static_acceleration_body[0] /= IMU_INIT_IER;
                static_acceleration_body[1] /= IMU_INIT_IER;
                static_acceleration_body[2] /= IMU_INIT_IER;
                
                init_gyro_bias_est[0] /= IMU_INIT_IER;
                init_gyro_bias_est[1] /= IMU_INIT_IER;
                init_gyro_bias_est[2] /= IMU_INIT_IER;
                
                x[10] = init_gyro_bias_est[0];
                x[11] = init_gyro_bias_est[1];
                x[12] = init_gyro_bias_est[2];
                
                initialise = false;
                Reb(&x[6],&static_acceleration_body[0],&static_acceleration_earth[0]);
            }
        }
    }    
}

void UDP_socket::float2ASCIIHex(const float val, char * return_string){
    char temp[4] = {0};
    memcpy(&temp[0], &val, sizeof(float));
    
    sprintf(return_string, "%.2X%.2X%.2X%.2X", (unsigned char)temp[0]
            ,(unsigned char)temp[1]
            ,(unsigned char)temp[2]
            ,(unsigned char)temp[3]);
}

void UDP_socket::shortInt2ASCIIHex(const short int val, char * return_string){
    char temp[2] = {0};
    memcpy(&temp[0], &val, sizeof(short int));
    
    sprintf(return_string, "%.2X%.2X", (unsigned char)temp[0]
            ,(unsigned char)temp[1]);
}

float UDP_socket::limitLengthTo5(float x){
    // limit length to 5 characters
    if (x >= 100000)
        x = 99999;
    if (x <= -10000)
        x = -9999;
    return x;
}

float UDP_socket::limitLengthTo4(float x){
    // limit length to 4 characters
    if (x >= 10000)
        x = 9999;
    if (x <= -1000)
        x = -999;
    return x;
}

// get filtered states
real_T UDP_socket::get_tx(void){
    return x[0]*1000;
}

real_T UDP_socket::get_ty(void){
    return x[1]*1000;
}

real_T UDP_socket::get_tz(void){
    return x[2]*1000;
}

real_T UDP_socket::get_vx(void){
    return x[3]*1000;
}

real_T UDP_socket::get_vy(void){
    return x[4]*1000;
}

real_T UDP_socket::get_vz(void){
    return x[5]*1000;
}

real_T UDP_socket::get_roll(void){
    return roll;
}

real_T UDP_socket::get_pitch(void){
    return pitch;
}

real_T UDP_socket::get_yaw(void){
    return yaw;
}

real_T UDP_socket::get_xgyrobias(void){
    return x[10];
}

real_T UDP_socket::get_ygyrobias(void){
    return x[11];
}

real_T UDP_socket::get_zgyrobias(void){
    return x[12];
}

real_T UDP_socket::get_xaccelbias(void){
    return x[13];
}

real_T UDP_socket::get_yaccelbias(void){
    return x[14];
}

real_T UDP_socket::get_zaccelbias(void){
    return x[15];
}

void UDP_socket::setTransmitRate(float Hz){
    tx_ms_delay = 1000/Hz;
    if (tx_ms_delay > 999)
        tx_ms_delay = 999;
    cout << tx_ms_delay << endl;
}

void UDP_socket::setHelicopterTxRate(char char_tx_rate){
    tx_rate = char_tx_rate;
    cout << tx_rate << endl;
}

void UDP_socket::setAutoMode(char c){
    auto_mode = c;
}

void UDP_socket::setGPSMode(char c){  
    gps_mode = c;
}

void UDP_socket::setVisualData(int tx, int ty, int tz, 
        float q0v, float q1v, float q2v, float q3v, int cam_q){
    room_x = tx;
    room_y = ty;
    room_z = tz;
    q0 = q0v;
    q1 = q1v;
    q2 = q2v;
    q3 = q3v;
    cam_quality = cam_q;
}

bool UDP_socket::isConnected(void){
    return sockfd_connected;
}

bool UDP_socket::isTransmitting(void){
    return transmitting;
}

bool UDP_socket::isReceving(void){
    return receiving;
}

void UDP_socket::close_rx(void)
{
    if (sockrx > 0)
    {    
        close(sockrx);
        cout << "UDP: sockrx closed." << endl;
    }
}

void UDP_socket::close_tx(void)
{
    if (sockfd > 0)
    {
        close(sockfd);
        cout << "UDP: sockfd closed." << endl;
    }
}

void UDP_socket::setLEDS(bool rd, bool gr, bool bl){
    leds = 0;
    
    if (rd)
        leds |= 0b1;
    else
        leds &= 0b11111110;
    
    if (gr)
        leds |= 0b10;    
    else
        leds &= 0b11111101;
    
    if (bl)
        leds |= 0b100; 
    else
        leds &= 0b11111011;
    
}

void UDP_socket::setCamQuality(int quality)
{
    cam_quality = quality;
}

signed short int * UDP_socket::getGyroData(void){
    return gyroscope;
}

signed short int * UDP_socket::getAccelData(void){
    return accelerometer;
}

signed short int * UDP_socket::getMagnetoData(void){
    return magnetometer;
}

unsigned long int * UDP_socket::getPressureData(void){
    return &pressure;
}

unsigned short int * UDP_socket::getUltrasonicData(void){
    return &ultrasonic_distance;
}

unsigned short int * UDP_socket::getVoltageData(void){
    return &voltage;
}

unsigned short int * UDP_socket::getSpeedData(void){
    return &head_speed;
}

float * UDP_socket::getTempBaroData(void){
    return &temperature_bmp;
}

unsigned short int UDP_socket::getESC(void){
    return esc_measured;
}

unsigned short int UDP_socket::getRudder(void){
    return servo_rudder_measured;
}

unsigned short int UDP_socket::getServoLeft(void){
    return servo_left_measured;
}

unsigned short int UDP_socket::getServoRight(void){
    return servo_right_measured;
}

unsigned short int UDP_socket::getServoRear(void){
    return servo_rear_measured;
}

unsigned short int UDP_socket::getGain(void){
    return gyro_gain;
}

float UDP_socket::getCollective(void){
    return collective;
}

float UDP_socket::getLateral(void){
    return lateral;
}

float UDP_socket::getLongitudinal(void){
    return longitudinal;
}

float UDP_socket::getLatency(void){
    return latency;
}

UDP_socket::~UDP_socket() {
    disconnect_socket();
    closeLogFile();
}

