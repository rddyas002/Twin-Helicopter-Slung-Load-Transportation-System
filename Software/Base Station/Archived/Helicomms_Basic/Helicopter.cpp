/* 
 * File:   Helicopter.cpp
 * Author: Yashren Reddi
 * 
 * Created on 19 May 2014, 11:35 AM
 */

#include "Helicopter.h"

#define HELI_MULTI_DATA "225.0.0.37"
#define HELI_MULTI_PORT "12345"

using namespace std;

Helicopter::Helicopter(int id, char * ip_add, char *ip_port) {
    broadcast = NULL;
    broadcast = new Multicast(HELI_MULTI_DATA, HELI_MULTI_PORT);
    helicopter_id = id;
    initializeVariables();
    initialiseEKFvariables();
    initialiseLog();
    
    // Create UDP connection to helicopter
    makeConnection(ip_add, ip_port);
    connect_time = udp_socket->timeSince(0);
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

bool Helicopter::makeConnection(const char * ip_address, const char * ip_port){
    udp_socket = new UDP_socket(ip_address, ip_port);
    start_t0 = udp_socket->timeSince(0);
    if (!udp_socket->connect_socket()){
        delete udp_socket;
        udp_socket = NULL;
        return false;
    }
    // Link data receive signal to decode function
    data_rx_connection = udp_socket->getDataReceivedSignal()->connect(boost::bind(&Helicopter::decodeData,this,_1));
    return udp_socket->setupReceive(2002);
}

void Helicopter::initializeVariables(void){
    gyroscope[0] = gyroscope[1] = gyroscope[2] = 0;
    gyro_temperature = 0;
    accelerometer[0] = accelerometer[1] = accelerometer[2] = 0;
    magnetometer[0] = magnetometer[1] = magnetometer[2] = 0;            
    temperature_bmp = 0.0;
    ultrasonic_distance = voltage = key = 0;
    pressure = 0;
    checksum = 0;
    head_speed = 0;
    
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
    
    sequence = 0;
    
    state_propagation_on = false;
    ekf_initialising = false;
}

void Helicopter::decodeData(char rx_data[]){
    static int sample = 0;
    // this function is called only when new data arrives
    static double prev_timestamp = 0;    
    static double total_time = 0;
    static int time_counter = 0;
    
    if (prev_timestamp == 0){
        prev_timestamp = udp_socket->getLastReceiveTime();
    }
    
    char checksum = 0;
    for (int i = 2; i < 32; i++){
        checksum ^= rx_data[i];
    }    
    
    // check if complete packet has been received
    if ((rx_data[0] == '*') && (rx_data[1] == '#') && (rx_data[33] == '@') && (rx_data[34] == '!')
        && (checksum == rx_data[32])){     
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
        body_acceleration[0] = (float)accelerometer[0]*9.81/272;
        body_acceleration[1] = (float)accelerometer[1]*9.81/258;
        body_acceleration[2] = (float)accelerometer[2]*9.81/225;

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
                
        MSB = ((unsigned short int)rx_data[index++] << 8);
        LSB = rx_data[index++];
        head_speed = (unsigned short int) (MSB | LSB);    
        
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
        
        double data_timestamp = (double)temp/32768 - T0_TIME;
        double time_data_rx = udp_socket->getLastReceiveTime() - T0_TIME;
        
        latency = (time_data_rx - data_timestamp)*1000;
        
//        float right_delta = servo_right_measured - SERVO_RIGHT_BIAS;
//        float left_delta = -(servo_left_measured - SERVO_LEFT_BIAS);
//        float rear_delta = -(servo_rear_measured - SERVO_REAR_BIAS);
//        
//        float servos_avg_height = (right_delta+left_delta+rear_delta)/3;
//        float servo_lon_diff = 2*rear_delta - (left_delta + right_delta);
//        float servo_lat_diff = right_delta - left_delta;
//        collective = servos_avg_height*SERVO_AVG_HEIGHT_2_COLLECTIVE_GRAD + SERVO_AVG_HEIGHT_2_COLLECTIVE_OFFSET;
//        longitudinal = servo_lon_diff*SERVO_LONG_DIFF_2_LONG_GRAD + SERVO_LONG_DIFF_2_LONG_OFFSET;
//        lateral = servo_lat_diff*SERVO_LAT_DIFF_2_LAT_GRAD + SERVO_LAT_DIFF_2_LAT_OFFSET;
        
        // log helicopter data
        sprintf(log_buffer,"%d,%d,%d,%d,%d,%d,%u,%.2f,%d,%d,%d,%d,%d,%d,%u,%.3f,%.3f,%.3f\r\n",
                gyroscope[0],gyroscope[1],gyroscope[2],
                accelerometer[0],accelerometer[1],accelerometer[2],
                ultrasonic_distance,
                getVoltage(),
                onboard_roll,
                onboard_pitch,
                onboard_yaw,
                tx_x_position,
                tx_y_position,
                tx_z_position,
                head_speed,
                data_timestamp,
                time_data_rx,
                latency);
        writeLog();       
                               
        if (state_propagation_on){
            real_T dt_ekf = udp_socket->getLastReceiveTime() - prev_timestamp;
            prev_timestamp = udp_socket->getLastReceiveTime();
                
            total_time += dt_ekf;
            time_counter++;
            if (time_counter == 20){
                time_counter = 0;
                ekf_rate = 20/total_time;
                total_time = 0; 
            }            
            
            real_T u[6] = {(real_T)gyroscope[0],(real_T)gyroscope[1],(real_T)gyroscope[2],
                (real_T)accelerometer[0],(real_T)accelerometer[1],(real_T)accelerometer[2]};
            
            runEKF(dt_ekf, u);
                
            if (timeSinceLastCorrection() > 0.1)
                checksum = 0;
            else
                checksum = 1;
            
            short int translation_int16[3] = {0};
            short int velocity_int16[3] = {0};
            float quaternion_float[4] = {0};
        
            char translation_s[3][20] = {{0},{0}};
            char velocity_s[3][20] = {{0},{0}};
            char quaternion_s[4][20] = {{0},{0}};
            char sequence_s[20] = {0};
            char checksum_s[20] = {0};      
            
            translation_int16[0] = (short int) (x_ekf[0]*1000);
            translation_int16[1] = (short int) (x_ekf[1]*1000);
            translation_int16[2] = (short int) (x_ekf[2]*1000);
        
            velocity_int16[0] = (short int) (x_ekf[3]*1000);
            velocity_int16[1] = (short int) (x_ekf[4]*1000);
            velocity_int16[2] = (short int) (x_ekf[5]*1000);      
        
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
            sendPacket(tx_data);
        }
    }        
}

short int Helicopter::returnTruncINT16(float val){
    if (val > 32000)
        return (short int) 32000;
    if (val < -32000)
        return -(short int) 32000;
    
    return (short int) val;
}

real_T Helicopter::timeSinceLastCorrection(void){
    return udp_socket->timeSince(0) - *lastCorrection();
}

double Helicopter::timeSinceStart(void){
    return udp_socket->timeSince(0) - connect_time;
}

void Helicopter::runEKF(float dt, real_T u[6]){
    static int sample = 0;
    static double accel_body_sum[3] = {0};
    static double ang_rate_body_sum[3] = {0};
    
    real_T angular_rate_body[3] = {u[0]*GYROSCOPE_2_SI,
        u[1]*GYROSCOPE_2_SI,
        u[2]*GYROSCOPE_2_SI};        
    
    real_T accel_body[3] = {u[3]*ACCELEROMETER_X_2_SI,
        u[4]*ACCELEROMETER_Y_2_SI,
        u[5]*ACCELEROMETER_Z_2_SI};
    
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
            // initial quaternion already set by poseestimator
            
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
            // Initial quaternion set above
            // Initial gyro biases set above
            // Initially the acceleration bias should be zero because any bias
            // should be contained in static_acceleration_earth
            x_ekf[13] = 0;
            x_ekf[14] = 0;//0.0051; 
            x_ekf[15] = 0;//0.0040;
            
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
            Q_ekf[114] = accel_xy_noise_var;
            Q_ekf[133] = accel_xy_noise_var;
            Q_ekf[152] = accel_z_noise_var;
            Q_ekf[171] = Q_ekf[190] = Q_ekf[209] = gyro_noise_var;
            Q_ekf[228] = Q_ekf[247] = Q_ekf[266] = 1e-20;
            Q_ekf[285] = Q_ekf[304] = Q_ekf[323] = 1e-20;
    
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
        }                       
    }
    else{
        real_T u_ekf[6] = {angular_rate_body[0],angular_rate_body[1],angular_rate_body[2],
            accel_body[0], accel_body[1], accel_body[2]};
        real_T mean[16];
        real_T Cov[256];

        projectStateAndCov(x_ekf, P_ekf, u_ekf, Q_ekf, dt, static_acceleration_earth, &mean[0], &Cov[0]);
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
        sprintf(ekf_buffer,"%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.3f\r\n",
                x_ekf[0],x_ekf[1],x_ekf[2],
                x_ekf[3],x_ekf[4],x_ekf[5],
                x_ekf[6],x_ekf[7],x_ekf[8],x_ekf[9],
                x_ekf[10],x_ekf[11],x_ekf[12],
                x_ekf[13],x_ekf[14],x_ekf[15],
                udp_socket->getLastReceiveTime() - T0_TIME);
        writeEKFLog();           
        broadcast->sendPacket(ekf_buffer);
    }
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

real_T * Helicopter::lastCorrection(void){
    return &last_correction;
}

real_T * Helicopter::lastProjection(void){
    return &last_projection;
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

double Helicopter::getEKFRate(void){
    return ekf_rate;
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

void Helicopter::float2ASCIIHex(const float val, char * return_string){
    char temp[4] = {0};
    memcpy(&temp[0], &val, sizeof(float));
    
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

Helicopter::~Helicopter() {
    if (broadcast != NULL){
        delete broadcast;
        broadcast = NULL;
    }
    
    if (udp_socket != NULL){
        data_rx_connection.disconnect();
        delete udp_socket;
        udp_socket = NULL;
    }
    closeLog();
}

