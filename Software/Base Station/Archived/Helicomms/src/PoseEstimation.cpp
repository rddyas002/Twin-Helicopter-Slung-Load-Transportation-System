/* 
 * File:   PoseEstimation.cpp
 * Author: yashren
 * 
 * Created on 13 May 2013, 5:45 AM
 */

#include "../include/PoseEstimation.h"
#include <locale.h>
#include <stdlib.h>

PoseEstimation::PoseEstimation() {
    
    char string_buffer[256];
    
    first_run = true;
    first_time_prop = true;
    
    FPS = 0;
    total_time = 0;
    FPS_counter = 0;
    
    clearCamDataReceived();
    camera_data.all_data_received = true;

    for (int v_cam = 0; v_cam < MAX_CAMERAS; v_cam++){
        if (v_cam < MAX_ACTIVE_CAMERAS){
            // create virtual cameras
            virtual_cam[v_cam] = new OpencvCamera(-1,v_cam + 1);    // front left of room
            virtual_cam[v_cam]->setIntrinsic((CvMat*)cvLoad(get_concat_string(string_buffer, PREAMBLE_TO_INTRINSIC, v_cam + 1)));
            virtual_cam[v_cam]->setDistortion((CvMat*)cvLoad(get_concat_string(string_buffer, PREAMBLE_TO_DISTORTION, v_cam + 1)));
            virtual_cam[v_cam]->setCamPosition((CvMat*)cvLoad(get_concat_string(string_buffer, PREAMBLE_TO_POSITION, v_cam + 1)));
            virtual_cam[v_cam]->loadRotationAngles((CvMat*)cvLoad(get_concat_string(string_buffer, PREAMBLE_TO_ROTATION, v_cam + 1)));
            virtual_cam[v_cam]->setCenter((CvMat*)cvLoad(PATH_TO_CENTER));
            virtual_cam[v_cam]->setCalibObjectPoints((CvMat*)cvLoad(PATH_TO_OBJECTS)); 
            virtual_cam[v_cam]->calcExtrinsics();
        }
        else
            virtual_cam[v_cam] = NULL;
    }
    
    tcp_server = NULL;
    helicopter_link = NULL;
    
    R1.set_size(3,3);
    R1.zeros();
    
    Rx.set_size(3,3);
    Rx.zeros();
    Rx(0,0) = 1; 
    Rx(1,1) = cos(M_PI); Rx(1,2) = -sin(M_PI);
    Rx(2,1) = sin(M_PI); Rx(2,2) = cos(M_PI);
    
    roll = pitch = yaw = 0;
    h1q.q0 = h1q.q1 = h1q.q2 = h1q.q3 = 0;
    tx = ty = tz = 0;    
    
    Red_platform.set_size(2,1);
    Green_platform.set_size(2,1);
    Blue_platform.set_size(2,1);
    
    Red_platform << RED_BLOB_X << endr
                 << RED_BLOB_Y << endr;
    Green_platform << GREEN_BLOB_X << endr
                 << GREEN_BLOB_Y << endr;
    Blue_platform << BLUE_BLOB_X << endr
                 << BLUE_BLOB_Y << endr;    
    
    calc_done_time = 0;
    num_cams_used = 0;
    active_blobs = 0;
    request_timestamp = 0.0;
    
    openLogFile();
    tcp_server = new TCP_server(2011);
}

ofstream PoseEstimation::visualLog;
ofstream PoseEstimation::visualRaw;
char PoseEstimation::write_buffer[512] = {0};
char PoseEstimation::write_buffer_raw[512] = {0};

void PoseEstimation::setUDPLink(UDP_socket * udp){
    if (helicopter_link == NULL)
        helicopter_link = udp;
}

void PoseEstimation::openLogFile(void){
    visualLog.open("visualLog.txt");
    visualRaw.open("visual_Raw.txt");
}

void PoseEstimation::closeLogFile(void){
    if (visualLog.is_open())
        visualLog.close();
    
    if (visualRaw.is_open())
        visualRaw.close();    
}

bool PoseEstimation::writeLogFile(void){
    if (visualLog.is_open()){
        visualLog << write_buffer;
        return true;
    }
    else
        return false;
}

bool PoseEstimation::writeRawFile(void){
    if (visualRaw.is_open()){
        visualRaw << write_buffer_raw;
        return true;
    }
    else
        return false;
}

OpencvCamera * PoseEstimation::getVirtualCamera(int index){
        return virtual_cam[index];
}

void PoseEstimation::setRequestTime(void){
    request_timestamp = timeSince(T0_TIME);
}

double PoseEstimation::getRequestTime(void){
    return request_timestamp;
}

quaternion * PoseEstimation::calculateQuaternion(mat R){
    float r11 = R(0,0), r12 = R(0,1),r13 = R(0,2);
    float r21 = R(1,0), r22 = R(1,1),r23 = R(1,2);
    float r31 = R(2,0), r32 = R(2,1),r33 = R(2,2);
    float qd;
    if ((r22>-r33) && (r11>-r22) && (r11>-r33)){
        qd = sqrt(1+r11+r22+r33);
        h1q.q0 = 0.5*qd;
        h1q.q1 = 0.5*(r23 - r32)/qd;
        h1q.q2 = 0.5*(r31 - r13)/qd;
        h1q.q3 = 0.5*(r12 - r21)/qd;
    }

    if ((r22<-r33) && (r11>r22) && (r11>r33)){    
        qd = sqrt(1+r11-r22-r33);
        h1q.q1 = 0.5*qd;
        h1q.q0 = 0.5*(r23 - r32)/qd;
        h1q.q2 = 0.5*(r12 + r21)/qd;
        h1q.q3 = 0.5*(r31 + r13)/qd;
    }

    if ((r22>r33) && (r11<r22) && (r11<-r33)){
        qd = sqrt(1-r11+r22-r33);
        h1q.q2 = 0.5*qd;
        h1q.q0 = 0.5*(r31 - r13)/qd;
        h1q.q1 = 0.5*(r12 + r21)/qd;
        h1q.q3 = 0.5*(r23 + r32)/qd;
    }

    if ((r22<r33) && (r11<-r22) && (r11<r33)){
        qd = sqrt(1-r11-r22+r33);
        h1q.q3 = 0.5*qd;
        h1q.q0 = 0.5*(r12 - r21)/qd;
        h1q.q1 = 0.5*(r31 + r13)/qd;
        h1q.q2 = 0.5*(r23 + r32)/qd;
    }   
    
    // Normalize quaternion
    float norm_q = sqrt(pow(h1q.q0,2)+pow(h1q.q1,2)+pow(h1q.q2,2)+pow(h1q.q3,2));    
    h1q.q0 /= -norm_q;
    h1q.q1 /= -norm_q;
    h1q.q2 /= -norm_q;
    h1q.q3 /= -norm_q;
   
    // This quaternion represents the rotation between helicopter body (classical)
    // and earth frame
    
    // complement quat for body to earth
    float q0 = h1q.q0;
    float q1 = h1q.q1;
    float q2 = h1q.q2;
    float q3 = h1q.q3;
    roll = -atan2(2*q2*q3+2*q0*q1,pow(q3,2)-pow(q2,2)-pow(q1,2)+pow(q0,2));
    if (roll > 0)
        roll -= M_PI;
    else
        roll += M_PI;
    pitch = asin(2*q1*q3-2*q0*q2);
    yaw = -atan2(2*q1*q2+2*q0*q3,pow(q1,2)+pow(q0,2)-pow(q3,2)-pow(q2,2));   
           
//    cout << "Roll: " << roll*180/M_PI << " Pitch: " << pitch*180/M_PI << " Yaw: " << yaw*180/M_PI << endl;
    
    return &h1q;
}

quaternion PoseEstimation::getQuaternion(void){
    return h1q;
}

void PoseEstimation::loadData(char * data){
    char data_cpy[256]={0};
    double blobs[21]={0}; 
    
    // find start character *
    int i = 0, start_char = 0, end_char = 0;
    while(i < 256){
        if (data[i] == '*')
        {
            start_char = i;
            break;
        }
        i++;
    }
    
    int rem = 255 - i;
    while(i < rem){
        if (data[i] == '#')
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
    
    memcpy(data_cpy, &data[start_char + 1], (end_char - start_char + 2));

    int num_red = 0, num_green = 0, num_blue = 0;
    int client_num = 0;
    char temp_buf[64];
    int temp_len;
    
    setlocale(LC_ALL,"C");
    
    // First extract num of blobs
    int scn_ret = sscanf(data_cpy, "C%d:%d,%d,%d", &client_num, &num_red, &num_green, &num_blue);  
    if (scn_ret != 4)
        std::cout << "scan err" << std::endl;
    
    
    std::cout << "CLIENT:" << client_num << "started." << std::endl;   
    
    int sptf_ret = sprintf(temp_buf, "C%d:%d,%d,%d", client_num, num_red, num_green, num_blue);
    
    temp_len = strlen(temp_buf);
    
    char * temp_char = (char *) malloc(sizeof(char) * 40);
    temp_char = strtok(&data_cpy[temp_len], "|");

    blobs[0] = atof(temp_char);
    i = 1;
    
    while((temp_char = strtok(NULL, "|"))){
        blobs[i++] = atof(temp_char);
    }
            
    // now update blob structs
    virtual_cam[client_num - 1]->blob_data.red_blobs = num_red;
    virtual_cam[client_num - 1]->blob_data.green_blobs = num_green;
    virtual_cam[client_num - 1]->blob_data.blue_blobs = num_blue;

    int j = 0;
    for (i = 0; i < num_red; i++){
        virtual_cam[client_num - 1]->blob_data.red[i].x = blobs[j++];
        virtual_cam[client_num - 1]->blob_data.red[i].y = blobs[j++];
    }
    
    for (i = 0; i < num_green; i++){
        virtual_cam[client_num - 1]->blob_data.green[i].x = blobs[j++];
        virtual_cam[client_num - 1]->blob_data.green[i].y = blobs[j++];
    }    
    
    for (i = 0; i < num_blue; i++){
        virtual_cam[client_num - 1]->blob_data.blue[i].x = blobs[j++];
        virtual_cam[client_num - 1]->blob_data.blue[i].y = blobs[j++];
    }        
    
    if (num_red || num_green || num_blue)
    {
        virtual_cam[client_num - 1]->blob_data.timestamp = blobs[j];
    }
    else
    {
        virtual_cam[client_num - 1]->blob_data.timestamp = blobs[0];
    }
    
    if (virtual_cam[client_num - 1]->blob_data.timestamp == 0)
        std::cout << "Trouble.." << std::endl;
    
    virtual_cam[client_num - 1]->blob_data.updated = true;
    
    int string_length = sprintf(write_buffer,"%s",data);
    sprintf(&(write_buffer[string_length-2]),"%.3f\r\n",request_timestamp);
    //cout << write_buffer << endl;
    writeLogFile();    
    
//    if(setCamDataReceived(client_num))
//    {
//        doEstimation();
//        clearCamDataReceived();
//    }
    
    std::cout << "CLIENT:" << client_num << "ended." << std::endl;    
    
    free(temp_char);
}

void PoseEstimation::clearCamDataReceived(void){
    memset(&camera_data,0,sizeof(cam_data_struct));
    camera_data.all_data_received = true;
}

void PoseEstimation::setAlldataReceived(bool val){
    camera_data.all_data_received = val;
}

void PoseEstimation::setCamDataReceived(int val){
    static double prev_correction_timestamp = 0;
    int num_red = 0, num_green = 0, num_blue = 0;
    
    if (camera_data.camera_data_received[val - 1] == true)
        std::cout << "scan error" << std::endl;
    camera_data.camera_data_received[val - 1] = true;
    
    int j = 0;
    for (int i = 0; i < MAX_CAMERAS; i++){
        if(camera_data.camera_data_received[i]){
            j++;
        }
    }
    
    if (j == MAX_CAMERAS){
        // Send dummy packet to helicopter to log identification time       
        // do correction here
        b_struct_T z[4];
        
        double curr_timestamp = timeSince(T0_TIME);
        double del = curr_timestamp - prev_correction_timestamp;
        prev_correction_timestamp = curr_timestamp;
        
        total_time += del;
        if (++FPS_counter == 10){
            FPS = (double)10/total_time;
            FPS_counter = 0;
            total_time = 0;
        }       
       
        for (int i = 0; i < 4; i++){
            setCamParam(&z[i], i);
            z[i].number_red = virtual_cam[i]->blob_data.red_blobs;
            z[i].number_green = virtual_cam[i]->blob_data.green_blobs;
            z[i].number_blue = virtual_cam[i]->blob_data.blue_blobs;
            
            num_red   += z[i].number_red;
            num_green += z[i].number_green;
            num_blue  += z[i].number_blue;
            
            for (int k = 0; k < z[i].number_red; k++){
                z[i].red_data[k] = virtual_cam[i]->blob_data.red[k].x;
                z[i].red_data[5+k] = virtual_cam[i]->blob_data.red[k].y;                
            }
            
            for (int k = 0; k < z[i].number_green; k++){
                z[i].green_data[k] = virtual_cam[i]->blob_data.green[k].x;
                z[i].green_data[5+k] = virtual_cam[i]->blob_data.green[k].y;                
            }

            for (int k = 0; k < z[i].number_blue; k++){
                z[i].blue_data[k] = virtual_cam[i]->blob_data.blue[k].x;
                z[i].blue_data[5+k] = virtual_cam[i]->blob_data.blue[k].y;                
            }            
            
            z[i].time = virtual_cam[i]->blob_data.timestamp;
        }
        
        real_T x[16];
        real_T P[256];
        
        memcpy(&x[0],helicopter_link->kalmanState(),sizeof(real_T)*16);
        memcpy(&P[0],helicopter_link->kalmanErrorCovariance(),sizeof(real_T)*256);
        
        real_T objectPoints[9] = {-140,264,0,-144,-263,0,314,0,0};
        real_T meas_noise = 3; // pixels
        real_T xCorr[16];
        real_T PCorr[256];
        
        correctStateAndCov(&x[0], &z[0], &P[0], &objectPoints[0], meas_noise, &xCorr[0], &PCorr[0]);

        sprintf(write_buffer,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%f\r\n",
                x[0],x[1],x[2],
                x[3],x[4],x[5],
                x[6],x[7],x[8],x[9],
                x[10],x[11],x[12],
                x[13],x[14],x[15],timeSince(T0_TIME));
        writeLogFile();
        
        memcpy(helicopter_link->kalmanState(),&xCorr[0],sizeof(double)*16);
        memcpy(helicopter_link->kalmanErrorCovariance(),&PCorr[0],sizeof(double)*256);
                
        clearCamDataReceived();

        active_blobs = num_red + num_green + num_blue;
        if ((num_red > 1) && (num_green > 1) && (num_blue > 1)){
            if (active_blobs > 6){
                if (P[34] < 0.2)
                    *helicopter_link->lastCorrection() = timeSince(0);
//                else
//                    printf("[%f],C1[%.0f,%.0f,%.0f],C2[%.0f,%.0f,%.0f],C3[%.0f,%.0f,%.0f],C4[%.0f,%.0f,%.0f]\r\n",
//                            P[34],
//                            z[0].number_red,z[0].number_green,z[0].number_blue,
//                            z[1].number_red,z[1].number_green,z[1].number_blue,
//                            z[2].number_red,z[2].number_green,z[2].number_blue,
//                            z[3].number_red,z[3].number_green,z[3].number_blue);
            }
        }
    }        
}

void PoseEstimation::setCamParam(b_struct_T * tmp_z, int index){
    mat R = virtual_cam[index]->getRArm();
    mat T = virtual_cam[index]->getTArm();
    mat A = virtual_cam[index]->getIntrinsicArm();    
    
    tmp_z->cam_param.translation_w2c[0] = T(0);
    tmp_z->cam_param.translation_w2c[1] = T(1);
    tmp_z->cam_param.translation_w2c[2] = T(2);
    
    tmp_z->cam_param.intrinsic_mat[0] = A(0,0);
    tmp_z->cam_param.intrinsic_mat[1] = A(1,0);
    tmp_z->cam_param.intrinsic_mat[2] = A(2,0);
    tmp_z->cam_param.intrinsic_mat[3] = A(0,1);
    tmp_z->cam_param.intrinsic_mat[4] = A(1,1);
    tmp_z->cam_param.intrinsic_mat[5] = A(2,1);
    tmp_z->cam_param.intrinsic_mat[6] = A(0,2);
    tmp_z->cam_param.intrinsic_mat[7] = A(1,2);
    tmp_z->cam_param.intrinsic_mat[8] = A(2,2);

    tmp_z->cam_param.rotation_w2c[0] = R(0,0);
    tmp_z->cam_param.rotation_w2c[1] = R(1,0);
    tmp_z->cam_param.rotation_w2c[2] = R(2,0);
    tmp_z->cam_param.rotation_w2c[3] = R(0,1);
    tmp_z->cam_param.rotation_w2c[4] = R(1,1);
    tmp_z->cam_param.rotation_w2c[5] = R(2,1);
    tmp_z->cam_param.rotation_w2c[6] = R(0,2);
    tmp_z->cam_param.rotation_w2c[7] = R(1,2);
    tmp_z->cam_param.rotation_w2c[8] = R(2,2);
}

int PoseEstimation::getNumOfCamerasUsed(void){
    return num_cams_used;
}

int PoseEstimation::getActiveBlobs(void){
    return active_blobs;
}

bool PoseEstimation::getAllDataReceived(void){
    return camera_data.all_data_received;
}

bool PoseEstimation::propagateMeasurements(OpencvCamera * v_cam, double t_current){
    if (first_time_prop){
        v_cam->prev_blob_data = v_cam->blob_data;
        
        first_time_prop = false;
        return false;
    }
    else
    {

        return true;
    }
}

float PoseEstimation::imagePointGrad(float pc_prev, float pc_prev2, float delta_t){
    // pc_prev is the last captured point
    // pc_prev2 is the point captured before that
    // delta_t is the time between the two points
    return ((pc_prev - pc_prev2)/delta_t);
}

float PoseEstimation::propPoint(float grad, float propTime, float prev_point){
    return 0;
}

bool PoseEstimation::doEstimation(void){
    int i = 0;

    if (first_run){
        prev_timestamp = timeSince(T0_TIME);
        
        first_run = false;
    }
    else
    {
        double curr_timestamp = timeSince(T0_TIME);
        double del = curr_timestamp - prev_timestamp;
        prev_timestamp = curr_timestamp;
        
        total_time += del;
        if (++FPS_counter == 10){
            FPS = (double)10/total_time;
            FPS_counter = 0;
            total_time = 0;
        }
    }    
    
    mat Pimage_red, Pimage_green, Pimage_blue;
    Pimage_red.set_size(3,MAX_CAMERAS);
    Pimage_green.set_size(3,MAX_CAMERAS);
    Pimage_blue.set_size(3,MAX_CAMERAS);
    Pimage_red.zeros();
    Pimage_green.zeros();
    Pimage_blue.zeros();
    
    bool valid_cam_data[MAX_CAMERAS] = {false};    
    
    for (i = 0; i < MAX_CAMERAS; i++){
        if (virtual_cam[i] != NULL){
            vec temp;
            
            // take the first blob only here
            
            // Fill in red blobs
            temp << virtual_cam[i]->blob_data.red[0].x << endr
                 << virtual_cam[i]->blob_data.red[0].y << endr
                 << 1 << endr;   
            Pimage_red.col(i) = temp;
            
            // Fill in green blobs
            temp << virtual_cam[i]->blob_data.green[0].x << endr
                 << virtual_cam[i]->blob_data.green[0].y << endr
                 << 1 << endr;  
            Pimage_green.col(i) = temp;
            
            // Fill in blue blobs
            temp << virtual_cam[i]->blob_data.blue[0].x << endr
                 << virtual_cam[i]->blob_data.blue[0].y << endr
                 << 1 << endr;  
            Pimage_blue.col(i) = temp;   
            
            if ((virtual_cam[i]->blob_data.red_blobs == 1) &&
                (virtual_cam[i]->blob_data.green_blobs == 1) &&
                (virtual_cam[i]->blob_data.blue_blobs == 1)){
                valid_cam_data[i] = true;
            }
        }
    }
        
    mat Trans, A_red, A_green, A_blue;
    cube Rot_t, Intr_inv;
    mat A_block; mat a_temp;
    vec B_block; vec b_temp;
        
    Trans.set_size(3,MAX_CAMERAS); Trans.zeros();
    A_red.set_size(3,MAX_CAMERAS); A_red.zeros();
    A_green.set_size(3,MAX_CAMERAS); A_green.zeros();
    A_blue.set_size(3,MAX_CAMERAS); A_blue.zeros();
    Rot_t.set_size(3,3,MAX_CAMERAS); Rot_t.zeros();
    Intr_inv.set_size(3,3,MAX_CAMERAS); Intr_inv.zeros();
        
    int j = 0;
    for (i = 0; i < MAX_CAMERAS; i++){
        if (valid_cam_data[i]){
            Trans.col(i) = virtual_cam[i]->getTArm();
            Rot_t.slice(i) = virtual_cam[i]->getR_transposedArm();
            Intr_inv.slice(i) = virtual_cam[i]->getIntrinsicInverseArm();
                
            A_red.col(i) = Rot_t.slice(i)*Intr_inv.slice(i)*Pimage_red.col(i);
            A_green.col(i) = Rot_t.slice(i)*Intr_inv.slice(i)*Pimage_green.col(i);
            A_blue.col(i) = Rot_t.slice(i)*Intr_inv.slice(i)*Pimage_blue.col(i);
                
            double a_temp0 = -A_red(0,i)/A_red(2,i);
            double a_temp1 = -A_red(1,i)/A_red(2,i);
            double a_temp2 = -A_green(0,i)/A_green(2,i);
            double a_temp3 = -A_green(1,i)/A_green(2,i); 
            double a_temp4 = -A_blue(0,i)/A_blue(2,i);
            double a_temp5 = -A_blue(1,i)/A_blue(2,i);                 
                
            a_temp << Red_platform(0) << Red_platform(1) << 0 
                << 0 << Red_platform(0)*a_temp0 << Red_platform(1)*a_temp0
                << 1 << 0 << a_temp0 << endr
                << 0 << 0 << Red_platform(0) 
                << Red_platform(1) << Red_platform(0)*a_temp1 << Red_platform(1)*a_temp1
                << 0 << 1 << a_temp1 << endr
                << Green_platform(0) << Green_platform(1) << 0 
                << 0 << Green_platform(0)*a_temp2 << Green_platform(1)*a_temp2
                << 1 << 0 << a_temp2 << endr
                << 0 << 0 << Green_platform(0) 
                << Green_platform(1) << Green_platform(0)*a_temp3 << Green_platform(1)*a_temp3
                << 0 << 1 << a_temp3 << endr
                << Blue_platform(0) << Blue_platform(1) << 0 
                << 0 << Blue_platform(0)*a_temp4 << Blue_platform(1)*a_temp4
                << 1 << 0 << a_temp4 << endr
                << 0 << 0 << Blue_platform(0) 
                << Blue_platform(1) << Blue_platform(0)*a_temp5 << Blue_platform(1)*a_temp5
                << 0 << 1 << a_temp5 << endr; 
                
            b_temp << Trans(0,i) - Trans(2,i)*-a_temp0 << endr
                << Trans(1,i) - Trans(2,i)*-a_temp1 << endr
                << Trans(0,i) - Trans(2,i)*-a_temp2 << endr
                << Trans(1,i) - Trans(2,i)*-a_temp3 << endr
                << Trans(0,i) - Trans(2,i)*-a_temp4 << endr
                << Trans(1,i) - Trans(2,i)*-a_temp5 << endr;
             
            if (j == 0){
                A_block = a_temp;
                B_block = b_temp;
            }
            else
            {
                A_block = join_cols(A_block, a_temp);
                B_block = join_cols(B_block, b_temp);
            }
                
//            std::cout << "Cam " << i + 1 << " used" << std::endl;
            
            j++;  // holds the number of actual blob equations we got to work with
        }
    }
    
    num_cams_used = j;
    
    // if there are two or more valid camera data available, then solve
    if (j > 1){
//        std::cout << j << std::endl;
        mat solx;
        
        if(solve(solx,A_block,B_block))
        {
            storeRotandTrans(solx);
            calc_done_time = timeSince(T0_TIME);
            data_ready(this);
            return 100;
        }
        else
        {
            cout << "No solution found." << endl;
            return 0;
        }  
    }
    else
        return 0;
}

double PoseEstimation::getCalcDoneTime(void){
    return calc_done_time;
}

void PoseEstimation::storeRotandTrans(mat x){
    mat x1, x2, x3;
    
    x1.set_size(3,1);x2.set_size(3,1);x3.set_size(3,1);
    x1(0,0) = x(0,0);
    x1(1,0) = x(2,0);
    x1(2,0) = x(4,0);
    
    x2(0,0) = x(1,0);
    x2(1,0) = x(3,0);
    x2(2,0) = x(5,0);

    x3 = cross(x1,x2);
    
    R1(0,0) = x(0,0); R1(1,0) = x(2,0); R1(2,0) = x(4,0);
    R1(0,1) = x(1,0); R1(1,1) = x(3,0); R1(2,1) = x(5,0);
    R1(0,2) = x3(0,0); R1(1,2) = x3(1,0); R1(2,2) = x3(2,0);
    
    // get R into the classical body frame i.e. z down
    R1 = R1*Rx;
    
    calculateQuaternion(R1);
    
//    yaw = atan2(-x(1,0),x(0));
//    pitch = asin(x3(0,0));
//    roll = atan2(-x3(1,0),x3(2,0));
    tx = x(6,0);
    ty = x(7,0);
    tz = x(8,0);
       
//    char string_buff[64];
//    sprintf(string_buff,"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
//            tx,ty,tz,roll,pitch,yaw);
//    tcp_server->write2allClients(string_buff);
//    std::cout << "Translation: " << tx << ", " << ty << ", " << tz << std::endl;
}

char * PoseEstimation::get_concat_string(char * buffer, const char * pream, int id){
    char ascii_terminal[10];
    sprintf(ascii_terminal,"%d.xml",id);
    sprintf(buffer,PATH_TO_SERVER_MOUNT);
    strcat(buffer,pream);
    strcat(buffer,ascii_terminal);
    return buffer;
}

double PoseEstimation::getFPS(void){
    return FPS;
}

void PoseEstimation::debugMessage(const char * str){
    printf("DBG [PoseEstimation] >> %s\r\n", str);
}

void PoseEstimation::errorMessage(const char * str){
    printf("ERR [PoseEstimation] >> %s\r\n", str);
}
/* 
 * return functions
 **************************************************
 ************************************************** 
 */

double PoseEstimation::getTrans_x(void){
    return tx;
}

double PoseEstimation::getTrans_y(void){
    return ty;
}

double PoseEstimation::getTrans_z(void){
    return tz;
}

double PoseEstimation::get_vx(void){
    return vx;
}

double PoseEstimation::get_vy(void){
    return vy;
}

double PoseEstimation::get_vz(void){
    return vz;
}

double PoseEstimation::getRoll(void){
    return (roll*180/M_PI);
}

double PoseEstimation::getPitch(void){
    return (pitch*180/M_PI);
}

double PoseEstimation::getYaw(void){
    return (yaw*180/M_PI);
}

PoseEstimation::~PoseEstimation() {
    closeLogFile();
    
    for (int i = 0; i < MAX_CAMERAS; i++){
        if (virtual_cam[i] != NULL)
            delete virtual_cam[i];
    }    
    
    if (tcp_server != NULL){
        delete tcp_server;
        tcp_server = NULL;
    }
}

