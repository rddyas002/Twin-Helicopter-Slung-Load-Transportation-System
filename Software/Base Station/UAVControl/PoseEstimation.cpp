
#include "PoseEstimation.h"
#include "solvePoseEst.h"
#include <locale.h>
#include <stdlib.h>

PoseEstimation::PoseEstimation(int num_active, Helicopter * helicopter_ref[MAX_NUM_OF_HELICOPTERS]){   
    int i = 0;
    
    udp_lsq = NULL;
    active_helicopters = num_active;
    
    // make internal reference copy of helicopters
    // Inactive objects should be set to null by parent
    for (i = 0; i < MAX_NUM_OF_HELICOPTERS; i++){
        helicopter[i] = helicopter_ref[i];
        try_find[i] = 0;
    }     
    
    // Set initialisation mode
    for (i = 0; i < active_helicopters; i++){
        helicopter[i]->initialiseOffsetVariables();        
        helicopter[i]->setInitialisingPose(true);
    }         

    stats_count = 0;
    fps_sum = 0.0;
    proc_time_sum = 0.0;  
    fps_first_enter = false;    
    delta_first_enter = false;    
    ekf_on = false;
    FPS = 0;
    process_time = 0;
    initialising = true;    
    current_state = PE_INITIALISING;
    
    clearCamDataReceived();

    if (pthread_mutex_init(&set_cam_mutex, NULL) != 0){
        std::cout << "Mutex initialisation failed" << std::endl;
    }

    char string_buffer[256];
    // create virtual cameras
    for (int v_cam = 0; v_cam < NUM_OF_CAMERAS; v_cam++){
        if (v_cam < NUM_OF_CAMERAS){
            virtual_cam[v_cam] = new OpencvCamera(-1,v_cam + 1);    
            virtual_cam[v_cam]->setIntrinsic((CvMat*)cvLoad(get_concat_string(string_buffer, PREAMBLE_TO_INTRINSIC, v_cam + 1)));
            virtual_cam[v_cam]->setDistortion((CvMat*)cvLoad(get_concat_string(string_buffer, PREAMBLE_TO_DISTORTION, v_cam + 1)));
            virtual_cam[v_cam]->setCamPosition((CvMat*)cvLoad(get_concat_string(string_buffer, PREAMBLE_TO_POSITION, v_cam + 1)));
            virtual_cam[v_cam]->loadRotationAngles((CvMat*)cvLoad(get_concat_string(string_buffer, PREAMBLE_TO_ROTATION, v_cam + 1)));
            virtual_cam[v_cam]->setCenter((CvMat*)cvLoad(PATH_TO_CENTER));
            virtual_cam[v_cam]->setCalibObjectPoints((CvMat*)cvLoad(PATH_TO_OBJECTS)); 
            virtual_cam[v_cam]->calcExtrinsics();
        }
        else{
            virtual_cam[v_cam] = NULL;
            errorMessage("Creating virtual camera");
        }
    }
    initialiseLog();
    makeUDP();
}

bool PoseEstimation::makeUDP(void){
    // only send to 1 pc, use helicopter 1 data as default
    udp_lsq = new UDP_socket(10, helicopter[0]->getHelicopterInfo()->ip_address_udp,  helicopter[0]->getHelicopterInfo()->ip_port_udp, 0);

    if (!udp_lsq->connect_socket()){
        delete udp_lsq;
        udp_lsq = NULL;
        return false;
    }
    return true;
}

void PoseEstimation::initialiseLog(void){
    struct tm * sTm;
    char file_buffer[128];
    char time_buffer[128];
    time_t now = time(0);
    sTm = localtime(&now);
    strftime(time_buffer,sizeof(time_buffer),"%Y-%m-%d_%H-%M", sTm);
    
    sprintf(file_buffer,"log/lsq_data_%s.txt", time_buffer);
    openLog(&file_buffer[0]);
}

void PoseEstimation::openLog(char * file_name){
    lsq_log.open(file_name);
}

void PoseEstimation::closeLog(void){
    if (lsq_log.is_open())
        lsq_log.close();
}

bool PoseEstimation::writeLog(void){
    if (lsq_log.is_open()){
        lsq_log << log_buffer;
        return true;
    }
    else
        return false;
}

double PoseEstimation::FPSCalc(void){
    struct timespec now;
    
    if (!fps_first_enter){
        fps_first_enter = true;
        clock_gettime(CLOCK_MONOTONIC_RAW, &fps_last);
        return 0.0;
    }
    else{
        clock_gettime(CLOCK_MONOTONIC_RAW, &now);
        double time_elaps = (now.tv_sec - fps_last.tv_sec) + (now.tv_nsec - fps_last.tv_nsec)/1e9;
        memcpy(&fps_last, &now, sizeof(timespec));
        if (time_elaps != 0)
            return 1/time_elaps;
        else
            return 0;
    }
}

double PoseEstimation::dtCalc(void){
    struct timespec now;
    
    if (!dtcalc_first_enter){
        dtcalc_first_enter = true;
        clock_gettime(CLOCK_MONOTONIC_RAW, &fps_last);
        return 20e-3;
    }
    else{
        clock_gettime(CLOCK_MONOTONIC_RAW, &now);
        double time_elaps = (now.tv_sec - dtcalc_last.tv_sec) + (now.tv_nsec - dtcalc_last.tv_nsec)/1e9;
        memcpy(&dtcalc_last, &now, sizeof(timespec));
        return time_elaps;
    }
}

double PoseEstimation::deltaTime(void){
    struct timespec now;
    
    if (!delta_first_enter){
        delta_first_enter = true;
        clock_gettime(CLOCK_MONOTONIC_RAW, &delta_last);
        return 0.0;
    }
    else{
        clock_gettime(CLOCK_MONOTONIC_RAW, &now);
        double time_elaps = (now.tv_sec - delta_last.tv_sec)*1e6 + (now.tv_nsec - delta_last.tv_nsec)/1e3;
        memcpy(&delta_last, &now, sizeof(timespec));
        return time_elaps;
    }
}

OpencvCamera * PoseEstimation::getVirtualCamera(int index){
        return virtual_cam[index];
}

void PoseEstimation::clearCamDataReceived(void){
    for (int i = 0; i < NUM_OF_CAMERAS; i++){
        dataFromCamReceived[i] = false;
    }    
}

bool PoseEstimation::allCamClear(void){
    for (int i = 0; i < NUM_OF_CAMERAS; i++){
        if(dataFromCamReceived[i])
            return false;
    }    
    
    return true;
}

bool PoseEstimation::lsqnon_Estimation(real_T init_x[7], const b_struct_T z[4], const real_T obj_pnts[9], real_T ret_x[7], real_T * resnorm){    
    for (int i = 0; i < LSQ_NON_ITERATONS; i++){
        solvePoseEst(init_x, z, obj_pnts, &ret_x[0], resnorm);
        memcpy(&init_x[0], &ret_x[0], sizeof(real_T)*7);
    }
    
    if ((double)*resnorm < RESNORM_MIN){
        return true;
    }
//    else{
//        real_T roll = 0, pitch = 0, yaw = 0;
//        real_T flip[4] = {0,1,0,0};
//        real_T ret_quat[4];
//        real_T quat[4] = {init_x[3],init_x[4],init_x[5],init_x[6]};        
//        quaternionRotation(&quat[0], &flip[0], &ret_quat[0]);
//        eulerAnglesFromQuaternion(&ret_quat[0], &roll, &pitch, &yaw);
//        printf("Res: %05.1f x: %04.0f y: %04.0f z: %04.0f r: %05.2f p: %05.2f y: %05.2f     \r", 
//                (double)(*resnorm), 
//                init_x[0]*1000, init_x[1]*1000, init_x[2]*1000,
//                roll, pitch, yaw);
//    }
    
    return false;
}

bool PoseEstimation::allDataReceived(void){
    pthread_mutex_lock(&set_cam_mutex);
    for (int i = 0; i < NUM_OF_CAMERAS; i++){
        if(!dataFromCamReceived[i]){
            pthread_mutex_unlock(&set_cam_mutex);
            return false;
        }
    }    
    return true;
    pthread_mutex_unlock(&set_cam_mutex);
}

void PoseEstimation::setCamDataReceived(int val){     
    int i = 0;
    pthread_mutex_lock(&set_cam_mutex);
    if (dataFromCamReceived[val - 1] == true)
        std::cout << "scan error" << std::endl;
    
    dataFromCamReceived[val - 1] = true;
    
    for (i = 0; i < NUM_OF_CAMERAS; i++){
        if(!dataFromCamReceived[i]){
            pthread_mutex_unlock(&set_cam_mutex);
            return;
        }
    }
    
    if (ekf_on){
//        b_struct_T z[4] = {{0}};
//        // copy camera data to structs
//        fillCamStruct(&z[0]);        
//        // calculate distance between blobs of same color
//        for (i = 0; i < 4; i++){
//            if (z[i].number_red == 2){
//                real_T red1[2] = {z[i].red_data[0], z[i].red_data[5]};
//                real_T red2[2] = {z[i].red_data[1], z[i].red_data[6]};
//                real_T dist1 = pixelDistance(red1, red2);
//                cout << "red_dist " << dist1 << endl;
//            }
//            if (z[i].number_green == 2){
//                real_T green1[2] = {z[i].green_data[0], z[i].green_data[5]};
//                real_T green2[2] = {z[i].green_data[1], z[i].green_data[6]};
//                real_T dist2 = pixelDistance(green1, green2);
//                cout << "green_dist " << dist2 << endl;
//            }
//            if (z[i].number_blue == 2){
//                real_T blue1[2] = {z[i].blue_data[0], z[i].blue_data[5]};
//                real_T blue2[2] = {z[i].blue_data[1], z[i].blue_data[6]};
//                real_T dist3 = pixelDistance(blue1, blue2);
//                cout << "blue_dist " << dist3 << endl;
//            }            
//        }
//        
        
        doCorrection();
    }
    else{
        bool helicopters_initialised = true;
        // Check if all helicopters have been initialised
        for (i = 0; i < active_helicopters; i++){
            helicopters_initialised = helicopters_initialised && !helicopter[i]->getInitialisingPose();
            // If helicopter has not been initialised then try again
            if (!helicopters_initialised){
                if (!initialiseHelicopter(helicopter[i])){
                    // if this instance of initialisation has failed accumulate for timeout
                    if (try_find[i]++ > INIT_QUAT_EST_ITER*3){
                        cout << "Tried to find Helicopter " << i + 1 << " " << try_find[i] << " times. Not in scene or residual too large." << endl;
                        current_state = PE_TIMEOUT;
                    }                    
                }
            }
        }       
        if (helicopters_initialised){
            initialising = false;
            if (helicopter[0]->getType() == ARNOQUAD){
                current_state = PE_LSQ_MODE;
                LSQEstimation(helicopter[0]);
                
                char tx_data[30] = {0};
                short int position_x = (short int)(*helicopter[0]->mocapPositionX()*1000);
                short int position_y = (short int)(*helicopter[0]->mocapPositionY()*1000);
                short int position_z = (short int)(*helicopter[0]->mocapPositionZ()*1000);
                short int q0 = (short int)(*helicopter[0]->mocapq0()*32000);
                short int q1 = (short int)(*helicopter[0]->mocapq1()*32000);
                short int q2 = (short int)(*helicopter[0]->mocapq2()*32000);
                short int q3 = (short int)(*helicopter[0]->mocapq3()*32000);
                unsigned int time_now = helicopter[0]->timeNow();
                tx_data[0] = '*';
                tx_data[1] = '#';
                tx_data[2] = (char)(position_x >> 8);
                tx_data[3] = (char) (position_x & 0xFF);
                tx_data[4] = (char)(position_y >> 8);
                tx_data[5] = (char) (position_y & 0xFF);
                tx_data[6] = (char)(position_z >> 8);
                tx_data[7] = (char) (position_z & 0xFF);                
                
                tx_data[8] = (char)(q0 >> 8);
                tx_data[9] = (char) (q0 & 0xFF);
                tx_data[10] = (char)(q1 >> 8);
                tx_data[11] = (char) (q1 & 0xFF);
                tx_data[12] = (char)(q2 >> 8);
                tx_data[13] = (char) (q2 & 0xFF);                
                tx_data[14] = (char)(q3 >> 8);
                tx_data[15] = (char) (q3 & 0xFF);                                
                
                tx_data[16] = (char)((time_now >> 24)& 0xFF);
                tx_data[17] = (char)((time_now >> 16)& 0xFF);
                tx_data[18] = (char)((time_now >> 8)& 0xFF);
                tx_data[19] = (char) (time_now & 0xFF);                                
                        
                char checksum2 = 0;
                for (int i = 0; i < 20; i++){
                    checksum2 ^= tx_data[i];
                }        
                tx_data[20] = checksum2;
                tx_data[21] = '\r';
                udp_lsq->sendPacketLen(tx_data,21);                
            }
            else
                current_state = PE_RUNNING;
        }
    }
        
    clearCamDataReceived();
    pthread_mutex_unlock(&set_cam_mutex);
    
    if (current_state != PE_TIMEOUT){
        // request for more data
        correction_done();    
    }
}

real_T PoseEstimation::estimateHeliPose(Helicopter * helicopter, real_T pose_est[7]){
    real_T x_heli[16];       
    // make a copy of the state and state covariance for each helicopter
    memcpy(&x_heli[0], helicopter->kalmanState(), sizeof(real_T)*16);
     
    // form pose array for each helicopter which is used to estimate most likely measurements
    real_T heli_pose[7] = {x_heli[0], x_heli[1], x_heli[2], x_heli[6], x_heli[7], x_heli[8], x_heli[9]};
    real_T norm = 0;    
    
    b_struct_T z[4];
    fillCamStruct(&z[0]);   
    
    // condition structure so correct blobs are used
    blobAllocate(heli_pose, helicopter->getRed3D(), helicopter->getGreen3D(), 
            helicopter->getBlue3D(), &z[0]);      
    
    lsqnon_Estimation(heli_pose, z, helicopter->getObjectPoints(), pose_est, &norm);
    
    real_T roll, pitch, yaw;
    real_T quat[4] = {-pose_est[4],pose_est[3],-pose_est[6],pose_est[5]};
    eulerAnglesFromQuaternion(&quat[0], &roll, &pitch, &yaw);            
    cout << norm << " - " << pose_est[0] << " " << pose_est[1] << " " << pose_est[2] << " " << roll << " " << pitch << " " << yaw << endl;      
    
    real_T offset[4] = {0};        
    offset[0] = -pose_est[4];
    offset[1] = pose_est[3];
    offset[2] = -pose_est[6];
    offset[3] = pose_est[5];
            
    // normalise average quaternion
    float norm_q = sqrt(pow(offset[0],2)+pow(offset[1],2)+pow(offset[2],2)+pow(offset[3],2));    
    offset[0] /= norm_q;
    offset[1] /= norm_q;
    offset[2] /= norm_q;
    offset[3] /= norm_q;                    
                    
    double temp[3];
    real_T newRed[3] = {0}, newGreen[3] = {0}, newBlue[3] = {0};
    Reb(&offset[0], helicopter->getRed3D(), &temp[0]);
    memcpy(&newRed[0],&temp[0], sizeof(double)*3);
    Reb(&offset[0], helicopter->getGreen3D(), &temp[0]);
    memcpy(&newGreen[0],&temp[0], sizeof(double)*3);
    Reb(&offset[0], helicopter->getBlue3D(), &temp[0]);
    memcpy(&newBlue[0],&temp[0], sizeof(double)*3);    
    
    b_struct_T z2[4];
    fillCamStruct(&z2[0]);     
    
   // condition structure so correct blobs are used
    blobAllocate(heli_pose, &newRed[0], &newGreen[0], &newBlue[0], &z2[0]);      
    real_T new_objects[9] = {newRed[0],newRed[1],newRed[2],
        newGreen[0],newGreen[1],newGreen[2],
        newBlue[0],newBlue[1],newBlue[2]};
    real_T pose_est2[7] = {0}, norm2 = 0;
    lsqnon_Estimation(heli_pose, z, &new_objects[0], pose_est2, &norm2);   
    
    real_T quat2[4] = {-pose_est2[4],pose_est2[3],-pose_est2[6],pose_est2[5]};
    eulerAnglesFromQuaternion(&quat2[0], &roll, &pitch, &yaw);            
    cout << norm << " - " << pose_est2[0] << " " << pose_est2[1] << " " << pose_est2[2] << " " << roll << " " << pitch << " " << yaw << endl;          
    
    return norm;
}

void PoseEstimation::LSQEstimation(Helicopter * h){
    // run nonlin est to compare results
    real_T pose_est[7] = {0};
    real_T norm = 0;
    
    b_struct_T z[4];
    fillCamStruct(&z[0]);       
    
    real_T pose[7] = {0};
    pose[0] = (double)*h->mocapPositionX();
    pose[1] = (double)*h->mocapPositionY();
    pose[2] = (double)*h->mocapPositionZ();
    pose[3] = (double)*h->mocapq0();
    pose[4] = (double)*h->mocapq1();
    pose[5] = (double)*h->mocapq2();
    pose[6] = (double)*h->mocapq3();    
    
    lsqnon_Estimation(pose, z, h->getObjectPoints(), &pose_est[0], &norm);
    
    //if residual is less than min then log/update as new data 
//    if (norm < RESNORM_MIN){
        *h->mocapPositionX() = (float)pose_est[0];
        *h->mocapPositionY() = (float)pose_est[1];
        *h->mocapPositionZ() = (float)pose_est[2];
    
        *h->mocapq0() = (float)pose_est[3];
        *h->mocapq1() = (float)pose_est[4];
        *h->mocapq2() = (float)pose_est[5];
        *h->mocapq3() = (float)pose_est[6];
    
        // get euler angles from quaternion
        real_T roll = 0, pitch = 0, yaw = 0;
        real_T quat[4] = {-pose_est[4],pose_est[3],-pose_est[6],pose_est[5]};
        eulerAnglesFromQuaternion(&quat[0], &roll, &pitch, &yaw);    
        *h->mocapRoll() = roll*M_PI/180;
        *h->mocapPitch() = pitch*M_PI/180;
        *h->mocapYaw() = yaw*M_PI/180;
        
       // printf("%5.2f %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\r\n",pose_est[0],pose_est[1],pose_est[2],roll, pitch,yaw, norm);
//    }
//    else{
//        //cout << "RENORM greater than " << RESNORM_MIN << endl;
//    }
}

bool PoseEstimation::initialiseHelicopter(Helicopter * helicopter){
    real_T init_x[7] = {0};
    memcpy(&init_x[0], helicopter->getInitPose(), sizeof(real_T)*7);
    real_T ret_x[7];
    real_T norm = 0;    
    
    b_struct_T z[4];
    fillCamStruct(&z[0]);   
    
    // if no blobs in scene return false
    if (activeBlobs(&z[0]) == 0)
        return false;

    // condition structure so correct blobs are used
    blobAllocate(init_x, helicopter->getRed3D(), helicopter->getGreen3D(), 
            helicopter->getBlue3D(), &z[0]);    
    
    if(lsqnon_Estimation(init_x, z, helicopter->getObjectPoints(), ret_x, &norm)){
        *(helicopter->getInitCounterSum()) = *(helicopter->getInitCounterSum()) + 1;
        
        *(helicopter->getInitxSum()) += ret_x[0];
        *(helicopter->getInitySum()) += ret_x[1];
        *(helicopter->getInitzSum()) += ret_x[2];        
        
        *(helicopter->getInitq0Sum()) += ret_x[3];
        *(helicopter->getInitq1Sum()) += ret_x[4];
        *(helicopter->getInitq2Sum()) += ret_x[5];
        *(helicopter->getInitq3Sum()) += ret_x[6];              
                
        if (*(helicopter->getInitCounterSum()) == INIT_QUAT_EST_ITER){
            double offset[4];
            *(helicopter->getInitq0Sum()) /= INIT_QUAT_EST_ITER;
            *(helicopter->getInitq1Sum()) /= INIT_QUAT_EST_ITER;
            *(helicopter->getInitq2Sum()) /= INIT_QUAT_EST_ITER;
            *(helicopter->getInitq3Sum()) /= INIT_QUAT_EST_ITER;
                    
            *(helicopter->getInitxSum()) /= INIT_QUAT_EST_ITER;
            *(helicopter->getInitySum()) /= INIT_QUAT_EST_ITER;
            *(helicopter->getInitzSum()) /= INIT_QUAT_EST_ITER;
                    
            offset[0] = -*(helicopter->getInitq1Sum());
            offset[1] = *(helicopter->getInitq0Sum());
            offset[2] = -*(helicopter->getInitq3Sum());
            offset[3] = *(helicopter->getInitq2Sum());                   
            
            // normalise average quaternion
            float norm_q = sqrt(pow(offset[0],2)+pow(offset[1],2)+pow(offset[2],2)+pow(offset[3],2));    
            offset[0] /= norm_q;
            offset[1] /= norm_q;
            offset[2] /= norm_q;
            offset[3] /= norm_q;                    
                    
            double temp[3];
            Reb(&offset[0], helicopter->getRed3D(), &temp[0]);
            memcpy(helicopter->getRed3D(),&temp[0], sizeof(double)*3);
            Reb(&offset[0], helicopter->getGreen3D(), &temp[0]);
            memcpy(helicopter->getGreen3D(),&temp[0], sizeof(double)*3);
            Reb(&offset[0], helicopter->getBlue3D(), &temp[0]);
            memcpy(helicopter->getBlue3D(),&temp[0], sizeof(double)*3);  
            
            // Must update blob positions i.e. copy individual blob positions to objectPoints[9]
            helicopter->updateObjectPoints();
                        
            real_T init_x2[7] = {0}; real_T ret_x2[7] = {0};
            init_x2[0] = *(helicopter->getInitxSum());
            init_x2[1] = *(helicopter->getInitySum());
            init_x2[2] = *(helicopter->getInitzSum());
            init_x2[3] = *(helicopter->getInitq0Sum());
            init_x2[4] = *(helicopter->getInitq1Sum());
            init_x2[5] = *(helicopter->getInitq2Sum());
            init_x2[6] = *(helicopter->getInitq3Sum());
            
            // Now get pose with updated blob positions
            lsqnon_Estimation(init_x2, z, helicopter->getObjectPoints(), ret_x2, &norm);
            // copy new translation
            memcpy(helicopter->getInitxSum(), &ret_x2[0], sizeof(real_T)*3);
            memcpy(helicopter->getInitq0Sum(), &ret_x2[3], sizeof(real_T)*4);
           
            std::cout << "Init H" << helicopter->getID() << " DONE" << std::endl;
            printf("New blob coordinates are: \r\n");
            printf("RED: %.2f, %.2f, %.2f\r\n", *helicopter->getRed3D(), *(helicopter->getRed3D()+1), *(helicopter->getRed3D()+2));
            printf("GREEN: %.2f, %.2f, %.2f\r\n", *helicopter->getGreen3D(), *(helicopter->getGreen3D()+1), *(helicopter->getGreen3D()+2));
            printf("BLUE: %.2f, %.2f, %.2f\r\n", *helicopter->getBlue3D(), *(helicopter->getBlue3D()+1), *(helicopter->getBlue3D()+2));
            printf("Initial translation: %.0f, %.0f, %.0f (mm)\r\n", *(helicopter->getInitxSum())*1000, *(helicopter->getInitySum())*1000, *(helicopter->getInitzSum())*1000);
            real_T roll_err, pitch_err, yaw_err;
            real_T quat_err[4] = {-ret_x2[4],ret_x2[3],-ret_x2[6],ret_x2[5]};
            eulerAnglesFromQuaternion(&quat_err[0], &roll_err, &pitch_err, &yaw_err);
            printf("Rotational error: %.4f, %.4f, %.4f (degrees)\r\n", roll_err, pitch_err, yaw_err);
                    
            // Replace initial state estimates
            real_T * ekf_state = helicopter->kalmanState();
                    
            ekf_state[0] = ret_x2[0];
            ekf_state[1] = ret_x2[1];
            ekf_state[2] = ret_x2[2];
            ekf_state[6] = ret_x2[3];
            ekf_state[7] = ret_x2[4];
            ekf_state[8] = ret_x2[5];
            ekf_state[9] = ret_x2[6];
            printf("H%d initial pose: %.0f, %.0f, %.0f | %7.4f,%7.4f,%7.4f,%7.4f \r\n\r\n", 
                    helicopter->getID(),
                    ekf_state[0]*1000, ekf_state[1]*1000, ekf_state[2]*1000,
                     ekf_state[6], ekf_state[7], ekf_state[8], ekf_state[9]); 
            
            helicopter->setInitialisingPose(false);
            
            return true;
        }
    }   

    return false;    
}

int PoseEstimation::activeBlobs(b_struct_T * tmp_z){
    int blobs = 0;
    for (int i = 0; i < 4; i++){
        blobs += (int)(tmp_z[i].number_red);
        blobs += (int)(tmp_z[i].number_green);
        blobs += (int)(tmp_z[i].number_blue);
    }
    return blobs;
}

bool PoseEstimation::isInitialised(void){
    return !initialising;
}

PoseEstimation::pe_state PoseEstimation::getState(void){
    return current_state;
}

void PoseEstimation::enableEKFCorrection(bool en){
    ekf_on = en;
}

double PoseEstimation::getFPS(void){
    return FPS;
}

void PoseEstimation::doCorrection(void){      
    int i;
    real_T dt = dtCalc();
    // log doCorrection start time
    deltaTime();
    
    for (i = 0; i < active_helicopters; i++){
        correctStates(helicopter[i], dt);        
    }
    
    clearCamDataReceived();
    
    // Compute average FPS for 100 samples
    ++stats_count;
    proc_time_sum += deltaTime();
    fps_sum += FPSCalc();
                    
    if (stats_count == POSEESTIMATION_STATS_SAMPLE){
        FPS = fps_sum/POSEESTIMATION_STATS_SAMPLE;
        process_time = proc_time_sum/POSEESTIMATION_STATS_SAMPLE;
        stats_count = 0;
        fps_sum = 0;
        proc_time_sum = 0;
//        printTiming();
    }    
}

void PoseEstimation::correctStates(Helicopter * h, real_T dt){
    b_struct_T z_heli[4] = {{0}};
    
    // copy camera data to structs
    fillCamStruct(&z_heli[0]);
        
    real_T x_heli[16];
    real_T P_heli[256];
    
    // correct state and covariance using modified measurement structs to current time
    // first project state and error covariance to current time if dt is greater than 10ms
    h->projectStateErrorCovarianceCurrent();    
        
    // make a copy of the state and state covariance for each helicopter
    memcpy(&x_heli[0], h->kalmanState(), sizeof(real_T)*16);
    memcpy(&P_heli[0], h->kalmanErrorCovariance(), sizeof(real_T)*256);
     
    // form pose array for each helicopter which is used to estimate most likely measurements
    real_T heli_pose[7] = {x_heli[0], x_heli[1], x_heli[2], x_heli[6], x_heli[7], x_heli[8], x_heli[9]};
    
//    blobAllocate(heli_pose, h->getRed3D(), h->getGreen3D(), h->getBlue3D(), &z_heli[0]);
    // Run nonlin lsq algorithm
    updateHelicopterMocapData(h, z_heli, heli_pose, dt);
    
    real_T meas_noise = MEASUREMENT_NOISE_VARIANCE;  // pixel^2
    real_T xCorr_h[16];
    real_T PCorr_h[256];    
    
    correctStateAndCov(&x_heli[0], &z_heli[0], &P_heli[0], h->getObjectPoints(), meas_noise, &xCorr_h[0], &PCorr_h[0]);    
        
    // replace internal ekf states and covariance by corrected ones
    memcpy(h->kalmanState(), &xCorr_h[0], sizeof(double)*16);
    memcpy(h->kalmanErrorCovariance(), &PCorr_h[0], sizeof(double)*256);    
    
    h->logCorrectionTime();   
}

void PoseEstimation::updateHelicopterMocapData(Helicopter * h, const b_struct_T z[4], real_T pose[7], real_T dt){
    // run nonlin est to compare results
    real_T pose_est[7] = {0};
    real_T norm = 0;
    lsqnon_Estimation(pose, z, h->getObjectPoints(), &pose_est[0], &norm);
    
    //if residual is less than min then log/update as new data 
    if (norm < RESNORM_MIN){
        // First calc and store velocities
        *h->mocapVelocityX() = (float)(pose_est[0] - *h->mocapPositionX())/(float)dt;
        *h->mocapVelocityY() = (float)(pose_est[1] - *h->mocapPositionY())/(float)dt;
        *h->mocapVelocityZ() = (float)(pose_est[2] - *h->mocapPositionZ())/(float)dt;
        *h->mocapPositionX() = (float)pose_est[0];
        *h->mocapPositionY() = (float)pose_est[1];
        *h->mocapPositionZ() = (float)pose_est[2];
        // calculate body rates
        // 2q_conj*dq/dt = [0 w]'
        float body_rates[4] = {0};
        float dq_dt[4] = {0};
        dq_dt[0] = (float)(pose_est[3] - *h->mocapq0())/(float)dt;
        dq_dt[1] = (float)(pose_est[4] - *h->mocapq1())/(float)dt;
        dq_dt[2] = (float)(pose_est[5] - *h->mocapq2())/(float)dt;
        dq_dt[3] = (float)(pose_est[6] - *h->mocapq3())/(float)dt;
        float q_curr_conj[4] = {2*(*h->mocapq0()),
            -2*(*h->mocapq1()),
            -2*(*h->mocapq2()),
            -2*(*h->mocapq3())};
        quaternionMultiply(q_curr_conj, dq_dt, &body_rates[0]);
        *h->mocapRollRate() = body_rates[1];
        *h->mocapPitchRate() = body_rates[2];
        *h->mocapYawRate() = body_rates[3];
    
        *h->mocapq0() = (float)pose_est[3];
        *h->mocapq1() = (float)pose_est[4];
        *h->mocapq2() = (float)pose_est[5];
        *h->mocapq3() = (float)pose_est[6];
    
        // get euler angles from quaternion
        real_T roll = 0, pitch = 0, yaw = 0;
        real_T quat[4] = {-pose_est[4],pose_est[3],-pose_est[6],pose_est[5]};
        eulerAnglesFromQuaternion(&quat[0], &roll, &pitch, &yaw);    
        *h->mocapRoll() = roll*M_PI/180;
        *h->mocapPitch() = pitch*M_PI/180;
        *h->mocapYaw() = yaw*M_PI/180;
    }
    
    sprintf(log_buffer, "%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%.4G,%u\r\n", 
            *h->mocapPositionX(), *h->mocapPositionY(), *h->mocapPositionZ(),
            *h->mocapVelocityX(), *h->mocapVelocityY(), *h->mocapVelocityZ(),
            *h->mocapq0(),*h->mocapq1(),*h->mocapq2(),*h->mocapq3(),
            *h->mocapRoll(), *h->mocapPitch(), *h->mocapYaw(),
            *h->mocapRollRate(), *h->mocapPitchRate(), *h->mocapYawRate(),
            h->timeNow());
    writeLog();
}

void PoseEstimation::quaternionMultiply(const float q1[4], const float q2[4], float q[4]){
    float b_q1[16];
    int i0;
    int i1;
    b_q1[0] = q1[0];
    b_q1[4] = -q1[1];
    b_q1[8] = -q1[2];
    b_q1[12] = -q1[3];
    b_q1[1] = q1[1];
    b_q1[5] = q1[0];
    b_q1[9] = -q1[3];
    b_q1[13] = q1[2];
    b_q1[2] = q1[2];
    b_q1[6] = q1[3];
    b_q1[10] = q1[0];
    b_q1[14] = -q1[1];
    b_q1[3] = q1[3];
    b_q1[7] = -q1[2];
    b_q1[11] = q1[1];
    b_q1[15] = q1[0];
    for (i0 = 0; i0 < 4; i0++) {
        q[i0] = 0.0F;
        for (i1 = 0; i1 < 4; i1++) {
            q[i0] += b_q1[i0 + (i1 << 2)] * q2[i1];
        }
    }
}

void PoseEstimation::printTiming(void){
    printf("[PoseEst] Process time: %07.0fus FPS: %04.0f\r\n", process_time, FPS);
}

// supply camera blob struct and 3D blob position to project
void PoseEstimation::projectObjectPoint(const real_T h_x[7], const b_struct_T z,
        const real_T object_points[3], real_T image_point[2]){
    // Modify blob struct so that first blob represents actual blob
    // blob selection is based on reprojecting blobs into each camera
    
    real_T A[9];  
    real_T b_z[3];
    int32_T i0;
    real_T b_object_points[3];
    
    for (i0 = 0; i0 < 9; i0++){
        A[i0] = z.cam_param.intrinsic_mat[i0] * 1000.0;
    }
    
    /*  convert mapping from pixel/mm to pixel/m */
    for (i0 = 0; i0 < 3; i0++) {
        b_z[i0] = z.cam_param.translation_w2c[i0] / 1000.0;
    }    
    
    for (i0 = 0; i0 < 3; i0++) {
        b_object_points[i0] = object_points[i0] / 1000.0;
    }
    
    // Project object points in body frame to camera frame
    project2_2D(A, z.cam_param.rotation_w2c, b_z, &h_x[0],
                  &h_x[3], b_object_points, image_point);
    
}

void PoseEstimation::getImagePoints(const real_T h_x[7], const b_struct_T z, 
        const real_T red3D[3], const real_T green3D[3], const real_T blue3D[3], real_T image_points[3][2]){    
    projectObjectPoint(h_x, z, red3D, &image_points[0][0]);
    projectObjectPoint(h_x, z, green3D, &image_points[1][0]);
    projectObjectPoint(h_x, z, blue3D, &image_points[2][0]);
}

// blobAllocate modifies the struct b_struct_T so that the first blob element is
// the most likely true element. h_x is the last known pose of the helicopter
// hx[0:2] translation
// hx[3:6] quaternion
// The blobs in the helicopter frame are projected on to each camera frame given hx
// if there is contention i.e. multiple blobs in the scene; the blob that produces the 
// minimum 2D distance is chosen as the candidate-done by replacing the first index and
// changing number of blobs to 1
// If this pixel distance is too large it is deleted
void PoseEstimation::blobAllocate(const real_T h_x[7],
        const real_T red3D[3], const real_T green3D[3], const real_T blue3D[3], b_struct_T * z){
    real_T image_points_cam0[3][2] = {{-1,-1},{-1,-1},{-1,-1}};
    real_T image_points_cam1[3][2] = {{-1,-1},{-1,-1},{-1,-1}};
    real_T image_points_cam2[3][2] = {{-1,-1},{-1,-1},{-1,-1}};
    real_T image_points_cam3[3][2] = {{-1,-1},{-1,-1},{-1,-1}};
    
    getImagePoints(h_x, z[0], red3D, green3D, blue3D, image_points_cam0);
    getImagePoints(h_x, z[1], red3D, green3D, blue3D, image_points_cam1);
    getImagePoints(h_x, z[2], red3D, green3D, blue3D, image_points_cam2);
    getImagePoints(h_x, z[3], red3D, green3D, blue3D, image_points_cam3); 
    
    real_T image_red0[5][2] = {{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}};
    real_T red_dist0[5] = {-1,-1,-1,-1,-1};
    real_T image_green0[5][2] = {{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}};
    real_T green_dist0[5] = {-1,-1,-1,-1,-1};
    real_T image_blue0[5][2] = {{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}};
    real_T blue_dist0[5] = {-1,-1,-1,-1,-1};    
    int minIndexRed0 = 0, minIndexGreen0 = 0, minIndexBlue0 = 0;
    findMostLikelyBlob(0, z, image_red0, image_green0, image_blue0, 
            red_dist0, green_dist0, blue_dist0, &minIndexRed0, 
            &minIndexGreen0, &minIndexBlue0, image_points_cam0);
    modifyCamStruct(&z[0], minIndexRed0, minIndexGreen0, minIndexBlue0,
            red_dist0, green_dist0, blue_dist0);
    
    real_T image_red1[5][2] = {{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}};
    real_T red_dist1[5] = {-1,-1,-1,-1,-1};
    real_T image_green1[5][2] = {{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}};
    real_T green_dist1[5] = {-1,-1,-1,-1,-1};
    real_T image_blue1[5][2] = {{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}};
    real_T blue_dist1[5] = {-1,-1,-1,-1,-1};    
    int minIndexRed1 = 0, minIndexGreen1 = 0, minIndexBlue1 = 0;
    findMostLikelyBlob(1, z, image_red1, image_green1, image_blue1, 
            red_dist1, green_dist1, blue_dist1, &minIndexRed1, 
            &minIndexGreen1, &minIndexBlue1, image_points_cam1);
    modifyCamStruct(&z[1], minIndexRed1, minIndexGreen1, minIndexBlue1,
            red_dist1, green_dist1, blue_dist1);

    real_T image_red2[5][2] = {{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}};
    real_T red_dist2[5] = {-1,-1,-1,-1,-1};
    real_T image_green2[5][2] = {{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}};
    real_T green_dist2[5] = {-1,-1,-1,-1,-1};
    real_T image_blue2[5][2] = {{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}};
    real_T blue_dist2[5] = {-1,-1,-1,-1,-1};    
    int minIndexRed2 = 0, minIndexGreen2 = 0, minIndexBlue2 = 0;
    findMostLikelyBlob(2, z, image_red2, image_green2, image_blue2, 
            red_dist2, green_dist2, blue_dist2, &minIndexRed2, 
            &minIndexGreen2, &minIndexBlue2, image_points_cam2);    
    modifyCamStruct(&z[2], minIndexRed2, minIndexGreen2, minIndexBlue2,
            red_dist2, green_dist2, blue_dist2);
    
    real_T image_red3[5][2] = {{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}};
    real_T red_dist3[5] = {-1,-1,-1,-1,-1};
    real_T image_green3[5][2] = {{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}};
    real_T green_dist3[5] = {-1,-1,-1,-1,-1};
    real_T image_blue3[5][2] = {{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}};
    real_T blue_dist3[5] = {-1,-1,-1,-1,-1};    
    int minIndexRed3 = 0, minIndexGreen3 = 0, minIndexBlue3 = 0;
    findMostLikelyBlob(3, z, image_red3, image_green3, image_blue3, 
            red_dist3, green_dist3, blue_dist3, &minIndexRed3, 
            &minIndexGreen3, &minIndexBlue3, image_points_cam3);
    modifyCamStruct(&z[3], minIndexRed3, minIndexGreen3, minIndexBlue3,
            red_dist3, green_dist3, blue_dist3);    

//    clearScreen();
//    fflush(stdout);
//    printf("%-12s%-21s%2s%-5s%-21s%2s%-5s%-21s%2s\r\n", "Red", " ", " ", "Green", " ", " ", "Blue", " ", " ");
//    printf("%-12s%7s%7s%7s%2s%-5s%7s%7s%7s%2s%-5s%7s%7s%7s%2s\r\n", " ", "X", "Y", "dist", " ", " ", "X", "Y", "dist", " ", " ", "X", "Y", "dist", " ");
//    
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n", "Cam Pred 1", image_points_cam0[0][0], image_points_cam0[0][1], 0.0, " ", " ", image_points_cam0[1][0], image_points_cam0[1][1], 0.0, " ", " ", image_points_cam0[2][0], image_points_cam0[2][1], 0.0, " ");
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n", "Measure  0", image_red0[0][0], image_red0[0][1], red_dist0[0], (minIndexRed0 == 0) ? "1" : "-", " ", image_green0[0][0], image_green0[0][1], green_dist0[0], (minIndexGreen0 == 0) ? "1" : "-", " ", image_blue0[0][0], image_blue0[0][1], blue_dist0[0], (minIndexBlue0 == 0) ? "1" : "-");
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n", "Measure  1", image_red0[1][0], image_red0[1][1], red_dist0[1], (minIndexRed0 == 1) ? "1" : "-", " ", image_green0[1][0], image_green0[1][1], green_dist0[1], (minIndexGreen0 == 1) ? "1" : "-", " ", image_blue0[1][0], image_blue0[1][1], blue_dist0[1], (minIndexBlue0 == 1) ? "1" : "-");
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n", "Measure  2", image_red0[2][0], image_red0[2][1], red_dist0[2], (minIndexRed0 == 2) ? "1" : "-", " ", image_green0[2][0], image_green0[2][1], green_dist0[2], (minIndexGreen0 == 2) ? "1" : "-", " ", image_blue0[2][0], image_blue0[2][1], blue_dist0[2], (minIndexBlue0 == 2) ? "1" : "-");    
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n", "Measure  3", image_red0[3][0], image_red0[3][1], red_dist0[3], (minIndexRed0 == 3) ? "1" : "-", " ", image_green0[3][0], image_green0[3][1], green_dist0[3], (minIndexGreen0 == 3) ? "1" : "-", " ", image_blue0[3][0], image_blue0[3][1], blue_dist0[3], (minIndexBlue0 == 3) ? "1" : "-");
//    
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n", "Cam Pred 2", image_points_cam1[0][0], image_points_cam1[0][1], 0.0, " ", " ", image_points_cam1[1][0], image_points_cam1[1][1], 0.0, " ", " ", image_points_cam1[2][0], image_points_cam1[2][1], 0.0, " ");
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n", "Measure  0", image_red1[0][0], image_red1[0][1], red_dist1[0], (minIndexRed1 == 0) ? "1" : "-", " ", image_green1[0][0], image_green1[0][1], green_dist1[0], (minIndexGreen1 == 0) ? "1" : "-", " ", image_blue1[0][0], image_blue1[0][1], blue_dist1[0], (minIndexBlue1 == 0) ? "1" : "-");
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n", "Measure  1", image_red1[1][0], image_red1[1][1], red_dist1[1], (minIndexRed1 == 1) ? "1" : "-", " ", image_green1[1][0], image_green1[1][1], green_dist1[1], (minIndexGreen1 == 1) ? "1" : "-", " ", image_blue1[1][0], image_blue1[1][1], blue_dist1[1], (minIndexBlue1 == 1) ? "1" : "-");
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n", "Measure  2", image_red1[2][0], image_red1[2][1], red_dist1[2], (minIndexRed1 == 2) ? "1" : "-", " ", image_green1[2][0], image_green1[2][1], green_dist1[2], (minIndexGreen1 == 2) ? "1" : "-", " ", image_blue1[2][0], image_blue1[2][1], blue_dist1[2], (minIndexBlue1 == 2) ? "1" : "-");    
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n", "Measure  3", image_red1[3][0], image_red1[3][1], red_dist1[3], (minIndexRed1 == 3) ? "1" : "-", " ", image_green1[3][0], image_green1[3][1], green_dist1[3], (minIndexGreen1 == 3) ? "1" : "-", " ", image_blue1[3][0], image_blue1[3][1], blue_dist1[3], (minIndexBlue1 == 3) ? "1" : "-");    
//    
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n", "Cam Pred 3", image_points_cam2[0][0], image_points_cam2[0][1], 0.0, " ", " ", image_points_cam2[1][0], image_points_cam2[1][1], 0.0, " ", " ", image_points_cam2[2][0], image_points_cam2[2][1], 0.0, " ");
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n", "Measure  0", image_red2[0][0], image_red2[0][1], red_dist2[0], (minIndexRed2 == 0) ? "1" : "-", " ", image_green2[0][0], image_green2[0][1], green_dist2[0], (minIndexGreen2 == 0) ? "1" : "-", " ", image_blue2[0][0], image_blue2[0][1], blue_dist2[0], (minIndexBlue2 == 0) ? "1" : "-");
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n", "Measure  1", image_red2[1][0], image_red2[1][1], red_dist2[1], (minIndexRed2 == 1) ? "1" : "-", " ", image_green2[1][0], image_green2[1][1], green_dist2[1], (minIndexGreen2 == 1) ? "1" : "-", " ", image_blue2[1][0], image_blue2[1][1], blue_dist2[1], (minIndexBlue2 == 1) ? "1" : "-");
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n", "Measure  2", image_red2[2][0], image_red2[2][1], red_dist2[2], (minIndexRed2 == 2) ? "1" : "-", " ", image_green2[2][0], image_green2[2][1], green_dist2[2], (minIndexGreen2 == 2) ? "1" : "-", " ", image_blue2[2][0], image_blue2[2][1], blue_dist2[2], (minIndexBlue2 == 2) ? "1" : "-");    
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n", "Measure  3", image_red2[3][0], image_red2[3][1], red_dist2[3], (minIndexRed2 == 3) ? "1" : "-", " ", image_green2[3][0], image_green2[3][1], green_dist2[3], (minIndexGreen2 == 3) ? "1" : "-", " ", image_blue2[3][0], image_blue2[3][1], blue_dist2[3], (minIndexBlue2 == 3) ? "1" : "-");    
//    
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n", "Cam Pred 4", image_points_cam3[0][0], image_points_cam3[0][1], 0.0, " ", " ", image_points_cam3[1][0], image_points_cam3[1][1], 0.0, " ", " ", image_points_cam3[2][0], image_points_cam3[2][1], 0.0, " ");
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n", "Measure  0", image_red3[0][0], image_red3[0][1], red_dist3[0], (minIndexRed3 == 0) ? "1" : "-", " ", image_green3[0][0], image_green3[0][1], green_dist3[0], (minIndexGreen3 == 0) ? "1" : "-", " ", image_blue3[0][0], image_blue3[0][1], blue_dist3[0], (minIndexBlue3 == 0) ? "1" : "-");
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n", "Measure  1", image_red3[1][0], image_red3[1][1], red_dist3[1], (minIndexRed3 == 1) ? "1" : "-", " ", image_green3[1][0], image_green3[1][1], green_dist3[1], (minIndexGreen3 == 1) ? "1" : "-", " ", image_blue3[1][0], image_blue3[1][1], blue_dist3[1], (minIndexBlue3 == 1) ? "1" : "-");
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n", "Measure  2", image_red3[2][0], image_red3[2][1], red_dist3[2], (minIndexRed3 == 2) ? "1" : "-", " ", image_green3[2][0], image_green3[2][1], green_dist3[2], (minIndexGreen3 == 2) ? "1" : "-", " ", image_blue3[2][0], image_blue3[2][1], blue_dist3[2], (minIndexBlue3 == 2) ? "1" : "-");    
//    printf("%-12s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s%-5s%7.1f%7.1f%7.1f%2s\r\n\r\n\r\n", "Measure  3", image_red3[3][0], image_red3[3][1], red_dist3[3], (minIndexRed3 == 3) ? "1" : "-", " ", image_green3[3][0], image_green3[3][1], green_dist3[3], (minIndexGreen3 == 3) ? "1" : "-", " ", image_blue3[3][0], image_blue3[3][1], blue_dist3[3], (minIndexBlue3 == 3) ? "1" : "-");
//    fflush(stdout);
}

real_T PoseEstimation::pixelDistance(const real_T a[2], const real_T b[2]){
    return sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2));
}

void PoseEstimation::clearScreen(void){
    int n;
    for (n = 0; n < 10; n++){
        printf("\n\n\n\n\n\n\n\n\n\n");
    }
}

int PoseEstimation::retMinDistanceIndex(const real_T dist[5], const real_T max_blobs){
    real_T min_dist = 1000;
    int min_dist_index = -1;
    int i = 0;
    for (i = 0; i < max_blobs; i++){
        if (dist[i] < min_dist){
            min_dist = dist[i];
            min_dist_index = i;
        }
    }
    return min_dist_index;
}

void PoseEstimation::findMostLikelyBlob(const int cam, const b_struct_T z[4], real_T image_red[5][2]
    , real_T image_green[5][2], real_T image_blue[5][2]
    , real_T red_dist[5], real_T green_dist[5], real_T blue_dist[5]
    , int * red_dist_min_index, int * green_dist_min_index, int * blue_dist_min_index
    , const real_T image_points_cam[3][2]){
    
    int i = 0;

    for (i = 0; i < z[cam].number_red; i++){
        image_red[i][0] = z[cam].red_data[i];
        image_red[i][1] = z[cam].red_data[i+5];
        red_dist[i] = pixelDistance(&image_points_cam[0][0], &image_red[i][0]);
    }
    *red_dist_min_index = retMinDistanceIndex(red_dist, z[cam].number_red);
   
    for (i = 0; i < z[cam].number_green; i++){
        image_green[i][0] = z[cam].green_data[i];
        image_green[i][1] = z[cam].green_data[i+5];
        green_dist[i] = pixelDistance(&image_points_cam[1][0], &image_green[i][0]);
    }
    *green_dist_min_index = retMinDistanceIndex(green_dist, z[cam].number_green);    
     
    for (i = 0; i < z[cam].number_blue; i++){
        image_blue[i][0] = z[cam].blue_data[i];
        image_blue[i][1] = z[cam].blue_data[i+5];
        blue_dist[i] = pixelDistance(&image_points_cam[2][0], &image_blue[i][0]);
    }
    *blue_dist_min_index = retMinDistanceIndex(blue_dist, z[cam].number_blue);  
}

void PoseEstimation::modifyCamStruct(b_struct_T * z, const int red_min_index, 
        const int green_min_index, const int blue_min_index,
        real_T red_dist[5], real_T green_dist[5], real_T blue_dist[5]){
    // Replace index #1 of multi-blob colors with blob most likely
    
    real_T min_dist_blob[2] = {0,0};
        
    if (z->number_red > 1){
        min_dist_blob[0] = z->red_data[red_min_index];
        min_dist_blob[1] = z->red_data[red_min_index+5];
        z->red_data[0] = min_dist_blob[0];
        z->red_data[5] = min_dist_blob[1];
        z->number_red = 1;
    }
    if (z->number_green > 1){
        min_dist_blob[0] = z->green_data[green_min_index];
        min_dist_blob[1] = z->green_data[green_min_index+5];
        z->green_data[0] = min_dist_blob[0];
        z->green_data[5] = min_dist_blob[1];
        z->number_green = 1;
    }
    if (z->number_blue > 1){
        min_dist_blob[0] = z->blue_data[blue_min_index];
        min_dist_blob[1] = z->blue_data[blue_min_index+5];
        z->blue_data[0] = min_dist_blob[0];
        z->blue_data[5] = min_dist_blob[1];
        z->number_blue = 1;
    }    
    
    // if the minimum distance is unacceptable then discard measurement
    if (red_dist[red_min_index] > PEST_MIN_DIST){
        cout << "Red min dist " << red_dist[red_min_index] << endl;
        z->number_red = 0;
    }
    if (green_dist[green_min_index] > PEST_MIN_DIST){
        cout << "Green min dist " << green_dist[green_min_index] << endl;
        z->number_green = 0;
    }
    if (blue_dist[blue_min_index] > PEST_MIN_DIST){
        cout << "Blue min dist " << blue_dist[blue_min_index] << endl;
        z->number_blue = 0;
    }        
}

void PoseEstimation::solveContention(int cam, b_struct_T z, const real_T image_points[3][2]){
    // determine if there is contention ... is there more than 1 blob detected?
    // if there is, then replace #1 with the blob that is closest to the predicted
    int i = 0;
    real_T red_dist[5] = {-1,-1,-1,-1,-1};
    real_T green_dist[5] = {-1,-1,-1,-1,-1};
    real_T blue_dist[5] = {-1,-1,-1,-1,-1};
    real_T image_meas[2] = {-1,-1};
    
    if (z.number_red > 1){
        for (i = 0; i < z.number_red; i++){
            image_meas[0] = z.red_data[i];
            image_meas[1] = z.red_data[i+5];
            red_dist[i] = pixelDistance(&image_points[0][0], &image_meas[0]);
        }
    }
    int indexRed = retMinDistanceIndex(red_dist, z.number_red);    
    
    if (z.number_green > 1){
        for (i = 0; i < z.number_green; i++){
            image_meas[0] = z.green_data[i];
            image_meas[1] = z.green_data[i+5];
            green_dist[i] = pixelDistance(&image_points[1][0], &image_meas[0]);            
        }        
    }
    int indexGreen = retMinDistanceIndex(green_dist, z.number_green);
    
    if (z.number_blue > 1){
        for (i = 0; i < z.number_blue; i++){
            image_meas[0] = z.blue_data[i];
            image_meas[1] = z.blue_data[i+5];
            blue_dist[i] = pixelDistance(&image_points[2][0], &image_meas[0]);            
        }        
    }    
    int indexBlue = retMinDistanceIndex(blue_dist, z.number_blue);
    
    // Replace index #1 of multi-blob colors with blob most likely
    real_T min_dist_blob[2] = {0,0};
    if (z.number_red > 1){
        min_dist_blob[0] = z.red_data[indexRed];
        min_dist_blob[1] = z.red_data[indexRed+5];
        z.red_data[0] = min_dist_blob[0];
        z.red_data[5] = min_dist_blob[1];
    }
    if (z.number_green > 1){
        min_dist_blob[0] = z.green_data[indexGreen];
        min_dist_blob[1] = z.green_data[indexGreen+5];
        z.green_data[0] = min_dist_blob[0];
        z.green_data[5] = min_dist_blob[1];
    }
    if (z.number_blue > 1){
        min_dist_blob[0] = z.blue_data[indexBlue];
        min_dist_blob[1] = z.blue_data[indexBlue+5];
        z.blue_data[0] = min_dist_blob[0];
        z.blue_data[5] = min_dist_blob[1];
    }        
}

void PoseEstimation::fillCamStruct(b_struct_T * tmp_z){
    for (int i = 0; i < 4; i++){
        setCamParam(&tmp_z[i], i);
        tmp_z[i].number_red = virtual_cam[i]->blob_data.red_blobs;
        tmp_z[i].number_green = virtual_cam[i]->blob_data.green_blobs;
        tmp_z[i].number_blue = virtual_cam[i]->blob_data.blue_blobs;
                        
        for (int k = 0; k < tmp_z[i].number_red; k++){
            tmp_z[i].red_data[k] = virtual_cam[i]->blob_data.red[k].x;
            tmp_z[i].red_data[5+k] = virtual_cam[i]->blob_data.red[k].y;                
        }
            
        for (int k = 0; k < tmp_z[i].number_green; k++){
            tmp_z[i].green_data[k] = virtual_cam[i]->blob_data.green[k].x;
            tmp_z[i].green_data[5+k] = virtual_cam[i]->blob_data.green[k].y;                
        }

        for (int k = 0; k < tmp_z[i].number_blue; k++){
            tmp_z[i].blue_data[k] = virtual_cam[i]->blob_data.blue[k].x;
            tmp_z[i].blue_data[5+k] = virtual_cam[i]->blob_data.blue[k].y;                
        }            
            
        tmp_z[i].time = virtual_cam[i]->blob_data.timestamp;
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

char * PoseEstimation::get_concat_string(char * buffer, const char * pream, int id){
    char ascii_terminal[10];
    sprintf(ascii_terminal,"%d.xml",id);
    sprintf(buffer,PATH_TO_SERVER_MOUNT);
    strcat(buffer,pream);
    strcat(buffer,ascii_terminal);
    return buffer;
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
void PoseEstimation::normalizeVector(float array[], int len){
    unsigned char k;
    float norm = 0.0;

    for (k = 0; k < len; k++){
        norm += pow(array[k],2);
    }

    norm = sqrt(norm);

    for (k = 0; k < len; k++){
        array[k] = array[k]/norm;
    }
}

void PoseEstimation::quaternion_error(const float p[4], const float q[4], float q_err[4]){
  float q_conj[4] = {0};
  int i = 0;
  float b_p[16] = {0};
  int i0 = 0;

  q_conj[0] = q[0];
  for (i = 0; i < 3; i++) {
    q_conj[i + 1] = -q[i + 1];
  }

  /*  do quaternion multiplication */
  b_p[0] = p[0];
  b_p[4] = -p[1];
  b_p[8] = -p[2];
  b_p[12] = -p[3];
  b_p[1] = p[1];
  b_p[5] = p[0];
  b_p[9] = -p[3];
  b_p[13] = p[2];
  b_p[2] = p[2];
  b_p[6] = p[3];
  b_p[10] = p[0];
  b_p[14] = -p[1];
  b_p[3] = p[3];
  b_p[7] = -p[2];
  b_p[11] = p[1];
  b_p[15] = p[0];
  for (i = 0; i < 4; i++) {
    q_err[i] = 0.0F;
    for (i0 = 0; i0 < 4; i0++) {
      q_err[i] += b_p[i + (i0 << 2)] * (float)q_conj[i0];
    }
  }
}

void PoseEstimation::quaternionRotate(const float q1[4], const float q2[4], float q[4]){
  float b_q2[16];
  int i9;
  int i10;

  /*  rotation q1 by q2 */
  b_q2[0] = q2[0];
  b_q2[4] = -q2[1];
  b_q2[8] = -q2[2];
  b_q2[12] = -q2[3];
  b_q2[1] = q2[1];
  b_q2[5] = q2[0];
  b_q2[9] = q2[3];
  b_q2[13] = q2[2];
  b_q2[2] = q2[2];
  b_q2[6] = q2[3];
  b_q2[10] = q2[0];
  b_q2[14] = -q2[1];
  b_q2[3] = q2[3];
  b_q2[7] = -q2[2];
  b_q2[11] = q2[1];
  b_q2[15] = q2[0];
  for (i9 = 0; i9 < 4; i9++) {
    q[i9] = 0.0;
    for (i10 = 0; i10 < 4; i10++) {
      q[i9] += b_q2[i9 + (i10 << 2)] * q1[i10];
    }
  }
}

PoseEstimation::~PoseEstimation() {
    closeLog();
    for (int i = 0; i < NUM_OF_CAMERAS; i++){
        if (virtual_cam[i] != NULL){
            delete virtual_cam[i];
        }
    }    
    if (udp_lsq != NULL){
        delete udp_lsq;
        udp_lsq = NULL;
    }      
    pthread_mutex_destroy(&set_cam_mutex);
}

