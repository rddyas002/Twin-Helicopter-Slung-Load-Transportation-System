
#include "PoseEstimation.h"
#include "solvePoseEst.h"
#include <locale.h>
#include <stdlib.h>

//tbb::mutex PoseEstimation::set_cam_mutex;

PoseEstimation::PoseEstimation() {
    
    char string_buffer[256];
    clearCamDataReceived();
    ekf_on = false;
    FPS_counter = 0;
    initialising = true;
    quaternion_offset[0] = quaternion_offset[2] = quaternion_offset[3] = 0;
    quaternion_offset[1] = 1;
    
    if (pthread_mutex_init(&set_cam_mutex, NULL) != 0){
        std::cout << "Mutex initialisation failed" << std::endl;
    }

    for (int v_cam = 0; v_cam < NUM_OF_CAMERAS; v_cam++){
        if (v_cam < NUM_OF_CAMERAS){
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
                 << -RED_BLOB_Y << endr;
    Green_platform << GREEN_BLOB_X << endr
                 << -GREEN_BLOB_Y << endr;
    Blue_platform << BLUE_BLOB_X << endr
                 << -BLUE_BLOB_Y << endr;    
    
    red_blob3D[0] = RED_BLOB_X;
    red_blob3D[1] = RED_BLOB_Y;
    red_blob3D[2] = RED_BLOB_Z;
    green_blob3D[0] = GREEN_BLOB_X;
    green_blob3D[1] = GREEN_BLOB_Y;
    green_blob3D[2] = GREEN_BLOB_Z;
    blue_blob3D[0] = BLUE_BLOB_X;
    blue_blob3D[1] = BLUE_BLOB_Y;
    blue_blob3D[2] = BLUE_BLOB_Z;
    
    setObjectPoints();    
}

OpencvCamera * PoseEstimation::getVirtualCamera(int index){
        return virtual_cam[index];
}

void PoseEstimation::setObjectPoints(void){
    objectPoints[0] = red_blob3D[0];
    objectPoints[1] = red_blob3D[1];
    objectPoints[2] = red_blob3D[2];
    objectPoints[3] = green_blob3D[0];
    objectPoints[4] = green_blob3D[1];
    objectPoints[5] = green_blob3D[2];
    objectPoints[6] = blue_blob3D[0];
    objectPoints[7] = blue_blob3D[1];
    objectPoints[8] = blue_blob3D[2];    
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
    real_T flip[4] = {0,1,0,0};
    real_T ret_quat[4];
    real_T quat[4] = {h1q.q0,h1q.q1,h1q.q2,h1q.q3};
    quaternionRotation(&quat[0], &flip[0], &ret_quat[0]);
    eulerAnglesFromQuaternion(&ret_quat[0], &roll, &pitch, &yaw);     
    roll *= M_PI/180;
    pitch *= M_PI/180;
    yaw *= M_PI/180;
    

    return &h1q;
}

quaternion PoseEstimation::getQuaternion(void){
    return h1q;
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

bool PoseEstimation::lsqnon_Estimation(real_T init_x[7], real_T ret_x[7], real_T * resnorm){
    b_struct_T z[4];
    // fill struct with camera data
    fillCamStruct(&z[0]);
    for (int i = 0; i < LSQ_NON_ITERATONS; i++){
        solvePoseEst(init_x, z, objectPoints, &ret_x[0], resnorm);
        memcpy(&init_x[0], &ret_x[0], sizeof(real_T)*7);
    }
    
    if (*resnorm < 20){
        return true;
    }
    
    return false;
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
    Pimage_red.set_size(3,NUM_OF_CAMERAS);
    Pimage_green.set_size(3,NUM_OF_CAMERAS);
    Pimage_blue.set_size(3,NUM_OF_CAMERAS);
    Pimage_red.zeros();
    Pimage_green.zeros();
    Pimage_blue.zeros();
    
    bool valid_cam_data[NUM_OF_CAMERAS] = {false};    
    
    for (i = 0; i < NUM_OF_CAMERAS; i++){
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
        
    Trans.set_size(3,NUM_OF_CAMERAS); Trans.zeros();
    A_red.set_size(3,NUM_OF_CAMERAS); A_red.zeros();
    A_green.set_size(3,NUM_OF_CAMERAS); A_green.zeros();
    A_blue.set_size(3,NUM_OF_CAMERAS); A_blue.zeros();
    Rot_t.set_size(3,3,NUM_OF_CAMERAS); Rot_t.zeros();
    Intr_inv.set_size(3,3,NUM_OF_CAMERAS); Intr_inv.zeros();
        
    int j = 0;
    for (i = 0; i < NUM_OF_CAMERAS; i++){
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
            return true;
        }
        else
        {
            cout << "No solution found." << endl;
            return false;
        }  
    }
    else
        return false;
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
    
    tx = x(6,0);
    ty = x(7,0);
    tz = x(8,0);
}

void PoseEstimation::setHelicopterReference(Helicopter * heli){
    helicopter_link = heli;
}

void PoseEstimation::setCamDataReceived(int val){   
    static double q_init_sum[4] = {0};
    static double trans_init_sum[3] = {0};
    static int counter_sum = 0;
    static int try_counter = 0;
    
    pthread_mutex_lock(&set_cam_mutex);
    if (dataFromCamReceived[val - 1] == true)
        std::cout << "scan error" << std::endl;
    
    dataFromCamReceived[val - 1] = true;
    
    for (int i = 0; i < NUM_OF_CAMERAS; i++){
        if(!dataFromCamReceived[i]){
            pthread_mutex_unlock(&set_cam_mutex);
            return;
        }
    }
       
    if (ekf_on)
        doCorrection();
    else{
        // If in initialisation phase get a sample of initial quaternion
        // for zero attitude estimate     
        if (initialising){
            // get 20 valid samples of the quaternion
            real_T init_x[7] = {INIT_POSE_X,INIT_POSE_Y,INIT_POSE_Z,
                INIT_POSE_Q1, INIT_POSE_Q2, INIT_POSE_Q3, INIT_POSE_Q4};
            real_T ret_x[7];
            real_T norm = 0;
            
            if(lsqnon_Estimation(init_x, ret_x, &norm)){
                counter_sum++;
                q_init_sum[0] += ret_x[3];//h1q.q0;
                q_init_sum[1] += ret_x[4];//h1q.q1;
                q_init_sum[2] += ret_x[5];//h1q.q2;
                q_init_sum[3] += ret_x[6];//h1q.q3;
                
                trans_init_sum[0] += ret_x[0];
                trans_init_sum[1] += ret_x[1];
                trans_init_sum[2] += ret_x[2];                
                
                if (counter_sum == INIT_QUAT_EST_ITER){
                    double offset[4];
                    q_init_sum[0] /= INIT_QUAT_EST_ITER;
                    q_init_sum[1] /= INIT_QUAT_EST_ITER;
                    q_init_sum[2] /= INIT_QUAT_EST_ITER;
                    q_init_sum[3] /= INIT_QUAT_EST_ITER;
                    
                    trans_init_sum[0] /= INIT_QUAT_EST_ITER;
                    trans_init_sum[1] /= INIT_QUAT_EST_ITER;
                    trans_init_sum[2] /= INIT_QUAT_EST_ITER;
                    
                    offset[0] = -q_init_sum[1];
                    offset[1] = q_init_sum[0];
                    offset[2] = -q_init_sum[3];
                    offset[3] = q_init_sum[2];                   
                    
////                    // normalise average quaternion
                    float norm_q = sqrt(pow(offset[0],2)+pow(offset[1],2)+pow(offset[2],2)+pow(offset[3],2));    
                    offset[0] /= norm_q;
                    offset[1] /= norm_q;
                    offset[2] /= norm_q;
                    offset[3] /= norm_q;                    
                    
                    double temp[3];
                    Reb(&offset[0], &red_blob3D[0], &temp[0]);
                    memcpy(&red_blob3D[0],&temp[0], sizeof(double)*3);
                    Reb(&offset[0], &green_blob3D[0], &temp[0]);
                    memcpy(&green_blob3D[0],&temp[0], sizeof(double)*3);
                    Reb(&offset[0], &blue_blob3D[0], &temp[0]);
                    memcpy(&blue_blob3D[0],&temp[0], sizeof(double)*3);  
                    
                    setObjectPoints();    
                    real_T init_x2[7] = {trans_init_sum[0],trans_init_sum[1],trans_init_sum[2],
                        q_init_sum[0],q_init_sum[1],q_init_sum[2],q_init_sum[3]};
                    real_T ret_x2[7] = {0};
                    // Now get pose with updated blob positions
                    lsqnon_Estimation(init_x2, ret_x2, &norm);
                    
                    initialising = false;
                    std::cout << "Initialisation done" << std::endl;
                    printf("New blob coordinates are: \r\n");
                    printf("RED: %.2f, %.2f, %.2f\r\n", red_blob3D[0], red_blob3D[1], red_blob3D[2]);
                    printf("GREEN: %.2f, %.2f, %.2f\r\n", green_blob3D[0], green_blob3D[1], green_blob3D[2]);
                    printf("BLUE: %.2f, %.2f, %.2f\r\n", blue_blob3D[0], blue_blob3D[1], blue_blob3D[2]);
                    printf("Initial translation: %.0f, %.0f, %.0f (mm)\r\n", ret_x2[0]*1000, ret_x2[1]*1000, ret_x2[2]*1000);
                    real_T roll_err, pitch_err, yaw_err;
                    real_T quat_err[4] = {-ret_x2[4],ret_x2[3],-ret_x2[6],ret_x2[5]};
                    eulerAnglesFromQuaternion(&quat_err[0], &roll_err, &pitch_err, &yaw_err);
                    printf("Rotational error: %.4f, %.4f, %.4f (degrees)\r\n\r\n", roll_err, pitch_err, yaw_err);
                    memcpy(&offset_quaternion[0],&offset[0],sizeof(real_T)*4);
                    memcpy(&initial_translation[0],&trans_init_sum[0],sizeof(real_T)*3);
                    
                    real_T * ekf_state = helicopter_link->kalmanState();
                    
                    ekf_state[0] = ret_x2[0];
                    ekf_state[1] = ret_x2[1];
                    ekf_state[2] = ret_x2[2];
                    ekf_state[6] = ret_x2[3];
                    ekf_state[7] = ret_x2[4];
                    ekf_state[8] = ret_x2[5];
                    ekf_state[9] = ret_x2[6];
                }
            }
        }
    }
    
    clearCamDataReceived();
    pthread_mutex_unlock(&set_cam_mutex);
}

bool PoseEstimation::isInitialised(void){
    return !initialising;
}

void PoseEstimation::enableEKFCorrection(bool en){
    ekf_on = en;
}

double PoseEstimation::getFPS(void){
    return FPS;
}

void PoseEstimation::doCorrection(void){
    static double prev_correction_timestamp = 0;    
    int num_red = 0, num_green = 0, num_blue = 0;
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
        
//    real_T objectPoints[9] = {-140,264,0,-144,-263,0,314,0,0};
//    real_T objectPoints[9] = {red_blob3D[0],red_blob3D[1],red_blob3D[2],
//        green_blob3D[0],green_blob3D[1],green_blob3D[2],
//        blue_blob3D[0],blue_blob3D[1],blue_blob3D[2]};
    real_T meas_noise = MEASUREMENT_NOISE_VARIANCE;// // pixel^2
    real_T xCorr[16];
    real_T PCorr[256];
        
    correctStateAndCov(&x[0], &z[0], &P[0], &objectPoints[0], meas_noise, &xCorr[0], &PCorr[0]);
        
    memcpy(helicopter_link->kalmanState(),&xCorr[0],sizeof(double)*16);
    memcpy(helicopter_link->kalmanErrorCovariance(),&PCorr[0],sizeof(double)*256);
    
    clearCamDataReceived();

    active_blobs = num_red + num_green + num_blue;
    if ((num_red > 1) && (num_green > 1) && (num_blue > 1)){
        if (active_blobs > 6){
            if (P[34] < 0.2){
                *helicopter_link->lastCorrection() = timeSince(0);
            }
//            else
//               printf("[%f],C1[%.0f,%.0f,%.0f],C2[%.0f,%.0f,%.0f],C3[%.0f,%.0f,%.0f],C4[%.0f,%.0f,%.0f]\r\n",
//                       P[34],
//                       z[0].number_red,z[0].number_green,z[0].number_blue,
//                       z[1].number_red,z[1].number_green,z[1].number_blue,
//                       z[2].number_red,z[2].number_green,z[2].number_blue,
//                       z[3].number_red,z[3].number_green,z[3].number_blue);
       }
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
    for (int i = 0; i < NUM_OF_CAMERAS; i++){
        if (virtual_cam[i] != NULL){
            delete virtual_cam[i];
        }
    }    
    pthread_mutex_destroy(&set_cam_mutex);
}

