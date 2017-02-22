/* 
 * File:   BlobAllocator.cpp
 * Author: Yashren Reddi
 * 
 * Created on 05 March 2013, 9:57 AM
 * 
 * The purpose of this class is to correctly allocate blobs so that the least squares
 * algorithm can use the right correspondence's. It does this by labeling each blob
 * in one image (say from camera 1) and finding the corresponding rays. Then the rays are
 * projected onto the image plane of the other cameras. For the blobs to correspond, they
 * must lie on the projected ray. 
 */

#include "../include/BlobAllocator.h"

BlobAllocator::BlobAllocator(OpencvCamera * cam1,OpencvCamera * cam2,Blob * blob_1, Blob * blob_2,
    Helicopter * heli1, Helicopter * heli2) {
    camera1 = cam1;
    camera2 = cam2;
    blob1 = blob_1;
    blob2 = blob_2;
    helicopter1 = heli1;
    helicopter2 = heli2;
    
    R1.set_size(3,3);
    R1.zeros();
    
    Rx.set_size(3,3);
    Rx.zeros();
    Rx(0,0) = 1; 
    Rx(1,1) = cos(M_PI); Rx(1,2) = -sin(M_PI);
    Rx(2,1) = sin(M_PI); Rx(2,2) = cos(M_PI);
    
    roll = pitch = yaw = 0;
    h1quat.q0 = h1quat.q1 = h1quat.q2 = h1quat.q3 = 0;
    tx = ty = tz = 0;
}

int BlobAllocator::allocateBlobs(void){
    // Use camera 1 as a reference camera, for which all other blobs are referred to
    
    // get the ray
    
//    double cam1[3] = {-2380,-1065,3200};
//    CvPoint3D32f ray_red = camera1->getRay(blob1->getBlob_data().red_blob_pos[0]);       
//    CvPoint3D32f ray_green = camera1->getRay(blob1->getBlob_data().green_blob_pos[0]);       
//    CvPoint3D32f ray_blue = camera1->getRay(blob1->getBlob_data().blue_blob_pos[0]);       
//    blob2->setRays(ray_red,ray_green,ray_blue);    
//    
//       double ray_red1[3] = {ray_red.x,ray_red.y,ray_red.z};
//        double ray_green1[3] = {ray_green.x,ray_green.y,ray_green.z};
//        double ray_blue1[3] = {ray_blue.x,ray_blue.y,ray_blue.z};    
//    
//    cvDrawLine(blob2->getFrame(),camera2->get2Dcoordinates(&ray_red1[0]),
//                camera2->get2Dcoordinates(&cam1[0]),CV_RGB(255,0,0),1);
//    cvDrawLine(blob2->getFrame(),camera2->get2Dcoordinates(&ray_green1[0]),
//                camera2->get2Dcoordinates(&cam1[0]),CV_RGB(0,255,0),1);
//    cvDrawLine(blob2->getFrame(),camera2->get2Dcoordinates(&ray_blue1[0]),
//                camera2->get2Dcoordinates(&cam1[0]),CV_RGB(0,0,255),1); 
    
    // For now we assume there is only one helicopter. So, order of blob_data is correct

//  lets just test the ls algorithm here for two cameras
    mat b, A, x, Pc1_red, Pc2_red, Pc1_green, Pc2_green, Pc1_blue, Pc2_blue;   
    
    // fill all "measurements"
    Pc1_red.set_size(3,1);
    Pc2_red.set_size(3,1);
    Pc1_red.zeros();
    Pc2_red.zeros();
    
    Pc1_red(0) = (blob1->getBlob_data())->red_blob_pos[0].x;
    Pc1_red(1) = (blob1->getBlob_data())->red_blob_pos[0].y;
    Pc1_red(2) = 1;
    
    Pc2_red(0) = (blob2->getBlob_data())->red_blob_pos[0].x;
    Pc2_red(1) = (blob2->getBlob_data())->red_blob_pos[0].y;
    Pc2_red(2) = 1;    
    
    Pc1_green.set_size(3,1);
    Pc2_green.set_size(3,1);
    Pc1_green.zeros();
    Pc2_green.zeros();    
    
    Pc1_green(0) = (blob1->getBlob_data())->green_blob_pos[0].x;
    Pc1_green(1) = (blob1->getBlob_data())->green_blob_pos[0].y;
    Pc1_green(2) = 1;
    
    Pc2_green(0) = (blob2->getBlob_data())->green_blob_pos[0].x;
    Pc2_green(1) = (blob2->getBlob_data())->green_blob_pos[0].y;
    Pc2_green(2) = 1;  
    
    Pc1_blue.set_size(3,1);
    Pc2_blue.set_size(3,1);
    Pc1_blue.zeros();
    Pc2_blue.zeros();    
    
    Pc1_blue(0) = (blob1->getBlob_data())->blue_blob_pos[0].x;
    Pc1_blue(1) = (blob1->getBlob_data())->blue_blob_pos[0].y;
    Pc1_blue(2) = 1;
    
    Pc2_blue(0) = (blob2->getBlob_data())->blue_blob_pos[0].x;
    Pc2_blue(1) = (blob2->getBlob_data())->blue_blob_pos[0].y;
    Pc2_blue(2) = 1;      
    
    if (((blob1->getBlob_data())->red_blob != 0) && 
            ((blob1->getBlob_data())->green_blob != 0) &&
            ((blob1->getBlob_data())->blue_blob != 0) &&
            ((blob2->getBlob_data())->red_blob != 0) && 
            ((blob2->getBlob_data())->green_blob != 0) &&
            ((blob2->getBlob_data())->blue_blob != 0)){
        
        mat T1 = camera1->getTArm();
        mat T2 = camera2->getTArm();
        mat Rcw1inv = camera1->getR_transposedArm();
        mat Rcw2inv = camera2->getR_transposedArm();
        mat Ainv1 = camera1->getIntrinsicInverseArm();
        mat Ainv2 = camera2->getIntrinsicInverseArm();
        
        mat red_blob_platform, green_blob_platform, blue_blob_platform;
        red_blob_platform.set_size(2,1);
        green_blob_platform.set_size(2,1);
        blue_blob_platform.set_size(2,1);
        
        red_blob_platform(0,0) = RED_BLOB_X;
        red_blob_platform(1,0) = RED_BLOB_Y;
        green_blob_platform(0,0) = GREEN_BLOB_X;
        green_blob_platform(1,0) = GREEN_BLOB_Y;        
        blue_blob_platform(0,0) = BLUE_BLOB_X;
        blue_blob_platform(1,0) = BLUE_BLOB_Y;        
        
        mat a1_red = Rcw1inv*Ainv1*Pc1_red;
        mat a1_green = Rcw1inv*Ainv1*Pc1_green;
        mat a1_blue = Rcw1inv*Ainv1*Pc1_blue;
                
        mat a2_red = Rcw2inv*Ainv2*Pc2_red;
        mat a2_green = Rcw2inv*Ainv2*Pc2_green;
        mat a2_blue = Rcw2inv*Ainv2*Pc2_blue;
        
        A.set_size(12,9);
        A.zeros();
        
        // row 1
        A(0,0) = red_blob_platform(0);         // red blob X
        A(0,1) = red_blob_platform(1);         // red blob Y
        A(0,4) = red_blob_platform(0)*-a1_red(0)/a1_red(2);
        A(0,5) = red_blob_platform(1)*-a1_red(0)/a1_red(2);
        A(0,6) = 1;
        A(0,7) = 0;
        A(0,8) = -a1_red(0)/a1_red(2);

        // row 2
        A(1,2) = red_blob_platform(0);       
        A(1,3) = red_blob_platform(1);       
        A(1,4) = red_blob_platform(0)*-a1_red(1)/a1_red(2);
        A(1,5) = red_blob_platform(1)*-a1_red(1)/a1_red(2);
        A(1,6) = 0;
        A(1,7) = 1;
        A(1,8) = -a1_red(1)/a1_red(2);
        
        // row 3
        A(2,0) = green_blob_platform(0);         
        A(2,1) = green_blob_platform(1);         
        A(2,4) = green_blob_platform(0)*-a1_green(0)/a1_green(2);
        A(2,5) = green_blob_platform(1)*-a1_green(0)/a1_green(2);
        A(2,6) = 1;
        A(2,7) = 0;
        A(2,8) = -a1_green(0)/a1_green(2);

        // row 4
        A(3,2) = green_blob_platform(0);       
        A(3,3) = green_blob_platform(1);       
        A(3,4) = green_blob_platform(0)*-a1_green(1)/a1_green(2);
        A(3,5) = green_blob_platform(1)*-a1_green(1)/a1_green(2);
        A(3,6) = 0;
        A(3,7) = 1;
        A(3,8) = -a1_green(1)/a1_green(2);        

        // row 5
        A(4,0) = blue_blob_platform(0);         
        A(4,1) = blue_blob_platform(1);         
        A(4,4) = blue_blob_platform(0)*-a1_blue(0)/a1_blue(2);
        A(4,5) = blue_blob_platform(1)*-a1_blue(0)/a1_blue(2);
        A(4,6) = 1;
        A(4,7) = 0;
        A(4,8) = -a1_blue(0)/a1_blue(2);

        // row 6
        A(5,2) = blue_blob_platform(0);       
        A(5,3) = blue_blob_platform(1);       
        A(5,4) = blue_blob_platform(0)*-a1_blue(1)/a1_blue(2);
        A(5,5) = blue_blob_platform(1)*-a1_blue(1)/a1_blue(2);
        A(5,6) = 0;
        A(5,7) = 1;
        A(5,8) = -a1_blue(1)/a1_blue(2);        
        
        // row 7
        A(6,0) = red_blob_platform(0);         // red blob X
        A(6,1) = red_blob_platform(1);         // red blob Y
        A(6,4) = red_blob_platform(0)*-a2_red(0)/a2_red(2);
        A(6,5) = red_blob_platform(1)*-a2_red(0)/a2_red(2);
        A(6,6) = 1;
        A(6,7) = 0;
        A(6,8) = -a2_red(0)/a2_red(2);

        // row 8
        A(7,2) = red_blob_platform(0);       
        A(7,3) = red_blob_platform(1);       
        A(7,4) = red_blob_platform(0)*-a2_red(1)/a2_red(2);
        A(7,5) = red_blob_platform(1)*-a2_red(1)/a2_red(2);
        A(7,6) = 0;
        A(7,7) = 1;
        A(7,8) = -a2_red(1)/a2_red(2);
        
        // row 9
        A(8,0) = green_blob_platform(0);         
        A(8,1) = green_blob_platform(1);         
        A(8,4) = green_blob_platform(0)*-a2_green(0)/a2_green(2);
        A(8,5) = green_blob_platform(1)*-a2_green(0)/a2_green(2);
        A(8,6) = 1;
        A(8,7) = 0;
        A(8,8) = -a2_green(0)/a2_green(2);

        // row 10
        A(9,2) = green_blob_platform(0);       
        A(9,3) = green_blob_platform(1);       
        A(9,4) = green_blob_platform(0)*-a2_green(1)/a2_green(2);
        A(9,5) = green_blob_platform(1)*-a2_green(1)/a2_green(2);
        A(9,6) = 0;
        A(9,7) = 1;
        A(9,8) = -a2_green(1)/a2_green(2);        

        // row 11
        A(10,0) = blue_blob_platform(0);         
        A(10,1) = blue_blob_platform(1);         
        A(10,4) = blue_blob_platform(0)*-a2_blue(0)/a2_blue(2);
        A(10,5) = blue_blob_platform(1)*-a2_blue(0)/a2_blue(2);
        A(10,6) = 1;
        A(10,7) = 0;
        A(10,8) = -a2_blue(0)/a2_blue(2);

        // row 12
        A(11,2) = blue_blob_platform(0);       
        A(11,3) = blue_blob_platform(1);       
        A(11,4) = blue_blob_platform(0)*-a2_blue(1)/a2_blue(2);
        A(11,5) = blue_blob_platform(1)*-a2_blue(1)/a2_blue(2);
        A(11,6) = 0;
        A(11,7) = 1;
        A(11,8) = -a2_blue(1)/a2_blue(2);                
  
        
        b.set_size(12,1);
        b(0,0) = T1(0,0)-T1(2,0)*a1_red(0,0)/a1_red(2,0);
        b(1,0) = T1(1,0)-T1(2,0)*a1_red(1,0)/a1_red(2,0);
        b(2,0) = T1(0,0)-T1(2,0)*a1_green(0,0)/a1_green(2,0);
        b(3,0) = T1(1,0)-T1(2,0)*a1_green(1,0)/a1_green(2,0);
        b(4,0) = T1(0,0)-T1(2,0)*a1_blue(0,0)/a1_blue(2,0);
        b(5,0) = T1(1,0)-T1(2,0)*a1_blue(1,0)/a1_blue(2,0);
        b(6,0) = T2(0,0)-T2(2,0)*a2_red(0,0)/a2_red(2,0);
        b(7,0) = T2(1,0)-T2(2,0)*a2_red(1,0)/a2_red(2,0);
        b(8,0) = T2(0,0)-T2(2,0)*a2_green(0,0)/a2_green(2,0);
        b(9,0) = T2(1,0)-T2(2,0)*a2_green(1,0)/a2_green(2,0);
        b(10,0) = T2(0,0)-T2(2,0)*a2_blue(0,0)/a2_blue(2,0);
        b(11,0) = T2(1,0)-T2(2,0)*a2_blue(1,0)/a2_blue(2,0);        
        
        if(solve(x,A,b))
        {
            storeRotandTrans(x);
            return 100;
        }
        else{
            cout << "No solution found." << endl;
            
            return 0;
        }
    }  
    else
    {
        return 0;
    }
}

void BlobAllocator::storeRotandTrans(mat x){
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
}

float BlobAllocator::getRoll(void){
    return roll;
}

float BlobAllocator::getPitch(void){
    return pitch;
}

float BlobAllocator::getYaw(void){
    return yaw;
}

float BlobAllocator::gettx(void){
    return tx;
}

float BlobAllocator::getty(void){
    return ty;
}

float BlobAllocator::gettz(void){
    return tz;
}


BlobAllocator::~BlobAllocator() {
}

struct quaternion * BlobAllocator::calculateQuaternion(mat R){
    float r11 = R(0,0), r12 = R(0,1),r13 = R(0,2);
    float r21 = R(1,0), r22 = R(1,1),r23 = R(1,2);
    float r31 = R(2,0), r32 = R(2,1),r33 = R(2,2);
    float qd;
    if ((r22>-r33) && (r11>-r22) && (r11>-r33)){
        qd = sqrt(1+r11+r22+r33);
        h1quat.q0 = 0.5*qd;
        h1quat.q1 = 0.5*(r23 - r32)/qd;
        h1quat.q2 = 0.5*(r31 - r13)/qd;
        h1quat.q3 = 0.5*(r12 - r21)/qd;
    }

    if ((r22<-r33) && (r11>r22) && (r11>r33)){    
        qd = sqrt(1+r11-r22-r33);
        h1quat.q1 = 0.5*qd;
        h1quat.q0 = 0.5*(r23 - r32)/qd;
        h1quat.q2 = 0.5*(r12 + r21)/qd;
        h1quat.q3 = 0.5*(r31 + r13)/qd;
    }

    if ((r22>r33) && (r11<r22) && (r11<-r33)){
        qd = sqrt(1-r11+r22-r33);
        h1quat.q2 = 0.5*qd;
        h1quat.q0 = 0.5*(r31 - r13)/qd;
        h1quat.q1 = 0.5*(r12 + r21)/qd;
        h1quat.q3 = 0.5*(r23 + r32)/qd;
    }

    if ((r22<r33) && (r11<-r22) && (r11<r33)){
        qd = sqrt(1-r11-r22+r33);
        h1quat.q3 = 0.5*qd;
        h1quat.q0 = 0.5*(r12 - r21)/qd;
        h1quat.q1 = 0.5*(r31 + r13)/qd;
        h1quat.q2 = 0.5*(r23 + r32)/qd;
    }   
    
    // Normalize quaternion
    float norm_q = sqrt(pow(h1quat.q0,2)+pow(h1quat.q1,2)+pow(h1quat.q2,2)+pow(h1quat.q3,2));    
    h1quat.q0 /= -norm_q;
    h1quat.q1 /= -norm_q;
    h1quat.q2 /= -norm_q;
    h1quat.q3 /= -norm_q;
   
    // This quaternion represents the rotation between helicopter body (classical)
    // and earth frame
    
    // complement quat for body to earth
    float q0 = h1quat.q0;
    float q1 = h1quat.q1;
    float q2 = h1quat.q2;
    float q3 = h1quat.q3;
    roll = -atan2(2*q2*q3+2*q0*q1,pow(q3,2)-pow(q2,2)-pow(q1,2)+pow(q0,2));
    if (roll > 0)
        roll -= M_PI;
    else
        roll += M_PI;
    pitch = asin(2*q1*q3-2*q0*q2);
    yaw = -atan2(2*q1*q2+2*q0*q3,pow(q1,2)+pow(q0,2)-pow(q3,2)-pow(q2,2));   
           
    return &h1quat;
}

struct quaternion * BlobAllocator::getQuaternion(void){
    return &h1quat;
}
    
float BlobAllocator::SIGN(float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

float BlobAllocator::NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}