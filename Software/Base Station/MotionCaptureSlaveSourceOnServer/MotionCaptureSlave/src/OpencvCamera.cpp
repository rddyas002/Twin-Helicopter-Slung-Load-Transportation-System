/* 
 * File:   OpencvCamera.cpp
 * Author: Yashren Reddi
 * 
 * Created on 13 November 2012, 10:59 AM
 * This class is intended to create the camera object and hold all the parameters 
 * relating to a specific camera i.e. resolution, distortion terms, intrinsic and
 * extrinsic matrices. It also has an autoCalibration method which fine tunes the 
 * rotation matrix to match the 2D projection.
 */

#include "../inc/OpencvCamera.h"

OpencvCamera::OpencvCamera(int fd, int ident) {
    id = ident;
    camera = NULL;
    pFile = NULL;
    object_points = NULL;
    intrinsic = NULL;
    distortion = NULL;

    // open object points for auto calibration
    pFile = fopen(PATH_TO_OBJECT_POINTS,"r");
    if (pFile == NULL)
        std::cout << "Cannot find object_points.txt" << std::endl;
    else
    {
        object_points = cvCreateMat(6,1,CV_32FC3);
        
        fscanf(pFile,"%f,%f,%f;%f,%f,%f;%f,%f,%f;%f,%f,%f;%f,%f,%f;%f,%f,%f;",
            &Po[0],&Po[1],&Po[2],
            &Po[3],&Po[4],&Po[5],
            &Po[6],&Po[7],&Po[8],
            &Po[9],&Po[10],&Po[11],
            &Po[12],&Po[13],&Po[14],
            &Po[15],&Po[16],&Po[17]); 
        
        CV_MAT_ELEM(*object_points,CvPoint3D32f,0,0) = cvPoint3D32f(Po[0],Po[1],Po[2]); 
        CV_MAT_ELEM(*object_points,CvPoint3D32f,1,0) = cvPoint3D32f(Po[3],Po[4],Po[5]); 
        CV_MAT_ELEM(*object_points,CvPoint3D32f,2,0) = cvPoint3D32f(Po[6],Po[7],Po[8]); 
        CV_MAT_ELEM(*object_points,CvPoint3D32f,3,0) = cvPoint3D32f(Po[9],Po[10],Po[11]); 
        CV_MAT_ELEM(*object_points,CvPoint3D32f,4,0) = cvPoint3D32f(Po[12],Po[13],Po[14]); 
        CV_MAT_ELEM(*object_points,CvPoint3D32f,5,0) = cvPoint3D32f(Po[15],Po[16],Po[17]);
        
        setCalibObjectPoints(object_points);

//        std::cout << Po[0] << " " << Po[1] << " " << Po[2] << std::endl;
//        std::cout << Po[3] << " " << Po[4] << " " << Po[5] << std::endl;
//        std::cout << Po[6] << " " << Po[7] << " " << Po[8] << std::endl;
//        std::cout << Po[9] << " " << Po[10] << " " << Po[11] << std::endl;
//        std::cout << Po[12] << " " << Po[15] << " " << Po[14] << std::endl;
//        std::cout << Po[15] << " " << Po[16] << " " << Po[17] << std::endl;
        
    }
        
    
    sprintf(windowName, "Camera %d", id);
       
    if (fd != -1)
    {    
        camera = cvCaptureFromCAM(fd);   
    
        if (camera == NULL)
        {
                std::cout << "Camera " << ident << " not found." << std::endl;
                //exit(1);
        }
        else
        {
                if (RES640X480){
                        cvSetCaptureProperty(camera, CV_CAP_PROP_FRAME_WIDTH, 640);
                        cvSetCaptureProperty(camera, CV_CAP_PROP_FRAME_HEIGHT, 480);
                }
                else{
                        cvSetCaptureProperty(camera, CV_CAP_PROP_FRAME_WIDTH, 320);
                        cvSetCaptureProperty(camera, CV_CAP_PROP_FRAME_HEIGHT, 240);
                }            
            
                double  w_frame = cvGetCaptureProperty(camera,CV_CAP_PROP_FRAME_WIDTH);
                double  h_frame = cvGetCaptureProperty(camera,CV_CAP_PROP_FRAME_HEIGHT);            
                
                std::cout << "Resolution set to " << w_frame << "x" << h_frame << std::endl;
                
                cvSetCaptureProperty(camera, CV_CAP_PROP_FPS, 125);
                std::cout << "FPS set to 125" << std::endl;
                
                // Do last - to use as reference size/image
                frame = cvQueryFrame(camera);          
//                cvNamedWindow(windowName,CV_WINDOW_AUTOSIZE);
//                cvShowImage(windowName, frame);
        }
    }

    // if fd is null -- means the object is used as a virtual camera to 
    // store projection data

}


IplImage * OpencvCamera::getFrame(void){
      cvGrabFrame(camera);
      cvGrabFrame(camera);      
      return cvRetrieveFrame(camera);
}

IplImage * OpencvCamera::queryFrame(void){
    cvQueryFrame(camera);
    return cvQueryFrame(camera);
}

void OpencvCamera::setIntrinsic(CvMat* intrins){
    intrinsic = intrins;
    
    intrinsic_array[0] = CV_MAT_ELEM(*intrinsic, float, 0, 0);
    fx = intrinsic_array[0];
    intrinsic_array[1] = CV_MAT_ELEM(*intrinsic, float, 0, 2);
    cx = intrinsic_array[1];
    intrinsic_array[2] = CV_MAT_ELEM(*intrinsic, float, 1, 1);
    fy = intrinsic_array[2];
    intrinsic_array[3] = CV_MAT_ELEM(*intrinsic, float, 1, 2);    
    cy = intrinsic_array[3];
}

void OpencvCamera::setCamPosition(CvMat* posit){
    
    position_array[0] = CV_MAT_ELEM(*posit, float, 0, 0);
    position_array[1] = CV_MAT_ELEM(*posit, float, 1, 0);
    position_array[2] = CV_MAT_ELEM(*posit, float, 2, 0);
}

void OpencvCamera::setCenter(CvMat* posit){
    
    center_array[0] = CV_MAT_ELEM(*posit, float, 0, 0);
    center_array[1] = CV_MAT_ELEM(*posit, float, 1, 0);
    center_array[2] = CV_MAT_ELEM(*posit, float, 2, 0);
}

void OpencvCamera::setCalibObjectPoints(CvMat* posit){
    Po1.set_size(3,1); Po2.set_size(3,1); Po3.set_size(3,1);  Po4.set_size(3,1);

    CvPoint3D32f p1 = CV_MAT_ELEM(*posit,CvPoint3D32f,0,0);
    CvPoint3D32f p2 = CV_MAT_ELEM(*posit,CvPoint3D32f,1,0);
    CvPoint3D32f p3 = CV_MAT_ELEM(*posit,CvPoint3D32f,2,0);
    CvPoint3D32f p4 = CV_MAT_ELEM(*posit,CvPoint3D32f,3,0);
    
    Po1(0,0) = p1.x; 
    Po1(1,0) = p1.y;
    Po1(2,0) = p1.z;
            
    Po2(0,0) = p2.x; 
    Po2(1,0) = p2.y;
    Po2(2,0) = p2.z;
    
    Po3(0,0) = p3.x;
    Po3(1,0) = p3.y;
    Po3(2,0) = p3.z;
    
    Po4(0,0) = p4.x;
    Po4(1,0) = p4.y;
    Po4(2,0) = p4.z;    
}

IplImage * OpencvCamera::get_mapx(void){
    return mapx;
}

IplImage * OpencvCamera::get_mapy(void){
    return mapy;    
}

mat OpencvCamera::getPo1(void){
    return Po1;
}

mat OpencvCamera::getPo2(void){
    return Po2;
}

mat OpencvCamera::getPo3(void){
    return Po3;
}

void OpencvCamera::loadRotationAngles(CvMat* thetas){
    theta_z = CV_MAT_ELEM(*thetas, float, 0, 0);
    theta_x = CV_MAT_ELEM(*thetas, float, 1, 0);
    theta_z2 = CV_MAT_ELEM(*thetas, float, 2, 0);
}

void OpencvCamera::calcExtrinsics(void){
    // Put intrinsic in matrix form
    A.set_size(3,3);
    A.zeros();
    A << intrinsic_array[0] << 0. << intrinsic_array[1] << endr
       << 0. << intrinsic_array[2] << intrinsic_array[3] << endr
       << 0. << 0. << 1. << endr;   
    
    Ainv = inv(A);
    
    T.set_size(3,1);
    T << CAMERA_X << endr
        << CAMERA_Y << endr
        << CAMERA_Z << endr;  
    
    // Set rotation matrix size
    Rz.set_size(3,3); Rx.set_size(3,3); Rz2.set_size(3,3);
    Rz.zeros(); Rx.zeros(); Rz2.zeros();    
    
    // calculate rotation angles
    //set_thetas();
    
    // generate rotation matrices
    Rz << cos(theta_z) << sin(theta_z) << 0 << endr
        << -sin(theta_z) << cos(theta_z) << 0 << endr
        << 0. << 0. << 1. << endr;

    Rx << 1 << 0 << 0 << endr
        << 0 << cos(theta_x) << sin(theta_x) << endr
        << 0 << -sin(theta_x) << cos(theta_x) << endr;

    Rz2 << cos(theta_z2) << sin(theta_z2) << 0 << endr
        << -sin(theta_z2) << cos(theta_z2) << 0 << endr
        << 0. << 0. << 1. << endr;   
        
    R = Rz2*Rx*Rz;
    
    Rtrans = inv(R);
                
    AR = A*R;
    ART = A*R*T;    
}

void OpencvCamera::set_thetas(void)
{
    double y_diff = CAMERA_Y - CENTER_Y;
    double x_diff = CAMERA_X - CENTER_X;

    if (y_diff > 0)
        theta_z = 180 - atan(x_diff/y_diff)*180/M_PI;
    else
        theta_z = -atan(x_diff/y_diff)*180/M_PI;

    theta_x = -(90 + atan((CAMERA_Z - CENTER_Z)/(sqrt(pow(x_diff, 2)+ pow(y_diff, 2))))*180/M_PI);

    // convert to radians
    theta_z *= M_PI/180;
    theta_x *= M_PI/180;
    theta_z2 = 0;
        
    cout << "Camera " << getID() << ": " << "Original angles - theta_x: " << theta_x*180/M_PI
            << " theta_z: " << theta_z*180/M_PI
            << " theta_z2: " << theta_z2*180/M_PI << endl;    
}

CvPoint OpencvCamera::get2Dcoordinates(double * objectPoint)
{
    mat Po;
    Po.set_size(3,1);
    Po.zeros();

    Po << objectPoint[0] << endr
       << objectPoint[1] << endr
       << objectPoint[2] << endr;

    // perform 3D to 2D mapping
    mat Pc = R*(Po - T);
    //std::cout << R << std::endl;
    mat q = A*Pc;
    q = q/q(2,0);

    return cvPoint((int)q(0,0), (int)q(1,0));
}

void OpencvCamera::autoCalib(CvMat* imagePoints){
    Pc1.set_size(3,1); Pc2.set_size(3,1); Pc3.set_size(3,1);Pc4.set_size(3,1);
    Pc1.zeros(); Pc2.zeros(); Pc3.zeros(); Pc4.zeros();
    
    CvPoint2D32f iP1 = CV_MAT_ELEM(*imagePoints,CvPoint2D32f,0,0);
    CvPoint2D32f iP2 = CV_MAT_ELEM(*imagePoints,CvPoint2D32f,1,0);
    CvPoint2D32f iP3 = CV_MAT_ELEM(*imagePoints,CvPoint2D32f,2,0);
    CvPoint2D32f iP4 = CV_MAT_ELEM(*imagePoints,CvPoint2D32f,3,0);
    
    Pc1(0,0) = iP1.x; 
    Pc1(1,0) = iP1.y; 
    Pc1(2,0) = 1;
    Pc2(0,0) = iP2.x; 
    Pc2(1,0) = iP2.y; 
    Pc2(2,0) = 1;
    Pc3(0,0) = iP3.x; 
    Pc3(1,0) = iP3.y; 
    Pc3(2,0) = 1;
    Pc4(0,0) = iP4.x; 
    Pc4(1,0) = iP4.y; 
    Pc4(2,0) = 1;    
       
    refineAngles();    
}

void OpencvCamera::refineAngles(void)
{
    int iterations = REFINE_CALIB_ITER;
    mat L1 = Po1 - T;
    mat L2 = Po2 - T;
    mat L3 = Po3 - T;
    mat L4 = Po4 - T;

    mat r1, r2, r3, r4;
    mat b1, b2, b3, b4;
    vec solution;

    mat Ablock;
    mat Bblock;
    
    old_theta_x = theta_x;
    old_theta_z = theta_z;
    old_theta_z = theta_z2;
    
    printf("Camera %d original angles\n\
    theta_x: %.2f\n\
    theta_z: %.2f\n\
    theta_z2: %.2f\r\n", getID(), theta_x*180/M_PI,theta_z*180/M_PI,theta_z2*180/M_PI);

    for (int i = 0; i < iterations; i++)
    {
        r1 = getRot(L1(0),L1(1),L1(2));
        r2 = getRot(L2(0),L2(1),L2(2));
        r3 = getRot(L3(0),L3(1),L3(2));
        r4 = getRot(L4(0),L4(1),L4(2));
        b1 = getB(L1(0),L1(1),L1(2), Pc1);
        b2 = getB(L2(0),L2(1),L2(2), Pc2);
        b3 = getB(L3(0),L3(1),L3(2), Pc3);
        b4 = getB(L4(0),L4(1),L4(2), Pc4);

        Bblock = join_cols(b1, b2);
        Bblock.insert_rows(Bblock.n_rows, b3);
        Bblock.insert_rows(Bblock.n_rows, b4);

        Ablock.insert_rows(Ablock.n_rows, r1);
        Ablock.insert_rows(Ablock.n_rows, r2);
        Ablock.insert_rows(Ablock.n_rows, r3);
        Ablock.insert_rows(Ablock.n_rows, r4);

        solve(solution,Ablock,Bblock);

        theta_x += solution(0);
        theta_z += solution(1);
        theta_z2 += solution(2);

        Ablock.reset();
        Bblock.reset();
    }
    
    printf("Camera %d refined angles\n\
    theta_x: %.2f\n\
    theta_z: %.2f\n\
    theta_z2: %.2f\r\n", getID(), theta_x*180/M_PI,theta_z*180/M_PI,theta_z2*180/M_PI);
    
    // generate new rotation matrices
    recalcExtrinsics();   
}    

void OpencvCamera::revert2oldParam(void){
    theta_x = old_theta_x;
    theta_z = old_theta_z;
    theta_z = old_theta_z2;    
    
    recalcExtrinsics();
}

void OpencvCamera::recalcExtrinsics(void){
    // generate rotation matrices
    Rz << cos(theta_z) << sin(theta_z) << 0 << endr
        << -sin(theta_z) << cos(theta_z) << 0 << endr
        << 0. << 0. << 1. << endr;

    Rx << 1 << 0 << 0 << endr
        << 0 << cos(theta_x) << sin(theta_x) << endr
        << 0 << -sin(theta_x) << cos(theta_x) << endr;

    Rz2 << cos(theta_z2) << sin(theta_z2) << 0 << endr
        << -sin(theta_z2) << cos(theta_z2) << 0 << endr
        << 0. << 0. << 1. << endr;

    R = Rz2*Rx*Rz;
      
    AR = A*R;
    ART = A*R*T;     
}

void OpencvCamera::writeParam(void){
    CvMat * Rotation = cvCreateMat(3,1,CV_32F);
    CV_MAT_ELEM(*Rotation,float,0,0) = theta_z;
    CV_MAT_ELEM(*Rotation,float,1,0) = theta_x;
    CV_MAT_ELEM(*Rotation,float,2,0) = theta_z2;
    
    char * sbuffer;
    sbuffer = (char *)malloc(sizeof (char) * 256);
    cvSave(get_concat_string(sbuffer, PREAMBLE_TO_ROTATION, getID()),Rotation);    
    std::cout << "New parameters have been stored in " << sbuffer << std::endl;
    free(sbuffer);
}

char * OpencvCamera::get_concat_string(char * buffer, const char * pream, int id){
    char ascii_terminal[20];
    sprintf(ascii_terminal,"%d.xml",id);
    sprintf(buffer,PATH_TO_SERVER_MOUNT);
    strcat(buffer,pream);
    strcat(buffer,ascii_terminal);
    return buffer;
}

mat OpencvCamera::getRot(double lx,double ly,double lz)
{
    double a11, a12, a13, a21, a22, a23, a31, a32, a33;
    mat rot;
    rot.set_size(3,3);
    rot.zeros();

    a11 = lz*cos(theta_x)*sin(theta_z2)
        - ly*cos(theta_z)*sin(theta_x)*sin(theta_z2)
        + lx*sin(theta_x)*sin(theta_z)*sin(theta_z2);

        a12 = ly*cos(theta_z)*cos(theta_z2) - lx*cos(theta_z2)*sin(theta_z)
            - lx*cos(theta_x)*cos(theta_z)*sin(theta_z2)
            - ly*cos(theta_x)*sin(theta_z)*sin(theta_z2);

        a13 = -lx*cos(theta_z)*sin(theta_z2)
            + lz*cos(theta_z2)*sin(theta_x) - ly*sin(theta_z)*sin(theta_z2)
            + ly*cos(theta_x)*cos(theta_z)*cos(theta_z2)
            - lx*cos(theta_x)*cos(theta_z2)*sin(theta_z);

        a21 = lz*cos(theta_x)*cos(theta_z2)- ly*cos(theta_z)*cos(theta_z2)*sin(theta_x)
            + lx*cos(theta_z2)*sin(theta_x)*sin(theta_z);

        a22 = -ly*cos(theta_z)*sin(theta_z2) + lx*sin(theta_z)*sin(theta_z2)
            - lx*cos(theta_x)*cos(theta_z)*cos(theta_z2)
            - ly*cos(theta_x)*cos(theta_z2)*sin(theta_z);

        a23 = -lx*cos(theta_z)*cos(theta_z2)
            - ly*cos(theta_z2)*sin(theta_z)
            - lz*sin(theta_x)*sin(theta_z2)
            - ly*cos(theta_x)*cos(theta_z)*sin(theta_z2)
            + lx*cos(theta_x)*sin(theta_z)*sin(theta_z2);

        a31 = -lz*sin(theta_x)
            - ly*cos(theta_x)*cos(theta_z)
            + lx*cos(theta_x)*sin(theta_z);
        a32 = lx*cos(theta_z)*sin(theta_x)
            + ly*sin(theta_x)*sin(theta_z);
        a33 = 0;

    rot(0,0) = a11; rot(0,1) = a12; rot(0,2) = a13;
    rot(1,0) = a21; rot(1,1) = a22; rot(1,2) = a23;
    rot(2,0) = a31; rot(2,1) = a32; rot(2,2) = a33;

    return rot;
}

mat OpencvCamera::getB(double lx, double ly, double lz, mat Pc)
{
    mat b1 = Ainv*Pc;

    b1(0) = b1(0) - (lx*cos(theta_z)*cos(theta_z2)
        + ly*cos(theta_z2)*sin(theta_z)
        + lz*sin(theta_x)*sin(theta_z2)
        + ly*cos(theta_x)*cos(theta_z)*sin(theta_z2)
        - lx*cos(theta_x)*sin(theta_z)*sin(theta_z2));
    b1(1) = b1(1) - (lz*cos(theta_z2)*sin(theta_x)
        - lx*cos(theta_z)*sin(theta_z2)
        - ly*sin(theta_z)*sin(theta_z2)
        + ly*cos(theta_x)*cos(theta_z)*cos(theta_z2)
        - lx*cos(theta_x)*cos(theta_z2)*sin(theta_z));
    b1(2) = b1(2) - (lz*cos(theta_x)
        - ly*cos(theta_z)*sin(theta_x)
        + lx*sin(theta_x)*sin(theta_z));

    return b1;
}

mat OpencvCamera::getAblock(CvPoint imagePoint)
{
    //image_projection point2D = Projection::get2Dcoordinates(objectPoint);

    AR.resize(3, 4);
    AR(0, 3) = -imagePoint.x;
    AR(1, 3) = -imagePoint.y;
    AR(2, 3) = -1;
    return AR;
}

mat OpencvCamera::getBblock(void)
{
    return ART;
}

void OpencvCamera::printCamPosition(void){
    cout << position_array[0] << ", " << position_array[1] << ", " << position_array[2] << endl;
}

CvMat * OpencvCamera::getDistortion(void){
    return distortion;
}

void OpencvCamera::setDistortion(CvMat* distor){
    distortion = distor;
}

CvMat * OpencvCamera::getIntrinsic(void){
    return intrinsic;
}

mat OpencvCamera::getIntrinsicArm(void){
    return A;
}

mat OpencvCamera::getIntrinsicInverseArm(void){
    return Ainv;
}

mat OpencvCamera::getTArm(void){
    return T;
}

mat OpencvCamera::getR_transposedArm(void){
    return Rtrans;
}

double OpencvCamera::getcx(void){
    return cx;
}

double OpencvCamera::getcy(void){
    return cy;
}

double OpencvCamera::getfx(void){
    return fx;
}

double OpencvCamera::getfy(void){
    return fy;
}

int OpencvCamera::getID(void){
    return id;
}

char * OpencvCamera::getWindowName(void){
    return windowName;
}

IplImage * OpencvCamera::getInitFrame(void){
    return frame;
}

void OpencvCamera::makeDistortionMap(void){
    CvSize frame_size = cvGetSize(frame);
    
    mapx = cvCreateImage(frame_size, IPL_DEPTH_32F, 1);
    mapy = cvCreateImage(frame_size, IPL_DEPTH_32F, 1);
    cvInitUndistortMap(intrinsic, distortion, mapx, mapy);    
    
//    std::cout << "frame size: " << frame_size.width << " x " << frame_size.height << std::endl;
}

CvPoint3D32f OpencvCamera::getRay(CvPoint2D32f image){
    CvPoint3D32f Po;
    float gamma = 10000;
    float sigma1 = gamma*(getcy() - image.y)/getfy();
    float sigma2 = gamma*(getcx() - image.x)/getfx();
    
    Po.x = gamma*R(2,0) + T(0) - R(0,0)*sigma2 - R(1,0)*sigma1;
    Po.y = gamma*R(2,1) + T(1) - R(0,1)*sigma2 - R(1,1)*sigma1;
    Po.z = gamma*R(2,2) + T(2) - R(0,2)*sigma2 - R(1,2)*sigma1;
    
    return Po;
}

OpencvCamera::~OpencvCamera() {
    if (pFile != NULL)
        fclose (pFile);
    if (camera != NULL)
        cvReleaseCapture(&camera);
    if (object_points != NULL){
        cvReleaseMat(&object_points);
    }
    if (intrinsic != NULL){
        cvReleaseMat(&intrinsic);
        intrinsic = NULL;
    }
    if (distortion != NULL){
        cvReleaseMat(&distortion);
        distortion = NULL;
    }    
}



