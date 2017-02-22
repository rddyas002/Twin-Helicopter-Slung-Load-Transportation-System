/* 
 * File:   OpencvCamera.h
 * Author: yashren2
 *
 * Created on 13 November 2012, 10:59 AM
 * This class is intended to create the camera object and hold all the parameters 
 * relating to a specific camera i.e. resolution, distortion terms, intrinsic and
 * extrinsic matrices. It also has an autoCalibration method which fine tunes the 
 * rotation matrix to match the 2D projection.
 */

#ifndef OPENCVCAMERA_H
#define	OPENCVCAMERA_H

#include <cstdio>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <armadillo>
#include <cmath>
#include "common.h"

#define RES640X480 0                    // Choose resolution
#define REFINE_CALIB_ITER 50000         // Number of iterations for rotation matrix calibration

// Holds the position of the center of the room, used for initial calibration
// All linear positions in mm
#define CENTER_X center_array[0]
#define CENTER_Y center_array[1]
#define CENTER_Z center_array[2]

// The camera position w.r.t. the world coordinate reference frame
#define CAMERA_X position_array[0]
#define CAMERA_Y position_array[1]
#define CAMERA_Z position_array[2]

// ******* This needs to move ****** //
#define CAM1_ON true
#define CAM2_ON true
#define CAM3_ON false
#define CAM4_ON false

#if RES640X480
        #define HEIGHT 480
        #define WIDTH 640
#else
        #define HEIGHT 240
        #define WIDTH 320
#endif

#define PATH_TO_OBJECT_POINTS "../../Mocap_Master/MotionCaptureSlave/Positions/object_points.txt"

using namespace cv;
using namespace std;
using namespace arma;

class OpencvCamera {
public:
    OpencvCamera(int fd, int ident);    // Used to initialize camera
    OpencvCamera(const OpencvCamera& orig);
    
    void setIntrinsic(CvMat* intrins);  
    void setCamPosition(CvMat* posit);
    CvMat * getIntrinsic(void);
    void setDistortion(CvMat* distor);
    void setCenter(CvMat* posit);

    IplImage * getFrame(void);          // Use cv method to get new frame
    IplImage * queryFrame(void);
    IplImage * getInitFrame(void);
    void calcExtrinsics(void);
    void recalcExtrinsics(void);
    void revert2oldParam(void);
    CvPoint get2Dcoordinates(double * objectPoint);     // 3D to 2D mapping
    
    void setCalibObjectPoints(CvMat* posit);
    void autoCalib(CvMat* imagePoints);
    void writeParam(void);
    char * get_concat_string(char * buffer, const char * pream, int id);
    void refineAngles(void);
    mat getAblock(CvPoint imagePoint);
    mat getBblock(void);
    mat getRot(double lx,double ly,double lz);
    mat getB(double lx, double ly, double lz, mat Pc);
    CvMat * getDistortion(void);   
    char * getWindowName(void);
    void loadRotationAngles(CvMat* thetas);
    CvPoint3D32f getRay(CvPoint2D32f image);   
    
    double getfx(void);
    double getfy(void);
    double getcx(void);
    double getcy(void);
    
    int getID(void);    
    void printCamPosition(void);
    
    void makeDistortionMap();           // Form distortion map for camera
    IplImage * get_mapx(void);
    IplImage * get_mapy(void);
    
    mat getIntrinsicArm(void);
    mat getIntrinsicInverseArm(void);
    mat getTArm(void);
    mat getR_transposedArm(void);
    mat getPo1(void);
    mat getPo2(void);
    mat getPo3(void);
    
    virtual ~OpencvCamera();
private:
        CvCapture* camera;
        int id;
        IplImage * frame;       // used to get size
        CvMat* intrinsic;
        CvMat* distortion;
        double intrinsic_array[4];
        double distortion_array[4];
        IplImage* mapx;
        IplImage* mapy;
        double cx;
        double cy;  
        double fx;
        double fy;          
        
        char windowName[20];
        int slider_end;
        int slider_max;
        int slider_min;
        
        // Projection stuff
        double position_array[3];
        double center_array[3];
        
        double theta_x, old_theta_x; // rotation about the x_axis
        double theta_z, old_theta_z; // rotation about the z_axis
        double theta_z2, old_theta_z2; // rotation about the y_axis

        mat Rz, Rx, Rz2, R, T, A;            //
        mat AR, ART;
        mat Rtrans, Ainv;                 // for mapping 2d to 3d
        void set_thetas(void);
        
        mat Po1, Po2, Po3, Po4;
        mat Pc1, Pc2, Pc3, Pc4;  
        
        FILE * pFile;
        float Po[18];
        CvMat * object_points;
};

#endif	/* OPENCVCAMERA_H */

