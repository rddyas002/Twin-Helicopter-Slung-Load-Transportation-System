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

#define MAX_BLOBS_HOLD 5

using namespace cv;
using namespace std;
using namespace arma;

class OpencvCamera {
public:
    typedef struct{
        unsigned int red_blobs;
        unsigned int green_blobs;
        unsigned int blue_blobs;
        
        CvPoint2D32f red[MAX_BLOBS_HOLD];
        CvPoint2D32f green[MAX_BLOBS_HOLD];
        CvPoint2D32f blue[MAX_BLOBS_HOLD];
        
        double timestamp;
        bool updated;
    } Blob_data;
    
    OpencvCamera(int fd, int ident);    // Used to initialize camera
    OpencvCamera(const OpencvCamera& orig);
    
    void setIntrinsic(CvMat* intrins);  
    void setCamPosition(CvMat* posit);
    CvMat * getIntrinsic(void);
    void setDistortion(CvMat* distor);
    void setCenter(CvMat* posit);
    
    // Add methods to retrieve camera parameters
    double getfx(void);
    double getfy(void);
    double getcx(void);
    double getcy(void);    
    double get_theta_z();
    double get_theta_x();
    double get_theta_z2();
    double get_Tcw_x();
    double get_Tcw_y();
    double get_Tcw_z();
     
    IplImage * getFrame(void);          // Use cv method to get new frame    
    void calcExtrinsics(void);
    CvPoint get2Dcoordinates(double * objectPoint);     // 3D to 2D mapping
    
    void setCalibObjectPoints(CvMat* posit);
    void autoCalib(CvPoint2D32f redImage,CvPoint2D32f greenImage,CvPoint2D32f blueImage);
    void refineAngles(void);
    mat getAblock(CvPoint imagePoint);
    mat getBblock(void);
    mat getRot(double lx,double ly,double lz);
    mat getB(double lx, double ly, double lz, mat Pc);
    CvMat * getDistortion(void);   
    char * getWindowName(void);
    void loadRotationAngles(CvMat* thetas);
    CvPoint3D32f getRay(CvPoint2D32f image);   
   
    int getID(void);    
    void printCamPosition(void);
    
    void makeDistortionMap();           // Form distortion map for camera
    
    mat getIntrinsicArm(void);
    mat getIntrinsicInverseArm(void);
    mat getTArm(void);
    mat getRArm(void);
    mat getR_transposedArm(void);
    mat getPo1(void);
    mat getPo2(void);
    mat getPo3(void);
    Blob_data blob_data;
    Blob_data prev_blob_data;
    Blob_data extrap_blob_data;
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
        
        double theta_x; // rotation about the x_axis
        double theta_z; // rotation about the z_axis
        double theta_z2; // rotation about the y_axis
        mat Rz, Rx, Rz2, R, T, A;            //
        mat AR, ART;
        mat Rtrans, Ainv;                 // for mapping 2d to 3d
        void set_thetas(void);
        
        mat Po1, Po2, Po3;
        mat Pc1, Pc2, Pc3;  
        
//        FILE * pFile;
        bool virtual_camera;
        
};

#endif	/* OPENCVCAMERA_H */

