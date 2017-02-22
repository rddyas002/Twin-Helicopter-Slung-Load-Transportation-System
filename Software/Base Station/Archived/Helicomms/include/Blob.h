/* 
 * File:   Blob.h
 * Author: yashren2
 *
 * Created on 13 November 2012, 11:26 AM
 */

#ifndef BLOB_H
#define	BLOB_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cvblob.h>
#include <math.h>
#include "OpencvCamera.h"
//#include "Helicopter.h"

#define BOUND_BORDER 35
#define UPSCALE 4

/* Tuning parameters
 Blob area filter:
 At center distance,  min 2600, max 5400
 At close range, min 3300, max 8200
 */

#define MINAREA 2600
#define MAXAREA 9200

#define MINAREA_GREEN 2300

#define CALIB_ITER 30

#define INITIAL_AREA_BLOB_FILTER_MIN 50
#define INITIAL_AREA_BLOB_FILTER_MAX 250

using namespace cvb;

struct Sblob_data{
    // holds the number of blobs per colour
    unsigned int red_blob;
    unsigned int green_blob;
    unsigned int blue_blob;
    
    // There can be at most 5 blobs of any colour in the picture
    CvPoint2D32f red_blob_pos[5];
    CvPoint2D32f green_blob_pos[5];
    CvPoint2D32f blue_blob_pos[5];
};

class Blob {
public:
    Blob(OpencvCamera * camera);//, Helicopter * helicopter1, Helicopter * helicopter2);
    void runProcess(void);
    IplImage * getSegmRed(void);
    IplImage * getSegmGreen(void);
    IplImage * getSegmBlue(void);
    IplImage * getSegmBlobs(void);  
    IplImage * getProcessedFrame(void);
    CvPoint2D32f getRedCalib(void);
    CvPoint2D32f getGreenCalib(void);
    CvPoint2D32f getBlueCalib(void);    
    void undistortBlobs(void);
    void undistortPoints(CvPoint2D32f cvpoint[], unsigned int num_blobs);
    
    CvRect getROI(CvBlobs &bl);
    void showBlobs(void);
    void setBlobData(void);
    struct Sblob_data * getBlob_data(void);
    void drawRoom(void);
    void drawVirtualBlobs(void);
    void setRays(CvPoint3D32f rO,CvPoint3D32f r1,CvPoint3D32f r2);
    IplImage * getFrame(void);
        
    void setMaxArea(int maxArea);
    void setMinArea(int minArea);
    void wait4ThreadEnd(void);
    float round(float num);
    virtual ~Blob();
    virtual void process(void);
private:
        volatile bool processRunning;
        OpencvCamera * cam;

        IplImage *hsv;
        IplImage *blue;
        IplImage *green;
        IplImage *red;
        IplImage *red1;
        IplImage *red2;
        IplImage *blobs;
        CvScalar blue_min;
        CvScalar blue_max;
        CvScalar green_min;
        CvScalar green_max;
        CvScalar red1_min;
        CvScalar red1_max;
        CvScalar red2_min;
        CvScalar red2_max;     
        
        pthread_t process_thread;
        
        void getBlob(void);
        bool first_run;
        IplImage * frame;    
        
        CvPoint2D32f redCenter32f;
        CvPoint2D32f greenCenter32f;
        CvPoint2D32f blueCenter32f;
        
        CvBlobs blob_red;
        CvTracks blob_red_track;
        CvBlobs blob_green;
        CvTracks blob_green_track;
        CvBlobs blob_blue; 
        CvTracks blob_blue_track;
        CvBlobs blobs_ROI; 
        CvRect ROI;      
        
        int min_area;
        int max_area;
        
        struct Sblob_data Blob_data;    
        
        float virtual_move_x;
        float virtual_move_y;
        float virtual_move_z;
        float virtual_counter;
        
//        Helicopter * heli1;
//        Helicopter * heli2;
        
        CvPoint3D32f ray_red;
        CvPoint3D32f ray_green;
        CvPoint3D32f ray_blue;
};

#endif	/* BLOB_H */

