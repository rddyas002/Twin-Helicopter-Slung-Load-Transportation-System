/* 
 * File:   Blob.h
 * Author: yashren2
 *
 * Created on 13 November 2012, 11:26 AM
 */

#ifndef BLOB_H
#define	BLOB_H

#include "common.h"
#include "OpencvCamera.h"

/* Tuning parameters
 Blob area filter:
 At center distance,  min 2600, max 5400
 At close range, min 3300, max 8200
 */

#define BOUND_BORDER 50
#define UPSCALE 4

#define MAX_BLOBS_HOLD 5

#define MINAREA 1800
#define MAXAREA 9200
#define MINAREA_GREEN 2000

#define INITIAL_AREA_BLOB_FILTER_MIN 50
#define INITIAL_AREA_BLOB_FILTER_MAX 250

#define STATS_SAMPLE (100)

using namespace cvb;

struct Sblob_data{
    // holds the number of blobs per colour
    unsigned int red_blob;
    unsigned int green_blob;
    unsigned int blue_blob;
    
    // There can be at most MAX_BLOBS_HOLD blobs of any colour in the picture
    CvPoint2D32f red_blob_pos[MAX_BLOBS_HOLD];
    CvPoint2D32f green_blob_pos[MAX_BLOBS_HOLD];
    CvPoint2D32f blue_blob_pos[MAX_BLOBS_HOLD];
};

class Blob {
public:
    // Initialize/Destroy
    Blob(OpencvCamera * camera, int id, bool draw_blob);
    virtual ~Blob();
    
    // Signals
    typedef boost::signals2::signal<void( char * )> Blob_Signal;
    
    // Set/get functions
    IplImage * getSegmRed(void);
    IplImage * getSegmGreen(void);
    IplImage * getSegmBlue(void);
    IplImage * getSegmBlobs(void);  
    IplImage * getProcessedFrame(void);
    void setMaxArea(int maxArea);
    void setMinArea(int minArea);    
    void setRays(CvPoint3D32f rO,CvPoint3D32f r1,CvPoint3D32f r2);
    IplImage * drawRoom(IplImage * temp_frame);
    void drawGrid(IplImage * tmp, int height, CvScalar line_color);
    void drawFloor2x2(IplImage * tmp);
    IplImage * getFrame(void);
    IplImage * getWindowFrame(void);    
    double getProgramRunTime(void);
    volatile bool blobProcessing(void);
    double deltaTime(void);
    double getAverageLatency(void);
    double FPSCalc(void);
    
    // autoThread functions
    void runAuto(void);
    void autoThread(void);
    void autoThreadEnd(void);
    
    // runProcess functions
    void runProcess(void);
    void processThread(void);
    void processThreadEnd(void);
    
    // calibThread   
    void calibMode(void);
    void calibThread(void);
    void doCalib(void);
    void calibThreadEnd(void);
    
    // main helper functions
    void releaseIplImage(IplImage * ImagePntr);    
    
    // Image processing related
    void drawVirtualBlobs(void);        // used for testing
    unsigned int setBlobData(void);             // store found blobs in struct
    void showBlobs(void);               // aesthetics
    CvRect getROI(CvBlobs &bl);         // based on found blobs, finds ROI
    void formBlobPacket(void);          // puts data into ASCII for transmission  
    void undistortBlobs(void);          // performs lens correction
    void undistortPoints(CvPoint2D32f cvpoint[], unsigned int num_blobs);
    
    // debugging functions
    void errorMessage(const char * str);
    void debugMessage(const char * str);
    
    struct Sblob_data * getBlob_data(void){return &Blob_data;}
    Blob_Signal* getSignal() { return &conversion_done; }
    float round(float num){return floor(num + 0.5);}
    inline double getFractionalSeconds(double start_time){
        struct timeval tv;
        gettimeofday(&tv,NULL);
        
        double t = (double) tv.tv_sec + (double) 1e-6*tv.tv_usec - start_time;
        return t;
    }    
    
private:
    // using to signal data send to client from TCP_server
    Blob_Signal conversion_done;
    
    int client_id;

    OpencvCamera * cam;
    IplImage *hsv;
    IplImage *blue;
    IplImage *green;
    IplImage *red;
    IplImage *red1;
    IplImage *red2;
    IplImage *blobs;
    IplImage * blobFrame;  
    IplImage * tempImage;
    IplImage * windowFrame;
    
    volatile bool blob_processing;
        
    CvScalar blue_min;
    CvScalar blue_max;
    CvScalar green_min;
    CvScalar green_max;
    CvScalar red1_min;
    CvScalar red1_max;
    CvScalar red2_min;
    CvScalar red2_max;     
    
    // Thread variables
    volatile bool processRunning, autoRunning, calibRunning;
    pthread_t process_thread,auto_thread,calib_thread;
    volatile bool auto_thread_run, calib_thread_run;
        
    void getBlob(void);
    bool first_run;
        
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

    // virtual blobs moving
    float virtual_move_x;
    float virtual_move_y;
    float virtual_move_z;
    float virtual_counter;
                
    CvPoint3D32f ray_red;
    CvPoint3D32f ray_green;
    CvPoint3D32f ray_blue;
        
    char data_packet[256];
            
    double time_t0;
    double latency;
    double fps;
    
    // used to indicate to Blob object whether to draw
    bool drawBlob;    
};

#endif	/* BLOB_H */

