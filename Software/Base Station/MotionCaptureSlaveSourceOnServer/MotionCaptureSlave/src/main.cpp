/*
 * Motion Capture Slave code
 * Slave computers act as servers waiting for request from master client for 
 * blob data
 * v1.1 - added autocalibration module - 19 May 2013
 * v1.2 - added test facility options -see and -draw from cmd to debug
 *        -see opens a window to see the frame and -draw draws the fake blobs
 * v1.3a - Trying to resolve crash upon repeated connections.
 * v1.4 - Real time optimised
 * v1.4a - added latency, fps, blobs in scene display
 * v1.5 - time synchronization with base station
 */


#include <cstdlib>
#include <iostream>
#include "../inc/MotionCapture.h"
#include <sched.h>

#define MOTIONCAPTURESLAVE_VERSION "MotionCaptureSlave v1.5"

MotionCapture * motionCaptureSlave = NULL;

// Calibration stuff
unsigned int calib_points_2d = 0;
CvMat * image_points = cvCreateMat(6,1,CV_32FC2);
CvMat * Rotation = cvCreateMat(3,3,CV_32F);
CvMat * Translation = cvCreateMat(3,1,CV_32F);
CvMat * Rodrigues = cvCreateMat(3,1,CV_32F);

void mouseCamEvent(int event, int x, int y, int flags, void* param){
    if (event == CV_EVENT_LBUTTONDOWN){
        if (calib_points_2d < 4){
            CV_MAT_ELEM(*image_points,CvPoint2D32f,calib_points_2d,0) = cvPoint2D32f(x,y);
            calib_points_2d++;
            cvDrawCircle(motionCaptureSlave->getBlobObject()->getWindowFrame(),cvPoint(x,y),
                5,CV_RGB(255,0,0),1);

            std::cout << "Point " << calib_points_2d <<  " x: " << x << " y: " << y << std::endl;
            std::cout << " " << std::endl;
        
            if (calib_points_2d < 4){
                std::cout << "Click on point " <<  (calib_points_2d + 1) << std::endl;
            }
            motionCaptureSlave->setState(UPDATE_IMAGE);
        }
    }
}

void realTimePriority(void){
    struct sched_param sp;
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0){
        cout << "Process priority setting failed" << endl;
    }
    else{
        cout << "Process priority setting succeeded" << endl;
    }
}

int main(int argc, char** argv) {
    int server_id, server_port;
    bool see_blobs = false, draw_blobs = false;

    if (argc < 3){
        std::cout << "Insufficient arguments. Program terminating..." << std::endl;
        return -1;
    }
    else
    {
        char * sbuffer;
        sbuffer = (char *) malloc(sizeof (char) * 20);

        server_id = atoi(argv[1]);
        server_port = atoi(argv[2]);
        
        if (argc > 3){
            sbuffer = argv[3];
            if (strcmp(sbuffer,"-see") == 0)
                see_blobs = true;
            else
                see_blobs = false;
        }
        
        if (argc > 4){
            sbuffer = argv[4];
            if (strcmp(sbuffer,"-draw") == 0)
                draw_blobs = true;
            else
                draw_blobs = false;
        }       
        
        
        if ((server_id < 1) || (server_id > MAX_CAMERAS))
        {
            std::cout << "Invalid argument. Program terminating..." << std::endl;
            return -1;
        }
        std::cout << "--------------------------------" << std::endl;
        std::cout << MOTIONCAPTURESLAVE_VERSION << std::endl;
        std::cout << "Motion Capture server " << server_id << std::endl;
        std::cout << "Server Port: " << server_port << std::endl;
    }    
	
//    realTimePriority();
    
    motionCaptureSlave = new MotionCapture(server_id,server_port,draw_blobs);
    int ch;
    
    while(motionCaptureSlave->getKeepRunning()){
        int c = cvWaitKey(100);
//        sleep(1);
        
        // Routine for calibration only
        MotionCaptureState st = motionCaptureSlave->getState();
        switch(st){
            case SHOW_IMAGE:
                // create window
                cvNamedWindow(motionCaptureSlave->getCameraObject()->getWindowName(),CV_WINDOW_NORMAL);
                cvShowImage(motionCaptureSlave->getCameraObject()->getWindowName(),
                        motionCaptureSlave->getBlobObject()->getWindowFrame()); 
                motionCaptureSlave->setState(REQUEST_CALIBRATION);
                break;
            case REQUEST_CALIBRATION:
                while(true){
                    std::cout << "Need calibration? [y/n]" << std::endl;
                    ch = getchar();
                    if (ch == 'y'){
                        std::cout << "Click on point " <<  (calib_points_2d + 1) << std::endl;
                        motionCaptureSlave->setState(WAIT_INPUT);
                        cvSetMouseCallback(motionCaptureSlave->getCameraObject()->getWindowName(), mouseCamEvent);
                        break;
                    }
                    else if (ch == 'n'){
                        std::cout << "Exiting calibration mode" << std::endl;
                        // close window and go back to idle
                        cvDestroyWindow(motionCaptureSlave->getCameraObject()->getWindowName());
                        motionCaptureSlave->setState(IDLE);
                        break;
                    }
                    else
                        std::cout << "Invalid input" << std::endl;
                }     
                break;
            case UPDATE_IMAGE:
                cvShowImage(motionCaptureSlave->getCameraObject()->getWindowName(),
                        motionCaptureSlave->getBlobObject()->getWindowFrame());   
                if (calib_points_2d < 4)
                    motionCaptureSlave->setState(WAIT_INPUT);
                else
                {
                    motionCaptureSlave->setState(DO_CALIBRATION);
                }
                break;
                
            case DO_CALIBRATION:
                    std::cout << "Performing calibration..." << std::endl;
                    motionCaptureSlave->getCameraObject()->autoCalib(image_points);
                    // redraw 
                    motionCaptureSlave->getBlobObject()->doCalib();
                    // show corrected image
                    cvShowImage(motionCaptureSlave->getCameraObject()->getWindowName(),
                        motionCaptureSlave->getBlobObject()->getWindowFrame());  
                    
                    motionCaptureSlave->setState(STORE_PARAMETERS);
                    break;
            case STORE_PARAMETERS:
                    while(true){
                        std::cout << "Do you want to store new paramters? [y/n]" << std::endl;
                        ch = getchar();
                        if (ch == 'y'){
                            motionCaptureSlave->getCameraObject()->writeParam();                     
                            break;
                        }
                        else if (ch == 'n'){
                            std::cout << "Parameters not saved." << std::endl;
                            motionCaptureSlave->getCameraObject()->revert2oldParam();
                            break;
                        }        
                        else
                            std::cout << "Invalid input" << std::endl;
                    }
                    cvDestroyWindow(motionCaptureSlave->getCameraObject()->getWindowName());                    
                    motionCaptureSlave->setState(IDLE);
                    calib_points_2d = 0;
                break;
            case AUTO:
                if (see_blobs){
                    cvShowImage(motionCaptureSlave->getCameraObject()->getWindowName(),
                        motionCaptureSlave->getBlobObject()->getWindowFrame());
                    cvWaitKey(1);
                }
                break;
        }
    }

    if (motionCaptureSlave != NULL)
        delete motionCaptureSlave;
    
    return 0;
}

