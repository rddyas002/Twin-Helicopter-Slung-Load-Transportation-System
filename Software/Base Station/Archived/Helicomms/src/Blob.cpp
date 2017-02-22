/* 
 * File:   Blob.cpp
 * Author: Yashren Reddi        
 * 
 * Created on 13 November 2012, 11:26 AM
 * This class is designed to extract the blob positions for each rgb LED.
 * Upon initialization it finds the average blob positions (32f) which are used
 * in the OpencvCamera autoCalibrate routine. The class is set to run on separate 
 * cores of the PC.
 */

#include "../include/Blob.h"

using namespace cv;

extern "C"
{
    // this C function will be used to receive the thread and pass it back to the Thread instance
    void* thread_catch(void* arg)
    {
        Blob* t = static_cast<Blob*>(arg);
        t->process();
        return 0;
    }
}

void Blob::process(void)
{
    processRunning = true;
    Blob::getBlob();
    processRunning = false;
}

Blob::Blob(OpencvCamera * camera){//, Helicopter * helicopter1, Helicopter * helicopter2) {
    cam = camera;
    first_run = true;
                
//    heli1 = helicopter1;
//    heli2 = helicopter2;
    
    CvSize size = cvGetSize(camera->getFrame());
    hsv = cvCreateImage(size, IPL_DEPTH_8U, 3);
    blue = cvCreateImage(size, IPL_DEPTH_8U, 1);
    green = cvCreateImage(size, IPL_DEPTH_8U, 1);
    red = cvCreateImage(size, IPL_DEPTH_8U, 1);
    red1 = cvCreateImage(size, IPL_DEPTH_8U, 1);
    red2 = cvCreateImage(size, IPL_DEPTH_8U, 1);
    blobs = cvCreateImage(size, IPL_DEPTH_8U, 1);

    blue_min = cvScalar(86,50,50,0);
    blue_max = cvScalar(126,255,255,0);
    green_min = cvScalar(55,50,50,0);
    green_max = cvScalar(85,255,255,0);
    red1_min = cvScalar(0,60,60,0);
    red1_max = cvScalar(16,255,255,0);
    red2_min = cvScalar(165,60,60,0);
    red2_max = cvScalar(179,255,255,0);    
    
    min_area = 85;
    max_area = 250;   
}

void Blob::runProcess(void)
{
    frame = cam->getFrame();
    //int c = cvWaitKey(1);
//    // Schedule core
//    cpu_set_t my_set;        
//    CPU_ZERO(&my_set);       
//    CPU_SET(cam->getID()+1, &my_set);     // set the bit that represents core 
//    sched_setaffinity(0, sizeof(cpu_set_t), &my_set); /* Set affinity of this process to */
//         
//    pthread_create(&process_thread, 0, &thread_catch, this);
}

void Blob::wait4ThreadEnd(void)
{
    pthread_join(process_thread, NULL);
}

float Blob::round(float num)
{
    return floor(num + 0.5);
}

void Blob::getBlob(void)
{
    CvRect newROI;

    // get frame from camera
    frame = cam->getFrame();
    
    // draw virtual blobs
    //drawVirtualBlobs();    

    if (first_run)
    {
        // find initial region of interest
        first_run = false;
        
        // convert to hsv space
        cvCvtColor(frame, hsv, CV_BGR2HSV);
        // extract color
        cvInRangeS(hsv, blue_min, blue_max, blue);
        cvInRangeS(hsv, green_min, green_max, green);
        cvInRangeS(hsv, red1_min, red1_max, red1);
        cvInRangeS(hsv, red2_min, red2_max, red2);
        // must or red regions because of discontinuity
        cvOr(red1, red2, red, NULL);

        // smooth out each channel
        int blur_val = 9;
        cvSmooth(blue, blue, CV_GAUSSIAN, blur_val, blur_val, 0, 0);
        cvSmooth(green, green, CV_GAUSSIAN, blur_val, blur_val, 0, 0);
        cvSmooth(red, red, CV_GAUSSIAN, blur_val, blur_val, 0, 0);        
        
        // get image of blobs or'd to determine extremes
        cvOr(blue, green, blobs, NULL);
        cvOr(blobs, red, blobs, NULL);
        
        //COM = Blob::getCOM(blobs);
        
        // use cvblob lib to get position of blobs
        IplImage *labelImg = cvCreateImage(cvGetSize(blobs), IPL_DEPTH_LABEL, 1);

        cvLabel(blobs, labelImg, blobs_ROI);   
        cvFilterByArea(blobs_ROI, INITIAL_AREA_BLOB_FILTER_MIN, INITIAL_AREA_BLOB_FILTER_MAX);
//        ROI = getROI(blobs_ROI);
//        
//        // Add a border to allow for movement
//        ROI.x -= BOUND_BORDER;
//        ROI.y -= BOUND_BORDER;
//        ROI.width += 2*BOUND_BORDER;
//        ROI.height += 2*BOUND_BORDER;
//        // Cannot go outside the frame...just check
//        if (ROI.x < 0) ROI.x = 0;
//        if (ROI.y < 0) ROI.y = 0;
//        if (ROI.x > WIDTH){ROI.x = 0; cout << "Error: ROI.x > WIDTH" << endl;}
//        if (ROI.y > HEIGHT){ROI.y = 0; cout << "Error: ROI.x > HEIGHT" << endl;}        
//        if ((ROI.x + ROI.width) > WIDTH){ROI.width = WIDTH - ROI.x;cout << "Error: ROI width outside frame width =" << ROI.x << " " <<  ROI.width << endl;}
//        if ((ROI.y + ROI.height) > HEIGHT){ROI.height = HEIGHT - ROI.y;cout << "Error: ROI height outside frame" << endl;}
               
        // Limit ROI to center of room first
        ROI.x = 1*BOUND_BORDER;
        ROI.y = 1*BOUND_BORDER;
        ROI.width = WIDTH - ROI.x*2;
        ROI.height = HEIGHT - ROI.y*2;        

        cvSetImageROI(frame, ROI);
        cvReleaseImage(&labelImg);
    }
    else        // use region of interest and adjust
    {
        // for some reason ROI wasn't set on first instance
        cvSetImageROI(frame, ROI);
        
        IplImage * hsv_roi = cvCreateImage(cvSize(ROI.width,ROI.height), IPL_DEPTH_8U, 3);
        IplImage * blue_roi = cvCreateImage(cvSize(ROI.width,ROI.height), IPL_DEPTH_8U, 1);
        IplImage * green_roi = cvCreateImage(cvSize(ROI.width,ROI.height), IPL_DEPTH_8U, 1);
        IplImage * red1_roi = cvCreateImage(cvSize(ROI.width,ROI.height), IPL_DEPTH_8U, 1);
        IplImage * red2_roi = cvCreateImage(cvSize(ROI.width,ROI.height), IPL_DEPTH_8U, 1);
        IplImage * red_roi = cvCreateImage(cvSize(ROI.width,ROI.height), IPL_DEPTH_8U, 1);
        IplImage * all_blobs = cvCreateImage(cvSize(ROI.width,ROI.height), IPL_DEPTH_8U, 1);
        CvBlobs all_blobs_roi;
        
        // convert ROI to hsv space
        cvCvtColor(frame, hsv_roi, CV_BGR2HSV);        
        
        // extract color
        cvInRangeS(hsv_roi, blue_min, blue_max, blue_roi);
        cvInRangeS(hsv_roi, green_min, green_max, green_roi);
        cvInRangeS(hsv_roi, red1_min, red1_max, red1_roi);
        cvInRangeS(hsv_roi, red2_min, red2_max, red2_roi);
        // must or red regions because of discontinuity
        cvOr(red1_roi, red2_roi, red_roi, NULL);        
        
        // smooth out each channel
        int blur_val = 9;
        cvSmooth(blue_roi, blue_roi, CV_GAUSSIAN, blur_val, blur_val, 0, 0);
        cvSmooth(green_roi, green_roi, CV_GAUSSIAN, blur_val, blur_val, 0, 0);
        cvSmooth(red_roi, red_roi, CV_GAUSSIAN, blur_val, blur_val, 0, 0);         

        // use cvblob lib to get position of blobs
        IplImage *labelImg0 = cvCreateImage(cvGetSize(hsv_roi), IPL_DEPTH_LABEL, 1);
        IplImage *labelImg1 = cvCreateImage(cvGetSize(hsv_roi), IPL_DEPTH_LABEL, 1);
        IplImage *labelImg2 = cvCreateImage(cvGetSize(hsv_roi), IPL_DEPTH_LABEL, 1);
        IplImage *labelImg3 = cvCreateImage(cvGetSize(hsv_roi), IPL_DEPTH_LABEL, 1);        

        cvLabel(red_roi, labelImg1, blob_red);
        cvLabel(green_roi, labelImg2, blob_green);
        cvLabel(blue_roi, labelImg3, blob_blue);          

        if (RES640X480)
        {
            min_area = 75;
            max_area = 250;
            cvFilterByArea(blob_red, min_area, max_area);
            cvFilterByArea(blob_green, MINAREA_GREEN, max_area);
            cvFilterByArea(blob_blue, min_area, max_area);
        }
        else
        {         
            min_area = 75;
            max_area = 250;
            cvFilterByArea(blob_red, min_area, max_area);
            cvFilterByArea(blob_green, min_area, max_area);
            cvFilterByArea(blob_blue, min_area, max_area);            
        }
               
//        double maxDist = 60;
//        unsigned int maxInactive = 20;
//        
//        cvRenderBlobs(labelImg1, blob_red, frame, frame, CV_BLOB_RENDER_BOUNDING_BOX);
//        cvUpdateTracks(blob_red, blob_red_track, maxDist, maxInactive);
//        cvRenderTracks(blob_red_track, frame, frame, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);
//        
//        cvRenderBlobs(labelImg2, blob_green, frame, frame, CV_BLOB_RENDER_BOUNDING_BOX);
//        cvUpdateTracks(blob_green, blob_green_track, maxDist, maxInactive);
//        cvRenderTracks(blob_green_track, frame, frame, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);
//
//        cvRenderBlobs(labelImg3, blob_blue, frame, frame, CV_BLOB_RENDER_BOUNDING_BOX);
//        cvUpdateTracks(blob_blue, blob_blue_track, maxDist, maxInactive);
//        cvRenderTracks(blob_blue_track, frame, frame, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);
//        
//        int green_blobs = 0;
//        // scan through list to find extremes
//        for (CvBlobs::const_iterator it=blob_green.begin(); it!=blob_green.end(); ++it)
//        {
//            green_blobs++;
//        }
//        if (green_blobs >= 2)
//            bool stop = true;        
        
        
        // Blobs may have moved, hence, we need to adjust ROI based on new positions
        // get image of blobs or'd to determine extremes
        // ** NOTE ROI not based on filtered blobs
        cvOr(red_roi, green_roi, all_blobs, NULL);
        cvOr(all_blobs, blue_roi, all_blobs, NULL);      
        
        cvLabel(all_blobs, labelImg0, all_blobs_roi);
        
        cvFilterByArea(all_blobs_roi, min_area, max_area);
        newROI = getROI(all_blobs_roi);
                     
        cvResetImageROI(frame);
        
        cvRectangle(frame, cvPoint(ROI.x, ROI.y), cvPoint(ROI.x+ROI.width, ROI.y+ROI.height), cvScalarAll(255));
        
        // undistort blobs and set in struct
        setBlobData();
        
        // Encircle blobs with appropriate colour
        showBlobs();
        
        cvReleaseImage(&blue_roi);
        cvReleaseImage(&red1_roi);
        cvReleaseImage(&red2_roi);
        cvReleaseImage(&red_roi);
        cvReleaseImage(&green_roi); 
        cvReleaseImage(&hsv_roi);
        cvReleaseImage(&all_blobs);
        
        cvReleaseImage(&labelImg0);
        cvReleaseImage(&labelImg1);
        cvReleaseImage(&labelImg2);
        cvReleaseImage(&labelImg3);
        
        drawRoom();
        // best to do it well in this thread
        //cvShowImage(cam->getWindowName(), frame);
        
        
        // Make sure this is the last thing you do
        // If we subtract the border from each newROI min 
        // then we move back to the frame region of interest ROI.x ROI.y
        ROI.x = (ROI.x + (newROI.x - BOUND_BORDER));
        ROI.y = (ROI.y + (newROI.y - BOUND_BORDER));
        ROI.width = newROI.width + 2*BOUND_BORDER;
        ROI.height = newROI.height + 2*BOUND_BORDER;
        
        // Cannot go outside the frame...just check
        if (ROI.x < 0) ROI.x = 0;
        if (ROI.y < 0) ROI.y = 0;
        if (ROI.x > WIDTH){ROI.x = 0; cout << "Error: ROI.x > WIDTH" << endl;}
        if (ROI.y > HEIGHT){ROI.y = 0; cout << "Error: ROI.x > HEIGHT" << endl;}        
        if ((ROI.x + ROI.width) > WIDTH)
        {
            ROI.width = WIDTH - ROI.x;
            //cout << "Error: ROI width outside frame width. ROI.x = " << ROI.x << " Width = " <<  ROI.width << endl;
        }
        if ((ROI.y + ROI.height) > HEIGHT)
        {
            ROI.height = HEIGHT - ROI.y;
            //cout << "Error: ROI height outside frame height. ROI.y = " << ROI.y << " Width = " <<  ROI.height << endl;
        }        
    }    
}

CvRect Blob::getROI(CvBlobs &bl){
    double max_x = 0, min_x = 9999, max_y = 0, min_y = 9999;

    // scan through list to find extremes
    for (CvBlobs::const_iterator it=bl.begin(); it!=bl.end(); ++it)
    {
        if (it->second->centroid.x > max_x)
            max_x = it->second->centroid.x;
        if (it->second->centroid.x < min_x)
            min_x = it->second->centroid.x;
        if (it->second->centroid.y > max_y)
            max_y = it->second->centroid.y;
        if (it->second->centroid.y < min_y)
            min_y = it->second->centroid.y;
    }

    if ((min_x > max_x) || (min_y > max_y))
        return cvRect(0, 0, WIDTH, HEIGHT);     // whole frame
    else    
        return cvRect(min_x, min_y, max_x-min_x, max_y-min_y);    
}

void Blob::showBlobs(void)
{
    for (unsigned char i = 0; i < Blob_data.red_blob; i++){
        cvCircle(frame, cvPoint(Blob_data.red_blob_pos[i].x,Blob_data.red_blob_pos[i].y), 
                9, CV_RGB(255,0,0), 1,8,0);     
    }
    
    for (unsigned char i = 0; i < Blob_data.green_blob; i++){
        cvCircle(frame, cvPoint(Blob_data.green_blob_pos[i].x,Blob_data.green_blob_pos[i].y), 
                9, CV_RGB(0,255,0), 1,8,0); 
    }    
    
    for (unsigned char i = 0; i < Blob_data.blue_blob; i++){
        cvCircle(frame, cvPoint(Blob_data.blue_blob_pos[i].x,Blob_data.blue_blob_pos[i].y), 
                9, CV_RGB(0,0,255), 1,8,0);         
    }        
}

void Blob::setBlobData(void){
        // This function just undistorts the blob position and loads it into the Blob_data struct
    
        // list blobs
        Blob_data.red_blob = 0;
        for (CvBlobs::const_iterator it=blob_red.begin(); it!=blob_red.end(); ++it)
        {
            Blob_data.red_blob_pos[Blob_data.red_blob].x = it->second->centroid.x + ROI.x;
            Blob_data.red_blob_pos[Blob_data.red_blob].y = it->second->centroid.y + ROI.y;
            Blob_data.red_blob++;
        }        
               
        Blob_data.green_blob = 0;
        for (CvBlobs::const_iterator it=blob_green.begin(); it!=blob_green.end(); ++it)
        {
            Blob_data.green_blob_pos[Blob_data.green_blob].x = it->second->centroid.x + ROI.x;
            Blob_data.green_blob_pos[Blob_data.green_blob].y = it->second->centroid.y + ROI.y;
            Blob_data.green_blob++;         
        }        
        
        Blob_data.blue_blob = 0;
        for (CvBlobs::const_iterator it=blob_blue.begin(); it!=blob_blue.end(); ++it)
        {
            Blob_data.blue_blob_pos[Blob_data.blue_blob].x = it->second->centroid.x + ROI.x;
            Blob_data.blue_blob_pos[Blob_data.blue_blob].y = it->second->centroid.y + ROI.y;
            Blob_data.blue_blob++;         
        }        

//        if (cam->getID() == 1){
//                cout << "Red blobs: " << Blob_data.red_blob << endl;//Blob_data.red_blob_pos[0].x << " " << Blob_data.red_blob_pos[0].y << endl;
//                cout << "Green blobs: " << Blob_data.green_blob << endl;
//                cout << "Blue blobs: " << Blob_data.blue_blob << endl;
//                cout << " " << endl;
//        }
        // For now just set the first blob as ..Center32f
        redCenter32f = Blob_data.red_blob_pos[0];
        greenCenter32f = Blob_data.green_blob_pos[0];
        blueCenter32f = Blob_data.blue_blob_pos[0];  
        
        undistortBlobs();   
}

IplImage * Blob::getFrame(void){
    return frame;
}

void Blob::undistortBlobs(void)
{
    //cout << "Before: " << Blob_data.red_blob_pos[0].x << " " << Blob_data.red_blob_pos[0].y << endl;
    undistortPoints(Blob_data.red_blob_pos, Blob_data.red_blob);
    //cout << "After: " << Blob_data.red_blob_pos[0].x << " " << Blob_data.red_blob_pos[0].y << endl;
    undistortPoints(Blob_data.green_blob_pos, Blob_data.green_blob);
    undistortPoints(Blob_data.blue_blob_pos, Blob_data.blue_blob);   
}

void Blob::undistortPoints(CvPoint2D32f cvpoint[], unsigned int num_blobs){

     if (num_blobs > 0){
        CvMat * inpoints = cvCreateMat(num_blobs,1,CV_32FC2);
        CvMat * outpoints = cvCreateMat(num_blobs,1,CV_32FC2);    
     
        for (unsigned char i = 0; i < num_blobs; i++){
                 CV_MAT_ELEM(*inpoints,CvPoint2D32f,i,0) = cvpoint[i];
        }
     
        cvUndistortPoints(inpoints, outpoints, cam->getIntrinsic(), cam->getDistortion());

        CvPoint2D32f temp;
        for (unsigned char i = 0; i < num_blobs; i++)
        {
            temp = CV_MAT_ELEM(*outpoints,CvPoint2D32f,i,0);
            cvpoint[i].x = temp.x*cam->getfx() + cam->getcx();
            cvpoint[i].y = temp.y*cam->getfy() + cam->getcy();
        }     
        cvReleaseMat(&inpoints);
        cvReleaseMat(&outpoints);         
     }
}

IplImage * Blob::getProcessedFrame(void){
    return frame;
}

IplImage * Blob::getSegmRed(void){
    return red;
}

IplImage * Blob::getSegmGreen(void){
    return green;
}

IplImage * Blob::getSegmBlue(void){
    return blue;
}

IplImage * Blob::getSegmBlobs(void){
    return blobs;
}

void Blob::setMinArea(int minArea){
    min_area = minArea;
}

void Blob::setMaxArea(int maxArea){
    max_area = maxArea;
}


Blob::~Blob() {
}

void Blob::drawRoom(void){
    
    CvScalar bottom_color = cvScalar(255,255,0,1);
    
    //draw left lower side
    double objectPoint1[3] = {-2500,-1565,0};
    double objectPoint2[3] = {1820,-1565,0};    
    cvDrawLine(frame,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]),
            bottom_color,1);
    
    //draw right lower side
    objectPoint1[0] = -2500;objectPoint1[1] = 1565;objectPoint1[2] = 0;
    objectPoint2[0] = 1820;objectPoint2[1] = 1565;objectPoint2[2] = 0;    
    cvDrawLine(frame,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]),
            bottom_color,1); 
    
    //draw top lower side
    objectPoint1[0] = 1820;objectPoint1[1] = -1565;objectPoint1[2] = 0;
    objectPoint2[0] = 1820;objectPoint2[1] = 1565;objectPoint2[2] = 0;    
    cvDrawLine(frame,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]),
            bottom_color,1);   
    
    //draw center
    objectPoint1[0] = 0;objectPoint1[1] = -1565;objectPoint1[2] = 0;
    objectPoint2[0] = 0;objectPoint2[1] = 1565;objectPoint2[2] = 0;    
    cvDrawLine(frame,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]),
            bottom_color,1);       
    //draw center
    objectPoint1[0] = -2500;objectPoint1[1] = 0;objectPoint1[2] = 0;
    objectPoint2[0] = 1820;objectPoint2[1] = 0;objectPoint2[2] = 0;    
    cvDrawLine(frame,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]),
            bottom_color,1);           
       
    // one level higher
    //draw left lower side
    objectPoint1[0] = -2500;objectPoint1[1] = -1565;objectPoint1[2] = 1000;
    objectPoint2[0] = 1820;objectPoint2[1] = -1565;objectPoint2[2] = 1000;     
    cvDrawLine(frame,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]),
            CV_RGB(0,255,0),1);
    
    //draw right lower side
    objectPoint1[0] = -2500;objectPoint1[1] = 1565;objectPoint1[2] = 1000;
    objectPoint2[0] = 1820;objectPoint2[1] = 1565;objectPoint2[2] = 1000;    
    cvDrawLine(frame,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]),
            CV_RGB(0,255,0),1); 
    
    //draw top lower side
    objectPoint1[0] = 1820;objectPoint1[1] = -1565;objectPoint1[2] = 1000;
    objectPoint2[0] = 1820;objectPoint2[1] = 1565;objectPoint2[2] = 1000;    
    cvDrawLine(frame,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]),
            CV_RGB(0,255,0),1);   
    
    //draw center
    objectPoint1[0] = 0;objectPoint1[1] = -1565;objectPoint1[2] = 1000;
    objectPoint2[0] = 0;objectPoint2[1] = 1565;objectPoint2[2] = 1000;    
    cvDrawLine(frame,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]),
            CV_RGB(0,255,0),1);       
    //draw center
    objectPoint1[0] = -2500;objectPoint1[1] = 0;objectPoint1[2] = 1000;
    objectPoint2[0] = 1820;objectPoint2[1] = 0;objectPoint2[2] = 1000;    
    cvDrawLine(frame,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]),
            CV_RGB(0,255,0),1);  
    
    CvScalar top_color = cvScalar(255,0,255,1);     
    // antoher level higher
    //draw left lower side
    objectPoint1[0] = -2500;objectPoint1[1] = -1565;objectPoint1[2] = 2000;
    objectPoint2[0] = 1820;objectPoint2[1] = -1565;objectPoint2[2] = 2000;     
    cvDrawLine(frame,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]),
            top_color,1);
    
    //draw right lower side
    objectPoint1[0] = -2500;objectPoint1[1] = 1565;objectPoint1[2] = 2000;
    objectPoint2[0] = 1820;objectPoint2[1] = 1565;objectPoint2[2] = 2000;    
    cvDrawLine(frame,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]),
            top_color,1); 
    
    //draw top lower side
    objectPoint1[0] = 1820;objectPoint1[1] = -1565;objectPoint1[2] = 2000;
    objectPoint2[0] = 1820;objectPoint2[1] = 1565;objectPoint2[2] = 2000;    
    cvDrawLine(frame,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]),
            top_color,1);   
    
    //draw center
    objectPoint1[0] = 0;objectPoint1[1] = -1565;objectPoint1[2] = 2000;
    objectPoint2[0] = 0;objectPoint2[1] = 1565;objectPoint2[2] = 2000;    
    cvDrawLine(frame,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]),
            top_color,1);       
    //draw center
    objectPoint1[0] = -2500;objectPoint1[1] = 0;objectPoint1[2] = 2000;
    objectPoint2[0] = 1820;objectPoint2[1] = 0;objectPoint2[2] = 2000;    
    cvDrawLine(frame,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]),
            top_color,1);    
    
//    char string_buffer[64];
//    sprintf(string_buffer, "ex-%d.jpg",cam->getID());
//    cvSaveImage(string_buffer ,frame);    
    
    
//    if (cam->getID()==2)
//    {
//        double cam1[3] = {-2380,-1065,3200};
//
//        double ray_red1[3] = {ray_red.x,ray_red.y,ray_red.z};
//        double ray_green1[3] = {ray_green.x,ray_green.y,ray_green.z};
//        double ray_blue1[3] = {ray_blue.x,ray_blue.y,ray_blue.z};
//        
//        //cout << ray_origin.x << " " << ray_origin.y << " " << ray_origin.z << endl;
//        cvDrawLine(frame,cam->get2Dcoordinates(&ray_red1[0]),
//                cam->get2Dcoordinates(&cam1[0]),CV_RGB(255,0,0),1);
//        cvDrawLine(frame,cam->get2Dcoordinates(&ray_green1[0]),
//                cam->get2Dcoordinates(&cam1[0]),CV_RGB(0,255,0),1);
//        cvDrawLine(frame,cam->get2Dcoordinates(&ray_blue1[0]),
//                cam->get2Dcoordinates(&cam1[0]),CV_RGB(0,0,255),1);        
//    }
         
}

void Blob::setRays(CvPoint3D32f rO,CvPoint3D32f r1,CvPoint3D32f r2){
    ray_red = rO;
    ray_green = r1;
    ray_blue = r2;
}

struct Sblob_data * Blob::getBlob_data(void){
    return &Blob_data;
}

void Blob::drawVirtualBlobs(void){

//    virtual_counter += M_PI/100;
//    virtual_move_y = 500*sin(virtual_counter);
//    virtual_move_x = 500*cos(virtual_counter);
//    virtual_move_z = 500*cos(virtual_counter);
//
//    // heli 1 blobs
//    double hx1 = heli1->getHeliPosition().x;
//    double hy1 = heli1->getHeliPosition().y;
//    double hz1 = heli1->getHeliPosition().z;
//    double hx2 = heli2->getHeliPosition().x+virtual_move_x;
//    double hy2 = heli2->getHeliPosition().y+virtual_move_y;
//    double hz2 = heli2->getHeliPosition().z;    
//    
//    double helicopter1red[3] = {hx1-200+virtual_move_x,hy1-280+virtual_move_y,hz1+virtual_move_z};
//    double helicopter1green[3] = {hx1-200+virtual_move_x,hy1+260+virtual_move_y,hz1+virtual_move_z};
//    double helicopter1blue[3] = {hx1+300+virtual_move_x,hy1+virtual_move_y,hz1+virtual_move_z};
//    
//    // heli 2 blobs
//    double helicopter2red[3] = {hx2-200,hy2-280,hz2};
//    double helicopter2green[3] = {hx2-200,hy2+260,hz2};
//    double helicopter2blue[3] = {hx2+300,hy2,hz2};
//    
////    cvDrawCircle(frame,cam->get2Dcoordinates(&helicopter1red[0]),3,CV_RGB(255,0,0),-1);
////    cvDrawCircle(frame,cam->get2Dcoordinates(&helicopter1green[0]),3,CV_RGB(0,255,0),-1);
////    cvDrawCircle(frame,cam->get2Dcoordinates(&helicopter1blue[0]),3,CV_RGB(0,0,255),-1);
//    
//    cvDrawCircle(frame,cam->get2Dcoordinates(&helicopter2red[0]),3,CV_RGB(255,0,0),-1);
//    cvDrawCircle(frame,cam->get2Dcoordinates(&helicopter2green[0]),3,CV_RGB(0,255,0),-1);
//    cvDrawCircle(frame,cam->get2Dcoordinates(&helicopter2blue[0]),3,CV_RGB(0,0,255),-1);
}