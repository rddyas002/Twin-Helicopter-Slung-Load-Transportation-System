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


#include "../inc/Blob.h"

// thread catching c functions
extern "C"
{
    // this C function will be used to receive the thread and pass it back to the Thread instance
    void* thread_catch(void* arg)
    {
        Blob* t = static_cast<Blob*>(arg);
        t->processThread();
        return 0;
    }
    
    // this C function will be used to receive the thread and pass it back to the Thread instance
    void* thread_catch2(void* arg)
    {
        Blob* t = static_cast<Blob*>(arg);
        t->autoThread();
        return 0;
    } 
    
    // this C function will be used to receive the thread and pass it back to the Thread instance
    void* thread_catch3(void* arg)
    {
        Blob* t = static_cast<Blob*>(arg);
        t->calibThread();
        return 0;
    }     
}

Blob::Blob(OpencvCamera * camera, int id, bool draw_blob) {
    // set local
    setlocale(LC_ALL,"C");
        
    // Initialize pointers
    cam = camera;
    blobFrame = NULL;
    windowFrame = NULL;
    tempImage = NULL;
    hsv = NULL;
    blue = NULL;
    green = NULL;
    red = NULL;
    red1 = NULL;
    red2 = NULL;
    blobs = NULL;
    
    calib_thread = NULL;
    
    client_id = id;
    first_run = true;
    auto_thread_run = false;
    calib_thread_run = false;
    drawBlob = draw_blob;
    
    autoRunning = false;
    calibRunning = false;
    processRunning = false;
    
    blob_processing = false;
        
    time_t0 = getFractionalSeconds(0.0);
                    
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
           
    // run getBlob() once to initialize
    getBlob();
}

Blob::~Blob() {
    autoThreadEnd();    
    calibThreadEnd();        
//    processThreadEnd();
    
    releaseIplImage(blobFrame);
    releaseIplImage(windowFrame);
    releaseIplImage(hsv);
    releaseIplImage(blue);
    releaseIplImage(green);
    releaseIplImage(red);
    releaseIplImage(red1);
    releaseIplImage(red2);
    releaseIplImage(blobs);  
}

void Blob::releaseIplImage(IplImage * ImagePntr){
    if (ImagePntr != NULL){
        cvReleaseImage(&ImagePntr);
        ImagePntr = NULL;
    }    
}

double Blob::deltaTime(void){
    static struct timespec t0;
    static bool first_enter = false;
    struct timespec now;
    
    if (!first_enter){
        first_enter = true;
        clock_gettime(CLOCK_MONOTONIC_RAW, &t0);
        return 0.0;
    }
    else{
        clock_gettime(CLOCK_MONOTONIC_RAW, &now);
        double time_elaps = (now.tv_sec - t0.tv_sec)*1e6 + (now.tv_nsec - t0.tv_nsec)/1e3;
        memcpy(&t0, &now, sizeof(timespec));
        return time_elaps;
    }
}

double Blob::FPSCalc(void){
    static struct timespec t0;
    static bool first_enter = false;
    struct timespec now;
    
    if (!first_enter){
        first_enter = true;
        clock_gettime(CLOCK_MONOTONIC_RAW, &t0);
        return 0.0;
    }
    else{
        clock_gettime(CLOCK_MONOTONIC_RAW, &now);
        double time_elaps = (now.tv_sec - t0.tv_sec) + (now.tv_nsec - t0.tv_nsec)/1e9;
        memcpy(&t0, &now, sizeof(timespec));
        if (time_elaps != 0)
            return 1/time_elaps;
        else
            return 0;
    }
}

/*******************************************************************************
    Auto mode functions - runs on separate thread - continous mode sampling
    void Blob::runAuto(void) // called to spawn thread
    void Blob::autoThread(void) // actual thread to run 
    void Blob::autoThreadEnd(void) // stops autoThread from running
*******************************************************************************/
void Blob::runAuto(void){
    
    if (!autoRunning){
        // Schedule core
        cpu_set_t my_set;        
        CPU_ZERO(&my_set);       
        CPU_SET(BLOB_AUTO_CORE, &my_set);     // set the bit that represents core 
        sched_setaffinity(0, sizeof(cpu_set_t), &my_set); /* Set affinity of this process to */
                 
        auto_thread_run = false;
        pthread_join(auto_thread, NULL);
        int ret = pthread_create(&auto_thread, 0, &thread_catch2, this);
        
        if (ret == 0){
            struct sched_param sp;
            sp.__sched_priority = AUTO_MODE_PRIORITY;
            pthread_setschedparam(auto_thread, SCHED_FIFO, &sp);        
        }
        else
        {
            errorMessage("Error creating auto-mode thread");
        }
    }
    else
    {
        debugMessage("Attempted to create thread while previous one was still running");
    }
}

void Blob::autoThread(void){
    autoRunning = true;
    auto_thread_run = true;
    debugMessage("Auto-sample mode ON");
    while(auto_thread_run){
        getBlob();       
    }
    debugMessage("Auto-sample mode OFF");
    autoRunning = false;
}

void Blob::autoThreadEnd(void){
    auto_thread_run = false;
//    pthread_join(auto_thread, NULL);
}

/*******************************************************************************
END   Auto mode functions - runs on separate thread - continous mode sampling
*******************************************************************************/


/*******************************************************************************
    runProcess functions - runs on separate thread - single sample moode
    void Blob::runProcess(void) // called to spawn thread
    void Blob::processThread(void) // actual thread to run 
    void Blob::autoThreadEnd(void) // stops autoThread from running
*******************************************************************************/
void Blob::runProcess(void)
{
    if (!processRunning){
        // Schedule core
        cpu_set_t my_set;        
        CPU_ZERO(&my_set);       
        CPU_SET(BLOB_PROCESS_CORE, &my_set);     // set the bit that represents core 
        sched_setaffinity(0, sizeof(cpu_set_t), &my_set); /* Set affinity of this process to */
         
        pthread_join(process_thread, NULL);
        int ret = pthread_create(&process_thread, 0, &thread_catch, this);
    
        if (ret == 0){
            struct sched_param sp;
            sp.__sched_priority = RUN_PROCESS_PRIORITY;
            pthread_setschedparam(process_thread, SCHED_FIFO, &sp);        
        }
        else
        {
            errorMessage("Error creating run process thread");
        }
    }
    else
    {
        debugMessage("Attempted to create thread while previous one was still running");
    }
}

void Blob::processThread(void)
{
    processRunning = true;
    Blob::getBlob();   
    processRunning = false;
}

void Blob::processThreadEnd(void)
{
    // no loop here
    pthread_join(process_thread, NULL);
}
/*******************************************************************************
END   prcess thread functions - runs on separate thread
*******************************************************************************/


/*******************************************************************************
    calibMode functions - runs on separate thread - just displays stream with room corners
    void Blob::calibMode(void) // called to spawn thread
    void Blob::calibThread(void) // actual thread to run 
    void Blob::calibThreadEnd(void) // stops autoThread from running
*******************************************************************************/
void Blob::calibMode(void){
    if (!calibRunning){
        // Schedule core
        cpu_set_t my_set;        
        CPU_ZERO(&my_set);       
        CPU_SET(BLOB_CALIB_CORE, &my_set);     // set the bit that represents core 
        sched_setaffinity(0, sizeof(cpu_set_t), &my_set); /* Set affinity of this process to */
                 
        calib_thread_run = false;
        pthread_join(calib_thread, NULL);
        int ret = pthread_create(&calib_thread, 0, &thread_catch3, this);
        
        if (ret == 0){
            struct sched_param sp;
            sp.__sched_priority = CALIB_MODE_PRIORITY;
            pthread_setschedparam(calib_thread, SCHED_FIFO, &sp);        
        }
        else
        {
            errorMessage("Error creating calibrate thread");
        }
    }
    else
    {
        debugMessage("Attempted to create thread while previous one was still running");
    }        
}

void Blob::calibThread(void){
    debugMessage("Calibration mode ON");
    calib_thread_run = true;
    while(calib_thread_run){
        doCalib();
    }
    debugMessage("Calibration mode OFF");
}

void Blob::doCalib(void){
    cvRemap(tempImage, windowFrame, cam->get_mapx(), cam->get_mapy());
    drawRoom(windowFrame);
}

void Blob::calibThreadEnd(void){
    calib_thread_run = false;
    if (calib_thread != NULL)
        pthread_join(calib_thread, NULL);
}
/*******************************************************************************
 *******************************************************************************
 END   prcess thread functions - runs on separate thread
 *******************************************************************************
*******************************************************************************/

/*******************************************************************************
    image processing functions
    void Blob::getBlob(void) // main processing
    CvRect Blob::getROI(CvBlobs &bl) 
    void Blob::showBlobs(void)
    void Blob::setBlobData(void)
    void Blob::formBlobPacket(void)
    void Blob::undistortBlobs(void)
    void Blob::undistortPoints(CvPoint2D32f cvpoint[], unsigned int num_blobs)
*******************************************************************************/
void Blob::getBlob(void)
{
    static int stats_count = 0;
    static double latency_sum = 0.0;
    static double fps_sum = 0.0;
    unsigned int blobs_scene = 0;
    
    CvRect newROI;
    blob_processing = true;
    // get blobFrame from camera
    deltaTime();
    blobFrame = cam->getFrame();
    
    if (drawBlob){
        // draw virtual blobs
        drawVirtualBlobs();    
    }

    if (first_run)
    {
        FPSCalc();
        // find initial region of interest
        first_run = false;
        
        tempImage = cvCloneImage(blobFrame);
        windowFrame = cvCloneImage(blobFrame);
        
        // convert to hsv space
        cvCvtColor(blobFrame, hsv, CV_BGR2HSV);
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
               
        // use cvblob lib to get position of blobs
        IplImage *labelImg = cvCreateImage(cvGetSize(blobs), IPL_DEPTH_LABEL, 1);

        cvLabel(blobs, labelImg, blobs_ROI);   
        cvFilterByArea(blobs_ROI, INITIAL_AREA_BLOB_FILTER_MIN, INITIAL_AREA_BLOB_FILTER_MAX);
           
        // Limit ROI to center of room first
        ROI.x = 1*BOUND_BORDER;
        ROI.y = 1*BOUND_BORDER;
        ROI.width = WIDTH - ROI.x*2;
        ROI.height = HEIGHT - ROI.y*2;        

        cvSetImageROI(blobFrame, ROI);
        cvReleaseImage(&labelImg);
    }
    else        // use region of interest and adjust
    {
        // for some reason ROI wasn't set on first instance
        cvSetImageROI(blobFrame, ROI);
        
        IplImage * hsv_roi = cvCreateImage(cvSize(ROI.width,ROI.height), IPL_DEPTH_8U, 3);
        IplImage * blue_roi = cvCreateImage(cvSize(ROI.width,ROI.height), IPL_DEPTH_8U, 1);
        IplImage * green_roi = cvCreateImage(cvSize(ROI.width,ROI.height), IPL_DEPTH_8U, 1);
        IplImage * red1_roi = cvCreateImage(cvSize(ROI.width,ROI.height), IPL_DEPTH_8U, 1);
        IplImage * red2_roi = cvCreateImage(cvSize(ROI.width,ROI.height), IPL_DEPTH_8U, 1);
        IplImage * red_roi = cvCreateImage(cvSize(ROI.width,ROI.height), IPL_DEPTH_8U, 1);
        IplImage * all_blobs = cvCreateImage(cvSize(ROI.width,ROI.height), IPL_DEPTH_8U, 1);
        CvBlobs all_blobs_roi;
        
        // convert ROI to hsv space
        cvCvtColor(blobFrame, hsv_roi, CV_BGR2HSV);        
        
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
            min_area = 45;
            max_area = 250;
            cvFilterByArea(blob_red, min_area, max_area);
            cvFilterByArea(blob_green, min_area, max_area);
            cvFilterByArea(blob_blue, min_area, max_area);            
        }
                      
        // Blobs may have moved, hence, we need to adjust ROI based on new positions
        // get image of blobs or'd to determine extremes
        // ** NOTE ROI not based on filtered blobs
        cvOr(red_roi, green_roi, all_blobs, NULL);
        cvOr(all_blobs, blue_roi, all_blobs, NULL);      
        
        cvLabel(all_blobs, labelImg0, all_blobs_roi);
        
        cvFilterByArea(all_blobs_roi, min_area, max_area);
        newROI = getROI(all_blobs_roi);
                     
        cvResetImageROI(blobFrame);
        
        cvRectangle(blobFrame, cvPoint(ROI.x, ROI.y), cvPoint(ROI.x+ROI.width, ROI.y+ROI.height), cvScalarAll(255));
        
        // undistort blobs and set in struct
        blobs_scene = setBlobData();
        
        if (SHOW_FOUND_BLOBS){
            // Encircle blobs with appropriate colour
            showBlobs();
        }
        
        // release memory used
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
                       
        // Make sure this is the last thing you do
        // If we subtract the border from each newROI min 
        // then we move back to the blobFrame region of interest ROI.x ROI.y
        ROI.x = (ROI.x + (newROI.x - BOUND_BORDER));
        ROI.y = (ROI.y + (newROI.y - BOUND_BORDER));
        ROI.width = newROI.width + 2*BOUND_BORDER;
        ROI.height = newROI.height + 2*BOUND_BORDER;
        
        // Cannot go outside the blobFrame...just check
        if (ROI.x < 0) ROI.x = 0;
        if (ROI.y < 0) ROI.y = 0;
        if (ROI.x > WIDTH){ROI.x = 0; cout << "Error: ROI.x > WIDTH" << endl;}
        if (ROI.y > HEIGHT){ROI.y = 0; cout << "Error: ROI.x > HEIGHT" << endl;}        
        if ((ROI.x + ROI.width) > WIDTH)
        {
            ROI.width = WIDTH - ROI.x;
            //cout << "Error: ROI width outside blobFrame width. ROI.x = " << ROI.x << " Width = " <<  ROI.width << endl;
        }
        if ((ROI.y + ROI.height) > HEIGHT)
        {
            ROI.height = HEIGHT - ROI.y;
            //cout << "Error: ROI height outside blobFrame height. ROI.y = " << ROI.y << " Width = " <<  ROI.height << endl;
        }        
        
        // emit signal that conversion is done
        conversion_done(&data_packet[0]);   
        cvCopy(blobFrame,windowFrame);    
    }
    
    // Compute average processing latency for 100 samples
    ++stats_count;
    latency_sum += deltaTime();
    fps_sum += FPSCalc();
        
    if (stats_count == STATS_SAMPLE){
        latency = latency_sum/STATS_SAMPLE;
        fps = fps_sum/STATS_SAMPLE;
        stats_count = 0;
        latency_sum = 0;
        fps_sum = 0;
        printf("\rAvg latency: %06.0fus Avg FPS: %03.1f Blobs: %u    ", latency ,fps, blobs_scene);
        fflush(stdout);
    }        

    blob_processing = false;
}

double Blob::getAverageLatency(void){
    return latency;
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
        return cvRect(0, 0, WIDTH, HEIGHT);     // whole blobFrame
    else    
        return cvRect(min_x, min_y, max_x-min_x, max_y-min_y);    
}

void Blob::showBlobs(void)
{
    for (unsigned char i = 0; i < Blob_data.red_blob; i++){
        cvCircle(blobFrame, cvPoint(Blob_data.red_blob_pos[i].x,Blob_data.red_blob_pos[i].y), 
                9, CV_RGB(255,0,0), 1,8,0);     
    }
    
    for (unsigned char i = 0; i < Blob_data.green_blob; i++){
        cvCircle(blobFrame, cvPoint(Blob_data.green_blob_pos[i].x,Blob_data.green_blob_pos[i].y), 
                9, CV_RGB(0,255,0), 1,8,0); 
    }    
    
    for (unsigned char i = 0; i < Blob_data.blue_blob; i++){
        cvCircle(blobFrame, cvPoint(Blob_data.blue_blob_pos[i].x,Blob_data.blue_blob_pos[i].y), 
                9, CV_RGB(0,0,255), 1,8,0);         
    }        
}

unsigned int Blob::setBlobData(void){
    unsigned int num_blobs = 0;
    // This function just undistorts the blob position and loads it into the Blob_data struct
    
    // list blobs
    Blob_data.red_blob = 0;
    for (CvBlobs::const_iterator it=blob_red.begin(); it!=blob_red.end(); ++it)
    {
        if ((Blob_data.red_blob + 1) > MAX_BLOBS_HOLD)
        {
            debugMessage("More blobs in image than array size");
            // break out of for loop
            break;
        }
        
        Blob_data.red_blob_pos[Blob_data.red_blob].x = it->second->centroid.x + ROI.x;
        Blob_data.red_blob_pos[Blob_data.red_blob].y = it->second->centroid.y + ROI.y;
        Blob_data.red_blob++;
        num_blobs++;
        
    }        
               
    Blob_data.green_blob = 0;
    for (CvBlobs::const_iterator it=blob_green.begin(); it!=blob_green.end(); ++it)
    {
        if ((Blob_data.green_blob + 1) > MAX_BLOBS_HOLD)
        {
            debugMessage("More blobs in image than array size");
            // break out of for loop
            break;
        }        
        
        Blob_data.green_blob_pos[Blob_data.green_blob].x = it->second->centroid.x + ROI.x;
        Blob_data.green_blob_pos[Blob_data.green_blob].y = it->second->centroid.y + ROI.y;
        Blob_data.green_blob++;  
        num_blobs++;
    }        
        
    Blob_data.blue_blob = 0;
    for (CvBlobs::const_iterator it=blob_blue.begin(); it!=blob_blue.end(); ++it)
    {
        if ((Blob_data.blue_blob + 1) > MAX_BLOBS_HOLD)
        {
            debugMessage("More blobs in image than array size");
            // break out of for loop
            break;
        }          
        
        Blob_data.blue_blob_pos[Blob_data.blue_blob].x = it->second->centroid.x + ROI.x;
        Blob_data.blue_blob_pos[Blob_data.blue_blob].y = it->second->centroid.y + ROI.y;
        Blob_data.blue_blob++;     
        num_blobs++;
    }        
       
    undistortBlobs();
    formBlobPacket();
    
    return num_blobs;
}

void Blob::formBlobPacket(void){
    char temp_packet[5][20];
    int str_len = 0;    
    
    setlocale(LC_ALL,"C");
    
    sprintf(&data_packet[0], "*C%d:", client_id);
    str_len = strlen(&data_packet[0]);
    
    sprintf(&data_packet[str_len], "%u,%u,%u", Blob_data.red_blob, Blob_data.green_blob, Blob_data.blue_blob);
    str_len = strlen(&data_packet[0]);
    
    for (int i = 0; i < Blob_data.red_blob; i++){
        sprintf(&temp_packet[i][0], "|%.2f|%.2f", Blob_data.red_blob_pos[i].x, Blob_data.red_blob_pos[i].y);
        strcpy(&data_packet[str_len], &temp_packet[i][0]);
        str_len = strlen(&data_packet[0]);
    }    
    
    for (int i = 0; i < Blob_data.green_blob; i++){
        sprintf(&temp_packet[i][0], "|%.2f|%.2f", Blob_data.green_blob_pos[i].x, Blob_data.green_blob_pos[i].y);
        strcpy(&data_packet[str_len], &temp_packet[i][0]);
        str_len = strlen(&data_packet[0]);
    }    

    for (int i = 0; i < Blob_data.blue_blob; i++){
        sprintf(&temp_packet[i][0], "|%.2f|%.2f", Blob_data.blue_blob_pos[i].x, Blob_data.blue_blob_pos[i].y);
        strcpy(&data_packet[str_len], &temp_packet[i][0]);
        str_len = strlen(&data_packet[0]);
    }        
    
    //sprintf(&data_packet[str_len], "|%.3f#\r\n",getFractionalSeconds(T0_TIME));
}

void Blob::undistortBlobs(void)
{
    undistortPoints(Blob_data.red_blob_pos, Blob_data.red_blob);
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
/******************************************************************************
 ******************************************************************************
 * END main processing functions
 ******************************************************************************
 ******************************************************************************/


/*******************************************************************************
  ******************************************************************************
    Some set/get functions
  ******************************************************************************
*******************************************************************************/
IplImage * Blob::getFrame(void){
    return blobFrame;
}

double Blob::getProgramRunTime(void){
    return getFractionalSeconds(time_t0);
}

IplImage * Blob::getProcessedFrame(void){
    return blobFrame;
}

IplImage * Blob::getWindowFrame(void){
    return windowFrame;
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

volatile bool Blob::blobProcessing(void){
    return blob_processing;
}

/******************************************************************************
 ******************************************************************************
 * END set/get functions
 ******************************************************************************
 ******************************************************************************/

IplImage* Blob::drawRoom(IplImage * temp_frame){   
    drawGrid(temp_frame, 0, cvScalar(255,255,0,1));
    drawGrid(temp_frame, 1000, CV_RGB(0,255,0));
    drawGrid(temp_frame, 2000, cvScalar(255,0,255,1));
    drawFloor2x2(temp_frame);
    return temp_frame;
}

void Blob::drawFloor2x2(IplImage * tmp){
    double objectPoint1[3] = {-1000,-1000,0};
    double objectPoint2[3] = {1000,-1000,0};
    cvDrawLine(tmp,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]), CV_RGB(255,255,255),1);

    objectPoint1[0] = 1000; objectPoint1[1] = -1000; objectPoint1[2] = 0;
    objectPoint2[0] = 1000; objectPoint2[1] = 1000 ; objectPoint2[2] = 0;    
    cvDrawLine(tmp,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]), CV_RGB(255,255,255),1);     
    
    objectPoint1[0] = 1000; objectPoint1[1] = 1000; objectPoint1[2] = 0;
    objectPoint2[0] = -1000; objectPoint2[1] = 1000 ; objectPoint2[2] = 0;    
    cvDrawLine(tmp,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]), CV_RGB(255,255,255),1);         
    
    objectPoint1[0] = -1000; objectPoint1[1] = 1000; objectPoint1[2] = 0;
    objectPoint2[0] = -1000; objectPoint2[1] = -1000 ; objectPoint2[2] = 0;    
    cvDrawLine(tmp,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]), CV_RGB(255,255,255),1);             
}

void Blob::drawGrid(IplImage * tmp, int height, CvScalar line_color){
    //draw Y_MAX_LINE
    double objectPoint1[3] = {ROOM_X_MIN,ROOM_Y_MIN,height};
    double objectPoint2[3] = {ROOM_X_MAX,ROOM_Y_MIN,height};    
    cvDrawLine(tmp,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]), line_color,1);
    
    //draw Y_MIN_LINE
    objectPoint1[0] = ROOM_X_MIN;objectPoint1[1] = ROOM_Y_MAX;objectPoint1[2] = height;
    objectPoint2[0] = ROOM_X_MAX;objectPoint2[1] = ROOM_Y_MAX;objectPoint2[2] = height;    
    cvDrawLine(tmp,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]), line_color,1); 
    
    //draw X_MAX_LINE
    objectPoint1[0] = ROOM_X_MAX;objectPoint1[1] = ROOM_Y_MIN;objectPoint1[2] = height;
    objectPoint2[0] = ROOM_X_MAX;objectPoint2[1] = ROOM_Y_MAX;objectPoint2[2] = height;    
    cvDrawLine(tmp,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]), line_color,1);
    
    //draw X_MIN_LINE
    objectPoint1[0] = ROOM_X_MIN;objectPoint1[1] = ROOM_Y_MIN;objectPoint1[2] = height;
    objectPoint2[0] = ROOM_X_MIN;objectPoint2[1] = ROOM_Y_MAX;objectPoint2[2] = height;    
    cvDrawLine(tmp,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]), line_color,1);    
    
    //draw center
    objectPoint1[0] = 0;objectPoint1[1] = ROOM_Y_MIN;objectPoint1[2] = height;
    objectPoint2[0] = 0;objectPoint2[1] = ROOM_Y_MAX;objectPoint2[2] = height;    
    cvDrawLine(tmp,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]), line_color,1);       
    //draw center
    objectPoint1[0] = ROOM_X_MIN;objectPoint1[1] = 0;objectPoint1[2] = height;
    objectPoint2[0] = ROOM_X_MAX;objectPoint2[1] = 0;objectPoint2[2] = height;    
    cvDrawLine(tmp,cam->get2Dcoordinates(&objectPoint1[0]),cam->get2Dcoordinates(&objectPoint2[0]), line_color,1);           
           
}

void Blob::setRays(CvPoint3D32f rO,CvPoint3D32f r1,CvPoint3D32f r2){
    ray_red = rO;
    ray_green = r1;
    ray_blue = r2;
}

void Blob::drawVirtualBlobs(void){

    virtual_counter += M_PI/100;
    virtual_move_y = 500*sin(virtual_counter);
    virtual_move_x = 500*cos(virtual_counter);
    virtual_move_z = 500*cos(virtual_counter);

    // heli 1 blobs
    double hx1 = 100;
    double hy1 = -300;
    double hz1 = 500;
    double hx2 = 400;
    double hy2 = 500;
    double hz2 = 500;    
//    
    double helicopter1red[3] = {hx1-200+virtual_move_x,hy1-280+virtual_move_y,hz1+virtual_move_z};
    double helicopter1green[3] = {hx1-200+virtual_move_x,hy1+260+virtual_move_y,hz1+virtual_move_z};
    double helicopter1blue[3] = {hx1+300+virtual_move_x,hy1+virtual_move_y,hz1+virtual_move_z};
//    
    // heli 2 blobs
    double helicopter2red[3] = {hx2-200,hy2-280,hz2};
    double helicopter2green[3] = {hx2-200,hy2+260,hz2};
    double helicopter2blue[3] = {hx2+300,hy2,hz2};
//    
    cvDrawCircle(blobFrame,cam->get2Dcoordinates(&helicopter1red[0]),3,CV_RGB(255,0,0),-1);
    cvDrawCircle(blobFrame,cam->get2Dcoordinates(&helicopter1green[0]),3,CV_RGB(0,255,0),-1);
    cvDrawCircle(blobFrame,cam->get2Dcoordinates(&helicopter1blue[0]),3,CV_RGB(0,0,255),-1);
//    
//    cvDrawCircle(blobFrame,cam->get2Dcoordinates(&helicopter2red[0]),3,CV_RGB(255,0,0),-1);
//    cvDrawCircle(blobFrame,cam->get2Dcoordinates(&helicopter2green[0]),3,CV_RGB(0,255,0),-1);
//    cvDrawCircle(blobFrame,cam->get2Dcoordinates(&helicopter2blue[0]),3,CV_RGB(0,0,255),-1);
}

void Blob::errorMessage(const char * str){
    printf("ERR Blob %d: %s\r\n", client_id, str);
}

void Blob::debugMessage(const char * str){
    printf("DBG Blob %d: %s\r\n", client_id, str);
}