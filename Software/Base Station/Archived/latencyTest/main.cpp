#include <cstdlib>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sched.h>

#define CV_WINDOW_NAME "Test Latency"

using namespace std;

IplImage * getFrame(void);
IplImage * queryFrame(void);
void stats_calc(double x_in, double * mean, double * var);

CvCapture * camera = NULL;

int main(int argc, char** argv) {
    struct timespec start,end;
    IplImage * image;    
    camera = cvCaptureFromCAM(200);   
 
    cvSetCaptureProperty(camera, CV_CAP_PROP_FRAME_WIDTH, 320);
    cvSetCaptureProperty(camera, CV_CAP_PROP_FRAME_HEIGHT, 240);    
    cvSetCaptureProperty(camera, CV_CAP_PROP_FPS, 125);   

    double width = cvGetCaptureProperty(camera, CV_CAP_PROP_FRAME_WIDTH);    
    double height = cvGetCaptureProperty(camera, CV_CAP_PROP_FRAME_HEIGHT);    
    double fps = cvGetCaptureProperty(camera, CV_CAP_PROP_FPS);    

    printf("Camera param %.0f FPS, %.0fx%.0f\r\n", fps, width, height);
    image = getFrame();
    cvWaitKey(10);
    image = getFrame();
    cvWaitKey(10);
    
    //cvNamedWindow(CV_WINDOW_NAME, CV_WINDOW_NORMAL);
    
    // Make this process a real-time process
    struct sched_param sp;
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);    
    if (sched_setscheduler(0, SCHED_FIFO, &sp) == -1){
        std::cout << "Process priority set failed" << std::endl;
    }
    else{
        std::cout << "Priority set to max = " << sp.sched_priority  << std::endl;
    }
        
    int c = 30;
    double mean, var;
    while(--c){
//        cvWaitKey(10);
        clock_gettime(CLOCK_REALTIME, &start);
        image = getFrame();
        clock_gettime(CLOCK_REALTIME, &end);
        double time_elapsed = (end.tv_sec - start.tv_sec)*1e6 + (end.tv_nsec - start.tv_nsec)/1e3;
        //stats_calc(time_elapsed, &mean, &var);
        printf("Elapsed time is %.0fus\r\n", time_elapsed);
        //cvShowImage(CV_WINDOW_NAME, image);            
    }
    
    cvReleaseCapture(&camera);
    cvReleaseImage(&image);
    
    return 0;
}

void stats_calc(double x_in, double * mean, double * var){
    static int count = 0;
    static double prev_mean = 0.0, current_mean = 0.0;
    static double prev_var = 0.0, current_var = 0.0;
    
    count++;
    
    if (count == 1){
        prev_mean = current_mean = x_in;
        prev_var = current_var = 0.0;
        *var = 0.0;
    }
    else{
        current_mean = prev_mean + (x_in - prev_mean)/count;
        current_var = prev_var + (x_in - prev_mean)*(x_in - current_mean);
        
        prev_mean = current_mean;
        prev_var = current_var;
        *var = current_var/(count - 1);
    }
    *mean = current_mean;
}



IplImage * getFrame(void){
    cvGrabFrame(camera);
    cvGrabFrame(camera);      
    return cvRetrieveFrame(camera);
}

IplImage * queryFrame(void){
    cvQueryFrame(camera);
    return cvQueryFrame(camera);
}
