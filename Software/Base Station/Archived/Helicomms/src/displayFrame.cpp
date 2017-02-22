/* 
 * File:   displayFrame.cpp
 * Author: yashren
 * 
 * Created on 13 March 2013, 11:43 AM
 */

#include "include/displayFrame.h"
#include <sys/time.h>
#include <iostream>

displayFrame::displayFrame(QObject *parent) : QThread(parent){
    stop = true;
    frameRate = 33;
    FPS = 0;
}

void displayFrame::run()
{
    qDebug() << "Starting frame display thread";
    int delay = (1000/frameRate);
    
    timeval start, end;
    double elapsedTime;
    double totalTime = 0;
    int iter = 0;
    while(!stop)
    {
        gettimeofday(&start, NULL);
        frame = cvQueryFrame(capture);
        
        emit processedImage(IplImage2QImage(frame));
        
        gettimeofday(&end, NULL);
        elapsedTime = (end.tv_sec - start.tv_sec) * 1000.0;      // sec to ms
        elapsedTime += (end.tv_usec - start.tv_usec) / 1000.0;   // us to ms
        totalTime += elapsedTime;

        if (++iter == 20)
        {
            FPS = 1000*(double)iter/totalTime;

            iter = 0;
            totalTime = 0;
        }
        
        this->msleep(1);
    }
    FPS = 0;
    QImage rect = QImage(320,240,QImage::Format_RGB32);
    rect.fill(Qt::black);    
    emit processedImage(rect);
    
    qDebug() << "Execution done";
}

double displayFrame::getFPS(void){
    return FPS;
}

bool displayFrame::initializeCamera(int fd) 
{
    capture = cvCaptureFromCAM(fd);
    if (capture != NULL)
    {
        return true;
    }
    else
        return false;
}

void displayFrame::Play()
{
    if (!isRunning()) 
    {
        if (isStopped())
        {
                stop = false;
        }
        start(HighPriority);
    }
}

void displayFrame::Stop()
{
    stop = true;
}

bool displayFrame::isStopped() const{
  return this->stop;
}

void displayFrame::msleep(int ms){
    struct timespec ts = { ms / 1000, (ms % 1000) * 1000 * 1000 };
    nanosleep(&ts, NULL);
}

displayFrame::~displayFrame()
{
    mutex.lock();
    stop = true;
    cvReleaseCapture(&capture);
    condition.wakeOne();
    mutex.unlock();
    wait();
}

QImage displayFrame::IplImage2QImage(const IplImage *iplImage)
{
       int height = iplImage->height;
       int width = iplImage->width;

       const uchar *qImageBuffer =(const uchar*)iplImage->imageData;
       QImage img(qImageBuffer, width, height, QImage::Format_RGB888);
       return img.rgbSwapped();
}