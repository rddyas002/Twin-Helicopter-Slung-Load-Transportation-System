/* 
 * File:   ImageProcessing.cpp
 * Author: yashren
 * 
 * Created on 14 March 2013, 6:29 AM
 */

#include "../include/ImageProcessing.h"
#include <iostream>

ImageProcessing::ImageProcessing(QObject *parent): QThread(parent){
    FPS = 0;
    camera_on_heap = false;
}

void ImageProcessing::initializeImageProcessing(void){
    // create new camera objects
    camera = new OpencvCamera(200,1);
    
    camera_on_heap = true;
    
    // load camera intrinsics and extrinsics
    if (RES640X480)
    {
        camera->setIntrinsic((CvMat*)cvLoad("calibration_param/Intrinsics1.xml"));
        camera->setDistortion((CvMat*)cvLoad("calibration_param/Distortion1.xml"));        
    }
    else
    {
        camera->setIntrinsic((CvMat*)cvLoad("calibration_param/Intrinsics1_320x240.xml"));
        camera->setDistortion((CvMat*)cvLoad("calibration_param/Distortion1_320x240.xml"));        
    }
    camera->setCamPosition((CvMat*)cvLoad("cam_positions/Position1.xml"));
    camera->setCenter((CvMat*)cvLoad("cam_positions/PositionCenter.xml"));
    camera->setCalibObjectPoints((CvMat*)cvLoad("calibration_param/ObjectPoints.xml"));
    camera->loadRotationAngles((CvMat*)cvLoad("calibration_param/RotationAnglesCamera1.xml"));
    camera->calcExtrinsics();
    
    helicopter = new Helicopter();
    helicopter2 = new Helicopter();
    helicopter->setID(1);
    helicopter2->setID(2);
    // separate heli's by a meter
    helicopter->setHeliPosition(cvPoint3D32f(0,-500,1000));
    helicopter2->setHeliPosition(cvPoint3D32f(0,500,1000));    
    
    blob = new Blob(camera, helicopter, helicopter2);   
}

void ImageProcessing::run()
{
//    blob->runProcess();
//    blob->wait4ThreadEnd(); 
  
    emit processingResult(IplImage2QImage(camera->getFrame()));//blob->getFrame()));
}

double ImageProcessing::getFPS(void){
    return FPS;
}

QImage ImageProcessing::getQImage(void){
    return IplImage2QImage(camera->getFrame());
}

void ImageProcessing::msleep(int ms){
    struct timespec ts = { ms / 1000, (ms % 1000) * 1000 * 1000 };
    nanosleep(&ts, NULL);
}

QImage ImageProcessing::IplImage2QImage(const IplImage *iplImage)
{
       int height = iplImage->height;
       int width = iplImage->width;

       const uchar *qImageBuffer =(const uchar*)iplImage->imageData;
       QImage img(qImageBuffer, width, height, QImage::Format_RGB888);
       return img.rgbSwapped();
}

ImageProcessing::~ImageProcessing() {
    mutex.lock();
    condition.wakeOne();
    mutex.unlock();
    wait();    
    if (camera_on_heap){
        delete camera;
        delete helicopter;
        delete helicopter2;    
    }
}

void ImageProcessing::kill_heap_objects() {
    mutex.lock();
    condition.wakeOne();
    mutex.unlock();
    wait();    
    
    delete camera;
    delete helicopter;
    delete helicopter2;
}

