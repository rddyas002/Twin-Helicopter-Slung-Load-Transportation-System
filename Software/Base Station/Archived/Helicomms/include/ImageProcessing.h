/* 
 * File:   ImageProcessing.h
 * Author: yashren
 *
 * Created on 14 March 2013, 6:29 AM
 */

#ifndef IMAGEPROCESSING_H
#define	IMAGEPROCESSING_H

#include <QtGui>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "OpencvCamera.h"
#include "Blob.h"
#include "Helicopter.h"
#include "BlobAllocator.h"

using namespace cv;

class ImageProcessing : public QThread {
    Q_OBJECT
public:
    ImageProcessing(QObject *parent);
    void initializeImageProcessing(void);
    void kill_heap_objects(void);
    double getFPS(void);
    QImage IplImage2QImage(const IplImage *iplImage);
    QImage getQImage(void);
    ~ImageProcessing();
signals:
    void processingResult(const QImage image1);    
protected:
        void run();
        void msleep(int ms);    
private:
    QMutex mutex;
    QWaitCondition condition;  
    QImage img;    
    double FPS;
    
    // Camera variables
    OpencvCamera * camera;
    Blob * blob;
    Helicopter * helicopter;
    Helicopter * helicopter2;

    BlobAllocator * blobAllocator;
    
    bool camera_on_heap;
};

#endif	/* IMAGEPROCESSING_H */
