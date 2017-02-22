/* 
 * File:   displayFrame.h
 * Author: yashren
 *
 * Created on 13 March 2013, 11:43 AM
 */

#ifndef DISPLAYFRAME_H
#define	DISPLAYFRAME_H

#include <QtGui>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;

class displayFrame : public QThread {
    Q_OBJECT
public:
    displayFrame(QObject *parent);
    QImage IplImage2QImage(const IplImage *iplImg);
    bool initializeCamera(int fd);
    void Play();
    void Stop();
    bool isStopped() const;
    double getFPS(void);
    ~displayFrame();
signals:
    void processedImage(const QImage image);    
protected:
        void run();
        void msleep(int ms);    
private:
    bool stop;
    CvCapture * capture;
    int frameRate;
    QMutex mutex;
    QWaitCondition condition;  
    IplImage * frame;
    QImage img;    
    double FPS;
};

#endif	/* DISPLAYFRAME_H */

