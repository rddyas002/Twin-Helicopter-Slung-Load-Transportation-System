/* 
 * File:   Helicopter.cpp
 * Author: Yashren Reddi
 * 
 * Created on 05 March 2013, 9:02 AM
 * 
 * This class holds all the data for a particular helicopter - all relevant
 * blob positions from each camera view, platform position and attitude.
 * Also runs the least squares estimation for finding the position and attitude. 
 * This will run on separate threads.
 * 
 */

#ifndef HELICOPTER_H
#define	HELICOPTER_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cvblob.h>
#include <math.h>
#include "../include/OpencvCamera.h"

using namespace cvb;

class Helicopter {
public:
    Helicopter();
    
    CvPoint3D32f getHeliPosition(void);
    void setHeliPosition(CvPoint3D32f pos);
    
    unsigned int getID(void);
    void setID(unsigned int Id);
    
    virtual ~Helicopter();
private:
    CvPoint3D32f heli_position;
    unsigned int id;
};

#endif	/* HELICOPTER_H */

