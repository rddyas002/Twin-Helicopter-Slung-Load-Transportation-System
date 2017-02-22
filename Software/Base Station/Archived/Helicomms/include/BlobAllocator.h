/* 
 * File:   BlobAllocator.h
 * Author: Yashren Reddi
 *
 * Created on 05 March 2013, 9:57 AM
 * The purpose of this class is to correctly allocate blobs so that the least squares
 * algorithm can use the right correspondence.
 */

#ifndef BLOBALLOCATOR_H
#define	BLOBALLOCATOR_H

#include "../include/Helicopter.h"
#include "../include/Blob.h"
#include "../include/OpencvCamera.h"
#include <armadillo>
#include <math.h>

#define RED_BLOB_X -240
#define RED_BLOB_Y -195
#define GREEN_BLOB_X -240
#define GREEN_BLOB_Y 198
#define BLUE_BLOB_X 314
#define BLUE_BLOB_Y 0

struct quaternion{
    float q0;
    float q1;
    float q2;
    float q3;
};

class BlobAllocator {
public:
    BlobAllocator(OpencvCamera * cam1,OpencvCamera * cam2,Blob * blob_1, Blob * blob_2,
            Helicopter * heli1, Helicopter * heli2);
    int allocateBlobs(void);
    void storeRotandTrans(mat x);
    float getRoll(void);
    float getPitch(void);
    float getYaw(void);
    float gettx(void);
    float getty(void);
    float gettz(void);    
    struct quaternion * calculateQuaternion(mat R);
    struct quaternion * getQuaternion(void);
    float SIGN(float x);
    float NORM(float a, float b, float c, float d);
    virtual ~BlobAllocator();
private:
    OpencvCamera * camera1;
    OpencvCamera * camera2;
    Blob * blob1;
    Blob * blob2;
    Helicopter * helicopter1;
    Helicopter * helicopter2;
    
    float roll, pitch, yaw;
    struct quaternion h1quat;
    mat R1, Rx;
    float tx, ty, tz;
};

#endif	/* BLOBALLOCATOR_H */

