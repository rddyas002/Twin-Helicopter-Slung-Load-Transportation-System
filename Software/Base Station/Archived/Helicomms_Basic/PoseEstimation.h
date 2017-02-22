
#ifndef POSEESTIMATION_H
#define	POSEESTIMATION_H

#include "OpencvCamera.h"
#include <stdlib.h>
#include <armadillo>
#include <math.h>
#include <boost/signals2.hpp>
#include <sys/time.h>
#include "correctStateAndCov.h"
#include "Helicopter.h"
#include <pthread.h>

#define PATH_TO_SERVER_MOUNT "../../Mocap_Master/MotionCaptureSlave/"
#define PREAMBLE_TO_INTRINSIC "Intrinsics/Intrinsics_320x240_"
#define PREAMBLE_TO_DISTORTION "Distortion/Distortion_320x240_"
#define PREAMBLE_TO_POSITION "Positions/Position_"
#define PREAMBLE_TO_ROTATION "Rotation/RotationAnglesCamera_"
#define PATH_TO_CENTER "../../Mocap_Master/MotionCaptureSlave/Positions/PositionCenter.xml"
#define PATH_TO_OBJECTS "../../Mocap_Master/MotionCaptureSlave/Positions/ObjectPoints.xml"

#define RED_BLOB_X -144
#define RED_BLOB_Y 255
#define RED_BLOB_Z 0
#define GREEN_BLOB_X -130
#define GREEN_BLOB_Y -265
#define GREEN_BLOB_Z 0
#define BLUE_BLOB_X 315
#define BLUE_BLOB_Y 0
#define BLUE_BLOB_Z 0

#define NUM_OF_CAMERAS 4

#define T0_TIME 1375679601

#define INIT_QUAT_EST_ITER  (20)

#define INIT_POSE_X     (0)
#define INIT_POSE_Y     (0)
#define INIT_POSE_Z     (0.045)
#define INIT_POSE_Q1     (0)
#define INIT_POSE_Q2     (1)
#define INIT_POSE_Q3     (0)
#define INIT_POSE_Q4     (0)
#define LSQ_NON_ITERATONS   (20)

#define MEASUREMENT_NOISE_VARIANCE  (10)//(14.65527E-3)

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
}quaternion;

typedef struct{
    bool camera_data_received[NUM_OF_CAMERAS];
    bool all_data_received;
} cam_data_struct;

class PoseEstimation {
public:
    // initialize/destroy
    PoseEstimation();
    virtual ~PoseEstimation();
    
    // helper
    char * get_concat_string(char * buffer, const char * pream, int id);
    
    // worker
    bool doEstimation(void);
    quaternion * calculateQuaternion(mat R);
    void storeRotandTrans(mat x);    
    bool propagateMeasurements(OpencvCamera * v_cam, double t_current);
    float imagePointGrad(float pc_prev, float pc_prev2, float delta_t);
    float propPoint(float grad, float propTime, float prev_point);
    void clearCamDataReceived(void);
    bool allCamClear(void);
    void setCamParam(b_struct_T * tmp_z, int index);
    void setCamDataReceived(int val);
    void setHelicopterReference(Helicopter * heli);
    void doCorrection(void);
    void enableEKFCorrection(bool en);
    bool isInitialised(void);
    void normalizeVector(float array[], int len);
    void quaternion_error(const float p[4], const float q[4], float q_err[4]);
    void quaternionRotate(const float q1[4], const float q2[4], float q[4]);
    void fillCamStruct(b_struct_T * tmp_z);
    bool lsqnon_Estimation(real_T init_x[7], real_T ret_x[7], real_T * resnorm);
    void setObjectPoints(void);
    
    // return functions
    double getTrans_x(void);
    double getTrans_y(void);
    double getTrans_z(void);
    double get_vx(void);
    double get_vy(void);
    double get_vz(void);    
    quaternion getQuaternion(void);
    double getRoll(void);
    double getPitch(void);
    double getYaw(void);    
    double getFPS(void);
    
    // development
    void debugMessage(const char * str);
    void errorMessage(const char * str);  
    
    OpencvCamera * getVirtualCamera(int index);
    
    pthread_mutex_t set_cam_mutex;
        
    inline double timeSince(double start_time){
        struct timeval tv;
        gettimeofday(&tv,NULL);
        double t = (double) tv.tv_sec + (double) 1e-6*tv.tv_usec - start_time;
        return t;
    }    
private:
    
    OpencvCamera * virtual_cam[NUM_OF_CAMERAS];
    Helicopter * helicopter_link;
    
    quaternion h1q;
    float quaternion_offset[4];
    mat R1, Rx;
    double tx, ty, tz; 
    double vx, vy, vz; 
    double roll, pitch, yaw;
    
    // fps timing
    bool first_run;
    double prev_timestamp;
    
    vec Red_platform, Green_platform, Blue_platform;
    
    double FPS;
    double total_time;
    int FPS_counter;
    
    double calc_done_time;
    int num_cams_used;
    int active_blobs;
    double request_timestamp;
    
    bool dataFromCamReceived[NUM_OF_CAMERAS];
    bool ekf_on;
    bool initialising;
    real_T red_blob3D[3];
    real_T green_blob3D[3];
    real_T blue_blob3D[3];
    real_T objectPoints[9];
    real_T offset_quaternion[4];
    real_T initial_translation[4];
};

#endif	/* POSEESTIMATION_H */

