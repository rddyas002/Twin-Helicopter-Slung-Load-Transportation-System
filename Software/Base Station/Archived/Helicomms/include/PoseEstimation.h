/* 
 * File:   PoseEstimation.h
 * Author: yashren
 *
 * Created on 13 May 2013, 5:45 AM
 */

#ifndef POSEESTIMATION_H
#define	POSEESTIMATION_H

#include "OpencvCamera.h"
#include <stdlib.h>
#include <armadillo>
#include <math.h>
#include <boost/signals2.hpp>
#include <sys/time.h>
#include "../include/TCP_server.h"
#include "../include/UDP_socket.h"
#include "correctStateAndCov.h"

#define PATH_TO_SERVER_MOUNT "../MotionCaptureSlave/"
#define PREAMBLE_TO_INTRINSIC "Intrinsics/Intrinsics_320x240_"
#define PREAMBLE_TO_DISTORTION "Distortion/Distortion_320x240_"
#define PREAMBLE_TO_POSITION "Positions/Position_"
#define PREAMBLE_TO_ROTATION "Rotation/RotationAnglesCamera_"
#define PATH_TO_CENTER "../MotionCaptureSlave/Positions/PositionCenter.xml"
#define PATH_TO_OBJECTS "../MotionCaptureSlave/Positions/ObjectPoints.xml"

#define RED_BLOB_X -140
#define RED_BLOB_Y -264
#define GREEN_BLOB_X -144
#define GREEN_BLOB_Y 263
#define BLUE_BLOB_X 314
#define BLUE_BLOB_Y 0

#define MAX_ACTIVE_CAMERAS 4
#define MAX_CAMERAS 4

#define T0_TIME 1375679601


typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
}quaternion;

typedef struct{
    bool camera_data_received[MAX_CAMERAS];
    bool all_data_received;
} cam_data_struct;

class PoseEstimation {
public:

    // initialize/destroy
    PoseEstimation();
    virtual ~PoseEstimation();
    
    typedef boost::signals2::signal<void( PoseEstimation * )> m_signal;
    
    // helper
    char * get_concat_string(char * buffer, const char * pream, int id);
    
    // worker
    void loadData(char * data);
    bool doEstimation(void);
    quaternion * calculateQuaternion(mat R);
    void storeRotandTrans(mat x);    
    bool propagateMeasurements(OpencvCamera * v_cam, double t_current);
    float imagePointGrad(float pc_prev, float pc_prev2, float delta_t);
    float propPoint(float grad, float propTime, float prev_point);
    void clearCamDataReceived(void);
    void setCamDataReceived(int val);
    bool getAllDataReceived(void);
    void setAlldataReceived(bool val);
    
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
    
    static std::ofstream visualLog;
    static std::ofstream visualRaw;
    static void openLogFile(void);
    static void closeLogFile(void);
    static bool writeLogFile(void);
    static bool writeRawFile(void);
    static char write_buffer[512];    
    static char write_buffer_raw[512];    
    
    double getCalcDoneTime(void);
    int getNumOfCamerasUsed(void);
    void setRequestTime(void);
    double getRequestTime(void);
    int getActiveBlobs(void);
    
    OpencvCamera * getVirtualCamera(int index);
    
    void setUDPLink(UDP_socket * udp);
    
    // get signal
    m_signal * getDataReadySignal(void){return &data_ready;}
    
    void setCamParam(b_struct_T * tmp_z, int index);
    
    inline double timeSince(double start_time){
        struct timeval tv;
        gettimeofday(&tv,NULL);
        double t = (double) tv.tv_sec + (double) 1e-6*tv.tv_usec - start_time;
        return t;
    }    
private:
    // signal used to trigger data transfer to helicopter
    m_signal data_ready;
    
    OpencvCamera * virtual_cam[MAX_CAMERAS];
    
    quaternion h1q;
    mat R1, Rx;
    float tx, ty, tz; 
    float vx, vy, vz; 
    float roll, pitch, yaw;
    
    // fps timing
    bool first_run;
    double prev_timestamp;
    
    vec Red_platform, Green_platform, Blue_platform;
    
    double FPS;
    double total_time;
    int FPS_counter;
    
    bool first_time_prop;
    
    cam_data_struct camera_data;
    
    double calc_done_time;
    int num_cams_used;
    int active_blobs;
    double request_timestamp;
    
    TCP_server * tcp_server;
    UDP_socket * helicopter_link;
};

#endif	/* POSEESTIMATION_H */

