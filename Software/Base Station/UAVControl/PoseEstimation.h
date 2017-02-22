
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

#define NUM_OF_CAMERAS 4
#define LSQ_NON_ITERATONS   (50)
#define INIT_QUAT_EST_ITER  (100)

#define MEASUREMENT_NOISE_VARIANCE (0.015)

#define POSEESTIMATION_STATS_SAMPLE (100)
#define PEST_MIN_DIST   (40)
#define RESNORM_MIN     (110)

#define MAX_NUM_OF_HELICOPTERS (5)

typedef struct{
    bool camera_data_received[NUM_OF_CAMERAS];
    bool all_data_received;
} cam_data_struct;

class PoseEstimation {
public:
    typedef boost::signals2::signal<void(  void )> m_Signal;
    
    enum pe_state{
        PE_INITIALISING,
        PE_RUNNING,
        PE_LSQ_MODE,
        PE_TIMEOUT
    };    
    
    // initialize/destroy
    PoseEstimation(int num_active, Helicopter * helicopter_ref[MAX_NUM_OF_HELICOPTERS]);//Helicopter * heli1, Helicopter * heli2);
    virtual ~PoseEstimation();
    
    // helper
    char * get_concat_string(char * buffer, const char * pream, int id);
    
    // worker
    void clearCamDataReceived(void);
    bool allCamClear(void);
    void setCamParam(b_struct_T * tmp_z, int index);
    void setCamDataReceived(int val);
    bool allDataReceived(void);
    void doCorrection(void);
    void enableEKFCorrection(bool en);
    bool isInitialised(void);
    pe_state getState(void);
    void normalizeVector(float array[], int len);
    void quaternion_error(const float p[4], const float q[4], float q_err[4]);
    void quaternionRotate(const float q1[4], const float q2[4], float q[4]);
    void fillCamStruct(b_struct_T * tmp_z);
    bool lsqnon_Estimation(real_T init_x[7], const b_struct_T z[4], const real_T obj_pnts[9], real_T ret_x[7], real_T * resnorm);
    void LSQEstimation(Helicopter * h);
    double FPSCalc(void);
    double deltaTime(void);
    double dtCalc(void);
    void projectObjectPoint(const real_T h_x[7], const b_struct_T z,
        const real_T object_points[3], real_T image_point[2]);
    void getImagePoints(const real_T h_x[7], const b_struct_T z, 
        const real_T red3D[3], const real_T green3D[3], const real_T blue3D[3], real_T image_points[3][2]);
    void blobAllocate(const real_T h_x[7],
        const real_T red3D[3], const real_T green3D[3], const real_T blue3D[3], b_struct_T * z);
    real_T pixelDistance(const real_T a[2],const real_T b[2]);
    int retMinDistanceIndex(const real_T dist[5], const real_T max_blobs);
    void solveContention(int cam, b_struct_T z, const real_T image_points[3][2]);
    void findMostLikelyBlob(const int cam, const b_struct_T z[4], real_T image_red[5][2]
    , real_T image_green[5][2], real_T image_blue[5][2]
    , real_T red_dist[5], real_T green_dist[5], real_T blue_dist[5]
    , int * red_dist_min_index, int * green_dist_min_index, int * blue_dist_min_index
    , const real_T image_points_cam[3][2]);
    void modifyCamStruct(b_struct_T * z, const int red_min_index, 
        const int green_min_index, const int blue_min_index,
        real_T red_dist[5], real_T green_dist[5], real_T blue_dist[5]);
    void clearScreen(void);
    void initialiseEKF(void);
    void printTiming(void);
    void correctStates(Helicopter * h, real_T dt);
    bool initialiseHelicopter(Helicopter * helicopter);
    int activeBlobs(b_struct_T * z);
    real_T estimateHeliPose(Helicopter * helicopter, real_T pose_est[7]);
    void updateHelicopterMocapData(Helicopter * h, const b_struct_T z[4], real_T pose[7], real_T dt);
    void quaternionMultiply(const float q1[4], const float q2[4], float q[4]);
    
    // return functions    
    double getFPS(void);
    
    // log
    void initialiseLog(void);
    std::ofstream lsq_log;
    void openLog(char * str);
    void closeLog(void);
    bool writeLog(void);      
    
    // development
    void debugMessage(const char * str);
    void errorMessage(const char * str);  
    
    OpencvCamera * getVirtualCamera(int index);
    
    m_Signal* getSignal() { return &correction_done; }
    
    pthread_mutex_t set_cam_mutex;
    bool makeUDP(void);

private:
    OpencvCamera * virtual_cam[NUM_OF_CAMERAS];
    Helicopter * helicopter[MAX_NUM_OF_HELICOPTERS];
    int try_find[MAX_NUM_OF_HELICOPTERS];
    int active_helicopters;
    
    double FPS, process_time;
    
    bool dataFromCamReceived[NUM_OF_CAMERAS];
    bool ekf_on;
    
    bool initialising;
    
    m_Signal correction_done;
    
    // static variables replacement
    struct timespec fps_last;
    bool fps_first_enter;    
    struct timespec delta_last;
    bool delta_first_enter;   
    struct timespec dtcalc_last;
    bool dtcalc_first_enter;    
    
    int stats_count;
    double fps_sum;
    double proc_time_sum;    
    
    pe_state current_state;
    char log_buffer[512];
    
    UDP_socket * udp_lsq;
};

#endif	/* POSEESTIMATION_H */

