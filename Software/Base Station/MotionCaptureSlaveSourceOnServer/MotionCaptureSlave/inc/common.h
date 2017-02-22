/* 
 * File:   common.h
 * Author: yashren
 *
 * Created on 04 May 2013, 12:15 PM
 */

#ifndef COMMON_H
#define	COMMON_H

#include <math.h>
#include <pthread.h>
#include <boost/signals2.hpp>
#include <locale.h>
#include <sys/time.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cvblob.h>

#define T0_TIME 1375679601

#define BLOB_PROCESS_CORE 1
#define BLOB_AUTO_CORE 1
#define BLOB_CALIB_CORE 1

#define RUN_PROCESS_PRIORITY 99
#define AUTO_MODE_PRIORITY 99
#define CALIB_MODE_PRIORITY 90

#define SHOW_FOUND_BLOBS 1
//     |------------------|-2465-------------|
//     |------------------|------------------|
//     |------------------|------------------|
//     |------------------|------------------|
//     |------------------|------------------|
//-1555|__________________|__________________|1565___>y
//     |------------------|------------------|
//     |------------------|------------------|
//     |------------------|------------------|
//     |------------------|------------------|    
//     |------------------|1815--------------|
//                        |
//                        |
//                       \ / x  
#define ROOM_X_MAX 1815
#define ROOM_X_MIN -2465
#define ROOM_Y_MAX 1565
#define ROOM_Y_MIN -1555

#define PATH_TO_SERVER "../../Mocap_Master"
#define PATH_TO_SERVER_MOUNT "../../Mocap_Master/MotionCaptureSlave/"
#define PREAMBLE_TO_INTRINSIC "Intrinsics/Intrinsics_320x240_"
#define PREAMBLE_TO_DISTORTION "Distortion/Distortion_320x240_"
#define PREAMBLE_TO_POSITION "Positions/Position_"
#define PREAMBLE_TO_ROTATION "Rotation/RotationAnglesCamera_"
#define PATH_TO_CENTER "../../Mocap_Master/MotionCaptureSlave/Positions/PositionCenter.xml"
#define PATH_TO_OBJECTS "../../Mocap_Master/MotionCaptureSlave/Positions/ObjectPoints.xml"

enum relay_callback_msg{
    GET_DATA_RELAY,
    START_AUTO_RELAY,
    STOP_AUTO_RELAY,
    CALIBRATE_RELAY,
    KILL_PROGRAM_RELAY,
    CLIENT_DISCONNECT_RELAY,
    WRITE_FAIL_RELAY,
    CONNECTION_MADE_RELAY
};  


#endif	/* COMMON_H */

