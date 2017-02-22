/* 
 * File:   MotionCapture.h
 * Author: yashren
 *
 * Created on 14 May 2013, 7:57 PM
 */

#ifndef MOTIONCAPTURE_H
#define	MOTIONCAPTURE_H

#include <cstdlib>
#include <iostream>
#include <string>
#include <boost/signals2.hpp>

#include "TCP_server.h"
#include "OpencvCamera.h"
#include "Blob.h"
#include "common.h"

enum MotionCaptureState{
    NULL_0,
    IDLE,
    AUTO,
    CALIBRATE,
    SHOW_IMAGE,
    REQUEST_CALIBRATION,
    WAIT_INPUT,
    UPDATE_IMAGE,
    DO_CALIBRATION,
    STORE_PARAMETERS
};

class MotionCapture {
public:
    
    MotionCapture(int id, int port, bool draw_blobs);
    virtual ~MotionCapture();
    
    // signal 
    typedef boost::signals2::signal<void( MotionCaptureState )> m_signal;
    
    // callback function from TCP_server
    void TCP_callback(relay_callback_msg msg);
	
    // Helper functions
    char * get_concat_string(char * buffer, const char * pream, int id);
    void debugMessage(const char * str);
    bool getKeepRunning(void); 
    MotionCaptureState getState(void);
    void setState(MotionCaptureState st);
        
    // just incase -- return points to important objects
    Blob * getBlobObject(void);
    TCP_server * getTCPServerObject(void);
    OpencvCamera * getCameraObject(void);
    
    // get signal
    m_signal * getMainThreadSignal(void){return &main_thread;}
private:
	TCP_server * tcp_server;
	OpencvCamera * camera;
	Blob * blob;
	
	// server parameters
	int server_id, server_port;
	
	// signal connections
	boost::signals2::connection conversion_complete;	// used to trigger blob data transmit
        // used to relay messages between motioncapture and tcp
	boost::signals2::connection motioncapture_connection;
        
        // signal to main thread
        m_signal main_thread;
	
	// Used to terminate main program
	bool keepRunning;
        
        // used to hold motioncapturestate
        MotionCaptureState system_state;
        
        // used to indicate to Blob object whether to draw
        bool drawBlobs;
};

#endif	/* MOTIONCAPTURE_H */

