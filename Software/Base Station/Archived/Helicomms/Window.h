/* 
 * File:   Window.h
 * Author: yashren
 *
 * Created on 11 March 2013, 9:37 AM
 */

#ifndef _WINDOW_H
#define	_WINDOW_H

#include "ui_Window.h"
#include "include/UDP_socket.h"
#include "include/getIP.h"
#include <QTimer>
#include <QComboBox>
#include "include/TCP_server.h"
#include "include/TCP_client.h"
#include "include/OpencvCamera.h"
#include "include/PoseEstimation.h"
#include <iostream>
#include <boost/signals2.hpp>

using namespace std;

#define SERVER_HOST_PORT 2010
#define GUI_UPDATE_MS 33
#define RQ_SERVER_MS 20
#define NUM_OF_CAMERAS 4
#define SCROLL_TIME_RANGE 10

class Window : public QDialog {
    Q_OBJECT
public:
    typedef boost::signals2::connection signal_connection;
    
    Window();
    void setupPlotting(void);
    virtual ~Window();
    
    void log(const QString& text);
    void send2allClients(TCP_client::client_msg msg);
    
    // TCP_client callback
    void TCP_Client_callback(TCP_client::callback_msg msg, int id);
    
    // Receives signal from poseEstimation to send data to helicopter
    void autoSend2Helicopter(void);
       
    static std::ofstream estimationLog;
    static void openLogFile(void);
    static void closeLogFile(void);
    static bool writeLogFile(void);
    static char write_buffer[512];      
    
public slots:
    void toggleComms(void);
    void toggleSend(void);
    void toggleReceive(void);
    void toggleMotionCapture(void);
    void updateData(void);
    void onRxRateChange(int index);
    void onTxRateChange( const QString & text);
    void onImageProcessChanged(int i);
    void onAutoChange(int i);
    void autoSample(int i);
    void syncSample(int i);
    void onGPSChange(int i);
    void processingDone(QImage image1);
    void dataRequestTimer(void);
    void startProcessing(void);
    void killServer(void);
    
    void onRLEDChanged(int i);
    void onGLEDChanged(int i);
    void onBLEDChanged(int i);    
    
private:
    Ui::Window widget;
    UDP_socket * helicopter_comms;
    bool UDP_connected;
    QTimer * updateGUITimer;
    QTimer * serverTXTimer;
    QTimer * imageProcessingTimer;
    getIP * ipadd;
    
    PoseEstimation * poseEstimation;

//    ImageProcessing * imageProcessingThread;
    
    timeval start, end;
    bool first_enter;
    double elapsedTime, totalTime;
    int iter;
    double FPS;
    
    bool red_led, green_led, blue_led;
    bool motionCaptureConnected;
    
    bool sendingViaImageProcessing;
    
    void initializeDataWindow(void);
    void initializeEstimationWindow(void);
    
    TCP_client * tcp_client[NUM_OF_CAMERAS];
    
    signal_connection tcp_callback_connection, data_received_connection[NUM_OF_CAMERAS];
    signal_connection pose_estimation_connection;
    
    const char * server_ip_addresses[NUM_OF_CAMERAS];
};

#endif	/* _WINDOW_H */
