/* 
 * File:   BasicWindow.h
 * Author: yashren
 *
 * Created on 20 May 2014, 8:53 AM
 */

#ifndef _BASICWINDOW_H
#define	_BASICWINDOW_H

#define NUM_OF_CAMERAS 4
#define SERVER_HOST_PORT 2010
#define GUI_UPDATE_MS 33
#define DATA_REQUEST_TIME 30
#define SCROLL_TIME_RANGE 20

#define GUI_VELOCITY_UPPER  (1500)
#define GUI_VELOCITY_LOWER  (-1500)
#define GUI_VOLTAGE_UPPER  (13)
#define GUI_VOLTAGE_LOWER  (0)
#define GUI_SPEED_UPPER  (4000)
#define GUI_SPEED_LOWER  (0)
#define GUI_PROTECTION_VOLTAGE (9.3)

#include "ui_BasicWindow.h"
#include "Helicopter.h"
#include "PoseEstimation.h"
#include "TCP_client.h"
#include <QTimer>
#include <QComboBox>

class BasicWindow : public QMainWindow {
    Q_OBJECT
public:
    typedef boost::signals2::connection signal_connection;
    
    BasicWindow();
    void errorMessage(const char * message);
    void connectClients(void);
    void disconnectClients(void);
    void TCP_Client_callback(TCP_client::callback_msg msg, int id);
    void setupPlotting(void);
    virtual ~BasicWindow();
public slots:
    void btnConnectHelicopter1Clicked(void); 
    void btnConnectHelicopter2Clicked(void);
    void btnGetData(void);
    void updateGUI(void);
private:
    Ui::BasicWindow widget;
    
    bool helicopter1connected;
    Helicopter * helicopter1;
    
    bool helicopter2connected;
    Helicopter * helicopter2;    
    
    PoseEstimation * poseEstimation;
    
    TCP_client * tcp_client[NUM_OF_CAMERAS];
    
    signal_connection tcp_callback_connection[NUM_OF_CAMERAS], data_received_connection[NUM_OF_CAMERAS];
    
    // Timers
    QTimer * updateGUITimer;
    QTimer * requestDataTimer;
    QTime runTime;
};

#endif	/* _BASICWINDOW_H */
