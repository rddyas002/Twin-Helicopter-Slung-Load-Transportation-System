/*
 * File:   Window.cpp
 * Author: yashren
 *
 * Created on 11 March 2013, 9:37 AM
 */

#include "Window.h"

Window::Window() {
    int i;
    widget.setupUi(this);
    
    // Setup buttons and checkboxed
    connect(widget.commButton,SIGNAL(clicked()), this, SLOT(toggleComms()));
    connect(widget.sendButton,SIGNAL(clicked()), this, SLOT(toggleSend()));
    connect(widget.recButton,SIGNAL(clicked()), this, SLOT(toggleReceive()));
    connect(widget.rxRateChangecomboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onRxRateChange(int)));
    connect(widget.txRateChangecomboBox, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(onTxRateChange( const QString&)));
    connect(widget.AutoModecheckBox,SIGNAL(stateChanged(int)),this,SLOT(onAutoChange(int)));
    connect(widget.GPScheckBox,SIGNAL(stateChanged(int)),this,SLOT(onGPSChange(int)));
    connect(widget.imageProcessingCheckbox,SIGNAL(stateChanged(int)),this,SLOT(onImageProcessChanged(int)));
    connect(widget.motionCaptureConnectButton,SIGNAL(clicked()), this, SLOT(toggleMotionCapture()));
    connect(widget.AutoModeCheckBoxVisual,SIGNAL(stateChanged(int)), this, SLOT(autoSample(int)));
    connect(widget.killServerConnectButton,SIGNAL(clicked()), this, SLOT(killServer()));
    connect(widget.AutoModeCheckBoxVisual2,SIGNAL(stateChanged(int)), this, SLOT(syncSample(int)));
    // LED checkboxed
    connect(widget.RLED_checkbox,SIGNAL(stateChanged(int)),this,SLOT(onRLEDChanged(int)));
    connect(widget.GLED_checkbox,SIGNAL(stateChanged(int)),this,SLOT(onGLEDChanged(int)));
    connect(widget.BLED_checkbox,SIGNAL(stateChanged(int)),this,SLOT(onBLEDChanged(int)));
   
    // print default values into gui
    initializeDataWindow();
    initializeEstimationWindow();
    ipadd = new getIP();    // get local IP
    widget.PCIPEdit->setText(QString::fromAscii(ipadd->ip()));
    
    helicopter_comms = NULL;
    
    widget.AutoModeCheckBoxVisual->setEnabled(false);
    widget.AutoModeCheckBoxVisual2->setEnabled(false);
    
    for (i = 0; i < NUM_OF_CAMERAS; i++){
        tcp_client[i] = NULL;
    }    
    
    // setup timer for updating gui data
    updateGUITimer = new QTimer(this);
    connect(updateGUITimer, SIGNAL(timeout()), this, SLOT(updateData()));  
    updateGUITimer->start(GUI_UPDATE_MS);
    
    // temp timer used to request data from server
    serverTXTimer = new QTimer(this);
    connect(serverTXTimer, SIGNAL(timeout()), this, SLOT(dataRequestTimer()));
    
//    // allocate space for image processing thread but don't initialize
//    imageProcessingThread = new ImageProcessing(this);
//    // connect signal between completion of image processing thread and function processingDone
//    connect(imageProcessingThread, SIGNAL(processingResult(QImage)), 
//            this, SLOT(processingDone(QImage)));

    // timer used to initiate image processing
    imageProcessingTimer = new QTimer(this);
    connect(imageProcessingTimer, SIGNAL(timeout()), this, SLOT(startProcessing())); 
        
    // Initialize variables
    red_led = green_led = blue_led = false; // all leds initially off
    
    motionCaptureConnected = false;
    
    first_enter = true;         // used for timing purposes for imageprocessing thread
    elapsedTime = totalTime = 0;
    iter = 0;
    
    sendingViaImageProcessing = false;  // using to indicate whether to send data using image processing frequency as reference
    UDP_connected = false;      // UDP not connected 
    
    poseEstimation = new PoseEstimation();
    
    setupPlotting();
}

void Window::setupPlotting(void){    
    // setup graph1
     widget.qplotGraph1->setNotAntialiasedElements(QCP::aeAll);
     QFont font;
     font.setStyleStrategy(QFont::NoAntialias);
     widget.qplotGraph1->xAxis->setTickLabelFont(font);
     widget.qplotGraph1->yAxis->setTickLabelFont(font);
     widget.qplotGraph1->legend->setFont(font);    
     
     // set title of plot:
     widget.qplotGraph1->plotLayout()->insertRow(0);
     QCPPlotTitle * qcpTitle = new QCPPlotTitle(widget.qplotGraph1, "Gyroscope");
     qcpTitle->setFont(QFont("Helvetica", 14));
     widget.qplotGraph1->plotLayout()->addElement(0, 0, qcpTitle);
     widget.qplotGraph1->xAxis->setLabel("Time");
     widget.qplotGraph1->yAxis->setLabel("Rate (deg/s)");
         
     widget.qplotGraph1->addGraph(); // blue line
     widget.qplotGraph1->graph(0)->setPen(QPen(Qt::blue));
     widget.qplotGraph1->graph(0)->setBrush(Qt::NoBrush);//QBrush(QColor(240, 255, 200)));
     widget.qplotGraph1->graph(0)->setAntialiasedFill(false);   
     widget.qplotGraph1->graph(0)->setName("x-axis");
     
     widget.qplotGraph1->addGraph(); // green line
     widget.qplotGraph1->graph(1)->setPen(QPen(Qt::green));
     widget.qplotGraph1->graph(1)->setBrush(Qt::NoBrush);//QBrush(QColor(240, 255, 200)));
     widget.qplotGraph1->graph(1)->setAntialiasedFill(false);        
     widget.qplotGraph1->graph(1)->setName("y-axis");
     
     widget.qplotGraph1->addGraph(); // red line
     widget.qplotGraph1->graph(2)->setPen(QPen(Qt::red));
     widget.qplotGraph1->graph(2)->setBrush(Qt::NoBrush);//QBrush(QColor(240, 255, 200)));
     widget.qplotGraph1->graph(2)->setAntialiasedFill(false);             
     widget.qplotGraph1->graph(2)->setName("z-axis");
     
     widget.qplotGraph1->xAxis->setTickLabelType(QCPAxis::ltDateTime);
     widget.qplotGraph1->xAxis->setDateTimeFormat("ss");
     widget.qplotGraph1->xAxis->setAutoTickStep(false);
     widget.qplotGraph1->xAxis->setTickStep(2);
     widget.qplotGraph1->axisRect()->setupFullAxesBox();
     
     widget.qplotGraph1->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignLeft);
     widget.qplotGraph1->legend->setVisible(true);     
     
     
     // make left and bottom axes transfer their ranges to right and top axes:
     connect(widget.qplotGraph1->xAxis, SIGNAL(rangeChanged(QCPRange)), widget.qplotGraph1->xAxis2, SLOT(setRange(QCPRange)));
     connect(widget.qplotGraph1->yAxis, SIGNAL(rangeChanged(QCPRange)), widget.qplotGraph1->yAxis2, SLOT(setRange(QCPRange)));     
     

     widget.qplotGraph2->setNotAntialiasedElements(QCP::aeAll);
     font.setStyleStrategy(QFont::NoAntialias);
     widget.qplotGraph2->xAxis->setTickLabelFont(font);
     widget.qplotGraph2->yAxis->setTickLabelFont(font);
     widget.qplotGraph2->yAxis2->setTickLabelFont(font);  
     widget.qplotGraph2->xAxis2->setTickLabelFont(font);  
     widget.qplotGraph2->legend->setFont(font);    
     
     // set title of plot:
     widget.qplotGraph2->plotLayout()->insertRow(0);
     QCPPlotTitle * qcpTitle2 = new QCPPlotTitle(widget.qplotGraph2, "Ultrasonic/Voltage/Speed");
     qcpTitle2->setFont(QFont("Helvetica", 14));
     widget.qplotGraph2->plotLayout()->addElement(0, 0, qcpTitle2);
     widget.qplotGraph2->xAxis->setLabel("Time");
     widget.qplotGraph2->yAxis->setLabel("Distance (cm)");
         
     widget.qplotGraph2->addGraph(widget.qplotGraph2->xAxis, widget.qplotGraph2->yAxis2); // blue line
     widget.qplotGraph2->graph(0)->setPen(QPen(Qt::blue));
     widget.qplotGraph2->graph(0)->setBrush(Qt::NoBrush);
     widget.qplotGraph2->graph(0)->setAntialiasedFill(false);   
     widget.qplotGraph2->graph(0)->setName("Voltage");
     
     widget.qplotGraph2->addGraph(); // green line
     widget.qplotGraph2->graph(1)->setPen(QPen(Qt::green));
     widget.qplotGraph2->graph(1)->setBrush(Qt::NoBrush);
     widget.qplotGraph2->graph(1)->setAntialiasedFill(false);                 
     widget.qplotGraph2->graph(1)->setName("Ultrasonic");
     
     widget.qplotGraph2->addGraph(); // red line
     widget.qplotGraph2->graph(2)->setPen(QPen(Qt::red));
     widget.qplotGraph2->graph(2)->setBrush(Qt::NoBrush);
     widget.qplotGraph2->graph(2)->setAntialiasedFill(false);                 
     widget.qplotGraph2->graph(2)->setName("Speed");     
     
     widget.qplotGraph2->xAxis->setTickLabelType(QCPAxis::ltDateTime);
     widget.qplotGraph2->xAxis->setDateTimeFormat("ss");
     widget.qplotGraph2->xAxis->setAutoTickStep(false);
     widget.qplotGraph2->xAxis->setTickStep(2);
     widget.qplotGraph2->axisRect()->setupFullAxesBox();
     widget.qplotGraph2->yAxis->setRange(0,200);
     widget.qplotGraph2->yAxis2->setRange(0,20);
     
     widget.qplotGraph2->yAxis2->setTickLength(3, 3);
     widget.qplotGraph2->yAxis2->setSubTickLength(1, 1);

     widget.qplotGraph2->yAxis2->setLabel("Voltage (V)");     
     widget.qplotGraph2->yAxis2->setVisible(true);
     
     widget.qplotGraph2->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignLeft);
     widget.qplotGraph2->legend->setVisible(true);
     
     widget.qplotGraph2->yAxis2->setVisible(true);
     
//     // make left and bottom axes transfer their ranges to right and top axes:
//     connect(widget.qplotGraph2->xAxis, SIGNAL(rangeChanged(QCPRange)), widget.qplotGraph2->xAxis2, SLOT(setRange(QCPRange)));
//     connect(widget.qplotGraph2->yAxis, SIGNAL(rangeChanged(QCPRange)), widget.qplotGraph2->yAxis2, SLOT(setRange(QCPRange)));          
}

/*
 * GUI handlers
 *****************************************************************************
 *****************************************************************************
 *****************************************************************************
 */

void Window::toggleComms(){
    if (!UDP_connected)
    {
        helicopter_comms = new UDP_socket(widget.heliIPEdit->text(),widget.heliPortEdit->text());
        
        poseEstimation->setUDPLink(helicopter_comms);
        
        // enable transmit rate combo box
        widget.rxRateChangecomboBox->setEnabled(true);
        widget.txRateChangecomboBox->setEnabled(true);
        
        if(helicopter_comms->connect_socket()){
            log("Starting TCP communication.");
            widget.commButton->setText("Disconnect");
            UDP_connected = true;
            // enable send button
            widget.sendButton->setEnabled(true);
            widget.recButton->setEnabled(true);
        }    
        else
        {
            log("Error initializing TCP communication.");
            UDP_connected = false;
        }
    }
    else
    {
        helicopter_comms->disconnect_socket();
        log("Ending TCP communication.");
        delete helicopter_comms;
        widget.commButton->setText("Connect");
        UDP_connected = false;
        // disable send button
        widget.sendButton->setEnabled(false);
        widget.recButton->setEnabled(false);
        
       // disable transmit rate combo box
        widget.rxRateChangecomboBox->setEnabled(false);
        widget.txRateChangecomboBox->setEnabled(false);        
    }
}

void Window::toggleSend(void){
    if(!helicopter_comms->isTransmitting())
    {
        helicopter_comms->enableTransmitThread(true);
        widget.sendButton->setText("Sending...");
        //openLogFile();
        //pose_estimation_connection = poseEstimation->getDataReadySignal()->connect(boost::bind(&Window::autoSend2Helicopter,this));
    }
    else
    {
        helicopter_comms->enableTransmitThread(false);
        widget.sendButton->setText("Send");
        
        //pose_estimation_connection.disconnect();
        //closeLogFile();
    }
}

void Window::toggleReceive(void){
    if(!helicopter_comms->isReceving())
    {
        helicopter_comms->setupReceive(widget.rxPortEdit->text().toInt());
       // helicopter_comms->setupBroadcastReceive();
       // helicopter_comms->enableBroadcastReceiveThread(true);
        widget.recButton->setText("Receiving...");
//        updateGUITimer->start(GUI_UPDATE_MS);
    }
    else
    {
        helicopter_comms->enableReceiveThread(false);
       // helicopter_comms->enableBroadcastReceiveThread(false);
        widget.recButton->setText("Receive");
//        updateGUITimer->stop();
        // initialize screen
        initializeDataWindow();
    }
}

void Window::onRxRateChange(int index){
    char tx_rate_char;
    switch(index)
    {
        case 0:
            tx_rate_char = '0';
            break;        
        case 1:
            tx_rate_char = '7';
            break;
        case 2:
            tx_rate_char = '5';
            break;
        case 3:
            tx_rate_char = '3';
            break;
        case 4:
            tx_rate_char = '2';
            break;  
        case 5:
            tx_rate_char = '1';
            break;            
        default:
            tx_rate_char = '0';
            break;
    }
    helicopter_comms->setHelicopterTxRate(tx_rate_char);
}

void Window::onTxRateChange( const QString & text){
    helicopter_comms->setTransmitRate(text.toFloat());
}

void Window::onAutoChange(int i){
    if (i)
    {
        helicopter_comms->setAutoMode('A');
    }
    else
    {
        helicopter_comms->setAutoMode('N');
    }
}

void Window::onGPSChange(int i){
    // don't setup gps just yet
    if (i)
    {
        //helicopter_comms->setGPSMode('A');
    }
    else
    {
        //helicopter_comms->setGPSMode('N');
    }
}

void Window::onImageProcessChanged(int i){
//    if (i){
//        imageProcessingThread->initializeImageProcessing();
//        //imageProcessingThread->setPriority(QThread::TimeCriticalPriority);
//        imageProcessingTimer->start(1);
////        if(helicopter_comms->isTransmitting()){
////            // If system is sending data to helicopter...stop that thread and only send
////            // when image processing step is done
////            log("Stopping transmit thread.");     
////            helicopter_comms->enableTransmitThread(false);
////            widget.sendButton->setEnabled(false);
////            sendingViaImageProcessing = true;
////        }
//        
//    }else{
////        if (sendingViaImageProcessing)
////        {
////            helicopter_comms->enableTransmitThread(true);
////            widget.sendButton->setEnabled(true);
////            sendingViaImageProcessing = false;
////        }
//        
//        imageProcessingTimer->stop();
//        QImage rect = QImage(320,240,QImage::Format_RGB32);
//        rect.fill(Qt::black);    
//        widget.frame_label1->setPixmap(QPixmap::fromImage(rect).scaled(widget.frame_label1->size(), Qt::KeepAspectRatio, Qt::FastTransformation));        
//        initializeEstimationWindow();
//        
//        // put leds off
//        widget.RLED_checkbox->setChecked(false);
//        widget.GLED_checkbox->setChecked(false);
//        widget.BLED_checkbox->setChecked(false);
////        helicopter_comms->setLEDS(false,false,false);
//        imageProcessingThread->~ImageProcessing();
//    }
}

// LEDs
void Window::onRLEDChanged(int i){
    if (i)
        red_led = true;
    else
        red_led = false;
    
    helicopter_comms->setLEDS(red_led,green_led,blue_led);
    if (sendingViaImageProcessing)
    {
        helicopter_comms->sendPacket();
    }    
}

void Window::onGLEDChanged(int i){
    if (i)
        green_led = true;
    else
        green_led = false;    
    
    helicopter_comms->setLEDS(red_led,green_led,blue_led);
    if (sendingViaImageProcessing)
    {
        helicopter_comms->sendPacket();
    }       
}

void Window::onBLEDChanged(int i){
    if (i)
        blue_led = true;
    else
        blue_led = false;    
    
    helicopter_comms->setLEDS(red_led,green_led,blue_led);
    if (sendingViaImageProcessing)
    {
        helicopter_comms->sendPacket();
    }       
}

void Window::toggleMotionCapture(void){
    int i;
    // extract char ip addresses
    for (i = 0; i < NUM_OF_CAMERAS; i++){
        server_ip_addresses[i] = (char*) malloc(20);
    }
    
    QByteArray ip_add_lat1 = widget.ips1Edit->text().toLatin1();
    QByteArray ip_add_lat2 = widget.ips2Edit->text().toLatin1();
    QByteArray ip_add_lat3 = widget.ips3Edit->text().toLatin1();
    QByteArray ip_add_lat4 = widget.ips4Edit->text().toLatin1();
    
    server_ip_addresses[0] = ip_add_lat1.data();    
    server_ip_addresses[1] = ip_add_lat2.data();    
    server_ip_addresses[2] = ip_add_lat3.data();    
    server_ip_addresses[3] = ip_add_lat4.data();    
    
    if(!motionCaptureConnected)
    {
        for (i = 0; i < NUM_OF_CAMERAS; i++){
            if (tcp_client[i] == NULL){
                tcp_client[i] = new TCP_client(i+1,SERVER_HOST_PORT,server_ip_addresses[i], poseEstimation->getVirtualCamera(i));
                if (!(tcp_client[i]->tryConnect()))
                {
                    delete tcp_client[i];
                    tcp_client[i] = NULL;
                }
                else
                {
                    tcp_callback_connection = tcp_client[i]->getCallbackSignal()->connect(boost::bind(&Window::TCP_Client_callback,this,_1,_2));
                    widget.AutoModeCheckBoxVisual->setEnabled(true);
                    widget.AutoModeCheckBoxVisual2->setEnabled(true);
                    widget.killServerConnectButton->setEnabled(true);
                    data_received_connection[i] = tcp_client[i]->getDataReceivedSignal()->connect(boost::bind(&PoseEstimation::setCamDataReceived,poseEstimation,_1));
                    widget.motionCaptureConnectButton->setText("Disconnect");
                    
                    motionCaptureConnected = true;
                }
            }
        }
        
        helicopter_comms->enableTransmitThread(false);
    }
    else
    {
        for (i = 0; i < NUM_OF_CAMERAS; i++){
            if (tcp_client[i] != NULL){
                data_received_connection[i].disconnect();
                
                widget.killServerConnectButton->setEnabled(false);
                widget.AutoModeCheckBoxVisual->setEnabled(false);
                widget.AutoModeCheckBoxVisual2->setEnabled(false);
                widget.motionCaptureConnectButton->setText("Connect");
                motionCaptureConnected = false;            
                delete tcp_client[i];
                tcp_client[i] = NULL;
            }
        }
        
        helicopter_comms->enableTransmitThread(true);
    }
}

void Window::autoSample(int i){
    if (i){
        widget.AutoModeCheckBoxVisual2->setEnabled(false);
        send2allClients(TCP_client::START_AUTO);
    }
    else
    {
        widget.AutoModeCheckBoxVisual2->setEnabled(true);
        send2allClients(TCP_client::STOP_AUTO);
    }
}

void Window::syncSample(int i){
    if (i){
        widget.AutoModeCheckBoxVisual->setEnabled(false);
        serverTXTimer->start(RQ_SERVER_MS);
    }
    else
    {
        widget.AutoModeCheckBoxVisual->setEnabled(true);
        serverTXTimer->stop();
    }
}

void Window::killServer(void){
    send2allClients(TCP_client::KILL_PROGRAM);
    if (!(QString::compare("Disconnect", widget.motionCaptureConnectButton->text(), Qt::CaseInsensitive)))
        toggleMotionCapture();
    
    widget.AutoModeCheckBoxVisual->setChecked(false);
    widget.AutoModeCheckBoxVisual->setEnabled(false);    
    widget.motionCaptureConnectButton->setEnabled(false);
    widget.killServerConnectButton->setEnabled(false);
}

/*
 * QSlots
 *****************************************************************************
 *****************************************************************************
 *****************************************************************************
 */

// Update GUI with data
void Window::updateData(void){
    if (helicopter_comms != NULL){
        if(helicopter_comms->isReceving()){
            signed short int * pnt = helicopter_comms->getGyroData();
    
            widget.gyroTempEdit->setText(QString::number(*pnt++));
            widget.gyroxEdit->setText(QString::number(*pnt++));
            widget.gyroyEdit->setText(QString::number(*pnt++));
            widget.gyrozEdit->setText(QString::number(*pnt++));
    
            pnt = helicopter_comms->getAccelData();
            widget.accelxEdit->setText(QString::number(*pnt++));
            widget.accelyEdit->setText(QString::number(*pnt++));
            widget.accelzEdit->setText(QString::number(*pnt++));   
            
            pnt = helicopter_comms->getMagnetoData();
            widget.magxEdit->setText(QString::number(*pnt++));
            widget.magyEdit->setText(QString::number(*pnt++));
            widget.magzEdit->setText(QString::number(*pnt++));       
            
            float * fpnt = helicopter_comms->getTempBaroData();
            widget.baroTempEdit->setText(QString::number(*fpnt));
            
            unsigned short int * upnt = helicopter_comms->getUltrasonicData();
            widget.ultraDistEdit->setText(QString::number(*upnt));
            
            upnt = helicopter_comms->getVoltageData();
            float fvoltage = ((float)(*upnt))/47.288136 + 0.33;
            widget.voltageEdit->setText(QString::number(fvoltage));
            if (fvoltage > 11.6)
                widget.voltageEdit->setStyleSheet("background: solid green");
            else if ((fvoltage > 10.6) && (fvoltage < 11.6))
                widget.voltageEdit->setStyleSheet("background: solid yellow");
            else
                widget.voltageEdit->setStyleSheet("background: solid red");
    
            upnt = helicopter_comms->getSpeedData();
            widget.speedEdit->setText(QString::number(*upnt));
    
            unsigned long int * lpnt = helicopter_comms->getPressureData();
            widget.pressureEdit->setText(QString::number(*lpnt));
            
            widget.escEdit->setText(QString::number(helicopter_comms->getESC()));
            widget.leftEdit->setText(QString::number(helicopter_comms->getServoLeft()));
            widget.rightEdit->setText(QString::number(helicopter_comms->getServoRight()));
            widget.rearEdit->setText(QString::number(helicopter_comms->getServoRear()));
            widget.gainEdit->setText(QString::number(helicopter_comms->getGain()));
            widget.rudderEdit->setText(QString::number(helicopter_comms->getRudder()));
            widget.runtimeEdit->setText(QString::number(helicopter_comms->getLatency(),'f',3));
            
            widget.collectiveEdit->setText(QString::number(helicopter_comms->getCollective()));
            widget.lateralEdit->setText(QString::number(helicopter_comms->getLateral()));
            widget.longitudinalEdit->setText(QString::number(helicopter_comms->getLongitudinal()));
            
             double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
             // add data to lines:
             widget.qplotGraph1->graph(0)->addData(key, widget.gyroxEdit->text().toFloat()/14.375 - helicopter_comms->get_xgyrobias());
             // remove data of lines that's outside visible range:
             widget.qplotGraph1->graph(0)->removeDataBefore(key-SCROLL_TIME_RANGE);
             
             widget.qplotGraph1->graph(1)->addData(key, widget.gyroyEdit->text().toFloat()/14.375 - helicopter_comms->get_ygyrobias());
             widget.qplotGraph1->graph(1)->removeDataBefore(key-SCROLL_TIME_RANGE);
             
             widget.qplotGraph1->graph(2)->addData(key, widget.gyrozEdit->text().toFloat()/14.375 - helicopter_comms->get_zgyrobias());
             widget.qplotGraph1->graph(2)->removeDataBefore(key-SCROLL_TIME_RANGE); 
             
             widget.qplotGraph1->rescaleAxes();
             
             // make key axis range scroll with the data (at a constant range size of 10):
             widget.qplotGraph1->xAxis->setRange(key+0.25, SCROLL_TIME_RANGE, Qt::AlignRight);
             widget.qplotGraph1->replot();              
             
             // add data to lines:
             widget.qplotGraph2->graph(0)->addData(key, fvoltage);
             widget.qplotGraph2->graph(0)->removeDataBefore(key-SCROLL_TIME_RANGE);
             widget.qplotGraph2->graph(1)->addData(key, widget.ultraDistEdit->text().toFloat());
             widget.qplotGraph2->graph(1)->removeDataBefore(key-SCROLL_TIME_RANGE);
             widget.qplotGraph2->graph(2)->addData(key, widget.speedEdit->text().toFloat()/60);
             widget.qplotGraph2->graph(2)->removeDataBefore(key-SCROLL_TIME_RANGE);             
             
             // make key axis range scroll with the data (at a constant range size of 10):
             widget.qplotGraph2->xAxis->setRange(key+0.25, SCROLL_TIME_RANGE, Qt::AlignRight);
             widget.qplotGraph2->replot();            
        }
        else
        {
            initializeDataWindow();
        }   
    }
    
    if (motionCaptureConnected){
        // update pose table
        widget.txEdit->setText(QString::number(helicopter_comms->get_tx(),'f',0));
        widget.tyEdit->setText(QString::number(helicopter_comms->get_ty(),'f',0));
        widget.tzEdit->setText(QString::number(helicopter_comms->get_tz(),'f',0));
        widget.vxEdit->setText(QString::number(helicopter_comms->get_vx(),'f',0));
        widget.vyEdit->setText(QString::number(helicopter_comms->get_vy(),'f',0));
        widget.vzEdit->setText(QString::number(helicopter_comms->get_vz(),'f',0));        
        widget.rollEdit->setText(QString::number(helicopter_comms->get_roll(),'f',1));
        widget.pitchEdit->setText(QString::number(helicopter_comms->get_pitch(),'f',1));
        widget.yawEdit->setText(QString::number(helicopter_comms->get_yaw(),'f',1));
        widget.activeCamEdit->setText(QString::number(poseEstimation->getActiveBlobs()));
        
        widget.fps_label->setText("FPS: " + QString::number(poseEstimation->getFPS(),'f',2));
    }
    else
    {
        initializeEstimationWindow();
    }
   
}

void Window::processingDone(QImage img1)
{
    if (imageProcessingTimer->isActive())
    {
//        // Send data to helicopter from here (if there is new information)
//        helicopter_comms->setVisualData(tx,ty,tz,q0,q1,q2,q3,cam_q);
//
//        if (sendingViaImageProcessing)
//        {
//            helicopter_comms->sendPacket();
//        }
        
        if (!img1.isNull())
        {
            widget.frame_label1->setAlignment(Qt::AlignCenter);
            widget.frame_label1->setPixmap(QPixmap::fromImage(img1).scaled(widget.frame_label1->size(), Qt::KeepAspectRatio, Qt::FastTransformation));        
        }
    
//        widget.rollEdit->setText(QString::number(roll*180/M_PI,'g',3));
//        widget.pitchEdit->setText(QString::number(pitch*180/M_PI,'g',3));
//        widget.yawEdit->setText(QString::number(yaw*180/M_PI,'g',3));
//        widget.txEdit->setText(QString::number(tx,'g',5));
//        widget.tyEdit->setText(QString::number(ty,'g',5));
//        widget.tzEdit->setText(QString::number(tz,'g',5));
    }
    else
    {
        QImage rect = QImage(320,240,QImage::Format_RGB32);
        rect.fill(Qt::black);    
        widget.frame_label1->setPixmap(QPixmap::fromImage(rect).scaled(widget.frame_label1->size(), Qt::KeepAspectRatio, Qt::FastTransformation));              
    }
}

// receives signal from PoseEstimation
void Window::autoSend2Helicopter(void){   
    float x = (float) poseEstimation->getTrans_x();
    float y = (float) poseEstimation->getTrans_y();
    float z = (float) poseEstimation->getTrans_z();
    
    quaternion hq = poseEstimation->getQuaternion();
    
    // log data here
    sprintf(write_buffer, "%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,%.5f,%.2f,%.2f,%.2f,%d,%.3f\r\n",
            x,y,z,hq.q0,hq.q1,hq.q2,hq.q3,
            poseEstimation->getRoll(),
            poseEstimation->getPitch(),
            poseEstimation->getYaw(),
            poseEstimation->getNumOfCamerasUsed(),
            poseEstimation->getCalcDoneTime());
    writeLogFile();
    
    // update data in helicopter packet
    helicopter_comms->setVisualData(x,y,z,hq.q0,hq.q1,hq.q2,hq.q3,100);
    
    helicopter_comms->sendPacket();
}

ofstream Window::estimationLog;
char Window::write_buffer[512] = {0};

void Window::openLogFile(void){
    estimationLog.open("estimationLog.txt");
}

void Window::closeLogFile(void){
    if (estimationLog.is_open())
        estimationLog.close();
}

bool Window::writeLogFile(void){
    if (estimationLog.is_open()){
        estimationLog << write_buffer;
        return true;
    }
    else
        return false;
}


void Window::dataRequestTimer(void){
    if (poseEstimation->getAllDataReceived()){
        poseEstimation->setAlldataReceived(false);
        poseEstimation->setRequestTime();
        send2allClients(TCP_client::GET_DATA);
    }
}

void Window::startProcessing(void){
//    if(!imageProcessingThread->isRunning()){
//        if (first_enter){
//            gettimeofday(&start, NULL);        
//            first_enter = false;
//        }
//        else
//        {
//            gettimeofday(&end, NULL);    
//            
//            elapsedTime = (end.tv_sec - start.tv_sec) * 1000.0;      // sec to ms
//            elapsedTime += (end.tv_usec - start.tv_usec) / 1000.0;   // us to ms            
//            totalTime += elapsedTime;
//            
//            if (++iter == 20){
//                FPS = (double)iter*1000/totalTime;
//                iter = totalTime = 0;
//            }
//            
//            first_enter = true;
//        }
//        
//       imageProcessingThread->start();
//    }
}

/*
 * Helper Functions
 *****************************************************************************
 *****************************************************************************
 *****************************************************************************
 */

void Window::initializeDataWindow(void){
    widget.accelxEdit->setText("0.0");
    widget.accelyEdit->setText("0.0");
    widget.accelzEdit->setText("0.0");
    widget.gyroxEdit->setText("0.0");
    widget.gyroyEdit->setText("0.0");
    widget.gyrozEdit->setText("0.0");
    widget.magxEdit->setText("0.0");
    widget.magyEdit->setText("0.0");
    widget.magzEdit->setText("0.0");
    
    widget.baroTempEdit->setText("0.0");
    widget.gyroTempEdit->setText("0.0");
    widget.pressureEdit->setText("0.0");
    widget.voltageEdit->setText("0.0");
    widget.speedEdit->setText("0.0");
    widget.ultraDistEdit->setText("0.0");
    widget.rssiEdit->setText("N/A");
    
    widget.escEdit->setText("0");
    widget.leftEdit->setText("0.0");
    widget.rightEdit->setText("0.0");
    widget.rearEdit->setText("0.0");
    widget.gainEdit->setText("0.0");
    widget.rudderEdit->setText("0.0");

    widget.collectiveEdit->setText("0.0");
    widget.lateralEdit->setText("0.0");
    widget.longitudinalEdit->setText("0.0");    
}

void Window::initializeEstimationWindow(void){
    widget.rollEdit->setText("0.0");
    widget.pitchEdit->setText("0.0");
    widget.yawEdit->setText("0.0");
    widget.txEdit->setText("0.0");
    widget.tyEdit->setText("0.0");
    widget.tzEdit->setText("0.0");
    widget.activeCamEdit->setText("0");
}

void Window::log(const QString& text)
{
    widget.logEdit->setText(widget.logEdit->toPlainText() + text.trimmed() + "\r\n");
    widget.logEdit->verticalScrollBar();
}

void Window::send2allClients(TCP_client::client_msg msg){
    for (int i = 0; i < NUM_OF_CAMERAS; i++){
        if (tcp_client[i] != NULL){
            tcp_client[i]->sendMsg(msg);
        }else
            std::cout << "Client " << i+1 << "NULL" << std::endl;
    }
}

/*
 * Callback functions
 *****************************************************************************
 *****************************************************************************
 *****************************************************************************
 */
// from TCP_client
void Window::TCP_Client_callback(TCP_client::callback_msg msg, int id){
    switch (msg){
        case TCP_client::TERMINATE_CLIENT:
            if (tcp_client[id] != NULL){
                delete tcp_client[id];
                tcp_client[id] = NULL;
            }            
            break;
        default:
            std::cout << "Unknown msg received in TCP_Client_callback" << std::endl;
            break;
    }
}

Window::~Window() {

    if (poseEstimation != NULL)
        delete poseEstimation;
    
    closeLogFile();
    
//    data_received_connection.disconnect();
    
    for (int i = 0; i < NUM_OF_CAMERAS; i++){
        if (tcp_client[i] != NULL){
            delete tcp_client[i];
            tcp_client[i] = NULL;
        }
    }
}

