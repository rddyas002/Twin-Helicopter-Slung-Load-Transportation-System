
#include "BasicWindow.h"
#include "qcustomplot.h"

BasicWindow::BasicWindow() {
    widget.setupUi(this);
    
    helicopter1 = NULL;
    helicopter1connected = false;
    helicopter2 = NULL;
    helicopter2connected = false;    
    
    poseEstimation = NULL;
    
    for (int i = 0; i < NUM_OF_CAMERAS; i++){
        tcp_client[i] = NULL;
    }        
    
    // Push buttons
    connect(widget.btnConnectHelicopter1,SIGNAL(clicked()), this, SLOT(btnConnectHelicopter1Clicked()));
    connect(widget.btnConnectHelicopter2,SIGNAL(clicked()), this, SLOT(btnConnectHelicopter2Clicked()));
      
    // setup timer for updating gui data
    updateGUITimer = new QTimer(this);
    connect(updateGUITimer, SIGNAL(timeout()), this, SLOT(updateGUI()));  
    updateGUITimer->start(GUI_UPDATE_MS);
    
    widget.lcdNumber->display("00:00");
    
    setupPlotting();
}

void BasicWindow::setupPlotting(void){
    // setup graph1
     widget.qplotGraph1->setNotAntialiasedElements(QCP::aeAll);
     QFont font;
     font.setStyleStrategy(QFont::NoAntialias);
     widget.qplotGraph1->xAxis->setTickLabelFont(font);
     widget.qplotGraph1->yAxis->setTickLabelFont(font);
     widget.qplotGraph1->legend->setFont(font);    
     
     // set title of plot:
     widget.qplotGraph1->plotLayout()->insertRow(0);
     QCPPlotTitle * qcpTitle = new QCPPlotTitle(widget.qplotGraph1, " ");
     qcpTitle->setFont(QFont("Helvetica", 11));
     widget.qplotGraph1->plotLayout()->addElement(0, 0, qcpTitle);
     widget.qplotGraph1->yAxis->setLabel("Euler angles (degrees)");
         
     widget.qplotGraph1->addGraph(); // blue line
     widget.qplotGraph1->graph(0)->setPen(QPen(Qt::blue));
     widget.qplotGraph1->graph(0)->setBrush(Qt::NoBrush);
     widget.qplotGraph1->graph(0)->setAntialiasedFill(false);   
     widget.qplotGraph1->graph(0)->setName("Roll");
     
     widget.qplotGraph1->addGraph(); // green line
     widget.qplotGraph1->graph(1)->setPen(QPen(Qt::green));
     widget.qplotGraph1->graph(1)->setBrush(Qt::NoBrush);
     widget.qplotGraph1->graph(1)->setAntialiasedFill(false);        
     widget.qplotGraph1->graph(1)->setName("Pitch");
     
     widget.qplotGraph1->addGraph(); // red line
     widget.qplotGraph1->graph(2)->setPen(QPen(Qt::red));
     widget.qplotGraph1->graph(2)->setBrush(Qt::NoBrush);
     widget.qplotGraph1->graph(2)->setAntialiasedFill(false);             
     widget.qplotGraph1->graph(2)->setName("Yaw");
     
     widget.qplotGraph1->addGraph(); // blue line
     widget.qplotGraph1->graph(3)->setPen(QPen(Qt::blue, 1.0, Qt::DotLine));
     widget.qplotGraph1->graph(3)->setBrush(Qt::NoBrush);
     widget.qplotGraph1->graph(3)->setAntialiasedFill(false);   
     widget.qplotGraph1->graph(3)->setName("Onboard Roll");
     
     widget.qplotGraph1->addGraph(); // green line
     widget.qplotGraph1->graph(4)->setPen(QPen(Qt::green, 1.0, Qt::DotLine));
     widget.qplotGraph1->graph(4)->setBrush(Qt::NoBrush);
     widget.qplotGraph1->graph(4)->setAntialiasedFill(false);        
     widget.qplotGraph1->graph(4)->setName("Onboard Pitch");
     
     widget.qplotGraph1->addGraph(); // red line
     widget.qplotGraph1->graph(5)->setPen(QPen(Qt::red, 1.0, Qt::DotLine));
     widget.qplotGraph1->graph(5)->setBrush(Qt::NoBrush);
     widget.qplotGraph1->graph(5)->setAntialiasedFill(false);             
     widget.qplotGraph1->graph(5)->setName("Onboard Yaw");     
     
     widget.qplotGraph1->xAxis->setTickLabelType(QCPAxis::ltDateTime);
     widget.qplotGraph1->xAxis->setDateTimeFormat("ss");
     widget.qplotGraph1->xAxis->setAutoTickStep(false);
     widget.qplotGraph1->xAxis->setTickStep(2);
     widget.qplotGraph1->axisRect()->setupFullAxesBox();
     
     widget.qplotGraph1->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignLeft);
     widget.qplotGraph1->legend->setVisible(false); 
     
     //////////////////////////////////////////////
     widget.qplotGraph2->setNotAntialiasedElements(QCP::aeAll);
     font.setStyleStrategy(QFont::NoAntialias);
     widget.qplotGraph2->xAxis->setTickLabelFont(font);
     widget.qplotGraph2->yAxis->setTickLabelFont(font);
     widget.qplotGraph2->legend->setFont(font);    
     
     // set title of plot:
     widget.qplotGraph2->plotLayout()->insertRow(0);
     qcpTitle = new QCPPlotTitle(widget.qplotGraph2, " ");
     qcpTitle->setFont(QFont("Helvetica", 11));     
     widget.qplotGraph2->plotLayout()->addElement(0, 0, qcpTitle);
     widget.qplotGraph2->yAxis->setLabel("Translation (mm)");
         
     widget.qplotGraph2->addGraph(); // blue line
     widget.qplotGraph2->graph(0)->setPen(QPen(Qt::blue));
     widget.qplotGraph2->graph(0)->setBrush(Qt::NoBrush);
     widget.qplotGraph2->graph(0)->setAntialiasedFill(false);   
     widget.qplotGraph2->graph(0)->setName("x");
     
     widget.qplotGraph2->addGraph(); // green line
     widget.qplotGraph2->graph(1)->setPen(QPen(Qt::green));
     widget.qplotGraph2->graph(1)->setBrush(Qt::NoBrush);
     widget.qplotGraph2->graph(1)->setAntialiasedFill(false);                 
     widget.qplotGraph2->graph(1)->setName("y");
     
     widget.qplotGraph2->addGraph(); // red line
     widget.qplotGraph2->graph(2)->setPen(QPen(Qt::red));
     widget.qplotGraph2->graph(2)->setBrush(Qt::NoBrush);
     widget.qplotGraph2->graph(2)->setAntialiasedFill(false);                 
     widget.qplotGraph2->graph(2)->setName("z");

     widget.qplotGraph2->addGraph(); // blue line
     widget.qplotGraph2->graph(3)->setPen(QPen(Qt::blue, 1.0, Qt::DotLine));
     widget.qplotGraph2->graph(3)->setBrush(Qt::NoBrush);
     widget.qplotGraph2->graph(3)->setAntialiasedFill(false);   
     widget.qplotGraph2->graph(3)->setName("Onboard tx");
     
     widget.qplotGraph2->addGraph(); // green line
     widget.qplotGraph2->graph(4)->setPen(QPen(Qt::green, 1.0, Qt::DotLine));
     widget.qplotGraph2->graph(4)->setBrush(Qt::NoBrush);
     widget.qplotGraph2->graph(4)->setAntialiasedFill(false);        
     widget.qplotGraph2->graph(4)->setName("Onboard ty");
     
     widget.qplotGraph2->addGraph(); // red line
     widget.qplotGraph2->graph(5)->setPen(QPen(Qt::red, 1.0, Qt::DotLine));
     widget.qplotGraph2->graph(5)->setBrush(Qt::NoBrush);
     widget.qplotGraph2->graph(5)->setAntialiasedFill(false);             
     widget.qplotGraph2->graph(5)->setName("Onboard tz");     
     
     widget.qplotGraph2->xAxis->setTickLabelType(QCPAxis::ltDateTime);
     widget.qplotGraph2->xAxis->setDateTimeFormat("ss");
     widget.qplotGraph2->xAxis->setAutoTickStep(false);
     widget.qplotGraph2->xAxis->setTickStep(2);
     widget.qplotGraph2->axisRect()->setupFullAxesBox();
     widget.qplotGraph2->yAxis->setRange(-2000,2000);
          
     widget.qplotGraph2->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignLeft);
     widget.qplotGraph2->legend->setVisible(false);
     
     widget.qplotGraph2->yAxis2->setVisible(true);
     //////////////////////////////////////////////
     widget.qplotGraph3->setNotAntialiasedElements(QCP::aeAll);
     font.setStyleStrategy(QFont::NoAntialias);
     widget.qplotGraph3->xAxis->setTickLabelFont(font);
     widget.qplotGraph3->yAxis->setTickLabelFont(font);
     widget.qplotGraph3->legend->setFont(font);      
     // set title of plot:
     widget.qplotGraph3->plotLayout()->insertRow(0);
     qcpTitle = new QCPPlotTitle(widget.qplotGraph3, " ");
     qcpTitle->setFont(QFont("Helvetica", 11));     
     widget.qplotGraph3->plotLayout()->addElement(0, 0, qcpTitle);
     widget.qplotGraph3->xAxis->setLabel("Time");
     widget.qplotGraph3->yAxis->setLabel("Velocity (mm/s)");
         
     widget.qplotGraph3->addGraph(); // blue line
     widget.qplotGraph3->graph(0)->setPen(QPen(Qt::blue));
     widget.qplotGraph3->graph(0)->setBrush(Qt::NoBrush);
     widget.qplotGraph3->graph(0)->setAntialiasedFill(false);   
     widget.qplotGraph3->graph(0)->setName("Vx");
     
     widget.qplotGraph3->addGraph(); // green line
     widget.qplotGraph3->graph(1)->setPen(QPen(Qt::green));
     widget.qplotGraph3->graph(1)->setBrush(Qt::NoBrush);
     widget.qplotGraph3->graph(1)->setAntialiasedFill(false);                 
     widget.qplotGraph3->graph(1)->setName("Vy");
     
     widget.qplotGraph3->addGraph(); // red line
     widget.qplotGraph3->graph(2)->setPen(QPen(Qt::red));
     widget.qplotGraph3->graph(2)->setBrush(Qt::NoBrush);
     widget.qplotGraph3->graph(2)->setAntialiasedFill(false);                 
     widget.qplotGraph3->graph(2)->setName("Vz");     
     
     widget.qplotGraph3->xAxis->setTickLabelType(QCPAxis::ltDateTime);
     widget.qplotGraph3->xAxis->setDateTimeFormat("ss");
     widget.qplotGraph3->xAxis->setAutoTickStep(false);
     widget.qplotGraph3->xAxis->setTickStep(2);
     widget.qplotGraph3->axisRect()->setupFullAxesBox();
     widget.qplotGraph3->yAxis->setRange(GUI_VELOCITY_LOWER,GUI_VELOCITY_UPPER);
     widget.qplotGraph3->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignLeft);
     widget.qplotGraph3->legend->setVisible(true);
     widget.qplotGraph3->yAxis2->setVisible(true);   
     
     //////////////////////////////////////////////
     widget.qplotGraph4->setNotAntialiasedElements(QCP::aeAll);
     font.setStyleStrategy(QFont::NoAntialias);
     widget.qplotGraph4->xAxis->setTickLabelFont(font);
     widget.qplotGraph4->yAxis->setTickLabelFont(font);
     widget.qplotGraph4->yAxis2->setTickLabelFont(font);
     widget.qplotGraph4->legend->setFont(font);      
     // set title of plot:
     widget.qplotGraph4->plotLayout()->insertRow(0);
     qcpTitle = new QCPPlotTitle(widget.qplotGraph4, " ");
     qcpTitle->setFont(QFont("Helvetica", 11));     
     widget.qplotGraph4->plotLayout()->addElement(0, 0, qcpTitle);
     widget.qplotGraph4->xAxis->setLabel("Time");
     widget.qplotGraph4->yAxis->setLabel("Voltage (volts)");
         
     widget.qplotGraph4->addGraph(); // blue line
     widget.qplotGraph4->graph(0)->setPen(QPen(Qt::blue));
     widget.qplotGraph4->graph(0)->setBrush(Qt::NoBrush);
     widget.qplotGraph4->graph(0)->setAntialiasedFill(false);   
     widget.qplotGraph4->graph(0)->setName("Batt_voltage");
     
     widget.qplotGraph4->addGraph(); // green line
     widget.qplotGraph4->graph(1)->setPen(QPen(Qt::green));
     widget.qplotGraph4->graph(1)->setBrush(Qt::NoBrush);
     widget.qplotGraph4->graph(1)->setAntialiasedFill(false);                 
     widget.qplotGraph4->graph(1)->setName("Speed");
     
     widget.qplotGraph4->addGraph(); // red line
     widget.qplotGraph4->graph(2)->setPen(QPen(Qt::red));
     widget.qplotGraph4->graph(2)->setBrush(Qt::NoBrush);
     widget.qplotGraph4->graph(2)->setAntialiasedFill(false);                 
     widget.qplotGraph4->graph(2)->setName("Signal strength");     
     
     widget.qplotGraph4->xAxis->setTickLabelType(QCPAxis::ltDateTime);
     widget.qplotGraph4->xAxis->setDateTimeFormat("ss");
     widget.qplotGraph4->xAxis->setAutoTickStep(false);
     widget.qplotGraph4->xAxis->setTickStep(2);
     widget.qplotGraph4->axisRect()->setupFullAxesBox();
     widget.qplotGraph4->yAxis->setRange(GUI_VOLTAGE_LOWER,GUI_VOLTAGE_UPPER);
     widget.qplotGraph4->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignLeft);
     widget.qplotGraph4->legend->setVisible(true);
     widget.qplotGraph4->yAxis->setAutoTickStep(false);
     widget.qplotGraph4->yAxis->setTickStep(1);
     widget.qplotGraph4->yAxis2->setLabel("Speed (rpm)");  
     widget.qplotGraph4->yAxis2->setAutoTickStep(false);
     widget.qplotGraph4->yAxis2->setAutoSubTicks(false);
     widget.qplotGraph4->yAxis2->setTickStep(1000);
     widget.qplotGraph4->yAxis2->setSubTickCount(5);
     widget.qplotGraph4->yAxis2->setRange(GUI_SPEED_LOWER,GUI_SPEED_UPPER);
     widget.qplotGraph4->yAxis2->setTickLabels(true); 
     widget.qplotGraph4->yAxis2->setVisible(true); 
}

void BasicWindow::btnConnectHelicopter1Clicked(void){
    char * heli_ip_address;
    heli_ip_address = (char*) malloc(20);
    
    QByteArray heli_ip_add = widget.editHeli1->text().toLatin1();   
    heli_ip_address = heli_ip_add.data();    
    
    if (helicopter1connected){
        if (helicopter1 != NULL){
            requestDataTimer->stop();
            delete requestDataTimer;            

            helicopter1->enableStatePropagation(false);
            poseEstimation->enableEKFCorrection(false);
            
            disconnectClients();
            
            helicopter1->enableStatePropagation(false);
            
            delete poseEstimation;
            poseEstimation = NULL;            
            
            delete helicopter1;
            helicopter1 = NULL;            
        }
        // Change button label
        widget.btnConnectHelicopter1->setText("Helicopter 1\nConnect");
        helicopter1connected = false;
    }
    else{
        if (helicopter1 == NULL){
            helicopter1 = new Helicopter(1,heli_ip_address, (char *) "2002");
            
            poseEstimation = new PoseEstimation();
            poseEstimation->setHelicopterReference(helicopter1);
                        
            connectClients();
            
            sleep(3);
            
            requestDataTimer = new QTimer(this);
            connect(requestDataTimer, SIGNAL(timeout()), this, SLOT(btnGetData()));  
            requestDataTimer->start(DATA_REQUEST_TIME);
            
//            // start timer to request data from clients but do not start ekf until initialisaton is complete
//            while(!poseEstimation->isInitialised());            
//            
//            helicopter1->enableStatePropagation(true);
//            poseEstimation->enableEKFCorrection(true);            
                        
            // Change button label
            widget.btnConnectHelicopter1->setText("Helicopter 1\nConnected");        
            helicopter1connected = true;
            runTime.start();
        }else{
            errorMessage("Tried to create helicopter1");
        }
    }
//    free(heli_ip_address);
}

void BasicWindow::btnConnectHelicopter2Clicked(void){
    char * heli_ip_address;
    heli_ip_address = (char*) malloc(20);
    
    QByteArray heli_ip_add = widget.editHeli2->text().toLatin1();   
    heli_ip_address = heli_ip_add.data();    
    
    if (helicopter2connected){
        if (helicopter2 != NULL){
            requestDataTimer->stop();
            delete requestDataTimer;            

            helicopter2->enableStatePropagation(false);
            poseEstimation->enableEKFCorrection(false);
            
            disconnectClients();
            
            helicopter2->enableStatePropagation(false);
            
            delete poseEstimation;
            poseEstimation = NULL;            
            
            delete helicopter2;
            helicopter2 = NULL;            
        }
        // Change button label
        widget.btnConnectHelicopter2->setText("Helicopter 2\nConnect");
        helicopter2connected = false;
    }
    else{
        if (helicopter2 == NULL){
            helicopter2 = new Helicopter(2,heli_ip_address, (char *) "2002");
            
            poseEstimation = new PoseEstimation();
            poseEstimation->setHelicopterReference(helicopter2);
                        
            connectClients();
            
            sleep(1);
            
            requestDataTimer = new QTimer(this);
            connect(requestDataTimer, SIGNAL(timeout()), this, SLOT(btnGetData()));  
            requestDataTimer->start(DATA_REQUEST_TIME);
            
//            // start timer to request data from clients but do not start ekf until initialisaton is complete
//            while(!poseEstimation->isInitialised());            
//            
//            helicopter1->enableStatePropagation(true);
//            poseEstimation->enableEKFCorrection(true);            
                        
            // Change button label
            widget.btnConnectHelicopter2->setText("Helicopter 2\nConnected");        
            helicopter2connected = true;
            runTime.start();
        }else{
            errorMessage("Tried to create helicopter2");
        }
    }
//    free(heli_ip_address);
}

void BasicWindow::btnGetData(void){
    if(poseEstimation->isInitialised()){
        if (helicopter1connected)
            helicopter1->enableStatePropagation(true);
        if (helicopter2connected)
            helicopter2->enableStatePropagation(true);
        
        poseEstimation->enableEKFCorrection(true);
    }
    
    if (poseEstimation->allCamClear()){
        for (int i = 0; i < NUM_OF_CAMERAS; i++){
            if (tcp_client[i] != NULL){
                tcp_client[i]->sendMsg(TCP_client::GET_DATA);
            }else
                std::cout << "Client " << i+1 << "NULL" << std::endl;
        }
    }
}

void BasicWindow::connectClients(void){
    int i;
    char * server_ip_addresses[NUM_OF_CAMERAS];
    // extract char ip addresses
    for (i = 0; i < NUM_OF_CAMERAS; i++){
        server_ip_addresses[i] = (char*) malloc(20);
    }
    
    QByteArray ip_add_lat1 = widget.editIP1->text().toLatin1();
    QByteArray ip_add_lat2 = widget.editIP2->text().toLatin1();
    QByteArray ip_add_lat3 = widget.editIP3->text().toLatin1();
    QByteArray ip_add_lat4 = widget.editIP4->text().toLatin1();
    
    server_ip_addresses[0] = ip_add_lat1.data();    
    server_ip_addresses[1] = ip_add_lat2.data();    
    server_ip_addresses[2] = ip_add_lat3.data();    
    server_ip_addresses[3] = ip_add_lat4.data();    
    
    for (i = 0; i < NUM_OF_CAMERAS; i++){
        if (tcp_client[i] == NULL){
            tcp_client[i] = new TCP_client(i+1,SERVER_HOST_PORT,server_ip_addresses[i], poseEstimation->getVirtualCamera(i));
            if (!(tcp_client[i]->tryConnect())){
                delete tcp_client[i];
                tcp_client[i] = NULL;
            }
            else{
                tcp_callback_connection[i] = tcp_client[i]->getCallbackSignal()->connect(boost::bind(&BasicWindow::TCP_Client_callback,this,_1,_2));
                data_received_connection[i] = tcp_client[i]->getDataReceivedSignal()->connect(boost::bind(&PoseEstimation::setCamDataReceived,poseEstimation,_1));
            }
        }
    }
    
//    for (i = 0; i < NUM_OF_CAMERAS; i++){
//        free(server_ip_addresses[i]);
//    }    
}

void BasicWindow::disconnectClients(void){
    for (int i = 0; i < NUM_OF_CAMERAS; i++){
        if (tcp_client[i] != NULL){
            tcp_callback_connection[i].disconnect();
            data_received_connection[i].disconnect();
            delete tcp_client[i];
            tcp_client[i] = NULL;
        }
    }    
}

void BasicWindow::TCP_Client_callback(TCP_client::callback_msg msg, int id){
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

void BasicWindow::updateGUI(void){
    if (helicopter1 != NULL){ 
        widget.lblEKFRate->setText(QString::number(helicopter1->getEKFRate(),'f',0) + " Hz");   
        widget.lblCorrectionRate->setText(QString::number(poseEstimation->getFPS(),'f',0) + " Hz");  
        float last_correction_ms = helicopter1->timeSinceLastCorrection()*1000;
        widget.lblTimeSinceLastCorrection->setText(QString::number(last_correction_ms,'f',0));
        if (last_correction_ms > 100){
            widget.lblTimeSinceLastCorrection->setForegroundRole(QPalette::Highlight);
        }else{
            widget.lblTimeSinceLastCorrection->setForegroundRole(QPalette::NoRole);
        }
        
        double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
        // add data to lines:
        widget.qplotGraph1->graph(0)->addData(key, helicopter1->getRoll());
        widget.qplotGraph1->graph(0)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph1->graph(1)->addData(key, helicopter1->getPitch());
        widget.qplotGraph1->graph(1)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph1->graph(2)->addData(key, helicopter1->getYaw());
        widget.qplotGraph1->graph(2)->removeDataBefore(key-SCROLL_TIME_RANGE); 
        widget.qplotGraph1->graph(3)->addData(key, helicopter1->getOnboardRoll());
        widget.qplotGraph1->graph(3)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph1->graph(4)->addData(key, helicopter1->getOnboardPitch());
        widget.qplotGraph1->graph(4)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph1->graph(5)->addData(key, helicopter1->getOnboardYaw());
        widget.qplotGraph1->graph(5)->removeDataBefore(key-SCROLL_TIME_RANGE);         
        widget.qplotGraph1->rescaleAxes();
        widget.qplotGraph1->xAxis->setRange(key+0.25, SCROLL_TIME_RANGE, Qt::AlignRight);
        widget.qplotGraph1->replot();            
        
        ////////
        widget.qplotGraph2->graph(0)->addData(key, helicopter1->getx());
        widget.qplotGraph2->graph(0)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph2->graph(1)->addData(key, helicopter1->gety());
        widget.qplotGraph2->graph(1)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph2->graph(2)->addData(key, helicopter1->getz());
        widget.qplotGraph2->graph(2)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph2->graph(3)->addData(key, helicopter1->getOnboardTx());
        widget.qplotGraph2->graph(3)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph2->graph(4)->addData(key, helicopter1->getOnboardTy());
        widget.qplotGraph2->graph(4)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph2->graph(5)->addData(key, helicopter1->getOnboardTz());
        widget.qplotGraph2->graph(5)->removeDataBefore(key-SCROLL_TIME_RANGE);        
        widget.qplotGraph2->rescaleAxes();
        widget.qplotGraph2->xAxis->setRange(key+0.25, SCROLL_TIME_RANGE, Qt::AlignRight);
        widget.qplotGraph2->replot();        
        
        ///////////////////////////        
        widget.qplotGraph3->graph(0)->addData(key, helicopter1->getVx());
        widget.qplotGraph3->graph(0)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph3->graph(1)->addData(key, helicopter1->getVy());
        widget.qplotGraph3->graph(1)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph3->graph(2)->addData(key, helicopter1->getVz());
        widget.qplotGraph3->graph(2)->removeDataBefore(key-SCROLL_TIME_RANGE); 
//        widget.qplotGraph3->rescaleAxes();
        widget.qplotGraph3->xAxis->setRange(key+0.25, SCROLL_TIME_RANGE, Qt::AlignRight);
        
        widget.qplotGraph3->replot();        
        
        ///////////////////////////        
        widget.qplotGraph4->graph(0)->addData(key, helicopter1->getVoltage());
        widget.qplotGraph4->graph(0)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph4->graph(1)->addData(key, helicopter1->getHeadSpeed()*GUI_VOLTAGE_UPPER/GUI_SPEED_UPPER);
        widget.qplotGraph4->graph(1)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph4->xAxis->setRange(key+0.25, SCROLL_TIME_RANGE, Qt::AlignRight);
        if (helicopter1->getVoltage() < GUI_PROTECTION_VOLTAGE){
            widget.qplotGraph4->setBackground(Qt::red);
        }
        else
        {
            widget.qplotGraph4->setBackground(Qt::white);
        }
        widget.qplotGraph4->replot();
        
        char buff[20];
        int tElapsed = runTime.elapsed();
        sprintf(buff, "%02d:%02d", (int)((float) tElapsed / 60000), (tElapsed % 60000)/1000);
        widget.lcdNumber->display(buff);
    }    
    
    if (helicopter2 != NULL){ 
        widget.lblEKFRate->setText(QString::number(helicopter2->getEKFRate(),'f',0) + " Hz");   
        widget.lblCorrectionRate->setText(QString::number(poseEstimation->getFPS(),'f',0) + " Hz");  
        float last_correction_ms = helicopter2->timeSinceLastCorrection()*1000;
        widget.lblTimeSinceLastCorrection->setText(QString::number(last_correction_ms,'f',0));
        if (last_correction_ms > 100){
            widget.lblTimeSinceLastCorrection->setForegroundRole(QPalette::Highlight);
        }else{
            widget.lblTimeSinceLastCorrection->setForegroundRole(QPalette::NoRole);
        }
        
        double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
        // add data to lines:
        widget.qplotGraph1->graph(0)->addData(key, helicopter2->getRoll());
        widget.qplotGraph1->graph(0)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph1->graph(1)->addData(key, helicopter2->getPitch());
        widget.qplotGraph1->graph(1)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph1->graph(2)->addData(key, helicopter2->getYaw());
        widget.qplotGraph1->graph(2)->removeDataBefore(key-SCROLL_TIME_RANGE); 
        widget.qplotGraph1->graph(3)->addData(key, helicopter2->getOnboardRoll());
        widget.qplotGraph1->graph(3)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph1->graph(4)->addData(key, helicopter2->getOnboardPitch());
        widget.qplotGraph1->graph(4)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph1->graph(5)->addData(key, helicopter2->getOnboardYaw());
        widget.qplotGraph1->graph(5)->removeDataBefore(key-SCROLL_TIME_RANGE);         
        widget.qplotGraph1->rescaleAxes();
        widget.qplotGraph1->xAxis->setRange(key+0.25, SCROLL_TIME_RANGE, Qt::AlignRight);
        widget.qplotGraph1->replot();            
        
        ////////
        widget.qplotGraph2->graph(0)->addData(key, helicopter2->getx());
        widget.qplotGraph2->graph(0)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph2->graph(1)->addData(key, helicopter2->gety());
        widget.qplotGraph2->graph(1)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph2->graph(2)->addData(key, helicopter2->getz());
        widget.qplotGraph2->graph(2)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph2->graph(3)->addData(key, helicopter2->getOnboardTx());
        widget.qplotGraph2->graph(3)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph2->graph(4)->addData(key, helicopter2->getOnboardTy());
        widget.qplotGraph2->graph(4)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph2->graph(5)->addData(key, helicopter2->getOnboardTz());
        widget.qplotGraph2->graph(5)->removeDataBefore(key-SCROLL_TIME_RANGE);        
        widget.qplotGraph2->rescaleAxes();
        widget.qplotGraph2->xAxis->setRange(key+0.25, SCROLL_TIME_RANGE, Qt::AlignRight);
        widget.qplotGraph2->replot();        
        
        ///////////////////////////        
        widget.qplotGraph3->graph(0)->addData(key, helicopter2->getVx());
        widget.qplotGraph3->graph(0)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph3->graph(1)->addData(key, helicopter2->getVy());
        widget.qplotGraph3->graph(1)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph3->graph(2)->addData(key, helicopter2->getVz());
        widget.qplotGraph3->graph(2)->removeDataBefore(key-SCROLL_TIME_RANGE); 
//        widget.qplotGraph3->rescaleAxes();
        widget.qplotGraph3->xAxis->setRange(key+0.25, SCROLL_TIME_RANGE, Qt::AlignRight);
        
        widget.qplotGraph3->replot();        
        
        ///////////////////////////        
        widget.qplotGraph4->graph(0)->addData(key, helicopter2->getVoltage());
        widget.qplotGraph4->graph(0)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph4->graph(1)->addData(key, helicopter2->getHeadSpeed()*GUI_VOLTAGE_UPPER/GUI_SPEED_UPPER);
        widget.qplotGraph4->graph(1)->removeDataBefore(key-SCROLL_TIME_RANGE);
        widget.qplotGraph4->xAxis->setRange(key+0.25, SCROLL_TIME_RANGE, Qt::AlignRight);
        if (helicopter2->getVoltage() < GUI_PROTECTION_VOLTAGE){
            widget.qplotGraph4->setBackground(Qt::red);
        }
        else
        {
            widget.qplotGraph4->setBackground(Qt::white);
        }
        widget.qplotGraph4->replot();
        
        char buff[20];
        int tElapsed = runTime.elapsed();
        sprintf(buff, "%02d:%02d", (int)((float) tElapsed / 60000), (tElapsed % 60000)/1000);
        widget.lcdNumber->display(buff);
    }        
    

}

void BasicWindow::errorMessage(const char * message){
    std::cout << "Window ERR: " << message << std::endl;
}

BasicWindow::~BasicWindow() {
    // If allocated pointers have not been deallocated...do so
    if (helicopter1 != NULL){
        delete helicopter1;
        helicopter1 = NULL;
    }
    if (poseEstimation != NULL){
        delete poseEstimation;
        poseEstimation = NULL;
    }    
}
