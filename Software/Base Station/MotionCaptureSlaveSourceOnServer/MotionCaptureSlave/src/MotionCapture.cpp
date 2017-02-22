/* 
 * File:   MotionCapture.cpp
 * Author: yashren
 * 
 * Created on 14 May 2013, 7:57 PM
 */

#include "../inc/MotionCapture.h"

MotionCapture::MotionCapture(int id, int port, bool draw_blobs) {
	server_id = id;
	server_port = port;
	keepRunning = true;
        drawBlobs = draw_blobs;
        
        system_state = NULL_0;
	
	// Initialize points with NULL
	tcp_server = NULL;
	camera = NULL;
	blob = NULL;
	
	if (tcp_server == NULL){
		tcp_server = new TCP_server(port);
                motioncapture_connection = tcp_server->getMainCallbackSignal()->connect(boost::bind(&MotionCapture::TCP_callback, this,_1));
	}    
}

void MotionCapture::TCP_callback(relay_callback_msg msg){
    // Used to hold dynamic file path for camera data
    char string_buffer[256];  
    
    switch(msg){
        case CONNECTION_MADE_RELAY:
            if (camera == NULL){
                // Set camera parameters
                camera = new OpencvCamera(200,server_id);
                camera->setIntrinsic((CvMat*)cvLoad(get_concat_string(string_buffer, PREAMBLE_TO_INTRINSIC, server_id)));
                camera->setDistortion((CvMat*)cvLoad(get_concat_string(string_buffer, PREAMBLE_TO_DISTORTION, server_id)));
                camera->setCamPosition((CvMat*)cvLoad(get_concat_string(string_buffer, PREAMBLE_TO_POSITION, server_id)));
                camera->makeDistortionMap();
                camera->loadRotationAngles((CvMat*)cvLoad(get_concat_string(string_buffer, PREAMBLE_TO_ROTATION, server_id)));
                camera->setCenter((CvMat*)cvLoad(PATH_TO_CENTER));
                camera->calcExtrinsics();
            
                if (blob == NULL){
                    blob = new Blob(camera,server_id,drawBlobs);

                    // connect blob conversion complete signal
                    if (tcp_server != NULL){
                        conversion_complete = blob->getSignal()->connect(boost::bind(&TCP_server::write2clients,tcp_server,_1));
                    }
                }
                
                system_state = IDLE;
            }
            break;
			
        case CLIENT_DISCONNECT_RELAY:
            if ((camera != NULL) && !(tcp_server->getNumClients())){
                
                // disconnect signal
                conversion_complete.disconnect();			
                if (blob != NULL){
                    delete blob;
                }
                blob = NULL;
                
                delete camera;
                camera = NULL;
            }
            system_state = IDLE;			
            break;  
            
        case GET_DATA_RELAY:
            if (blob != NULL)
                blob->processThread();//runProcess();
	    else
		debugMessage("Trying to run process with no blob object");
				
            break;
            
        case START_AUTO_RELAY:
            if (blob != NULL){
                switch (system_state){
                    case AUTO:
                        blob->autoThreadEnd();
                        system_state = IDLE;
                        break;
                    case CALIBRATE:
                        blob->calibThreadEnd();
                        system_state = IDLE;
                        break;
                    case IDLE:
                        blob->runAuto();
                        system_state = AUTO;
                        break;                        
                }                
            }
	    else
		debugMessage("Trying to access blob with no blob object");
				
            break; 
            
        case STOP_AUTO_RELAY:
            if (blob != NULL){
                blob->autoThreadEnd();
                system_state = IDLE;   
            }
	    else
		debugMessage("Trying to access blob with no blob object");
				
            break;  
            
        case CALIBRATE_RELAY:
            if (blob != NULL){
                switch (system_state){
                    case AUTO:
                        // stop auto-mode...just to keep track of things
                        blob->autoThreadEnd();
                        system_state = IDLE;
                        break;
                    case CALIBRATE:
                        blob->calibThreadEnd();
                        system_state = IDLE;
                        break;
                    case IDLE:
                        //blob->calibMode();
                        blob->doCalib();
//                        blob->doCalib();
                        // send signal to main thread
//                        main_thread(CALIBRATE);
                        system_state = SHOW_IMAGE;
                        break;                        
                }
            }
	    else
		debugMessage("Trying to access blob with no blob object");
				
            break;              
            
        case KILL_PROGRAM_RELAY:
            if (camera != NULL){
                if (blob != NULL){
                    delete blob;
                }
                blob = NULL;
                
                delete camera;
                camera = NULL;
            }
			
            keepRunning = false;
			
            break;             
 
        default:
            break;
    }
}

char * MotionCapture::get_concat_string(char * buffer, const char * pream, int id){
    char ascii_terminal[10];
    sprintf(ascii_terminal,"%d.xml",id);
    sprintf(buffer,PATH_TO_SERVER_MOUNT);
    strcat(buffer,pream);
    strcat(buffer,ascii_terminal);
    return buffer;
}

void MotionCapture::debugMessage(const char * str){
	printf("DBG>> %s \n", str);
}

bool MotionCapture::getKeepRunning(void){
	return keepRunning;
}

MotionCaptureState MotionCapture::getState(void){
    return system_state;
}

void MotionCapture::setState(MotionCaptureState st){
    system_state = st;
}

// return objects
Blob * MotionCapture::getBlobObject(void){
    return blob;
}
TCP_server * MotionCapture::getTCPServerObject(void){
    return tcp_server;
}
OpencvCamera * MotionCapture::getCameraObject(void){
    return camera;
}

MotionCapture::~MotionCapture() {
	if (tcp_server != NULL){
		delete tcp_server;
        tcp_server = NULL;
	}       
    
    std::cout << "Deallocating tcp_server" << std::endl;
}

