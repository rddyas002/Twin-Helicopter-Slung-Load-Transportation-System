/*
 * UAVControl Application
 * Function: Retrieve IMU data from UAV's, fuse with data from camera subsystem
 * form an estimate of position, velocity, acceleration, rotation and IMU biases
 * using Extended Kalman Filter
 */

#include "UAV.h"

#define VERSION "1.0"

using namespace std;
    
int main(int argc, char** argv) {   
    UAV * uav = NULL;
        
    cout << "UAV Control Version " << VERSION <<  endl;    
    
    if (argc < 2){
        cout << "ERR: No configuration file supplied" << endl;
        return 0;
    }
    else{
        uav = new UAV(argv[1]);    
    }
        
    int c = 0;
    do{
        c = getchar();
    } while (c != 'q');
    
    delete uav;
    uav = NULL;
    
    cout << "Exiting" << endl;
    return 0;
}

