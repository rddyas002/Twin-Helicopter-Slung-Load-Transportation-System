/* 
 * File:   Helicopter.cpp
 * Author: Yashren Reddi
 * 
 * Created on 05 March 2013, 9:02 AM
 * 
 * This class holds all the data for a particular helicopter - all relevant
 * blob positions from each camera view, platform position and attitude.
 * Also runs the least squares estimation for finding the position and attitude. 
 * This will run on separate threads.
 * 
 */

#include "../include/Helicopter.h"

Helicopter::Helicopter() {
}

Helicopter::~Helicopter() {
}

CvPoint3D32f Helicopter::getHeliPosition(void){
    return heli_position;
}

void Helicopter::setHeliPosition(CvPoint3D32f pos){
    heli_position = pos;
}

unsigned int Helicopter::getID(void){
    return id;
}

void Helicopter::setID(unsigned int Id){
    id = Id;
}

