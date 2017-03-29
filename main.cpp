// Copyright 2006-2016 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.3.2 on August 29th 2016

// Make sure to have the server side running in V-REP: 
// in a child script of a V-REP scene, add following command
// to be executed just once, at simulation start:
//
// simExtRemoteApiStart(19999)
//
// then start simulation, and run this program.
//
// IMPORTANT: for each successful call to simxStart, there
// should be a corresponding call to simxFinish at the end!

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
extern "C" {
    #include "extApi.h"
}


float wheel_radius = 0.079259/2; 
float max_speed = 1; // m/s  
int main(int argc,char* argv[])
{
    int clientID=simxStart((simxChar*)"127.0.0.1",19997,true,true,2000,5);
    if (clientID!=-1)
    {
        //printf("Connected to remote API server\n");
        std::cout<< "Connected"<< std::endl;
        float curr_pos[7], targetpos[5];  // x,y,z,theta,vel 
        targetpos[4] = 0;

        float max_w = max_speed/wheel_radius;  // angular velocity max
        float curr_vel = 0;
        int steeringLeftHandle, steeringRightHandle,motorLeftHandle,motorRightHandle, targethandle, GPS ;  
        simxGetObjectHandle(clientID,"target", &targethandle, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID,"GPS", &GPS, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID,"nakedCar_steeringLeft", &steeringLeftHandle, simx_opmode_oneshot_wait); 
		simxGetObjectHandle(clientID, "nakedCar_steeringRight",&steeringRightHandle, simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, "nakedCar_motorLeft",&motorLeftHandle, simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, "nakedCar_motorRight",&motorRightHandle, simx_opmode_oneshot_wait);
        while (simxGetConnectionId(clientID)!=-1)
        {
           simxGetObjectPosition(clientID,GPS,-1, &curr_pos[0] ,simx_opmode_oneshot);
           simxGetObjectPosition(clientID,targethandle, GPS, &targetpos[0] ,simx_opmode_oneshot);
           simxGetObjectVelocity(clientID, GPS, &curr_pos[4], NULL, simx_opmode_oneshot); 
           
           float r = sqrt(targetpos[2]*targetpos[2] + targetpos[1]*targetpos[1]);
           float alpha = atan(targetpos[2]/targetpos[1]);
           curr_vel = sqrt(curr_pos[5]*curr_pos[5] + curr_pos[4]*curr_pos[4]);
           std::cout << " r " << r<< " alpha "<< alpha*180/3.14 << " Velocity :"<< curr_vel<< "  "<< wheel_radius*max_w <<std::endl;
		           




           if(r >= 0.5) 
           {
           		simxSetJointTargetVelocity(clientID,motorLeftHandle,max_w,simx_opmode_oneshot);            
           		simxSetJointTargetVelocity(clientID,motorRightHandle,max_w,simx_opmode_oneshot);           
           }
           else
           {
           		simxSetJointTargetVelocity(clientID,motorLeftHandle,0,simx_opmode_oneshot);            
           		simxSetJointTargetVelocity(clientID,motorRightHandle,0,simx_opmode_oneshot);           
           }
            
            extApi_sleepMs(5);
        }
        simxFinish(clientID);
    }
    return(0);
}
