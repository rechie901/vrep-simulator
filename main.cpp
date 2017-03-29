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
