#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
//#include <chrono>

extern "C" {
    #include "extApi.h"
}

// using ns = chrono::milliseconds;
// using get_time = chrono::steady_clock;

float wheel_radius = 0.079259/2; 
float max_speed = 1; // m/s
float max_accel = 1;
float max_brake = 1; // m/(s*s)  
int main(int argc,char* argv[])
{
    int clientID=simxStart((simxChar*)"127.0.0.1",19997,true,true,2000,5);
    if (clientID!=-1)
    {
        //printf("Connected to remote API server\n");
        std::cout<< "Connected"<< std::endl;
        simxStartSimulation(clientID, simx_opmode_oneshot);
        //auto time_start = get_time::now();
        float curr_pos[7], targetpos[5];  // x,y,z,theta,vel 
        targetpos[4] = 0;
        float accel = 1, brake = 1*100;

        float ang_vel = 0;  // angular velocity of wheel joint
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
        	// auto time_now = get_time::now();
        	// auto diff = time_now - time_start;
           simxGetObjectPosition(clientID,GPS,-1, &curr_pos[0] ,simx_opmode_oneshot);
           simxGetObjectPosition(clientID,targethandle, GPS, &targetpos[0] ,simx_opmode_oneshot);
           simxGetObjectVelocity(clientID, GPS, &curr_pos[4], NULL, simx_opmode_oneshot); 
           
           float r = sqrt(targetpos[2]*targetpos[2] + targetpos[1]*targetpos[1]);
           float alpha = atan(targetpos[2]/targetpos[1]);
           curr_vel = sqrt(curr_pos[5]*curr_pos[5] + curr_pos[4]*curr_pos[4]);

           

          // std::cout << " r " << r<< " alpha "<< alpha*180/3.14 << " Velocity :"<< curr_vel<<std::endl;
		           




           if(r >= 0.8 && curr_vel < 1.0) 
           {	
           		if(ang_vel < max_speed/wheel_radius)
           			ang_vel = ang_vel + accel*0.005/wheel_radius;
           		else
           			ang_vel = ang_vel;
           		std::cout << "curr_vel "<< curr_vel<< std::endl;
           		simxSetJointTargetVelocity(clientID,motorLeftHandle,ang_vel,simx_opmode_oneshot);            
           		simxSetJointTargetVelocity(clientID,motorRightHandle,ang_vel,simx_opmode_oneshot);           
           }
           else
           {	
           		if(curr_vel != 0.0 && ang_vel>=0)
           		{	
           			
           			
           			ang_vel = ang_vel - brake*0.005/wheel_radius;
           			std::cout << "curr_vel "<< curr_vel<< std::endl;
           			simxSetJointTargetVelocity(clientID,motorLeftHandle,ang_vel,simx_opmode_oneshot);            
           			simxSetJointTargetVelocity(clientID,motorRightHandle,ang_vel,simx_opmode_oneshot);           
 				}
 				// else
 				// {
 				// 	std::cout << "brake "<< curr_vel<< std::endl;
 				// 	break;
 				// }          
           }
            
            extApi_sleepMs(5);
        }
        simxFinish(clientID);
    }
    return(0);
}
