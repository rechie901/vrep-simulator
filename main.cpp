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
float d=0.0944; //-- 2*d=distance between left and right wheels
float l=0.3222; //-- l=distance between front and rear wheels

float max_speed = 1.0; // m/s
float max_accel = 1.0;
float max_brake = 1.0; // m/(s*s)
float max_steerangle = 40; 
int steeringLeftHandle, steeringRightHandle,motorLeftHandle,motorRightHandle, targethandle, GPS ;  

struct state{
	float x[],y[],heading[],steer_angle[],velocity[];
};
struct dynamics{
	float x,y,heading,steer_angle,velocity;
};
struct control{
	float acceleration[],steer_vel[];
};
struct control_parameters{
	float a1, b1, c1, a2, b2, c2;
};
void compute_v(state v, float accel[], int i){
	v.velocity[i] = v.velocity[0] + accel;
}
void compute_a(control control, control_parameters params, float tg, float step_size){
	for (int i = 0; i < tg; ++i)
	{
		control.acceleration[i] = params.a2 + 2*params.b2*step_size*i + 3*params.c2*pow(step_size*i ,2) ;
		if(control.acceleration[i] > max_accel){
			control.acceleration[i] = max_accel;
		}
		if(control.acceleration[i] < -max_accel){
			control.acceleration[i] = -max_accel;
		}

	}

	 
}
void compute_u(){

}
void accelerate(int ID ,float accel_input,float angular_vel, float current_vel, float delta_t){
	if(current_vel < max_speed){
        angular_vel = angular_vel + ((accel_input*delta_t)/wheel_radius); 
    	//std::cout <<"angular_vel " << angular_vel << std::endl;
    }
    else
        angular_vel = angular_vel;
    
    simxSetJointTargetVelocity(ID,motorLeftHandle,angular_vel,simx_opmode_oneshot);            
    simxSetJointTargetVelocity(ID,motorRightHandle,angular_vel,simx_opmode_oneshot);
    //return(angular_vel);         	

}
void brake(int ID ,float brake_input,float angular_vel, float current_vel, float delta_t){
	   if(angular_vel > 0)
        {			
           	angular_vel = angular_vel - ((brake_input*delta_t)/wheel_radius);
           	simxSetJointTargetVelocity(ID,motorLeftHandle,angular_vel,simx_opmode_oneshot);            
           	simxSetJointTargetVelocity(ID,motorRightHandle,angular_vel,simx_opmode_oneshot);           
 		}
 		else
 		{
 			simxSetJointTargetVelocity(ID,motorLeftHandle,0,simx_opmode_oneshot);            
        	simxSetJointTargetVelocity(ID,motorRightHandle,0,simx_opmode_oneshot);
 			//std::cout << "brake "<< curr_vel<< std::endl;
 			//simxStopSimulation(ID, simx_opmode_oneshot);
 		}             	
 		//return(angular_vel);
} 
void steer(int ID, float desiredSteeringAngle){
	
	float theta;
	if(desiredSteeringAngle < 0)
		theta = (90 + desiredSteeringAngle);
	else if(desiredSteeringAngle > 0)
		theta = -1 * (90 - desiredSteeringAngle);
	if(theta < - max_steerangle)
		theta = max_steerangle;
	if(theta > max_steerangle)
		theta = -1 * max_steerangle;
	theta = theta * 3.14/180;
	float steeringAngleLeft = atan(l/(-d+l/tan(theta)));
    float steeringAngleRight = atan(l/(d+l/tan(theta)));
    simxSetJointTargetPosition(ID,steeringLeftHandle,steeringAngleLeft,simx_opmode_oneshot);
    simxSetJointTargetPosition(ID,steeringRightHandle,steeringAngleRight,simx_opmode_oneshot);

}

void handle_init(int clientID){
		simxGetObjectHandle(clientID,"target", &targethandle, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID,"GPS", &GPS, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID,"nakedCar_steeringLeft", &steeringLeftHandle, simx_opmode_oneshot_wait); 
		simxGetObjectHandle(clientID, "nakedCar_steeringRight",&steeringRightHandle, simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, "nakedCar_motorLeft",&motorLeftHandle, simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, "nakedCar_motorRight",&motorRightHandle, simx_opmode_oneshot_wait);
}

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
        float accel = 199*2.5, deccel = 50;

        float ang_vel = 0.0;  // angular velocity of wheel joint
        float curr_vel = 0.0;
        float curr_steerpos = 0.0;
        handle_init(clientID);
        
        float n = 0.0; // time
        float step = 0.002; // step size -- 2 ms
        while (simxGetConnectionId(clientID)!=-1)
        {
        	// auto time_now = get_time::now();
        	// auto diff = time_now - time_start;


            simxGetObjectPosition(clientID,GPS,-1, &curr_pos[0] ,simx_opmode_oneshot);
            simxGetObjectPosition(clientID,targethandle, GPS, &targetpos[0] ,simx_opmode_oneshot);
            simxGetObjectVelocity(clientID, GPS, &curr_pos[4], NULL, simx_opmode_oneshot); 
            simxGetJointPosition(clientID, steeringLeftHandle, &curr_steerpos, simx_opmode_oneshot);
            float r = sqrt(targetpos[2]*targetpos[2] + targetpos[1]*targetpos[1]);
            float alpha = atan(targetpos[2]/targetpos[1]);
            curr_vel = sqrt(curr_pos[5]*curr_pos[5] + curr_pos[4]*curr_pos[4]);
            alpha = alpha*180/3.14 ;
           

          	//std::cout << " r " << r << " alpha "<< alpha << " Velocity :"<< curr_steerpos*180/3.14 <<std::endl;
		    if(r > 0.8 && curr_vel < 1.0) 
           {	
    			accelerate(clientID ,accel , ang_vel, curr_vel , step);
    			steer(clientID,alpha);
    			
    			//std::cout << "curr_vel "<< curr_vel<< std::endl;       		  
            }
           else
           {	
           		//std::cout << "brake"<< curr_vel<< std::endl;
           		brake(clientID ,deccel ,ang_vel, curr_vel, step);
           		steer(clientID,alpha);
          	}
            n = n+step;
            extApi_sleepMs(2);
            std::cout << " n " << n << std::endl;
        }
        simxFinish(clientID);
    }
    return(0);
}
