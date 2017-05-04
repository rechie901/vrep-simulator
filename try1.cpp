#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
//#include <chrono>

extern "C" {
    #include "extApi.h"
}

// using ns = chrono::milliseconds;
// using get_time = chrono::steady_clock;
using namespace Eigen;
float wheel_radius = 0.079259/2;
float d=0.0944; //-- 2*d=distance between left and right wheels
float l=0.3222; //-- l=distance between front and rear wheels

float max_speed = 1.0; // m/s
float max_accel = 1.0;
float max_brake = 1.0; // m/(s*s)
float max_steervel = 1.0;
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
struct terminal_states{
	float x, y, heading, steer_angle, velocity;
};
void compute_v(state v, control accel, float step_size, int tg, float curr_velocity){
	v.velocity[tg];
	//float init_vel = v.velocity[0];
	for (int i = 0; i < tg; ++i)
	{
		v.velocity[i] = curr_velocity + accel.acceleration[i]*step_size*i;
		if(v.velocity[i] > max_speed){
			v.velocity[i] = max_speed;
		}
		curr_velocity = v.velocity[i]; 
		// if (v.velocity[i] < -v_max)
		// {
		// 	/* code */
		// }
	}
	
}
void compute_steer(state v, control steer, float step_size, int tg, float curr_steerangle){
	v.steer_angle[tg];
	//float init_angle = v.steer_angle[0];
	for (int i = 0; i < tg; ++i)
	{
		v.steer_angle[i] = curr_steerangle + steer.steer_vel[i]*step_size*i;
		if(v.steer_angle[i] > max_steerangle){
			v.steer_angle[i] = max_steerangle;
		}
		if (v.steer_angle[i] < -max_steerangle)
		{
			v.steer_angle[i] = -max_steerangle;
		}
		curr_steerangle = v.steer_angle[i];
	}
	
}
void compute_states(state v, float step_size, int tg, float curr_x, float curr_y, float curr_heading){
	v.x[tg];
	v.y[tg];
	v.heading[tg];
	//float init_x = v.x[0], init_y = v.y[0], init_heading = v.heading[0]; 
	for (int i = 0; i < tg; ++i)
	{
		v.heading[i] = curr_heading + v.velocity[i] * tan(v.steer_angle[i])*step_size/l;
		v.x[i] = curr_x + v.velocity[i] * cos(v.heading[i]) * step_size;  ///////////// ---- here
		v.y[i] = curr_y + v.velocity[i] * sin(v.heading[i]) * step_size;
	}
}
void compute_u(control control, control_parameters params, int tg, float step_size){
	control.acceleration[tg];
	control.steer_vel[tg];
	for (int i = 0; i < tg; ++i)
	{

		control.acceleration[i] = params.a2 + 2*params.b2*step_size*i + 3*params.c2*pow(step_size*i ,2) ;
		control.steer_vel[i] = params.a1 + 2*params.b1*step_size*i + 3*params.c1*pow(step_size*i ,2) ;
		if(control.acceleration[i] > max_accel){
			control.acceleration[i] = max_accel;
		}
		if(control.acceleration[i] < -max_accel){
			control.acceleration[i] = -max_accel;
		}
		if(control.steer_vel[i] < -max_steervel){
			control.steer_vel[i] = -max_steervel;
		}
		if(control.steer_vel[i] >  max_steervel){
			control.steer_vel[i] = max_steervel;
		}
		//std::cout << "accel " << control.acceleration[i] << std::endl ;
	}

	 
}
void compute_j(terminal_states t_state, state final, float j, float beta[4], int tg)
{
	j = sqrt(beta[0] * (final.x[tg] - t_state.x)*(final.x[tg] - t_state.x) + beta[1]*(final.y[tg] - t_state.y)*(final.y[tg] - t_state.y) + beta[2]*(final.velocity[tg] - t_state.velocity)*(final.velocity[tg] - t_state.velocity) + beta[3]*(final.heading[tg] - t_state.heading)*(final.heading[tg] - t_state.heading));
	 
}

void compute_cor(float e, control_parameters params, control control, int tg, float step_size, state current_state, terminal_states desired_state, float coeff, float curr_x, float curr_y, float curr_heading, float curr_steerpos, float curr_velocity)
{	
	control_parameters del_params;
	control_parameters new_params;
	state updated_state;
	updated_state.x[tg];
	updated_state.y[tg];
	updated_state.heading[tg];
	updated_state.steer_angle[tg];
	updated_state.velocity[tg];
	VectorXf del_p(6);
	VectorXf del_s(5);
	MatrixXf jacobian_mat(5,6);
	for (int i = 0; i < 6; ++i)
	{
		new_params = params ; 

		if(i == 0)
			new_params.a1 = params.a1 + e;
		if (i == 1)
			new_params.b1 = params.b1 + e;
		if(i == 2)
			new_params.c1 = params.c1 + e;
		if (i == 3)
			new_params.a2 = params.a2 + e;
		if(i == 4)
			new_params.b2 = params.b2 + e;
		if (i == 5)
			new_params.c2 = params.c2 + e;

		compute_u(control, new_params, tg, step_size);
		std::cout<< " here " << std::endl;            //             problem here 
		compute_v(updated_state, control, step_size, tg, curr_velocity);
		
		compute_steer(updated_state, control, step_size, tg, curr_steerpos);
		compute_states(updated_state, step_size, tg, curr_x, curr_y, curr_heading);

		
		jacobian_mat(0,i) = (current_state.x[tg] - updated_state.x[tg])/e ;
		jacobian_mat(1,i) = (current_state.y[tg] - updated_state.y[tg])/e ;
		jacobian_mat(2,i) = (current_state.heading[tg] - updated_state.heading[tg])/e ;
		jacobian_mat(3,i) = (current_state.steer_angle[tg] - updated_state.steer_angle[tg])/e ;
		jacobian_mat(4,i) = (current_state.velocity[tg] - updated_state.velocity[tg])/e ;

	}
	del_p = jacobian_mat.jacobiSvd(ComputeThinU | ComputeThinV).solve(del_s);
	params.a1 = -coeff*del_p[0] + params.a1;
	params.b1 = -coeff*del_p[1] + params.b1;
	params.c1 = -coeff*del_p[2] + params.c1;
	params.a2 = -coeff*del_p[3] + params.a2;
	params.b2 = -coeff*del_p[4] + params.b2;
	params.c2 = -coeff*del_p[5] + params.c2;
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
        
        // float n = 0.0; // time
        // float step = 0.002; // step size -- 2 ms
        while (simxGetConnectionId(clientID)!=-1)
        {
        	// auto time_now = get_time::now();
        	// auto diff = time_now - time_start;


            simxGetObjectPosition(clientID,GPS,-1, &curr_pos[0] ,simx_opmode_oneshot);
            simxGetObjectPosition(clientID,targethandle, -1, &targetpos[0] ,simx_opmode_oneshot);
            simxGetObjectVelocity(clientID, GPS, &curr_pos[4], NULL, simx_opmode_oneshot); 
            simxGetJointPosition(clientID, steeringLeftHandle, &curr_steerpos, simx_opmode_oneshot);
            float r = sqrt(targetpos[2]*targetpos[2] + targetpos[1]*targetpos[1]);
            float alpha = atan(targetpos[2]/targetpos[1]);
            curr_vel = sqrt(curr_pos[5]*curr_pos[5] + curr_pos[4]*curr_pos[4]); // current velocity of the vehicle
            alpha = alpha*180/3.14 ;
           

          	//std::cout << " r " << r << " alpha "<< alpha << " Velocity :"<< curr_steerpos*180/3.14 <<std::endl;
		    // if(r > 0.8 && curr_vel < 1.0) 
      //      {	
    		// 	accelerate(clientID ,accel , ang_vel, curr_vel , step);
    		// 	steer(clientID,alpha);
    			
    		// 	//std::cout << "curr_vel "<< curr_vel<< std::endl;       		  
      //       }
      //      else
      //      {	
      //      		//std::cout << "brake"<< curr_vel<< std::endl;
      //      		brake(clientID ,deccel ,ang_vel, curr_vel, step);
      //      		steer(clientID,alpha);
      //     	}
            int tg = 20;
            float step_size = 0.002;
            float J = 0.0;
            float epsilon = 0.1;
            int counter = 0;
            float beta[4] = {0.01, 0.01, 0.01, 0.01};
            float tuning_coeff = 1.0;
            float e = 0.01;

            control_parameters params;
            params.a1 = 0.1;
            params.b1 = 0.1;
            params.c1 = 0.1;
            params.a2 = 0.1;
            params.b2 = 0.1;
            params.c2 = 0.1;
            control control_signals;
            // control_signals.acceleration[tg];
            // control_signals.steer_vel[tg];
            state predicted_state;
            // predicted_state.x[tg];
            // predicted_state.y[tg];
            // predicted_state.heading[tg];
            // predicted_state.steer_angle[tg];
            // predicted_state.velocity[tg];
            // predicted_state.x[0] = curr_pos[1];
            // predicted_state.y[0] = curr_pos[2];
            float curr_heading = atan(curr_pos[2]/curr_pos[1]);
            // predicted_state.steer_angle[0] = curr_steerpos;
            // predicted_state.velocity[0] = curr_vel;


            terminal_states des_state;
            des_state.x = targetpos[0];
            des_state.y = targetpos[1];
            des_state.heading = atan(targetpos[1]/targetpos[0]);
            des_state.steer_angle = 0.0 ; // steer zero
            des_state.velocity = 0.0 ; // end vel zero

            

            do
            {	
            	compute_u(control_signals, params, tg, step_size);
            	
            	compute_v(predicted_state, control_signals, step_size, tg, curr_vel);
            	
				compute_steer(predicted_state, control_signals, step_size, tg, curr_steerpos);
				compute_states(predicted_state, step_size, tg, curr_pos[1], curr_pos[2], curr_heading);
				compute_j(des_state, predicted_state, J, beta, tg);

				compute_cor(e, params, control_signals, tg, step_size, predicted_state, des_state, tuning_coeff, curr_pos[0], curr_pos[1], curr_heading, curr_steerpos, curr_vel);
				std::cout << " print " << counter << std::endl;
				counter++;

            }while(J >= epsilon || counter < 20);


            // n = n+step;
            extApi_sleepMs(2);
            std::cout << " J " << J << std::endl;
            std::cout << " params " << params.a1 << std::endl;
            std::cout << " final state x : y " << predicted_state.x[tg]<<" , "<<predicted_state.y[tg] << std::endl;
            std::cout << " control_signals " << control_signals.acceleration[0] << " , "<< control_signals.steer_vel[0] << std::endl;
            //break;
        }
        simxFinish(clientID);
    }
    return(0);
}
