/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <geometry_msgs/Twist.h>
#include <iostream>
#include "convert.h"


using namespace UNITREE_LEGGED_SDK;

unitree_legged_msgs::HighCmd SendHighROS;
unitree_legged_msgs::HighState RecvHighROS;


template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}


void control_callback(const geometry_msgs::Twist& cmd_vel)
{
	ROS_INFO("Linear Components:[%f,%f,%f]", cmd_vel.linear.x,  cmd_vel.linear.y,  cmd_vel.linear.z);
	ROS_INFO("Angular Components:[%f,%f,%f]",cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z);
	
	SendHighROS.velocity[0] = cmd_vel.linear.x;
	SendHighROS.velocity[1] = cmd_vel.linear.y;
	SendHighROS.yawSpeed = cmd_vel.angular.z;

	if(SendHighROS.yawSpeed > 1.5){
		SendHighROS.yawSpeed = 1.5;
        	ROS_INFO("Angular Components changed:[%f]", SendHighROS.yawSpeed);

	} else if (SendHighROS.yawSpeed <-1.5){
		SendHighROS.yawSpeed = -1.5;
        	ROS_INFO("Angular Components changed:[%f]", SendHighROS.yawSpeed);

	}else {

	}

        if(SendHighROS.velocity[0] >0.3){
                SendHighROS.velocity[0] = 0.3;
                ROS_INFO("Linear Components changed:[%f]", SendHighROS.velocity[0]);

        } else if (SendHighROS.velocity[0] <-0.3){
                SendHighROS.velocity[0] = -0.3;
                ROS_INFO("Linear Components changed:[%f]", SendHighROS.velocity[0]);

        }else {

        }

        if(SendHighROS.velocity[1] >0.2){
                SendHighROS.velocity[1] = 0.2;
                ROS_INFO("Linear Components changed:[%f]", SendHighROS.velocity[1]);

        } else if (SendHighROS.velocity[1] <-0.2){
                SendHighROS.velocity[1] = -0.2;
                ROS_INFO("Linear Components changed:[%f]", SendHighROS.velocity[1]);

        }else {

        }

}



template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl;
         //     << "Make sure the robot is standing on the ground." << std::endl
          //    << "Press Enter to continue..." << std::endl;
    //std::cin.ignore();

    ros::Rate loop_rate(500);
	
    TCmd SendHighLCM = {0};
    TState RecvHighLCM = {0};

    SendHighROS.mode = 0;
    //SendHighROS.gaitType = 0;
    SendHighROS.speedLevel = 0;
    SendHighROS.footRaiseHeight = 0;
    SendHighROS.bodyHeight = 0;
    SendHighROS.euler[0] = 0;
    SendHighROS.euler[1] = 0;
    SendHighROS.euler[2] = 0;
    SendHighROS.velocity[0] = 0.0f;
    SendHighROS.velocity[1] = 0.0f;
    SendHighROS.yawSpeed = 0.0f;
    SendHighROS.reserve = 0;
    
    roslcm.SubscribeState();
    
    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

	while (ros::ok())
	{
		roslcm.Get(RecvHighLCM); // receive the A1's message.
		RecvHighROS = ToRos(RecvHighLCM);
	    

        	//if(RecvHighROS.velocity[1] != 0)
        	//{
            	//	SendHighROS.velocity[1] = RecvHighROS.velocity[1]>0 ? -0.001:0.001;	
		//}

        // Just give a simple example: receive the velocity.
		if(SendHighROS.velocity[0] != 0 
		    || SendHighROS.velocity[1] != 0 
		    || SendHighROS.yawSpeed !=0)
		{
    			SendHighROS.gaitType = 1;
			SendHighROS.mode = 2;
		}
		else
		{
    			SendHighROS.gaitType = 0;
			SendHighROS.mode =1;
		}
        //memcpy(&SendHighLCM, &SendHighROS, sizeof(HighCmd));
		SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
        roslcm.Send(SendHighLCM);
        ros::spinOnce();
        loop_rate.sleep(); 
    }
    return 0;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "base_controller_node");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/cmd_vel", 1, control_callback);

    UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);

}

