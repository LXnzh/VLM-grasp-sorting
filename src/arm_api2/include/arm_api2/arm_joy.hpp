/*******************************************************************************
*
 * Copyright (c) 2024, Crobotic Solutions d.o.o.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*      Title       : moveit2_iface.cpp
 *      Project     : arm_api2
 *      Created     : 05/10/2024
 *      Author      : Filip Zoric
 *
 *      Description : Header for the joystick control class. 
 */


#ifndef ARM_JOY_H
#define ARM_JOY_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

//* ros 
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

//* msgs
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <control_msgs/msg/joint_jog.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <control_msgs/msg/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "geometry_msgs/msg/twist_stamped.hpp"

//* srvs
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp" 

using namespace std::chrono_literals; 
using std::placeholders::_1; 
using std::placeholders::_2; 

class JoyCtl: public rclcpp::Node 
{
	public:
		JoyCtl(); 

	private:

		// vars
		bool 		enableJoy_; 
		mutable float scale_factor;  
        rclcpp::Clock clock_;
		std::string frame_to_publish_;
		bool joint_states_received_ = false;
		double joint_vel_cmd_ = 1.0;
		double joint1_vel_cmd_;
		double joint2_vel_cmd_;
		double joint3_vel_cmd_;
		double joint4_vel_cmd_;
		double joint5_vel_cmd_;
		double joint6_vel_cmd_;
		bool task_space_ = true;
		bool joint_space_ = false;
		// list of joint names
		std::vector<std::string> joint_names_; 
	    
		// publishers	
		rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmdVelPub_;
		rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
		
		// subscribers
		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySub_; 
		rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
		// clients 
 		rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client_;

		void init(); 
		void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg); 

		// Setting them as const to be usable by joy_callback which is also const
		void setScaleFactor(float value); 
		float getScaleFactor() const; 
		void setEnableJoy(bool val); 
		bool getEnableJoy() const; 

		// TODO: Add service to turn joystick on and off
		void send_gripper_command(double position);
};

#endif