/*******************************************************************************
 * BSD 3-Clause License
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

/*      Title       : arm_joy.cpp
 *      Project     : arm_api2
 *      Created     : 14/11/2024
 *      Author      : Guanqi Chen
 *
 *      Description : Joystick control code for Servoing
 */

#include "arm_api2/arm_joy.hpp"

// Some constants used in the Servo Teleop demo
const std::string TWIST_TOPIC = "/arm/delta_twist_cmds";
const std::string JOINT_TOPIC = "/arm/delta_joint_cmds";
const std::string JOINT_STATE_TOPIC = "/joint_states";
const std::string GRIPPER_SERVICE = "robotiq_2f_urcap_adapter/gripper_command";
const size_t ROS_QUEUE_SIZE = 10;
const std::string EEF_FRAME_ID = "tcp";
const std::string BASE_FRAME_ID = "base_link";

JoyCtl::JoyCtl(): Node("joy_ctl")
{
    init();

    frame_to_publish_ = EEF_FRAME_ID;

    setScaleFactor(0.5); 

    enableJoy_ = false; 


}

void JoyCtl::init()
{

    // publishers
    cmdVelPub_ 		    = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE); 
    joint_pub_ 		    = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, ROS_QUEUE_SIZE);
    // subscribers
    joySub_ 		    = this->create_subscription<sensor_msgs::msg::Joy>("/joy", ROS_QUEUE_SIZE, std::bind(&JoyCtl::joy_callback, this, _1)); 
    joint_state_sub_    = this->create_subscription<sensor_msgs::msg::JointState>(
        JOINT_STATE_TOPIC, ROS_QUEUE_SIZE,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            (void)msg;
            joint_states_received_ = true;
            // save joint names
            joint_names_ = msg->name;
        });
    // client
    gripper_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(this,GRIPPER_SERVICE);
    RCLCPP_INFO(this->get_logger(), "Initialized joy_ctl with gripper control");
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
    RCLCPP_INFO(this->get_logger(), "Joint %d: %s", int(i + 1), joint_names_[i].c_str());
    }

    puts("Reading from Joystick");
    puts("---------------------------");
    puts("Hold 'LT' button to turn on/off the joystick control.");
    puts("Use 'LB' and 'RB' buttons to swticth between end effector frame and base frame - Default is end effector frame.");
    puts("Use 'A' or 'B' buttons to switch between joint space or task space control - Default is task space control.");
    puts("Use back and start buttons to accelerate and decelerate the motion.");
    puts("Use 'X' to close the gripper, 'Y' to open the gripper.");
    puts("------------For task space control:-------------");
    puts("  ");
    puts("Use 'right stick - left/right' for x-direction movement.");
    puts("Use 'right stick - up/down' for y-direction movement.");
    puts("Use 'left stick - up/down' for z-direction movement.");
    puts("Cross axes left|right for x rotation.");
    puts("Cross axes up|down for y rotation.");
    puts("Use 'left stick - left/right' for z rotation.");
    puts("  ");
    puts("------------For joint space control:------------");
    puts("  ");
    puts("Use 'left stick - left/right' for joint1.");
    puts("Use 'left stick - up/down' for joint2.");
    puts("Use 'cross - left/right' for joint3.");
    puts("Use 'cross - up/down' for joint4.");
    puts("Use 'right stick - left/right' for joint5.");
    puts("Use 'right stick - up/down' for joint6.");
    puts("  ");
 
}

void JoyCtl::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) 
{   
	float x_dir, y_dir, z_dir, yaw, pitch, roll;

    auto teleop_msg 	    = geometry_msgs::msg::TwistStamped();
    auto joint_msg          = control_msgs::msg::JointJog();

    // Enabling joystick functionality
    // xbox button pressed --> on/off switch
    int LOG_JOY_STATE_T = 5000; 
    if (msg->axes.at(2) == -1)
    { 
        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), clock_, LOG_JOY_STATE_T, "ON"); 
        setEnableJoy(true);
    }
    else if (msg->axes.at(2) == 1)
    {
        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), clock_, LOG_JOY_STATE_T, "OFF"); 
        setEnableJoy(false);
    }

    enableJoy_ = getEnableJoy();

    float sF = getScaleFactor();

    // Velocity control
    if (msg->buttons.at(7) == 1){
        
        if (sF > 0 && sF < 10)
        {
          sF += 0.1; 
          RCLCPP_INFO_STREAM(this->get_logger(), "Increasing scale factor: " << sF); 
        }
        else{sF = 0.5;}
    } // button right select
    if (msg->buttons.at(6) == 1){
       if (sF > 0 && sF < 10) 
       {
        sF -= 0.1; 
        RCLCPP_INFO_STREAM(this->get_logger(), "Decreasing scale factor: " << sF); 
       }
       else{sF = 0.5;}
    } // button left select

    // Frame selection
    if(msg->buttons.at(4)==1)
    {
        frame_to_publish_ = EEF_FRAME_ID;
        RCLCPP_INFO(this->get_logger(), "End Effector Frame Selected");
    }
    if(msg->buttons.at(5)==1)
    {
        frame_to_publish_ = BASE_FRAME_ID;
        RCLCPP_INFO(this->get_logger(), "Base Frame Selected");
    }
    
    // -----------Task Space------------
    if (msg->buttons.at(0) == 1)
    {
        task_space_ = true;
        joint_space_ = false;
        RCLCPP_INFO(this->get_logger(), "Task Space Control");
    }
    if(msg->buttons.at(1) == 1){
        joint_space_ = true;
        task_space_ = false;
        RCLCPP_INFO(this->get_logger(), "Joint Space Control");
    }


    // Motion control
    if (task_space_)
    {
        x_dir = msg->axes.at(3); // right joystick left/right
        y_dir = msg->axes.at(4); // right joystick up/down
        z_dir = msg->axes.at(1); // left joystick up/down
        yaw = msg->axes.at(0); // left joystick left/right, z rotation
        roll = msg->axes.at(6); // cross left/right, x rotation
        pitch = msg->axes.at(7); // cross up/down, y rotation

        joint_msg.joint_names.clear();
        joint_msg.velocities.push_back(0.0);
    }else if(joint_space_)
    {
        x_dir = 0;
        y_dir = 0;
        z_dir = 0;
        yaw = 0;
        roll = 0;
        pitch = 0;

        joint1_vel_cmd_ = msg->axes.at(0); // left joystick left/right
        joint2_vel_cmd_ = msg->axes.at(1); // left joystick up/down
        joint3_vel_cmd_ = msg->axes.at(6); // cross left/right
        joint4_vel_cmd_ = msg->axes.at(7); // cross up/down
        joint5_vel_cmd_ = msg->axes.at(3); // right joystick left/right
        joint6_vel_cmd_ = msg->axes.at(4); // right joystick up/down

        for(auto i=0u;i<joint_names_.size();i++)
        {
            joint_msg.joint_names.push_back(joint_names_[i]);
            joint_msg.velocities.push_back(
                (i == 0) ? joint2_vel_cmd_ * sF  :
                (i == 1) ? joint3_vel_cmd_ * sF  :
                (i == 2) ? joint4_vel_cmd_ * sF  :
                (i == 3) ? joint5_vel_cmd_ * sF  :
                (i == 4) ? joint6_vel_cmd_ * sF  :
                (i == 5) ? joint1_vel_cmd_ * sF : 0.0  // default to 0 if index out of range
            );
        }

    }

    // Gripper control
    if (msg->buttons.at(2) == 1)
    {
        send_gripper_command(0.8); // close gripper
    }
    if (msg->buttons.at(3) == 1)
    {
        send_gripper_command(0.0); // open gripper
    }

    setScaleFactor(sF); 

    // Modify the message
    teleop_msg.header.stamp = this->get_clock()->now();
    teleop_msg.header.frame_id = frame_to_publish_; 
    teleop_msg.twist.linear.x	= x_dir  * sF; 
    teleop_msg.twist.linear.y 	= y_dir   * sF;
    teleop_msg.twist.linear.z 	= z_dir   * sF; 
    teleop_msg.twist.angular.z  = yaw    * sF;
    teleop_msg.twist.angular.y  = pitch * sF;
    teleop_msg.twist.angular.x  = roll  * sF; 
    
    if (enableJoy_){
        if (joint_space_){
            joint_pub_->publish(joint_msg);
        }
        else{
            cmdVelPub_->publish(teleop_msg);
        }
    }
    else{
        teleop_msg.twist.linear.x = 0;
        teleop_msg.twist.linear.y = 0;
        teleop_msg.twist.linear.z = 0;
        teleop_msg.twist.angular.z = 0;
        teleop_msg.twist.angular.y = 0;
        teleop_msg.twist.angular.x = 0;
	    cmdVelPub_->publish(teleop_msg);

        joint_msg.joint_names.clear();
        joint_msg.velocities.push_back(0.0);
        joint_pub_->publish(joint_msg); 
    }


}

// Methods that set scale factor 
void JoyCtl::setScaleFactor(float value)
{
    scale_factor = value; 
}

float JoyCtl::getScaleFactor() const
{
    return scale_factor; 
}

void JoyCtl::setEnableJoy(bool val) 
{
    enableJoy_ = val; 
}

bool JoyCtl::getEnableJoy() const
{
    return enableJoy_; 
}

void JoyCtl::send_gripper_command(double position)
{
    if (!gripper_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Gripper action server not available!");
        return;
    }
    auto goal = control_msgs::action::GripperCommand::Goal();
    goal.command.position = position;
    goal.command.max_effort = 140.0;
    
    auto result = gripper_client_->async_send_goal(goal);
    RCLCPP_INFO(this->get_logger(), "Sending gripper command");
}