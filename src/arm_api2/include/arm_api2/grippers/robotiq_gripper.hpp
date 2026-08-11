/*******************************************************************************
*
 * Copyright (c) 2024, Crobotic Solutions d.o.o. (www.crobotics.tech)
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

/*      Title       : utils.hpp
 *      Project     : arm_api2
 *      Created     : 01/19/2025
 *      Author      : Filip Zoric
 *
 *      Description : Implementation of the gripper class for the Robotiq gripper
 */

#ifndef ROBOTIQ_GRIPPER_H
#define ROBOTIQ_GRIPPER_H

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <control_msgs/msg/gripper_command.hpp>
#include "gripper.hpp"
#include <rclcpp/executors.hpp>

class RobotiqGripper: public Gripper {

public:
    // Constructor
    RobotiqGripper(std::shared_ptr<rclcpp::Node> node) : node_(node){
        isOpen = false;
        gripper_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(node_,"robotiq_2f_urcap_adapter/gripper_command");
    };
    ~RobotiqGripper(){};

    // Method to open the gripper
    void open(){
        send_gripper_command(0.0); // open gripper
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Gripper opened.");
        isOpen = true;
    }; 

    // Method to close the gripper
    void close(){
        send_gripper_command(0.8); // close gripper
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Gripper closed.");
        isOpen = false; 
    };

    bool send_gripper_command(double position, double max_effort = 140.0)
    {
        if (!gripper_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Gripper action server not available!");
            return false;
        }
        auto goal = control_msgs::action::GripperCommand::Goal();
        goal.command.position = position;
        goal.command.max_effort = max_effort;
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending gripper command ...");
        /*auto result = gripper_client_->async_send_goal(goal);

        // Wait for the result
        if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to send gripper command");
            return;
        }

        std::shared_ptr<rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>> response = result.get();
        auto result_response = response->get_result();
        float pos = result_response.position;*/

        auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&RobotiqGripper::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&RobotiqGripper::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&RobotiqGripper::result_callback, this, std::placeholders::_1);

        gripper_client_->async_send_goal(goal, send_goal_options);

        is_done = false;

        while (!is_done) {
            //rclcpp::spin_some(node_);
        }

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper moved to position %f", last_position);
        return success;
    };

    float get_position(){
        return last_position;
    };

    float get_effort(){
        return last_effort;
    };

    bool is_stalled(){
        return last_stalled;
    };

    bool reached_goal(){
        return last_reached_goal;
    };

private:
    std::shared_ptr<rclcpp::Node> node_;
    bool isOpen;

    float last_position = 0.0;
    float last_effort = 0.0;
    bool last_stalled = false;
    bool last_reached_goal = false;
    bool success = false;
    bool is_done = false;

    // client
    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client_;

    void goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>> goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        std::shared_ptr<rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>> goal_handle,
        const std::shared_ptr<const control_msgs::action::GripperCommand::Feedback> feedback) {
        (void)goal_handle;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Got feedback: position = %f, effort = %f, stalled = %d, reached_goal = %d",
                    feedback->position, feedback->effort, feedback->stalled, feedback->reached_goal);
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal succeeded!");
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: position = %f, effort = %f, stalled = %d, reached_goal = %d",
                    result.result->position, result.result->effort, result.result->stalled, result.result->reached_goal);
                last_position = result.result->position;
                last_effort = result.result->effort;
                last_stalled = result.result->stalled;
                last_reached_goal = result.result->reached_goal;
                success = true;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was aborted");
                success = false;
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was canceled");
                success = false;
                break;
            default:
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown result code");
                break;
        }


        is_done = true;
    }

};

#endif // ROBOTIQ_GRIPPER_H