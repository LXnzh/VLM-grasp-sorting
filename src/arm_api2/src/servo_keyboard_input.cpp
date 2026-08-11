/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, KIT
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : servo_keyboard_input.cpp
 *      Project   : arm_api2
 *      Created   : 14/11/2024
 *      Author    : Guanqi Chen
 *      Description : Keyboard control code for Servoing
 */

// Code modified from https://github.com/moveit/moveit2_tutorials/blob/humble/doc/examples/realtime_servo/src/servo_keyboard_input.cpp

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <control_msgs/msg/gripper_command.hpp>

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

// Define used keys
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_PERIOD 0x2E
#define KEYCODE_SEMICOLON 0x3B
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72
#define KEYCODE_PLUS 0x2B
#define KEYCODE_MINUS 0x2D
#define KEYCODE_SPACE 0x20
#define KEYCODE_L 0x6C
#define KEYCODE_K 0x6B
#define KEYCODE_J 0x6A
#define KEYCODE_I 0x69
#define KEYCODE_M 0x6D
#define KEYCODE_COMMA 0x2C
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64

// Some constants used in the Servo Teleop demo
const std::string TWIST_TOPIC = "/arm/delta_twist_cmds";
const std::string JOINT_TOPIC = "/arm/delta_joint_cmds";
const std::string JOINT_STATE_TOPIC = "/joint_states";
const std::string GRIPPER_SERVICE = "robotiq_2f_urcap_adapter/gripper_command";
const size_t ROS_QUEUE_SIZE = 10;
const std::string EEF_FRAME_ID = "tcp";
const std::string BASE_FRAME_ID = "base_link";

// A class for reading the key inputs from the terminal
class KeyboardReader
{
public:
  KeyboardReader() : kfd(0)
  {
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
  }
  void readOne(char* c)
  {
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
  }
  void shutdown()
  {
    tcsetattr(kfd, TCSANOW, &cooked);
  }

private:
  int kfd;
  struct termios cooked;
};

// Converts key-presses to Twist or Jog commands for Servo, in lieu of a controller
class KeyboardServo
{
public:
  KeyboardServo();
  int keyLoop();
  void send_gripper_command(double position);

private:
  void spin();

  rclcpp::Node::SharedPtr nh_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client_;

  std::string frame_to_publish_;
  double joint_vel_cmd_;
  double vel_scale_;
  bool joint_states_received_;
  // list of joint names
  std::vector<std::string> joint_names_;
};

KeyboardServo::KeyboardServo() : frame_to_publish_(BASE_FRAME_ID), joint_vel_cmd_(1.0), vel_scale_(0.5), joint_states_received_(false)
{
  nh_ = rclcpp::Node::make_shared("servo_keyboard_input");

  twist_pub_ = nh_->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
  joint_pub_ = nh_->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, ROS_QUEUE_SIZE);
  joint_state_sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>(
      JOINT_STATE_TOPIC, ROS_QUEUE_SIZE,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        (void)msg;
        joint_states_received_ = true;
        // save joint names
        joint_names_ = msg->name;
      });
  gripper_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(nh_,GRIPPER_SERVICE);
}

KeyboardReader input;

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  KeyboardServo keyboard_servo;

  signal(SIGINT, quit);

  int rc = keyboard_servo.keyLoop();
  input.shutdown();
  rclcpp::shutdown();

  return rc;
}

void KeyboardServo::spin()
{
  while (rclcpp::ok())
  {
    rclcpp::spin_some(nh_);
  }
}

int KeyboardServo::keyLoop()
{
  char c;
  bool publish_twist = false;
  bool publish_joint = false;

  std::thread{ std::bind(&KeyboardServo::spin, this) }.detach();

  while(!joint_states_received_)
  {
    RCLCPP_INFO(nh_->get_logger(), "Waiting for joint states...");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  // print joint names with index (starting from 1)
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    RCLCPP_INFO(nh_->get_logger(), "Joint %d: %s", int(i + 1), joint_names_[i].c_str());
  }

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys and the '.' and ';' keys to Cartesian jog");
  puts("Use 'I' and 'K' to pitch(y rotate), 'J' and 'L' to yaw(z rotate), and 'M' and ',' to roll(x rotate)");
  puts("Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame");
  puts("Use 1|2|3|4|5|6|7 keys to joint jog. 'R' to reverse the direction of jogging.");
  puts("Use '+' and '-' to increase/decrease the speed.");
  puts("Use 'A' to close the gripper, 'D' to open the gripper.");
  puts("Press Space to stop all motions.");
  puts("'Q' to quit.");
  puts("  ");

  for (;;)
  {
    // get the next event from the keyboard
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error&)
    {
      perror("read():");
      return -1;
    }

    RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X\n", c);

    // // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();


    // Use read key-press
    switch (c)
    {
      // Cartesian motions
      case KEYCODE_LEFT:
        RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
        twist_msg->twist.linear.y = -1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_RIGHT:
        RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
        twist_msg->twist.linear.y = 1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_UP:
        RCLCPP_DEBUG(nh_->get_logger(), "UP");
        twist_msg->twist.linear.x = 1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_DOWN:
        RCLCPP_DEBUG(nh_->get_logger(), "DOWN");
        twist_msg->twist.linear.x = -1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_PERIOD:
        RCLCPP_DEBUG(nh_->get_logger(), "PERIOD");
        twist_msg->twist.linear.z = -1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_SEMICOLON:
        RCLCPP_DEBUG(nh_->get_logger(), "SEMICOLON");
        twist_msg->twist.linear.z = 1.0 * vel_scale_;
        publish_twist = true;
        break;
      
      // rotational motions
      case KEYCODE_J:
        RCLCPP_DEBUG(nh_->get_logger(), "J");
        twist_msg->twist.angular.z = 1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_L:
        RCLCPP_DEBUG(nh_->get_logger(), "L");
        twist_msg->twist.angular.z = -1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_I:
        RCLCPP_DEBUG(nh_->get_logger(), "I");
        twist_msg->twist.angular.y = 1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_K:
        RCLCPP_DEBUG(nh_->get_logger(), "K");
        twist_msg->twist.angular.y = -1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_M:
        RCLCPP_DEBUG(nh_->get_logger(), "M");
        twist_msg->twist.angular.x = 1.0 * vel_scale_;
        publish_twist = true;
        break;
      case KEYCODE_COMMA:
        RCLCPP_DEBUG(nh_->get_logger(), "COMMA");
        twist_msg->twist.angular.x = -1.0 * vel_scale_;
        publish_twist = true;
        break;

      // Frame selection
      case KEYCODE_E:
        RCLCPP_DEBUG(nh_->get_logger(), "E");
        frame_to_publish_ = EEF_FRAME_ID;
        break;
      case KEYCODE_W:
        RCLCPP_DEBUG(nh_->get_logger(), "W");
        frame_to_publish_ = BASE_FRAME_ID;
        break;

      // Joint motions
      case KEYCODE_1:
        RCLCPP_INFO(nh_->get_logger(), "1");
        joint_msg->joint_names.push_back(joint_names_[0]);
        joint_msg->velocities.push_back(joint_vel_cmd_ * vel_scale_);
        RCLCPP_INFO(nh_->get_logger(), "Now is controlling: %s", joint_msg->joint_names[0].c_str());
        publish_joint = true;
        break;
      case KEYCODE_2:
        RCLCPP_INFO(nh_->get_logger(), "2");
        joint_msg->joint_names.push_back(joint_names_[1]);
        joint_msg->velocities.push_back(joint_vel_cmd_ * vel_scale_);
        publish_joint = true;
        RCLCPP_INFO(nh_->get_logger(), "Now is controlling: %s", joint_msg->joint_names[0].c_str());
        break;
      case KEYCODE_3:
        RCLCPP_INFO(nh_->get_logger(), "3");
        joint_msg->joint_names.push_back(joint_names_[2]);
        joint_msg->velocities.push_back(joint_vel_cmd_ * vel_scale_);
        RCLCPP_INFO(nh_->get_logger(), "Now is controlling: %s", joint_msg->joint_names[0].c_str());
        publish_joint = true;
        break;
      case KEYCODE_4:
        RCLCPP_INFO(nh_->get_logger(), "4");
        joint_msg->joint_names.push_back(joint_names_[3]);
        joint_msg->velocities.push_back(joint_vel_cmd_ * vel_scale_);
        RCLCPP_INFO(nh_->get_logger(), "Now is controlling: %s", joint_msg->joint_names[0].c_str());
        publish_joint = true;
        break;
      case KEYCODE_5:
        RCLCPP_INFO(nh_->get_logger(), "5");
        joint_msg->joint_names.push_back(joint_names_[4]);
        joint_msg->velocities.push_back(joint_vel_cmd_ * vel_scale_);
        RCLCPP_INFO(nh_->get_logger(), "Now is controlling: %s", joint_msg->joint_names[0].c_str());
        publish_joint = true;
        break;
      case KEYCODE_6:
        RCLCPP_INFO(nh_->get_logger(), "6");
        joint_msg->joint_names.push_back(joint_names_[5]);
        joint_msg->velocities.push_back(joint_vel_cmd_ * vel_scale_);
        RCLCPP_INFO(nh_->get_logger(), "Now is controlling: %s", joint_msg->joint_names[0].c_str());
        publish_joint = true;
        break;
      case KEYCODE_7:
        if (joint_names_.size() < 7)
        {
          RCLCPP_WARN(nh_->get_logger(), "Not enough joints for key 7");
          break;
        }
        RCLCPP_INFO(nh_->get_logger(), "7");
        joint_msg->joint_names.push_back(joint_names_[6]);
        joint_msg->velocities.push_back(joint_vel_cmd_ * vel_scale_);
        RCLCPP_INFO(nh_->get_logger(), "Now is controlling: %s", joint_msg->joint_names[0].c_str());
        publish_joint = true;
        break;
      case KEYCODE_R:
        RCLCPP_INFO(nh_->get_logger(), "R");
        joint_vel_cmd_ *= -1;
        break;
      
      // Stop actions
      case KEYCODE_Q:
        RCLCPP_INFO(nh_->get_logger(), "quit");
        return 0;
      case KEYCODE_SPACE:
        RCLCPP_INFO(nh_ -> get_logger(), "STOP");
        twist_msg->twist.linear.x = 0.0;
        twist_msg->twist.linear.y = 0.0;
        twist_msg->twist.linear.z = 0.0;
        twist_msg->twist.angular.x = 0.0;
        twist_msg->twist.angular.y = 0.0;
        twist_msg->twist.angular.z = 0.0;
        joint_msg->joint_names.clear();
        joint_msg->velocities.push_back(0.0);
        twist_pub_->publish(std::move(twist_msg));
        joint_pub_->publish(std::move(joint_msg));
        publish_twist = false;
        publish_joint = false;
        break;

      // Velocity control
      case KEYCODE_PLUS:
        RCLCPP_DEBUG(nh_->get_logger(), "PLUS");
        vel_scale_ += 0.01;
        RCLCPP_INFO(nh_->get_logger(), "Velocity scale: %f", vel_scale_);
        break;
      case KEYCODE_MINUS:
        RCLCPP_DEBUG(nh_->get_logger(), "MINUS");
        vel_scale_ -= 0.01;
        if (vel_scale_ < 0.01)
        {
          vel_scale_ = 0.01;
        }
        RCLCPP_INFO(nh_->get_logger(), "Velocity scale: %f", vel_scale_);
        break;

      // Gripper control
      case KEYCODE_A:
        RCLCPP_INFO(nh_->get_logger(), "close gripper");
        send_gripper_command(0.8); // close gripper
        break;
      case KEYCODE_D:
        RCLCPP_INFO(nh_->get_logger(), "open gripper");
        send_gripper_command(0.0); // open gripper
        break;
    }

    // If a key requiring a publish was pressed, publish the message now
    if (publish_twist)
    {
      twist_msg->header.stamp = nh_->now();
      twist_msg->header.frame_id = frame_to_publish_;
      twist_pub_->publish(std::move(twist_msg));
      publish_twist = false;
    }
    else if (publish_joint)
    {
      joint_msg->header.stamp = nh_->now();
      joint_msg->header.frame_id = BASE_FRAME_ID;
      joint_pub_->publish(std::move(joint_msg));
      publish_joint = false;
    }
  }
  return 0;
}

void KeyboardServo::send_gripper_command(double position)
{
    if (!gripper_client_->wait_for_action_server(std::chrono::seconds(5))) {
        return;
    }
    auto goal = control_msgs::action::GripperCommand::Goal();
    goal.command.position = position;
    goal.command.max_effort = 140.0;
    
    auto result = gripper_client_->async_send_goal(goal);
}
