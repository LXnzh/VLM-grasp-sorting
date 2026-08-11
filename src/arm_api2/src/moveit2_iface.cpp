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

/*      Title       : moveit2_iface.cpp
 *      Project     : arm_api2
 *      Created     : 05/10/2024
 *      Author      : Filip Zoric
 *
 *      Description : The core robot manipulator and MoveIt2! ROS 2 interfacing header class.
 */

#include "arm_api2/moveit2_iface.hpp"
#include <future>
#include <vector>
#include <tf2_eigen/tf2_eigen.hpp>

m2Iface::m2Iface(const rclcpp::NodeOptions &options)
    : Node("moveit2_iface", options), node_(std::make_shared<rclcpp::Node>("moveit2_iface_node")), 
     executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()), gripper(node_) 
{   
    this->get_parameter("config_path", config_path);
    this->get_parameter("enable_servo", enable_servo); 
    this->get_parameter("dt", dt); 

    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded config!");

    // TODO: Add as reconfigurable param 
    std::chrono::duration<double> SYSTEM_DT(dt);
    timer_ = this->create_wall_timer(SYSTEM_DT, std::bind(&m2Iface::run, this));

    std::chrono::duration<double> interval(1.0 / 200.0);
    servo_timer_ = this->create_wall_timer(interval, std::bind(&m2Iface::servo_loop_cb, this));

    rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    last_target_time_ = steady_clock.now();


    // Load arm basically --> two important params
    // Manual param specification --> https://github.com/moveit/moveit2_tutorials/blob/8eaef05bfbabde3f35910ad054a819d79e70d3fc/doc/tutorials/quickstart_in_rviz/launch/demo.launch.py#L105
    config              = init_config(config_path);  
    PLANNING_GROUP      = config["robot"]["arm_name"].as<std::string>(); 
    EE_LINK_NAME        = config["robot"]["ee_link_name"].as<std::string>();
    ROBOT_DESC          = config["robot"]["robot_desc"].as<std::string>();  
    PLANNING_FRAME      = config["robot"]["planning_frame"].as<std::string>(); 
    PLANNING_SCENE      = config["robot"]["planning_scene"].as<std::string>(); 
    MOVE_GROUP_NS       = config["robot"]["move_group_ns"].as<std::string>(); 
    JOINT_STATES        = config["robot"]["joint_states"].as<std::string>();
    WITH_PLANNER        = config["robot"]["with_planner"].as<bool>();
    max_vel_scaling_factor = config["robot"]["max_vel_scaling_factor"].as<float>();
    max_acc_scaling_factor = config["robot"]["max_acc_scaling_factor"].as<float>();
    
    // Currently not used :) 
    ns_ = this->get_namespace(); 	
    init_publishers(); 
    init_subscribers(); 
    init_services(); 
    init_moveit(); 
    init_actionservers();
    if (enable_servo) {servoPtr = init_servo();};
    
    // TF2 listener
    tfBufferPtr = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transformListenerPtr = std::make_shared<tf2_ros::TransformListener>(*tfBufferPtr);

    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized node!"); 

    // Init anything for the old pose because it is non-existent at the beggining
    nodeInit = true; 
}

YAML::Node m2Iface::init_config(std::string yaml_path)
{   
    RCLCPP_INFO_STREAM(this->get_logger(), "Config yaml path is: " << yaml_path); 
    return YAML::LoadFile(yaml_path);
}

void m2Iface::init_publishers()
{   
    auto pose_state_name = config["topic"]["pub"]["current_pose"]["name"].as<std::string>();
    auto robot_state_name = config["topic"]["pub"]["current_robot_state"]["name"].as<std::string>(); 
    auto joint_command_name = config["topic"]["pub"]["joint_command"]["name"].as<std::string>();
    pose_state_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(ns_ + pose_state_name, 1);
    robot_state_pub_ = this->create_publisher<std_msgs::msg::String>(ns_ + robot_state_name, 1);
    joint_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(joint_command_name, 10); 

    if (config["topic"]["pub"]["joint_command_traj"] && config["topic"]["pub"]["joint_command_traj"]["name"])
    {
        auto joint_command_traj_name = config["topic"]["pub"]["joint_command_traj"]["name"].as<std::string>();
        joint_command_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(joint_command_traj_name, 10);
        has_joint_command_traj_topic_ = true;
        RCLCPP_INFO_STREAM(this->get_logger(), "Initialized optional joint_command_traj publisher: " << joint_command_traj_name);
    }
    else
    {
        has_joint_command_traj_topic_ = false;
        RCLCPP_INFO_STREAM(this->get_logger(), "joint_command_traj not defined in config. Using joint_command only.");
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized publishers!");
}

void m2Iface::init_subscribers()
{
    auto joint_states_name = config["topic"]["sub"]["joint_states"]["name"].as<std::string>();
    auto target_pose_name = config["topic"]["sub"]["target_pose"]["name"].as<std::string>();
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(ns_ + joint_states_name, 1, std::bind(&m2Iface::joint_state_cb, this, _1));
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(ns_ + target_pose_name, rclcpp::QoS(10), std::bind(&m2Iface::target_pose_cb, this, _1));
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized subscribers!");
}

void m2Iface::init_services()
{
    auto change_state_name = config["srv"]["change_robot_state"]["name"].as<std::string>(); 
    auto set_vel_acc_name = config["srv"]["set_vel_acc"]["name"].as<std::string>();
    auto set_eelink_name = config["srv"]["set_eelink"]["name"].as<std::string>();
    auto set_plan_only_name = config["srv"]["set_planonly"]["name"].as<std::string>();
    change_state_srv_ = this->create_service<arm_api2_msgs::srv::ChangeState>(ns_ + change_state_name, std::bind(&m2Iface::change_state_cb, this, _1, _2)); 
    set_vel_acc_srv_ = this->create_service<arm_api2_msgs::srv::SetVelAcc>(ns_ + set_vel_acc_name, std::bind(&m2Iface::set_vel_acc_cb, this, _1, _2));
    set_eelink_srv_ = this->create_service<arm_api2_msgs::srv::SetStringParam>(ns_ + set_eelink_name, std::bind(&m2Iface::set_eelink_cb, this, _1, _2));
    set_plan_only_srv_ = this->create_service<std_srvs::srv::SetBool>(ns_ + set_plan_only_name, std::bind(&m2Iface::set_plan_only_cb, this, _1, _2));
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized services!"); 
}

void m2Iface::init_actionservers()
{
    auto move_to_pose_name = config["action"]["move_to_pose"]["name"].as<std::string>();
    auto move_to_joint_name = config["action"]["move_to_joint"]["name"].as<std::string>();
    auto move_to_pose_path_name = config["action"]["move_to_pose_path"]["name"].as<std::string>();
    auto gripper_control_name = config["action"]["gripper_control"]["name"].as<std::string>();
    move_to_pose_as_ = rclcpp_action::create_server<arm_api2_msgs::action::MoveCartesian>(this,
                                                                                        ns_ + move_to_pose_name,
                                                                                        std::bind(&m2Iface::move_to_pose_goal_cb, this, _1, _2),
                                                                                        std::bind(&m2Iface::move_to_pose_cancel_cb, this, _1),
                                                                                        std::bind(&m2Iface::move_to_pose_accepted_cb, this, _1));
    move_to_joint_as_ = rclcpp_action::create_server<arm_api2_msgs::action::MoveJoint>(this,
                                                                                        ns_ + move_to_joint_name,
                                                                                        std::bind(&m2Iface::move_to_joint_goal_cb, this, _1, _2),
                                                                                        std::bind(&m2Iface::move_to_joint_cancel_cb, this, _1),
                                                                                        std::bind(&m2Iface::move_to_joint_accepted_cb, this, _1));
    move_to_pose_path_as_ = rclcpp_action::create_server<arm_api2_msgs::action::MoveCartesianPath>(this,
                                                                                        ns_ + move_to_pose_path_name,
                                                                                        std::bind(&m2Iface::move_to_pose_path_goal_cb, this, _1, _2),
                                                                                        std::bind(&m2Iface::move_to_pose_path_cancel_cb, this, _1),
                                                                                        std::bind(&m2Iface::move_to_pose_path_accepted_cb, this, _1));
    gripper_control_as_ = rclcpp_action::create_server<control_msgs::action::GripperCommand>(this,
                                                                                        ns_ + gripper_control_name,
                                                                                        std::bind(&m2Iface::gripper_control_goal_cb, this, _1, _2),
                                                                                        std::bind(&m2Iface::gripper_control_cancel_cb, this, _1),
                                                                                        std::bind(&m2Iface::gripper_control_accepted_cb, this, _1));

    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized action servers!");
}
void m2Iface::init_moveit()
{

    RCLCPP_INFO_STREAM(this->get_logger(), "robot_description: " << ROBOT_DESC); 
    RCLCPP_INFO_STREAM(this->get_logger(), "planning_group: " << PLANNING_GROUP);
    RCLCPP_INFO_STREAM(this->get_logger(), "planning_frame: " << PLANNING_FRAME); 
    RCLCPP_INFO_STREAM(this->get_logger(), "move_group_ns: " << MOVE_GROUP_NS);  
    // MoveIt related things!
    moveGroupInit       = setMoveGroup(node_, PLANNING_GROUP, MOVE_GROUP_NS); 
    pSceneMonitorInit   = setPlanningSceneMonitor(node_, ROBOT_DESC);
    robotModelInit      = setRobotModel(node_);
}

// TODO: Try to replace with auto
std::unique_ptr<moveit_servo::Servo> m2Iface::init_servo()
{   
    auto nodeParameters = node_->get_node_parameters_interface(); 
    auto servoParams = moveit_servo::ServoParameters::makeServoParameters(node_); 
    RCLCPP_INFO_STREAM(this->get_logger(), "ee_frame_name: " << servoParams->ee_frame_name);  
    servoParams->get("moveit_servo", nodeParameters);


    //auto servoParamsPtr = std::make_shared<moveit_servo::ServoParameters>(std::move(servoParams));
    //auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_); 
    // Servo parameters need to bee constSharedPtr
    auto servo = std::make_unique<moveit_servo::Servo>(node_, servoParams, m_pSceneMonitorPtr); 
    RCLCPP_INFO(this->get_logger(), "Servo initialized!"); 
    return servo;
}

void m2Iface::joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{   
    std::vector<std::string> jointNames = msg->name;
    std::vector<double> jointPositions = msg->position;
    m_currJointNames = jointNames;
    if(robotModelInit) {m_robotStatePtr->setVariablePositions(jointNames, jointPositions);}; 

}

void m2Iface::target_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (msg->header.frame_id != PLANNING_FRAME)
    {
        RCLCPP_WARN(this->get_logger(), "Target pose in frame [%s], expected [%s]",
                    msg->header.frame_id.c_str(), PLANNING_FRAME.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Received new target pose for servo control.");
    move_to_pose_servo(msg->pose);
}

void m2Iface::set_vel_acc_cb(const std::shared_ptr<arm_api2_msgs::srv::SetVelAcc::Request> req, const std::shared_ptr<arm_api2_msgs::srv::SetVelAcc::Response> res)
{
    if(req->max_vel < 0 || req->max_acc < 0 || req->max_vel > 1 || req->max_acc > 1)
    {
        res->success = false;
        RCLCPP_ERROR_STREAM(this->get_logger(), "Velocity and acceleration must be in the range [0, 1]!");
        return;
    }
    max_vel_scaling_factor = float(req->max_vel);
    max_acc_scaling_factor = float(req->max_acc);
    res->success = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "Set velocity and acceleration to " << max_vel_scaling_factor << " " << max_acc_scaling_factor);

}

void m2Iface::set_eelink_cb(const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Request> req, const std::shared_ptr<arm_api2_msgs::srv::SetStringParam::Response> res)
{
    EE_LINK_NAME = req->value;
    m_moveGroupPtr->setEndEffectorLink(EE_LINK_NAME);
    res->success = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "Set end effector link to " << req->value);
}

void m2Iface::set_plan_only_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
    planOnly = req->data;
    res->success = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "Set plan only to " << req->data);
}

rclcpp_action::GoalResponse m2Iface::move_to_joint_goal_cb(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const arm_api2_msgs::action::MoveJoint::Goal> goal)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request for joint control! Number of joints: "  << goal->joint_state.position.size());
    if(robotState != JOINT_TRAJ_CTL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Robot is not in joint control mode!");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if(goal->joint_state.position.size() != m_currJointPosition.size())
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Number of joint positions does not match the number of joints!");
        return rclcpp_action::GoalResponse::REJECT;
    }
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse m2Iface::move_to_joint_cancel_cb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveJoint>> goal_handle)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received request to cancel joint control!");
    (void)goal_handle;
    m_moveGroupPtr->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void m2Iface::move_to_joint_accepted_cb(std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveJoint>> goal_handle)
{
    m_moveToJointGoalHandle_ = goal_handle;
    recivCmd = true;
}

rclcpp_action::GoalResponse m2Iface::move_to_pose_goal_cb(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const arm_api2_msgs::action::MoveCartesian::Goal> goal)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request for Cartesian control!");
    if(robotState != CART_TRAJ_CTL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Robot is not in Cartesian control mode!");
        return rclcpp_action::GoalResponse::REJECT;
    }
    //if(goal->goal.header.frame_id != PLANNING_FRAME)
    //{
    //    RCLCPP_ERROR_STREAM(this->get_logger(), "Pose frame_id is not planning frame! PLANNING_FRAME: " << PLANNING_FRAME);
    //    return rclcpp_action::GoalResponse::REJECT;
    //}
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse m2Iface::move_to_pose_cancel_cb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesian>> goal_handle)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received request to cancel Cartesian control!");
    (void)goal_handle;
    m_moveGroupPtr->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void m2Iface::move_to_pose_accepted_cb(std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesian>> goal_handle)
{
    m_moveToPoseGoalHandle_ = goal_handle;
    recivCmd = true;
}

rclcpp_action::GoalResponse m2Iface::move_to_pose_path_goal_cb(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const arm_api2_msgs::action::MoveCartesianPath::Goal> goal)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request for Cartesian path control!");
    if(robotState != CART_TRAJ_CTL)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Robot is not in Cartesian control mode!");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if(goal->poses.size() < 2)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Number of poses in path is less than 2!");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if(goal->poses[0].header.frame_id != PLANNING_FRAME)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Path frame_id is not planning frame! PLANNING_FRAME: " << PLANNING_FRAME);
        return rclcpp_action::GoalResponse::REJECT;
    }
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse m2Iface::move_to_pose_path_cancel_cb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesianPath>> goal_handle)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received request to cancel Cartesian path control!");
    (void)goal_handle;
    m_moveGroupPtr->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void m2Iface::move_to_pose_path_accepted_cb(std::shared_ptr<rclcpp_action::ServerGoalHandle<arm_api2_msgs::action::MoveCartesianPath>> goal_handle)
{
    m_moveToPosePathGoalHandle_ = goal_handle;
    recivTraj = true;
}

rclcpp_action::GoalResponse m2Iface::gripper_control_goal_cb(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const control_msgs::action::GripperCommand::Goal> goal)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request for gripper control!");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse m2Iface::gripper_control_cancel_cb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle)
{
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void m2Iface::gripper_control_accepted_cb(std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle)
{
    m_gripperControlGoalHandle_ = goal_handle;
    recivGripperCmd = true;
}

void m2Iface::change_state_cb(const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Request> req, 
                              const std::shared_ptr<arm_api2_msgs::srv::ChangeState::Response> res)
{
    auto itr = std::find(std::begin(stateNames), std::end(stateNames), req->state); 
    
    if ( itr != std::end(stateNames))
    {
        int wantedIndex_ = std::distance(stateNames, itr); 
        robotState  = (state)wantedIndex_; 
        RCLCPP_INFO_STREAM(this->get_logger(), "Switching state!");
        res->success = true;  
    }else{
        RCLCPP_INFO_STREAM(this->get_logger(), "Failed switching to state " << req->state); 
        res->success = false; 
    } 
}

bool m2Iface::setMoveGroup(rclcpp::Node::SharedPtr nodePtr, std::string groupName, std::string moveNs)
{
    // check if moveNs is empty
    if (moveNs == "null") moveNs=""; 

    //https://github.com/moveit/moveit2/issues/496
    m_moveGroupPtr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(nodePtr, 
        moveit::planning_interface::MoveGroupInterface::Options(
            groupName,
            "robot_description",
            moveNs));

    double POS_TOL = 0.0000001; 
    // set move group stuff
    m_moveGroupPtr->setEndEffectorLink(EE_LINK_NAME); 
    m_moveGroupPtr->setPoseReferenceFrame(PLANNING_FRAME); 

    m_moveGroupPtr->setGoalPositionTolerance(POS_TOL);
    m_moveGroupPtr->startStateMonitor(); 

    // velocity scaling
    m_moveGroupPtr->setMaxVelocityScalingFactor(0.05);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(0.05);
    // executor
    executor_->add_node(node_); 
    executor_thread_ = std::thread([this]() {executor_->spin();});
    RCLCPP_INFO_STREAM(this->get_logger(), "Move group interface set up!"); 
    return true; 
}

/* This is not neccessary*/
bool m2Iface::setRobotModel(rclcpp::Node::SharedPtr nodePtr)
{
    robot_model_loader::RobotModelLoader robot_model_loader(nodePtr);
    kinematic_model = robot_model_loader.getModel(); 
    // Find nicer way to do this
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    m_robotStatePtr = kinematic_state;
    m_robotStatePtr->setToDefaultValues();
    RCLCPP_INFO_STREAM(this->get_logger(), "Robot model loaded!");
    RCLCPP_INFO_STREAM(this->get_logger(), "Robot model frame is: " << kinematic_model->getModelFrame().c_str());
    return true;
}

bool m2Iface::setPlanningSceneMonitor(rclcpp::Node::SharedPtr nodePtr, std::string name)
{
    // https://moveit.picknik.ai/main/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html
    // https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/planning_scene/src/planning_scene_tutorial.cpp
    m_pSceneMonitorPtr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(nodePtr, name); 
    m_pSceneMonitorPtr->startSceneMonitor(PLANNING_SCENE); 
    if (m_pSceneMonitorPtr->getPlanningScene())
    {
        m_pSceneMonitorPtr->startStateMonitor(JOINT_STATES); 
        m_pSceneMonitorPtr->setPlanningScenePublishingFrequency(25);
        m_pSceneMonitorPtr->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                         "/moveit_servo/publish_planning_scene");
        m_pSceneMonitorPtr->startSceneMonitor(); 
        m_pSceneMonitorPtr->providePlanningSceneService(); 
    }
    else 
    {
        RCLCPP_ERROR(this->get_logger(), "Planning scene not configured!"); 
        return EXIT_FAILURE; 
    }
    
    //TODO: Check what's difference between planning_Scene and planning_scene_monitor
    RCLCPP_INFO_STREAM(this->get_logger(), "Created planning scene monitor!");
    return true; 
}

void m2Iface::planAndExecJoint()
{   
    const auto feedback = std::make_shared<arm_api2_msgs::action::MoveJoint::Feedback>();
    const auto result = std::make_shared<arm_api2_msgs::action::MoveJoint::Result>();
    feedback->set__status("planning");
    m_moveToJointGoalHandle_->publish_feedback(feedback);

    const auto goalJointState = m_moveToJointGoalHandle_->get_goal()->joint_state;
    m_moveGroupPtr->setJointValueTarget(goalJointState);
    m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool success = planWithPlanner(plan);
    RCLCPP_INFO_STREAM(this->get_logger(), "Planning to joint space goal: " << (success ? "SUCCEEDED" : "FAILED"));
    
    if (success && planOnly) {
        RCLCPP_INFO_STREAM(this->get_logger(), "#####################################################");
        RCLCPP_INFO_STREAM(this->get_logger(), "                   Plan only mode!");
        RCLCPP_INFO_STREAM(this->get_logger(), "#####################################################");
        result->success = true;
        m_moveToJointGoalHandle_->succeed(result);
    }
    else if (success) {
        //addTimestampsToTrajectory(plan.trajectory_);
        //printTimestamps(plan.trajectory_);

        feedback->set__status("executing");
        m_moveToJointGoalHandle_->publish_feedback(feedback);
        auto errorcode = m_moveGroupPtr->execute(plan);
        if(errorcode == moveit::core::MoveItErrorCode::SUCCESS){
            RCLCPP_INFO_STREAM(this->get_logger(), "Execution succeeded!");
            result->success = true;
            m_moveToJointGoalHandle_->succeed(result);
        }
        else{
            RCLCPP_ERROR_STREAM(this->get_logger(), "Execution failed with error code");
            result->success = true;
            m_moveToJointGoalHandle_->abort(result); 
        }
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        result->success = false;
        m_moveToJointGoalHandle_->abort(result); 
    }
}

void m2Iface::planAndExecPose()
{   
    const auto feedback = std::make_shared<arm_api2_msgs::action::MoveCartesian::Feedback>();
    const auto result = std::make_shared<arm_api2_msgs::action::MoveCartesian::Result>();
    feedback->set__status("planning");
    m_moveToPoseGoalHandle_->publish_feedback(feedback);

    geometry_msgs::msg::PoseStamped goalPose = m_moveToPoseGoalHandle_->get_goal()->goal;
    RCLCPP_INFO_STREAM(this->get_logger(), "Planning to Cartesian Pose!");
    RCLCPP_INFO_STREAM(this->get_logger(), "Current pose is: " << m_currPoseState.pose.position.x << " " << m_currPoseState.pose.position.y << " " << m_currPoseState.pose.position.z);
    RCLCPP_INFO_STREAM(this->get_logger(), "Target pose is: " << goalPose.pose.position.x << " " << goalPose.pose.position.y << " " << goalPose.pose.position.z);
    RCLCPP_INFO_STREAM(this->get_logger(), "Pose frame_id is: " << goalPose.header.frame_id);
    RCLCPP_INFO_STREAM(this->get_logger(), "Using planner: " << (WITH_PLANNER ? "YES" : "NO"));

    if(goalPose.header.frame_id != PLANNING_FRAME){
        RCLCPP_INFO_STREAM(this->get_logger(), "Pose frame_id is not planning frame! PLANNING_FRAME: " << PLANNING_FRAME);
        RCLCPP_INFO_STREAM(this->get_logger(), "transforming pose to planning frame!");
        goalPose = transformPoseToFrame(goalPose, goalPose.header.frame_id, PLANNING_FRAME);
        if(goalPose.header.frame_id != PLANNING_FRAME){
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to transform pose to planning frame!");
            result->success = false;
            m_moveToPoseGoalHandle_->abort(result);
            return;
        }
    }

    m_moveGroupPtr->setPoseTarget(goalPose);
    m_moveGroupPtr->setMaxVelocityScalingFactor(max_vel_scaling_factor);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(max_acc_scaling_factor);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool success = planWithPlanner(plan);
    RCLCPP_INFO_STREAM(this->get_logger(), "Planning to pose goal: " << (success ? "SUCCEEDED" : "FAILED"));

    if (success && planOnly) {
        RCLCPP_INFO_STREAM(this->get_logger(), "#####################################################");
        RCLCPP_INFO_STREAM(this->get_logger(), "                   Plan only mode!");
        RCLCPP_INFO_STREAM(this->get_logger(), "#####################################################");
        result->success = true;
        m_moveToPoseGoalHandle_->succeed(result);
    }
    else if(success){
        //addTimestampsToTrajectory(plan.trajectory_);
        //printTimestamps(plan.trajectory_);

        feedback->set__status("executing");
        m_moveToPoseGoalHandle_->publish_feedback(feedback);
        auto errorcode = m_moveGroupPtr->execute(plan);
        if(errorcode == moveit::core::MoveItErrorCode::SUCCESS){
            RCLCPP_INFO_STREAM(this->get_logger(), "Execution succeeded!");
            result->success = true;
            m_moveToPoseGoalHandle_->succeed(result);
        }
        else{
            RCLCPP_ERROR_STREAM(this->get_logger(), "Execution failed with error code, time stamps:");
            printTimestamps(plan.trajectory_);
            result->success = false;
            m_moveToPoseGoalHandle_->abort(result); 
        }
    }
    else{
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        result->success = false;
        m_moveToPoseGoalHandle_->abort(result); 
    }
}

void m2Iface::planAndExecPosePath()
{   
    const auto feedback = std::make_shared<arm_api2_msgs::action::MoveCartesianPath::Feedback>();
    const auto result = std::make_shared<arm_api2_msgs::action::MoveCartesianPath::Result>();
    feedback->set__status("planning");
    m_moveToPosePathGoalHandle_->publish_feedback(feedback);

    auto goalPoseStampeds = m_moveToPosePathGoalHandle_->get_goal()->poses;
    RCLCPP_INFO_STREAM(this->get_logger(), "Planning Cartesian path!");
    RCLCPP_INFO_STREAM(this->get_logger(), "Current pose is: " << m_currPoseState.pose.position.x << " " << m_currPoseState.pose.position.y << " " << m_currPoseState.pose.position.z);
    RCLCPP_INFO_STREAM(this->get_logger(), "Target pose is: " << goalPoseStampeds[goalPoseStampeds.size()-1].pose.position.x << " " << goalPoseStampeds[goalPoseStampeds.size()-1].pose.position.y << " " << goalPoseStampeds[goalPoseStampeds.size()-1].pose.position.z);

    // extract all poses from the PoseStamped messages
    std::vector<geometry_msgs::msg::Pose> goalPoses;
    for (auto pose : goalPoseStampeds)
    {
        goalPoses.push_back(pose.pose);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    // TODO: Set as params that can be configured in YAML!
    double jumpThr = 0.0; 
    double eefStep = 0.02; 
    bool success = (m_moveGroupPtr->computeCartesianPath(goalPoses, eefStep, jumpThr, trajectory, true) == 1.0);

    if (success && planOnly) {
        RCLCPP_INFO_STREAM(this->get_logger(), "#####################################################");
        RCLCPP_INFO_STREAM(this->get_logger(), "                   Plan only mode!");
        RCLCPP_INFO_STREAM(this->get_logger(), "#####################################################");
        result->success = true;
        m_moveToPosePathGoalHandle_->succeed(result);
    }
    else if(success){
        addTimestampsToTrajectory(trajectory);

        feedback->set__status("executing");
        m_moveToPosePathGoalHandle_->publish_feedback(feedback);
        m_moveGroupPtr->execute(trajectory);
        result->success = true;
        m_moveToPosePathGoalHandle_->succeed(result);
    }
    else{
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        result->success = false;
        m_moveToPosePathGoalHandle_->abort(result); 
    }
    

}

void m2Iface::printTimestamps(const moveit_msgs::msg::RobotTrajectory &trajectory){
    trajectory_msgs::msg::JointTrajectory jointTrajectory = trajectory.joint_trajectory;
    std::vector<std::string> joint_names = jointTrajectory.joint_names;
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points = jointTrajectory.points;
    for (long unsigned int i = 0; i < points.size(); i++){
        trajectory_msgs::msg::JointTrajectoryPoint point = points[i];
        rclcpp::Duration duration = point.time_from_start;
        RCLCPP_INFO_STREAM(this->get_logger(), "Point " << i << " - time_from_start [s]: " << duration.seconds());
    } 
}

void m2Iface::addTimestampsToTrajectory(moveit_msgs::msg::RobotTrajectory &trajectory){
    // The trajectory created with computeCartesianPath() needs to be modified so it will include velocities as well.
    // reference: https://groups.google.com/g/moveit-users/c/MOoFxy2exT4
    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(m_moveGroupPtr->getCurrentState()->getRobotModel(), PLANNING_GROUP);
    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*m_moveGroupPtr->getCurrentState(), trajectory);
    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // Fourth compute computeTimeStamps
    bool success = iptp.computeTimeStamps(rt, max_vel_scaling_factor, max_acc_scaling_factor);
    RCLCPP_INFO_STREAM(this->get_logger(), "Computed time stamp " << (success ? "SUCCEEDED" : "FAILED"));
    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory);
} 

bool m2Iface::planWithPlanner(moveit::planning_interface::MoveGroupInterface::Plan &plan){
    // Planning priority:
    // 1. cuMotion planner from cuMotion (if have)
    // 2. LIN planner from pilz_industrial_motion_planner
    // 3. EST planner from ompl
    // 4. PRM planner from ompl
    // for EST and PRM create three plans and choose the best one
    //-----------------------------------------------------------------------------------------------
    m_moveGroupPtr->setPlanningPipelineId("isaac_ros_cumotion");
    m_moveGroupPtr->setPlannerId("cuMotion");
    std::list<moveit::planning_interface::MoveGroupInterface::Plan> all_plans;
    moveit::planning_interface::MoveGroupInterface::Plan cuMotion_plan;

    // TODO: if EE_LINK_NAME != tcp then move_group.setEndEffectorLink(tcp) need to be called before planning 
    // with cuMotion and the target pose needs to be transformed to tcp frame and sets again

    bool cuMotion_success = (m_moveGroupPtr->plan(cuMotion_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (cuMotion_success) {
        plan = cuMotion_plan;
        RCLCPP_INFO(this->get_logger(), "cuMotion planner succeeded. Immediate return.");
        return true;
    }

    std::vector<std::pair<std::string, std::string>> planners = {
        {"pilz_industrial_motion_planner", "LIN"},
        {"ompl", "EST"},
        {"ompl", "PRM"}
    };

    bool success = false;
    int tries_per_planner = 3;
    for(int i = 0; i < int(tries_per_planner*planners.size()); i++){

        int planner_index = i / tries_per_planner;
        int planner_try = i % tries_per_planner;

        m_moveGroupPtr->setPlanningPipelineId(planners[planner_index].first);
        m_moveGroupPtr->setPlannerId(planners[planner_index].second);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        success = static_cast<bool>(m_moveGroupPtr->plan(plan));

        if(success){
            RCLCPP_INFO(this->get_logger(), "%s found plan %d with %d points", 
                planners[planner_index].second.c_str(), i, int(plan.trajectory_.joint_trajectory.points.size()));
            all_plans.push_back(plan);
        }
        else {
            RCLCPP_INFO(this->get_logger(), "%s failed to find plan %d", 
                planners[planner_index].second.c_str(), i);
        }

        if(planner_try == tries_per_planner - 1 && all_plans.size() >= 3){
            RCLCPP_INFO(this->get_logger(), "Found %d plans, stopping planning", int(all_plans.size()));
        }
    }

    if(all_plans.size() == 0){
        RCLCPP_INFO_STREAM(this->get_logger(), "All planners failed!");
        return false;
    }
    else{
        RCLCPP_INFO_STREAM(this->get_logger(), "Found " << all_plans.size() << " plans!");
    }

    // find the best plan from the list of plans
    auto best_plan = std::min_element(all_plans.begin(), all_plans.end(), [](auto const& a, auto const& b){
        return a.trajectory_.joint_trajectory.points.size() < b.trajectory_.joint_trajectory.points.size();
    });

    plan = *best_plan;
    RCLCPP_INFO(this->get_logger(), "Best plan selected with %d points.", int(best_plan->trajectory_.joint_trajectory.points.size()));
    return true;
}

std::optional<std::vector<double>> m2Iface::calculateIK(const geometry_msgs::msg::Pose& target_pose)
{
    const moveit::core::JointModelGroup* joint_model_group = m_robotStatePtr->getJointModelGroup(PLANNING_GROUP);
    if (!joint_model_group) {
        RCLCPP_ERROR(this->get_logger(), "JointModelGroup %s not found!", PLANNING_GROUP.c_str());
        return std::nullopt;
    }

    // Convert geometry_msgs::Pose to Eigen::Isometry3d
    Eigen::Isometry3d target_pose_eigen;
    tf2::fromMsg(target_pose, target_pose_eigen);

    // Try to compute IK
    bool found_ik = m_robotStatePtr->setFromIK(joint_model_group, target_pose_eigen, EE_LINK_NAME, 0.1);

    if (found_ik)
    {
        
        // Collision checking
        if (!m_robotStatePtr) {
            RCLCPP_ERROR(this->get_logger(), "Robot state is NULL!");
            return std::nullopt;
        }
        m_robotStatePtr->update();
        // 🔐 Acquire thread-safe read-only access to the planning scene
        planning_scene_monitor::LockedPlanningSceneRO scene(m_pSceneMonitorPtr);
        if (!scene) {
            RCLCPP_ERROR(this->get_logger(), "Failed to lock planning scene.");
            return std::nullopt;
        }

        bool is_valid = scene->isStateValid(*m_robotStatePtr, PLANNING_GROUP);
        if (is_valid)
        {
            RCLCPP_INFO(this->get_logger(), "IK solution found and is collision-free.");
            std::vector<double> joint_values;
            m_robotStatePtr->copyJointGroupPositions(joint_model_group, joint_values);
            return joint_values;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "IK solution found but is in collision or invalid.");
            std::vector<std::string> colliding_links;
            m_pSceneMonitorPtr->getPlanningScene()->getCollidingLinks(colliding_links, *m_robotStatePtr);
            // print colliding links
            if (!colliding_links.empty()) {
                std::string links_str;
                for (size_t i = 0; i < colliding_links.size(); ++i) {
                    links_str += colliding_links[i];
                    if (i < colliding_links.size() - 1) {
                        links_str += ", ";
                    }
                }
                RCLCPP_WARN(this->get_logger(), "Colliding links: %s", links_str.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "No specific colliding links found, but state is invalid.");
            }
            return std::nullopt; // Return empty optional if in collision or invalid
        }
        
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "IK solution not found for given pose.");
        return std::nullopt;
    }
}

void m2Iface::move_to_pose_servo(const geometry_msgs::msg::Pose& target_pose)
{
    // check if the target pose is near to 
    double dx = m_currPoseState.pose.position.x - target_pose.position.x;
    double dy = m_currPoseState.pose.position.y - target_pose.position.y;
    double dz = m_currPoseState.pose.position.z - target_pose.position.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    constexpr double MAX_DIST = 0.15;  // meters

    if (distance > MAX_DIST)
    {
        RCLCPP_WARN(this->get_logger(), "Target pose too far (%.3f m > %.3f m) - ignored.", distance, MAX_DIST);
        // print target pose position
        RCLCPP_INFO_STREAM(this->get_logger(), "Target pose position: " << target_pose.position.x << ", "
            << target_pose.position.y << ", " << target_pose.position.z);
        RCLCPP_INFO_STREAM(this->get_logger(), "Current pose position: " << m_currPoseState.pose.position.x << ", "
            << m_currPoseState.pose.position.y << ", " << m_currPoseState.pose.position.z);
        return;
    }

    auto ik_result = calculateIK(target_pose);
    if (!ik_result)
    {
        RCLCPP_ERROR(this->get_logger(), "IK solution not found for target pose.");
        return;
    }

    const std::vector<double>& joint_positions = *ik_result;
    //RCLCPP_INFO_STREAM(this->get_logger(), "Moving to pose: " << target_pose.position.x << ", " 
    //    << target_pose.position.y << ", " << target_pose.position.z);


    rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    const rclcpp::Time now = steady_clock.now();
    constexpr double SAME_TARGET_EPS = 1e-4;

    // Identical final-pose commands are expected during feedback convergence.
    // Refresh the watchdog without restarting interpolation from stale commands.
    {
        std::lock_guard<std::mutex> lock(interpolation_mutex_);
        double max_joint_delta = -1.0;
        bool same_target = false;
        if (
            active_target_positions_ &&
            active_target_positions_->size() == joint_positions.size()
        ) {
            max_joint_delta = 0.0;
            for (size_t i = 0; i < joint_positions.size(); ++i) {
                max_joint_delta = std::max(
                    max_joint_delta,
                    std::abs((*active_target_positions_)[i] - joint_positions[i])
                );
            }
            same_target = max_joint_delta <= SAME_TARGET_EPS;
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Servo target max joint delta=%.6f rad, same_target=%s",
            max_joint_delta,
            same_target ? "true" : "false"
        );

        if (same_target) {
            last_target_time_ = now;
            return;
        }

        // A changed target must begin from measured joints, not the preceding
        // commanded target. If state is unavailable, leave the active motion intact.
        m_robotStatePtr = m_moveGroupPtr->getCurrentState(1.0);
        if (!m_robotStatePtr) {
            RCLCPP_WARN(this->get_logger(), "Current robot state unavailable; keeping active servo interpolation.");
            return;
        }
        const moveit::core::JointModelGroup* joint_model_group =
            m_robotStatePtr->getJointModelGroup(PLANNING_GROUP);
        if (!joint_model_group) {
            RCLCPP_WARN(this->get_logger(), "Joint model group unavailable; keeping active servo interpolation.");
            return;
        }

        std::vector<double> current_joint_positions;
        m_robotStatePtr->copyJointGroupPositions(
            joint_model_group,
            current_joint_positions
        );
        if (current_joint_positions.size() != joint_positions.size()) {
            RCLCPP_WARN(this->get_logger(), "Current joint state size mismatch; keeping active servo interpolation.");
            return;
        }

        const double dt = (now - last_target_time_).seconds();
        active_start_positions_ = current_joint_positions;
        active_target_positions_ = joint_positions;
        active_start_time_ = now;
        last_target_time_ = now;

        // Clamp between reasonable bounds: 50 Hz to 5 Hz.
        const double duration = std::clamp(dt, 0.02, 0.2);
        interpolation_duration_ = rclcpp::Duration::from_seconds(duration);
    }
    //RCLCPP_INFO(this->get_logger(), "Starting servo control with interpolation.");

}


void m2Iface::getArmState()
{   
    const moveit::core::JointModelGroup* joint_model_group = m_robotStatePtr->getJointModelGroup(PLANNING_GROUP);
    m_currJointNames = joint_model_group->getVariableNames();
    m_robotStatePtr->copyJointGroupPositions(joint_model_group, m_currJointPosition);
    // get current ee pose
    m_currPoseState = m_moveGroupPtr->getCurrentPose(EE_LINK_NAME); 
    // current_state_monitor
    m_robotStatePtr = m_moveGroupPtr->getCurrentState();
    // by default timeout is 10 secs
    m_robotStatePtr->update();
    
    Eigen::Isometry3d currentPose_ = m_moveGroupPtr->getCurrentState()->getFrameTransform(EE_LINK_NAME);
    m_currPoseState = utils::convertIsometryToMsg(currentPose_);
    auto frame_id = m_moveGroupPtr->getPlanningFrame().c_str();
    m_currPoseState.header.frame_id = frame_id;
}

geometry_msgs::msg::PoseStamped m2Iface::transformPoseToFrame(geometry_msgs::msg::PoseStamped pose, std::string source_frame, std::string frame_id)
{
    geometry_msgs::msg::TransformStamped transformStamped;
    geometry_msgs::msg::PoseStamped transformedPose;
    try{
        transformStamped = tfBufferPtr->lookupTransform(frame_id, source_frame, tf2::TimePointZero);
        tf2::doTransform(pose, transformedPose, transformStamped);
        transformedPose.header.frame_id = frame_id;
    }
    catch (tf2::TransformException &ex){
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    }

    return transformedPose;
}


bool m2Iface::run()
{
    if(!nodeInit)       {RCLCPP_ERROR(this->get_logger(), "Node not fully initialized!"); return false;} 
    if(!moveGroupInit)  {RCLCPP_ERROR(this->get_logger(), "MoveIt interface not initialized!"); return false;} 

    getArmState(); 
    pose_state_pub_->publish(m_currPoseState);
    std_msgs::msg::String stateMsg;
    stateMsg.data = stateNames[robotState];
    robot_state_pub_->publish(stateMsg);

    rclcpp::Clock steady_clock; 
    int LOG_STATE_TIMEOUT=10000; 

    // STATE MACHINE
    if (robotState == IDLE)
    {   
        active_start_positions_.reset();
        active_target_positions_.reset();
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, LOG_STATE_TIMEOUT, "arm_api2 is in IDLE mode."); 
    }
    else{
        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), steady_clock, LOG_STATE_TIMEOUT, "arm_api2 is in " << stateNames[robotState] << " mode."); 
    }

    // Check if servo active, to deactivate before sending to another pose 
    if (robotState != SERVO_CTL && servoEntered) {servoPtr->setPaused(true); servoEntered=false;} 

    if (robotState == JOINT_TRAJ_CTL)
    {
        if (recivCmd) {
            planAndExecJoint();
            recivCmd = false; 
        }
        active_start_positions_.reset();
        active_target_positions_.reset(); 
    }

    if (robotState == CART_TRAJ_CTL)
    {   
        // TODO: Beware if both are true at the same time, shouldn't occur, 
        if (recivCmd) {
            planAndExecPose();
            recivCmd = false; 
        } 

        if (recivTraj){
            planAndExecPosePath();
            recivTraj = false; 
        }
        active_start_positions_.reset();
        active_target_positions_.reset();
    }

    if (robotState == SERVO_CTL)
    {   
        if (!servoEntered)
        {   
            // Moveit servo status codes: https://github.com/moveit/moveit2/blob/main/moveit_ros/moveit_servo/include/moveit_servo/utils/datatypes.hpp
            servoPtr->start(); 
            servoEntered = true; 
        }
        active_start_positions_.reset();
        active_target_positions_.reset();
    }

    if(recivGripperCmd){
        recivGripperCmd = false;

        std::lock_guard<std::mutex> lock(gripper_goal_mutex_);
        auto goal_handle = m_gripperControlGoalHandle_;

        RCLCPP_INFO(this->get_logger(), "Launching gripper thread with goal...");

        std::thread([this, goal_handle]() {
            if (!goal_handle || !goal_handle->is_active()) {
                RCLCPP_WARN(this->get_logger(), "Gripper goal handle is not active anymore.");
                return;
            }

            auto goal = goal_handle->get_goal();
            float position = goal->command.position;
            float effort = goal->command.max_effort;

            auto result = std::make_shared<control_msgs::action::GripperCommand::Result>();
            RCLCPP_INFO(this->get_logger(), "Executing gripper command: pos = %f, effort = %f", position, effort);

            bool success = false;
            try {
                success = gripper.send_gripper_command(position, effort);
                RCLCPP_INFO(this->get_logger(), "Gripper command finished. Success: %d", success);
            } catch (const std::exception& e) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Exception during gripper command: " << e.what());
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Unknown exception during gripper command.");
            }

            if (!goal_handle->is_active()) {
                RCLCPP_WARN(this->get_logger(), "Gripper goal was completed/canceled before command finished.");
                return;
            }

            if (success) {
                result->position = gripper.get_position();
                result->effort = gripper.get_effort();
                result->stalled = gripper.is_stalled();
                result->reached_goal = gripper.reached_goal();
                goal_handle->succeed(result);
            } else {
                result->reached_goal = gripper.reached_goal();
                goal_handle->abort(result);
            }
        }).detach();
    }

    return true;     
}

static inline double clamp(double x, double lo, double hi)
{
    return std::max(lo, std::min(hi, x));
}

void m2Iface::servo_loop_cb()
{
    if (robotState != SERVO_POS_CTL) return;

    // Check if 5 seconds have passed since last target pose
    rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    rclcpp::Time now = steady_clock.now();
    double time_since_last_target = (now - last_target_time_).seconds();
    
    if (time_since_last_target > 5.0)
    {
        std::lock_guard<std::mutex> lock(interpolation_mutex_);
        if (active_start_positions_ || active_target_positions_)
        {
            RCLCPP_INFO(this->get_logger(), "No new target pose for 5 seconds, resetting servo targets.");
            active_start_positions_.reset();
            active_target_positions_.reset();
            smoother_init_ = false;
        }
        return;
    }

    std::optional<std::vector<double>> start_pos, target_pos;
    rclcpp::Time start_time;
    rclcpp::Duration duration = rclcpp::Duration::from_seconds(0.0);

    {
        std::lock_guard<std::mutex> lock(interpolation_mutex_);
        if (!active_start_positions_ || !active_target_positions_) return;

        start_pos = active_start_positions_;
        target_pos = active_target_positions_;
        start_time = active_start_time_;
        duration = interpolation_duration_;
    }

    double t = (now - start_time).seconds() / duration.seconds();
    t = std::clamp(t, 0.0, 1.0);
    // print the interpolation progress t
    //RCLCPP_INFO_STREAM(this->get_logger(), "Interpolation progress: " << t << " (from " 
    //    << start_time.seconds() << " to " << now.seconds() << ")" << " duration: " << duration.seconds());

    std::vector<double> interp(target_pos->size());
    for (size_t i = 0; i < interp.size(); ++i)
        interp[i] = (1.0 - t) * (*start_pos)[i] + t * (*target_pos)[i];


    // dt from steady clock (200Hz loop)
    static rclcpp::Time last = rclcpp::Clock(RCL_STEADY_TIME).now();
    double dt = (now - last).seconds();
    last = now;
    dt = clamp(dt, 1.0/500.0, 1.0/50.0);  // keep sane if scheduler jitters

    const auto &q_ref = interp; // your current interpolated target

    // init smoother state
    if (!smoother_init_ || q_out_.size() != q_ref.size())
    {
        q_out_ = q_ref;
        dq_out_.assign(q_ref.size(), 0.0);
        smoother_init_ = true;
    }

    // velocity/acceleration-limited tracking
    for (size_t i = 0; i < q_ref.size(); ++i)
    {
        double e = q_ref[i] - q_out_[i];

        // desired velocity to reduce error with time constant tau_
        double dq_des = clamp(e / tau_, -v_max_, v_max_);

        // accel-limit towards desired velocity
        double ddq = clamp((dq_des - dq_out_[i]) / dt, -a_max_, a_max_);

        dq_out_[i] += ddq * dt;
        q_out_[i]  += dq_out_[i] * dt;
    }

    // publish q_out_ instead of interp
    std_msgs::msg::Float64MultiArray msg;
    msg.data = q_out_;
    joint_command_pub_->publish(msg);

    if (has_joint_command_traj_topic_ && joint_command_traj_pub_)
    {
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.header.stamp = this->now();
        traj_msg.joint_names = m_currJointNames;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = q_out_;
        point.time_from_start.sec = 0;
        point.time_from_start.nanosec = 100000000;
        traj_msg.points.push_back(point);

        joint_command_traj_pub_->publish(traj_msg);
    }

}
