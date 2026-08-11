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

/*      Title       : utils.cpp
 *      Project     : arm_api2
 *      Created     : 05/10/2024
 *      Author      : Filip Zoric
 *
 *      Description : Contains all functional implementations not directly related to MoveIt! or ROS 2
 */


#include "arm_api2/utils.hpp"

// TODO: move to utils
std::vector<geometry_msgs::msg::Pose> utils::createCartesianWaypoints(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2, int n) 
{
    std::vector<geometry_msgs::msg::Pose> result;
    geometry_msgs::msg::Pose pose_;
    if (n <= 1) {
        result.push_back(p1);
        return result;
    }
    double dX = (p2.position.x - p1.position.x) / (n - 1);
    double dY = (p2.position.y - p1.position.y) / (n - 1); 
    double dZ = (p2.position.z - p1.position.z) / (n - 1); 
    // Set i == 1 because start point doesn't have to be included into waypoint list 
    // https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/
    for (int i = 1; i < n; ++i) {
        pose_.position.x = p1.position.x + i * dX; 
        pose_.position.y = p1.position.y + i * dY; 
        pose_.position.z = p1.position.z + i * dZ; 
        pose_.orientation.x = p1.orientation.x; 
        pose_.orientation.y = p1.orientation.y;
        pose_.orientation.z = p1.orientation.z;
        pose_.orientation.w = p1.orientation.w; 
        result.push_back(pose_);
    }
    return result;
}


// TODO: Move to utils
bool utils::comparePosition(geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2)
{   
    // Returns false if different positions
    bool x_cond = false;  bool y_cond = false;  bool z_cond = false; 
    double d = 0.01; 

    if (std::abs(p1.pose.position.x - p2.pose.position.x) < d) x_cond = true; 
    if (std::abs(p1.pose.position.y - p2.pose.position.y) < d) y_cond = true; 
    if (std::abs(p1.pose.position.z - p2.pose.position.z) < d) z_cond = true; 

    bool cond = x_cond && y_cond && z_cond;  
    return cond; 
}

// TODO: Move to utils
bool utils::compareOrientation(geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2)
{   
    // Returns false if different positions
    bool p_cond = false;  bool r_cond = false;  bool y_cond = false; 
    double d = 0.01; 

    geometry_msgs::msg::Quaternion q1c, q2c; 
    q1c.x = p1.pose.orientation.x; q2c.x = p2.pose.orientation.x; 
    q1c.y = p1.pose.orientation.y; q2c.y = p2.pose.orientation.y; 
    q1c.z = p1.pose.orientation.z; q2c.z = p2.pose.orientation.z; 
    q1c.w = p1.pose.orientation.w; q2c.w = p2.pose.orientation.w; 

    tf2::Quaternion q1(q1c.x, q1c.y, q1c.z, q1c.w); 
    tf2::Quaternion q2(q2c.x, q2c.y, q2c.z, q2c.w); 
    
    double r1, r2, pi1, pi2, y1, y2; 
    tf2::Matrix3x3(q1).getEulerYPR(y1, pi1, r1); 
    tf2::Matrix3x3(q2).getEulerYPR(y2, pi2, r2);

    d = 0.01; 
    if (std::abs(pi1 - pi2) < d) p_cond = true; 
    if (std::abs(r1 - r2) < d) r_cond = true; 
    if (std::abs(y1 - y2) < d) y_cond = true; 

    //TODO: Convert quat1 to roll & pitch & yaw 
    bool cond = p_cond && r_cond && y_cond; 
    return cond; 
}

// TODO: move to utils
bool utils::comparePose(geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2)
{
    bool position, orientation; 
    position    = comparePosition(p1, p2); 
    orientation = compareOrientation(p1, p2); 
    return position && orientation; 
}

//TODO: move to utils
geometry_msgs::msg::PoseStamped utils::normalizeOrientation(geometry_msgs::msg::PoseStamped p)
{
    geometry_msgs::msg::PoseStamped p_; 
    tf2::Quaternion q(p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w); 
    q.normalize(); 
    p_.pose.orientation.x = q.x(); 
    p_.pose.orientation.y = q.y(); 
    p_.pose.orientation.z = q.z(); 
    p_.pose.orientation.w = q.w(); 
    p_.pose.position = p.pose.position; 
    return p_; 
}

// There's also built-in method in tf2 however, didn't manage to get it to work
geometry_msgs::msg::PoseStamped utils::convertIsometryToMsg(Eigen::Isometry3d pose)
{
    geometry_msgs::msg::PoseStamped p; 
    p.pose.position.x = pose.translation().x(); 
    p.pose.position.y = pose.translation().y(); 
    p.pose.position.z = pose.translation().z(); 
    Eigen::Quaterniond q(pose.rotation()); 
    p.pose.orientation.x = q.x(); 
    p.pose.orientation.y = q.y(); 
    p.pose.orientation.z = q.z(); 
    p.pose.orientation.w = q.w(); 
    return p; 
}