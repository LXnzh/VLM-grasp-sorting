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
 *      Created     : 05/10/2024
 *      Author      : Filip Zoric
 *
 *      Description : Header for the utils.cpp file
 */

#ifndef UTILS_HPP
#define UTILS_HPP

#include <stdio.h>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "Eigen/Dense"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace utils {

    bool                                            comparePosition             (geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2);
    bool                                            compareOrientation          (geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2);
    bool                                            comparePose                 (geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2);
    std::vector<geometry_msgs::msg::Pose>           createCartesianWaypoints    (geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2, int n); 
    geometry_msgs::msg::PoseStamped                 normalizeOrientation        (geometry_msgs::msg::PoseStamped p);
    geometry_msgs::msg::PoseStamped                 convertIsometryToMsg       (Eigen::Isometry3d pose);

} // namespace utils

#endif // UTILS_HPP