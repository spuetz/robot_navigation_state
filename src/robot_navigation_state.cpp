/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by the University of Osnabrück
 *  Copyright (c) 2015, University of Osnabrück
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 *  robot_navigation_state.cpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#include "robot_navigation_state/robot_navigation_state.h"

namespace robot_navigation_state{

bool getRobotPose(
  const tf::TransformListener& tf_listener,
  const std::string& robot_frame,
  const std::string& global_frame,
  geometry_msgs::PoseStamped& robot_pose 
)
{

  tf::Stamped < tf::Pose > local_pose, global_pose;
  global_pose.setIdentity();
  local_pose.setIdentity();
  local_pose.frame_id_ = robot_frame;
  local_pose.stamp_ = ros::Time();

  // get the global pose of the robot
  try
  {
    tf_listener.transformPose(global_frame, local_pose, global_pose);
  }
  catch (tf::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  tf::poseStampedTFToMsg(global_pose, robot_pose);

  return true;
}

bool toGlobalFrame(
    const tf::TransformListener& tf_listener,
    const std::string& global_frame,
    const geometry_msgs::PoseStamped& in,
    geometry_msgs::PoseStamped& out 
){
  tf::Stamped<tf::Pose> in_tf_pose, global_pose;
  tf::poseStampedMsgToTF(in, in_tf_pose);

  //just get the latest available transform... for accuracy they should send
  in_tf_pose.stamp_ = ros::Time();

  try{
    tf_listener.transformPose(global_frame, in_tf_pose, global_pose);
  }
  catch(tf::TransformException& ex){
    ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
        in_tf_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
    return false;
  }
  tf::poseStampedTFToMsg(global_pose, out);
  return true;
}



} /* namespace robot_navigation_state */
