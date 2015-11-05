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
  const ros::Duration& timeout,
  geometry_msgs::PoseStamped& robot_pose 
)
{
  tf::Stamped < tf::Pose > local_pose;
  local_pose.setIdentity();
  local_pose.frame_id_ = robot_frame;
  local_pose.stamp_ = ros::Time::now();
  geometry_msgs::PoseStamped local_pose_msg;
  tf::poseStampedTFToMsg(local_pose, local_pose_msg);
  return robot_navigation_state::transformPose(
    tf_listener,
    global_frame,
    local_pose.stamp_,
    timeout,
    local_pose_msg,
    global_frame,
    robot_pose);
}

bool transformPose(
    const tf::TransformListener& tf_listener,
    const std::string& target_frame,
    const ros::Time& target_time,
    const ros::Duration& timeout,
    const geometry_msgs::PoseStamped& in,
    const std::string& fixed_frame,
    geometry_msgs::PoseStamped& out
){
  std::string error_msg;

  bool success = tf_listener.waitForTransform(
    target_frame,
    in.header.frame_id,
    in.header.stamp,
    timeout,
    ros::Duration(0.01),
    &error_msg   
  );
  
  if(!success){
    ROS_WARN("Failed to look up transform from %s into the %s frame: %s",
    in.header.frame_id.c_str(), target_frame.c_str(), error_msg.c_str());
    return false;
  }
    
  try{
    tf_listener.transformPose(target_frame, target_time, in, fixed_frame, out);
  }
  catch(tf::TransformException& ex){
    ROS_WARN("Failed to transform pose from %s into the %s frame: %s",
      in.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
  return true;
}



} /* namespace robot_navigation_state */
