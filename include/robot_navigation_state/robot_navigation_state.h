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
 *  robot_navigation_state.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#ifndef ROBOT_NAVIGATION_STATE_ROBOT_NAVIGATION_STATE_H_
#define ROBOT_NAVIGATION_STATE_ROBOT_NAVIGATION_STATE_H_

#include <tf/transform_listener.h>

namespace robot_navigation_state{

enum State {
  IDLE,
  PLANNING,
  MOVING,
};

namespace idle{
  enum Input {
    WAIT_FOR_GOAL,
    RECEIVED_GOAL
  };
}

namespace planning{
  enum Input {
	STOPPED,
    PLANNING,
    FOUND_PLAN,
    NO_PLAN_FOUND
  };
}

namespace moving{
  enum Input {
	STOPPED,
    NO_LOCAL_CMD,
    GOT_LOCAL_CMD,
    ARRIVED_GOAL
  };
}

bool toGlobalFrame(
  const tf::TransformListener& tf_listener,
  const std::string& global_frame,
  const geometry_msgs::PoseStamped& in,
  geometry_msgs::PoseStamped& out
);

bool getRobotPose(
  const tf::TransformListener& tf_listener,
  const std::string& robot_frame,
  const std::string& global_frame,
  geometry_msgs::PoseStamped& robot_pose
);

}; /* namespace robot_navigation_state */

#endif /* robot_navigation_state.h */
