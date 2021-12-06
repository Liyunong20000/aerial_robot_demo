// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
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

#include <dragon/model/full_vectoring_robot_model.h>
#include <sstream>
#include <nlopt.hpp>

class OptimizeValvePose
{
public:
  OptimizeValvePose(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~OptimizeValvePose() {}

  void recursiveSearch(boost::shared_ptr<Dragon::HydrusLikeRobotModel> robot_model, KDL::Rotation root_rot, sensor_msgs::JointState joint_state, std::vector<double> joint_uppers, int id, sensor_msgs::JointState& opt_joint_state, double& max_torque);

  double maxValveTorque(boost::shared_ptr<Dragon::HydrusLikeRobotModel> robot_model, const KDL::Rotation& root_rot, const sensor_msgs::JointState& joint_state, double max_joint_torque, double max_thrust,  bool verbose = false);


public: // easy for nlopt cost function to access
  ros::NodeHandle nh_, nhp_;;

  boost::shared_ptr<Dragon::FullVectoringRobotModel> robot_model_;
  KDL::Rotation root_rot_;

  double init_torque_;
  double max_thrust_;
  double max_joint_torque_;

  double joint_delta_;

  std::vector<double> joint_lowers_, joint_uppers_;
  std::vector<int> active_joint_;

  bool hydrus_model_; // true: HydrusLikeModel; false: FullVectoring
};
