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

int main (int argc, char **argv)
{
  ros::init (argc, argv, "max_torque");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  double init_torque, increment, max_thrust;
  nhp.param("init_torque", init_torque, -10.0);
  nhp.param("increment", increment, -1.0);
  nhp.param("max_thrust", max_thrust, 30.0);

  // TODO: FullVectoringRobotModel has bug about joint torque, use HydrusLikeRobotModel
  //boost::shared_ptr<Dragon::FullVectoringRobotModel> robot_model = boost::make_shared<Dragon::FullVectoringRobotModel>();
  boost::shared_ptr<Dragon::HydrusLikeRobotModel> robot_model = boost::make_shared<Dragon::HydrusLikeRobotModel>();

  // a joint configuration to operate a upward valve
  sensor_msgs::JointState joint_state;
  joint_state.name.push_back("joint1_pitch");
  joint_state.position.push_back(M_PI/2);
  joint_state.name.push_back("joint1_yaw");
  joint_state.position.push_back(M_PI/2);
  joint_state.name.push_back("joint2_pitch");
  joint_state.position.push_back(0);
  joint_state.name.push_back("joint2_yaw");
  joint_state.position.push_back(M_PI/2);
  joint_state.name.push_back("joint3_pitch");
  joint_state.position.push_back(0);
  joint_state.name.push_back("joint3_yaw");
  joint_state.position.push_back(M_PI/2);


  // update the robot model
  // the desired CoG frame is horizontal
  robot_model->updateRobotModel(joint_state);
  int rotor_num = robot_model->getRotorNum();
  for(auto name: robot_model->getJointNames())
    ROS_DEBUG_STREAM("joint name: " << name);
  ROS_INFO_STREAM("hover thrust: " << robot_model->getStaticThrust().transpose());

  robot_model->calcCoGMomentumJacobian(); // should be processed first!
  robot_model->calcBasicKinematicsJacobian(); // need cog_jacobian_
  robot_model->calcJointTorque(false);
  ROS_INFO_STREAM("hover joint torque: " << robot_model->getJointTorque().transpose());

  geometry_msgs::Point offset;
  offset.x = 0;
  offset.y = 0;
  offset.z = 0;
  geometry_msgs::Wrench wrench;
  wrench.force.x = 0;
  wrench.force.y = 0;
  wrench.force.z = 0;
  wrench.torque.x = 0;
  wrench.torque.y = 0;
  double torque = init_torque;

  while(true)
    {
      robot_model->updateRobotModel(joint_state);
      robot_model->calcJointTorque(false);

      wrench.torque.z = torque; //valve torque
      robot_model->resetExternalStaticWrench();
      robot_model->addExternalStaticWrench("valve", "link1", offset, wrench); // cog causes map at error in KDL

      robot_model->calcExternalWrenchCompThrust();
      Eigen::VectorXd ex_wrench_comp_f = robot_model->getExWrenchCompensateVectoringThrust();
      ROS_INFO_STREAM("ex_wrench_comp_f: " << ex_wrench_comp_f.transpose());
      robot_model->addCompThrustToStaticThrust();
      ROS_INFO_STREAM("final thrust: " << robot_model->getStaticThrust().transpose());

#if 0
      Eigen::VectorXd thrust_vec = Eigen::VectorXd::Zero(rotor_num);
      for(int i = 0; i < rotor_num; i++)
        {
          Eigen::VectorXd f = hover_vectoring_f.segment(3 * i, 3) + ex_wrench_comp_f.segment(3 * i, 3);
          thrust_vec(i) = f.norm();
        }
      ROS_INFO_STREAM("final thrust: " << thrust_vec.transpose());
#endif

      robot_model->addCompThrustToJointTorque();
      ROS_INFO_STREAM("final joint torque: " << robot_model->getJointTorque().transpose());

      if(robot_model->getStaticThrust().maxCoeff() > max_thrust) break;

      torque += increment; // increment
    }

  ROS_INFO("max turn torque at end effector is %f", torque);
  return 0;
}









