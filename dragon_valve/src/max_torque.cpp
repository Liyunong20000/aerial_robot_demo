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

#include <dragon_valve/max_torque.h>

namespace
{
  int cnt = 0;

  double maxTorque(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
  {
    OptimizeValvePose *planner = reinterpret_cast<OptimizeValvePose*>(ptr);

    sensor_msgs::JointState joint_state;
    joint_state.name = planner->robot_model_->getLinkJointNames();
    joint_state.position = planner->joint_lowers_;
    for(int i = 0; i < planner->active_joint_.size(); i++)
      joint_state.position.at(planner->active_joint_.at(i)) = x.at(i);

    double torque = planner->maxValveTorque(planner->robot_model_, planner->root_rot_, joint_state, planner->max_joint_torque_, planner->max_thrust_);

    cnt++;
    if(cnt % 1000 == 0)
      {
        std::stringstream ss;
        ss << "cnt: " << cnt;
        ss << ", angles: ";
        for(auto v : x) ss << v << ",";
        ss << "torque: " << torque;
        ss << "; torque threshold: " << planner->max_joint_torque_;
        ss << "; thrust threshold: " << planner->max_thrust_;
        std::cout << ss.str() << std::endl;
      }

    return fabs(torque);
  }
}

OptimizeValvePose::OptimizeValvePose(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp), active_joint_(0)
{
  nhp.param("init_torque", init_torque_, -10.0);
  nhp.param("max_thrust", max_thrust_, 30.0);
  nhp.param("max_joint_torque", max_joint_torque_, 7.0);
  nhp.param("hydrus_model", hydrus_model_, false);

  robot_model_ = boost::make_shared<Dragon::FullVectoringRobotModel>();
  // heurisitc sample: a joint configuration to operate a upward valve
  robot_model_->setBaselinkName(std::string("link1"));


  std::vector<double> root_rpy{0, -M_PI/2, 0}; // upward
  if(nhp.hasParam("root_rpy")) nhp.getParam("root_rpy", root_rpy);
  root_rot_ = KDL::Rotation::EulerZYX(root_rpy.at(2), root_rpy.at(1), root_rpy.at(0));

  bool joint_full_search;
  nhp.param("joint_full_search", joint_full_search, false);

  bool stopper_search;
  nhp.param("stopper_search", stopper_search, false);

  if(joint_full_search)
    {
      if(!nhp.hasParam("joint_lowers") || !nhp.hasParam("joint_uppers"))
        {
          ROS_ERROR("Please assign the lower and upper limits for joint angles");
          return;
        }

      nhp.getParam("joint_lowers", joint_lowers_);
      nhp.getParam("joint_uppers", joint_uppers_);
      nhp.getParam("joint_delta", joint_delta_);

      for(int i = 0; i < joint_lowers_.size(); i++)
        {
          if(joint_lowers_.at(i) < joint_uppers_.at(i)) active_joint_.push_back(i);
        }

      bool brute_search;
      nhp.param("brute_search", brute_search, false);

      if(brute_search)
        {
          Eigen::VectorXd max_torque_candidates = Eigen::VectorXd::Zero(int(std::pow(2, active_joint_.size())));
          std::vector<sensor_msgs::JointState> opt_joint_state_candidates(max_torque_candidates.size());
#pragma omp parallel for
          for(int i = 0; i < max_torque_candidates.size(); i++)
            {
              std::vector<double> sub_joint_lowers = joint_lowers_;
              std::vector<double>  sub_joint_uppers = joint_uppers_;
              for(int j = 0; j < active_joint_.size(); j++)
                {
                  int k = active_joint_.at(j);
                  if((1 << j) & i)
                    sub_joint_lowers.at(k) = (joint_lowers_.at(k) + joint_uppers_.at(k)) / 2;
                  else
                    sub_joint_uppers.at(k) = (joint_lowers_.at(k) + joint_uppers_.at(k)) / 2;
                }

              // std::stringstream ss;
              // for(int j = 0; j < joint_lowers_.size(); j++)
              //     ss << "(" << sub_joint_lowers.at(j) << ", " << sub_joint_uppers.at(j) << "), ";
              // ROS_INFO_STREAM("thread: " << omp_get_thread_num() << "; i " << i << ", " << ss.str());

              // TODO: FullVectoringRobotModel has bug about joint torque, use HydrusLikeRobotModel
              boost::shared_ptr<Dragon::FullVectoringRobotModel> robot_model = boost::make_shared<Dragon::FullVectoringRobotModel>();
              //boost::shared_ptr<Dragon::HydrusLikeRobotModel> robot_model = boost::make_shared<Dragon::HydrusLikeRobotModel>();
              robot_model->setBaselinkName(std::string("link1"));

              sensor_msgs::JointState joint_state;
              joint_state.name = robot_model->getLinkJointNames();
              joint_state.position = sub_joint_lowers;

              sensor_msgs::JointState opt_joint_state = joint_state;
              double max_torque = 0;

              recursiveSearch(robot_model, root_rot_, joint_state, sub_joint_uppers, 0, opt_joint_state, max_torque);

              std::stringstream ss;
              for(int i = 0; i < opt_joint_state.position.size(); i++)
                ss << "(" << opt_joint_state.name.at(i) << ": " << opt_joint_state.position.at(i) << ")";
              ROS_WARN_STREAM("finish iteration" << i << ", thread: " << omp_get_thread_num() << ", update the optimize joint state: " << ss.str() << ", generate valve torque of " << max_torque);

              max_torque_candidates(i) = max_torque;
              opt_joint_state_candidates.at(i) = opt_joint_state;
            }

          int best_index = 0;
          max_torque_candidates.cwiseAbs().maxCoeff(&best_index);

          std::stringstream ss;
          ss << "joints: ";
          for(int i = 0; i < opt_joint_state_candidates.at(best_index).name.size(); i++)
            ss << opt_joint_state_candidates.at(best_index).position.at(i) << ", ";
          ROS_INFO_STREAM("optimize max torque:" << max_torque_candidates(best_index) << ", " << ss.str());
        }
      else
        {
          // TODO: non gradient optimization cannot catch the complementary constraints
          auto algorithm_type = nlopt::GN_ISRES;
          nlopt::opt nl_solver(algorithm_type, active_joint_.size());
          nl_solver.set_max_objective(maxTorque, this);

          nl_solver.set_xtol_rel(1e-3); // rad
          nl_solver.set_ftol_rel(1e-4); // Nm
          nl_solver.set_maxeval(active_joint_.size() * 3000);

          std::vector<double> nlopt_lb;
          std::vector<double> nlopt_ub;
          std::vector<double> x;
          for(auto index: active_joint_)
            {
              nlopt_lb.push_back(joint_lowers_.at(index));
              nlopt_ub.push_back(joint_uppers_.at(index));
              x.push_back((joint_lowers_.at(index) + joint_uppers_.at(index))/2);
            }

          nl_solver.set_lower_bounds(nlopt_lb);
          nl_solver.set_upper_bounds(nlopt_ub);


          double max_torque;
          try{
            auto t_start = std::chrono::high_resolution_clock::now();
            nlopt::result result = nl_solver.optimize(x, max_torque);
            auto t_end = std::chrono::high_resolution_clock::now();
            double t = std::chrono::duration<double, std::milli>(t_end-t_start).count();

            std::vector<double> opt_joint_angles = joint_lowers_;
            for(int i = 0; i < active_joint_.size(); i++)
              opt_joint_angles.at(active_joint_.at(i)) = x.at(i);

            std::stringstream ss;
            ss << "joints: ";
            for(int j = 0; j < joint_lowers_.size(); j++)
              ss << opt_joint_angles.at(j) << ", ";
            ROS_INFO_STREAM("NLOPT: " << t << "[ms]" << ", max torque: " << max_torque << "; " << ss.str());


            sensor_msgs::JointState joint_state;
            joint_state.name = robot_model_->getLinkJointNames();
            joint_state.position = opt_joint_angles;
            maxValveTorque(robot_model_, root_rot_, joint_state, max_joint_torque_, max_thrust_, true);

          }
          catch(std::exception &e) {
            ROS_ERROR_STREAM("nlopt failed: " << e.what());
          }
        }
    }
  else if(stopper_search)
    {
      double valve_lower_angle = 0;
      double valve_upper_angle = 2 * M_PI;
      if(fabs(fabs(root_rpy.at(1)) - M_PI/2) < 0.01) valve_upper_angle = 0;

      for(double angle = M_PI / 4 ; angle <= 2 * M_PI / (robot_model_->getRotorNum() -1); angle+=0.01)
        {
          sensor_msgs::JointState joint_state;
          joint_state.name = robot_model_->getLinkJointNames();
          joint_state.position.push_back(M_PI/2);
          joint_state.position.push_back(angle);
          for(int i = 1; i < joint_state.name.size() / 2; i++)
            {
              joint_state.position.push_back(0);
              joint_state.position.push_back(angle);
            }

          int cnt = 0;
          double ave_torque = 0;
          for(double roll = valve_lower_angle; roll <= valve_upper_angle; roll+=0.02)
            {
              KDL::Rotation root_rot = KDL::Rotation::EulerZYX(root_rpy.at(2), root_rpy.at(1), roll);

              double torque = maxValveTorque(robot_model_, root_rot, joint_state, 0, max_thrust_, false);
              //ROS_INFO("max turn torque at valve rotation (%f, %f, %f) is %f", roll, root_rpy.at(1), root_rpy.at(2), torque);
              cnt ++;
              ave_torque += torque;
            }
          std::cout << "average max turn torque at stopper angle: " << angle << "; torque: " << ave_torque/cnt << std::endl;
        }
    }
  else
    {

      bool rotate_valve;
      nhp.param("rotate_valve", rotate_valve, false);

      double valve_lower_angle = 0;
      double valve_upper_angle = 2 * M_PI;
      if(!rotate_valve)
        {
          valve_lower_angle = root_rpy.at(0);
          valve_upper_angle = root_rpy.at(0);
        }

      int cnt = 0;
      double ave_torque = 0;
      double min_torque = 1e6;
      double max_torque = 0;
      for(double roll = valve_lower_angle; roll <= valve_upper_angle; roll+=0.02)
        {
          KDL::Rotation root_rot = KDL::Rotation::EulerZYX(root_rpy.at(2), root_rpy.at(1), roll);

          sensor_msgs::JointState joint_state;
          joint_state.name = robot_model_->getLinkJointNames();
          nhp.getParam("joint_angles", joint_state.position);

          if(joint_state.position.size() != joint_state.name.size())
            {
              ROS_ERROR("the size of joint position and name are not equal: %d, %d",
                        (int)joint_state.position.size(), (int)joint_state.name.size());
              return;
            }

          double torque = maxValveTorque(robot_model_, root_rot, joint_state, max_joint_torque_, max_thrust_, !rotate_valve);
          ROS_INFO("max turn torque at valve rotation (%f, %f, %f) is %f", roll, root_rpy.at(1), root_rpy.at(2), torque);

          cnt ++;
          ave_torque += torque;
          if(fabs(torque) > fabs(max_torque)) max_torque = torque;
          if(fabs(torque) < fabs(min_torque)) min_torque = torque;
        }
      if(cnt > 1)
        ROS_INFO("rotate average torque: %f, [%f, %f]", ave_torque/cnt, min_torque, max_torque);
    }
}

void OptimizeValvePose::recursiveSearch(boost::shared_ptr<Dragon::HydrusLikeRobotModel> robot_model, KDL::Rotation root_rot, sensor_msgs::JointState joint_state, std::vector<double> joint_uppers, int id, sensor_msgs::JointState& opt_joint_state, double& max_torque)
{
  if(id == joint_state.position.size())
    {
      double torque = maxValveTorque(robot_model, root_rot, joint_state, max_joint_torque_, max_thrust_);

      if(fabs(torque) > fabs(max_torque))
        {
          max_torque = torque;
          opt_joint_state = joint_state;

          // std::stringstream ss;
          // for(int i = 0; i < opt_joint_state.position.size(); i++)
          //   ss << "(" << opt_joint_state.name.at(i) << ": " << opt_joint_state.position.at(i) << ")";
          // ROS_INFO_STREAM("thread: " << omp_get_thread_num() << ", update the optimize joint state: " << ss.str() << ", generate valve torque of " << max_torque);
        }
    }
  else
    {
      while(joint_state.position.at(id) <= joint_uppers.at(id))
        {
          recursiveSearch(robot_model, root_rot, joint_state, joint_uppers, id+1, opt_joint_state, max_torque);

          if (joint_state.position.at(id) < joint_uppers.at(id) && joint_state.position.at(id) + joint_delta_ > joint_uppers.at(id))
            joint_state.position.at(id) = joint_uppers.at(id);
          else
            joint_state.position.at(id) += joint_delta_;
        }
    }
}

double OptimizeValvePose::maxValveTorque(boost::shared_ptr<Dragon::HydrusLikeRobotModel> robot_model, const KDL::Rotation& root_rot, const sensor_msgs::JointState& joint_state, double max_joint_torque, double max_thrust,  bool verbose)
{
  // update the robot model
  robot_model->setCogDesireOrientation(root_rot); // update the cog orientation
  robot_model->updateRobotModel(joint_state);
  if(verbose) ROS_INFO_STREAM("hover thrust: " << robot_model->getStaticThrust().transpose());

  robot_model->calcJointTorque(true);
  if(verbose) ROS_INFO_STREAM("hover joint torque: " << robot_model->getJointTorque().transpose());

  geometry_msgs::Point offset;
  offset.x = 0;
  offset.y = 0;
  offset.z = 0;
  geometry_msgs::Wrench wrench;
  wrench.force.x = 0;
  wrench.force.y = 0;
  wrench.force.z = 0;

  double upper_torque = 2 * init_torque_;
  double lower_torque = 0;
  double torque = 0;

  int cnt = 0;
  while(true)
    {
      robot_model->updateRobotModel(joint_state);
      robot_model->calcJointTorque(false);

      torque = (upper_torque + lower_torque) / 2;
      KDL::Vector torque_v = root_rot * KDL::Vector(torque, 0, 0);
      //KDL::Vector torque_v = KDL::Vector(0, 0, torque); // debug, downward end-effector
      wrench.torque.x = torque_v.x();
      wrench.torque.y = torque_v.y();
      wrench.torque.z = torque_v.z();
      robot_model->resetExternalStaticWrench();
      robot_model->addExternalStaticWrench("valve", "link1", offset, wrench); // the reaction force, but not the exerting force

      robot_model->calcExternalWrenchCompThrust();
      Eigen::VectorXd ex_wrench_comp_f = robot_model->getExWrenchCompensateVectoringThrust();
      if(verbose) ROS_INFO_STREAM("ex_wrench_comp_f: " << ex_wrench_comp_f.transpose());
      robot_model->addCompThrustToStaticThrust();
      if(verbose) ROS_INFO_STREAM("final thrust: " << robot_model->getStaticThrust().transpose());

      robot_model->addCompThrustToJointTorque();

      Eigen::VectorXd joint_torque = Eigen::VectorXd::Zero(joint_state.position.size());

      std::stringstream ss;
      int j = 0;
      for(int i = 0; i < robot_model->getJointNames().size(); i++)
        {
          if(robot_model->getJointNames().at(i).find("joint") == 0)
            {
              joint_torque(j) = robot_model->getJointTorque()(i);
              ss << "(" << robot_model->getJointNames().at(i) << ", " << joint_torque(j) << ")";
              j ++;
            }
        }
      if(verbose) ROS_INFO_STREAM("joint torque: " << ss.str());

      // Note: we set heusristic priority to the joint torque
      double f_diff = robot_model->getStaticThrust().maxCoeff() - max_thrust;

      Eigen::VectorXd psuedo_joint_torque = joint_torque;

      // consider the stopper
      for(int i = 0; i < robot_model->getLinkJointNames().size(); i++)
        {
          if(fabs(joint_state.position.at(i) - M_PI/2) < 0.001 && joint_torque(i) < 0)
            psuedo_joint_torque(i) = 0;
          if(fabs(joint_state.position.at(i) + M_PI/2) < 0.001 && joint_torque(i) > 0)
            psuedo_joint_torque(i) = 0;
        }
      double t_diff = fabs(psuedo_joint_torque.cwiseAbs().maxCoeff()) - max_joint_torque;


      if(max_joint_torque > 0)
        {
          //ROS_WARN("torque constraint");
          if(fabs(t_diff) < 1e-3)
            {
              if(f_diff > 1e-3) // consider the thrust constraint
                {
                  max_joint_torque = 0;
                  upper_torque = torque;
                  lower_torque = torque/2;
                }
              else break;
            }
          else
            {
              if(t_diff > 0) upper_torque = torque;
              if(t_diff < 0)
                {
                  if(f_diff > 1e-3) // consider the thrust constraint
                    {
                      upper_torque = torque;
                      max_joint_torque = 0;
                    }
                  else lower_torque = torque;
                }
            }
        }
      else
        {
          //ROS_WARN("force constraint");
          if(fabs(f_diff) < 1e-3) break;
          if(f_diff > 0) upper_torque = torque;
          if(f_diff < 0) lower_torque = torque;
        }

      cnt ++;
      if(cnt > 100) break;

      if(verbose) ROS_INFO("cnt: %d, thrust diff: %f, joint torque diff: %f, valve torque: %f", cnt, f_diff, t_diff, torque);
    }

  if(verbose) ROS_INFO("max turn torque at end effector is %f", torque);
  return torque;
}

int main (int argc, char **argv)
{
  ros::init (argc, argv, "max_torque");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  OptimizeValvePose optimizer(nh, nhp);

  return 0;
}









