/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Pasquale Oppido, University of Sannio in Benevento, Italy
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <ros/console.h> 

#include "position_controller_node.h"

#include "teamsannio_med_control/parameters_ros.h"

namespace teamsannio_med_control {

PositionControllerNode::PositionControllerNode() {
    InitializeParams();

    ros::NodeHandle nh;

    cmd_pose_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_POSE, 1, &PositionControllerNode::CommandPoseCallback, this);
    
    cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,  &PositionControllerNode::MultiDofJointTrajectoryCallback, this);

    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &PositionControllerNode::OdometryCallback, this);

    motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    command_timer_ = nh.createTimer(ros::Duration(0), &PositionControllerNode::TimedCommandCallback, this,
                                  true, false);

}

PositionControllerNode::~PositionControllerNode(){}

void PositionControllerNode::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  ROS_INFO("PositionController got first MultiDOFJointTrajectory message.");

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  for (size_t i = 1; i < n_commands; ++i) {
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];

    mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

    commands_.push_back(eigen_reference);
    command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
  }

  // We can trigger the first command immediately.
  position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void PositionControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void PositionControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Read parameters from rosparam.
  GetRosParameter(pnh, "xy_gain_kp/beta_x",
                  position_controller_.controller_parameters_.xy_gain_kp_.x(),
                  &position_controller_.controller_parameters_.xy_gain_kp_.x());
  GetRosParameter(pnh, "xy_gain_kp/beta_y",
                  position_controller_.controller_parameters_.xy_gain_kp_.y(),
                  &position_controller_.controller_parameters_.xy_gain_kp_.y());
  GetRosParameter(pnh, "z_gain_kp/beta_z",
                  position_controller_.controller_parameters_.z_gain_kp_,
                  &position_controller_.controller_parameters_.z_gain_kp_);
  
  GetRosParameter(pnh, "roll_gain_kp/beta_roll",
                  position_controller_.controller_parameters_.roll_gain_kp_,
                  &position_controller_.controller_parameters_.roll_gain_kp_);
  GetRosParameter(pnh, "pitch_gain_kp/beta_pitch",
                  position_controller_.controller_parameters_.pitch_gain_kp_,
                  &position_controller_.controller_parameters_.pitch_gain_kp_);
  GetRosParameter(pnh, "yaw_rate_gain_kp/beta_yaw",
                  position_controller_.controller_parameters_.yaw_rate_gain_kp_,
                  &position_controller_.controller_parameters_.yaw_rate_gain_kp_);

  GetRosParameter(pnh, "mu_xy/mu_x",
                  position_controller_.controller_parameters_.mu_xy_.x(),
                  &position_controller_.controller_parameters_.mu_xy_.x());
  GetRosParameter(pnh, "mu_xy/mu_y",
                  position_controller_.controller_parameters_.mu_xy_.y(),
                  &position_controller_.controller_parameters_.mu_xy_.y());
  GetRosParameter(pnh, "mu_z/mu_z",
                  position_controller_.controller_parameters_.mu_z_,
                  &position_controller_.controller_parameters_.mu_z_);
  
  GetRosParameter(pnh, "mu_roll/mu_roll",
                  position_controller_.controller_parameters_.mu_roll_,
                  &position_controller_.controller_parameters_.mu_roll_);
  GetRosParameter(pnh, "mu_pitch/mu_pitch",
                  position_controller_.controller_parameters_.mu_pitch_,
                  &position_controller_.controller_parameters_.mu_pitch_);
  GetRosParameter(pnh, "mu_yaw_rate/mu_yaw",
                  position_controller_.controller_parameters_.mu_yaw_,
                  &position_controller_.controller_parameters_.mu_yaw_);

  GetVehicleParameters(pnh, &position_controller_.vehicle_parameters_);
  
}

void PositionControllerNode::Publish(){
}

void PositionControllerNode::CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg){

    // Clear all pending commands.
    command_timer_.stop();
    commands_.clear();
    command_waiting_times_.clear();

    mav_msgs::EigenTrajectoryPoint eigen_reference;
    mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
    commands_.push_front(eigen_reference);

    position_controller_.SetTrajectoryPoint(commands_.front());
    commands_.pop_front();
}

void PositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("PositionController got first odometry message.");

    //This functions allows to put the odometry message into the odometry variable--> _position, _orientation,_velocit_body,
    //_angular_velocity
    EigenOdometry odometry;
    eigenOdometryFromMsg(odometry_msg, &odometry);
    position_controller_.SetOdometry(odometry);
 
    Eigen::Vector4d ref_rotor_velocities;
    position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

    //creating a new mav message. actuator_msg is used to send the velocities of the propellers.  
    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

    //we use clear because we later want to be sure that we used the previously calculated velocity.
    actuator_msg->angular_velocities.clear();
    //for all propellers, we put them into actuator_msg so they will later be used to control the crazyflie.
    for (int i = 0; i < ref_rotor_velocities.size(); i++)
       actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
    actuator_msg->header.stamp = odometry_msg->header.stamp;

    //ROS_INFO("%f %f %f %f", actuator_msg->angular_velocities[0], actuator_msg->angular_velocities[1], 
    //		actuator_msg->angular_velocities[2], actuator_msg->angular_velocities[3]);
    
    motor_velocity_reference_pub_.publish(actuator_msg);
    
}


}

int main(int argc, char** argv){
    ros::init(argc, argv, "position_controller_node");
    
    ros::NodeHandle nh2;
    
    teamsannio_med_control::PositionControllerNode position_controller_node;

    ros::spin();

    return 0;
}
