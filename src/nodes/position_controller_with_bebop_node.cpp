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
#include <ros/console.h> 

#include <std_msgs/Empty.h>

#include "position_controller_with_bebop_node.h"
#include "teamsannio_med_control/parameters_ros.h"
#include "bebop_msgs/default_topics.h"

namespace teamsannio_med_control {

PositionControllerWithBebopNode::PositionControllerWithBebopNode() {

    ROS_INFO_ONCE("Started position controller with Bebop");

    InitializeParams();

    ros::NodeHandle nh;

    cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,  &PositionControllerWithBebopNode::MultiDofJointTrajectoryCallback, this);

    odom_sub_ = nh.subscribe(bebop_msgs::default_topics::ODOM, 30, &PositionControllerWithBebopNode::OdomCallback, this);

    motor_velocity_reference_pub_ = nh.advertise<geometry_msgs::Twist>(bebop_msgs::default_topics::COMMAND_VEL, 1);

    takeoff_pub_ = nh.advertise<std_msgs::Empty>(bebop_msgs::default_topics::TAKE_OFF, 1);

}

PositionControllerWithBebopNode::~PositionControllerWithBebopNode(){}

void PositionControllerWithBebopNode::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg){
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  // We can trigger the first command immediately.
  position_controller_.SetTrajectoryPoint(eigen_reference);
  commands_.pop_front();

  if (n_commands >= 1) {
    waypointHasBeenPublished_ = true;
    ROS_INFO("PositionController with AR.Drone node got first MultiDOFJointTrajectory message.");
  }
}

void PositionControllerWithBebopNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Read parameters from rosparam.
  GetRosParameter(pnh, "beta_xy/beta_x",
                  position_controller_.controller_parameters_.beta_xy_.x(),
                  &position_controller_.controller_parameters_.beta_xy_.x());
  GetRosParameter(pnh, "beta_xy/beta_y",
                  position_controller_.controller_parameters_.beta_xy_.y(),
                  &position_controller_.controller_parameters_.beta_xy_.y());
  GetRosParameter(pnh, "beta_z/beta_z",
                  position_controller_.controller_parameters_.beta_z_,
                  &position_controller_.controller_parameters_.beta_z_);
  
  GetRosParameter(pnh, "beta_phi/beta_phi",
                  position_controller_.controller_parameters_.beta_phi_,
                  &position_controller_.controller_parameters_.beta_phi_);
  GetRosParameter(pnh, "beta_theta/beta_theta",
                  position_controller_.controller_parameters_.beta_theta_,
                  &position_controller_.controller_parameters_.beta_theta_);
  GetRosParameter(pnh, "beta_psi/beta_psi",
                  position_controller_.controller_parameters_.beta_psi_,
                  &position_controller_.controller_parameters_.beta_psi_);

  GetRosParameter(pnh, "mu_xy/mu_x",
                  position_controller_.controller_parameters_.mu_xy_.x(),
                  &position_controller_.controller_parameters_.mu_xy_.x());
  GetRosParameter(pnh, "mu_xy/mu_y",
                  position_controller_.controller_parameters_.mu_xy_.y(),
                  &position_controller_.controller_parameters_.mu_xy_.y());
  GetRosParameter(pnh, "mu_z/mu_z",
                  position_controller_.controller_parameters_.mu_z_,
                  &position_controller_.controller_parameters_.mu_z_);
  
  GetRosParameter(pnh, "mu_phi/mu_phi",
                  position_controller_.controller_parameters_.mu_phi_,
                  &position_controller_.controller_parameters_.mu_phi_);
  GetRosParameter(pnh, "mu_theta/mu_theta",
                  position_controller_.controller_parameters_.mu_theta_,
                  &position_controller_.controller_parameters_.mu_theta_);
  GetRosParameter(pnh, "mu_psi/mu_psi",
                  position_controller_.controller_parameters_.mu_psi_,
                  &position_controller_.controller_parameters_.mu_psi_);

  GetVehicleParameters(pnh, &position_controller_.vehicle_parameters_);

  position_controller_.SetControllerGains();
  position_controller_.SetVehicleParameters();

}

void PositionControllerWithBebopNode::Publish(){
}

void PositionControllerWithBebopNode::OdomCallback(const nav_msgs::OdometryConstPtr& odom_msg) {

    ROS_INFO_ONCE("PositionController with Bebop got first odometry message.");

    if (waypointHasBeenPublished_){

	    EigenOdometry odom;
	    eigenOdometryFromMsg(odom_msg, &odom);
	    position_controller_.SetOdom(odom);

            if (!takeOffMsgHasBeenSent_)            
                TakeOff();

	    geometry_msgs::Twist ref_command_signals;
	    position_controller_.CalculateCommandSignals(&ref_command_signals);
	    motor_velocity_reference_pub_.publish(ref_command_signals);
    }	 
}

void PositionControllerWithBebopNode::TakeOff(){

    //The drone takes off from the ground only if the waypoint to reach has been published and the IMU
    //message has been received. In this way, we are sure the communication is done
    
    std_msgs::Empty empty_msg;
    takeoff_pub_.publish(empty_msg);

    takeOffMsgHasBeenSent_ = true;

}

}

int main(int argc, char** argv){
    ros::init(argc, argv, "position_controller_with_bebop_node");

    ros::NodeHandle nh2;
    
    teamsannio_med_control::PositionControllerWithBebopNode position_controller_with_bebop_node;

    ros::spin();

    return 0;
}

