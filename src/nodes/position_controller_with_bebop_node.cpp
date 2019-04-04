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

#include <std_msgs/Empty.h>

#include "position_controller_with_bebop_node.h"

#include "bebop_simulator/parameters_ros.h"

#include "bebop_simulator_msgs/default_topics.h"
#include "bebop_msgs/default_topics.h"

namespace bebop_simulator {

PositionControllerWithBebopNode::PositionControllerWithBebopNode() {

    ROS_INFO_ONCE("Started position controller with Bebop");

    InitializeParams();

    ros::NodeHandle nh;

    cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,  &PositionControllerWithBebopNode::MultiDofJointTrajectoryCallback, this);

    odom_sub_ = nh.subscribe(bebop_msgs::default_topics::ODOM, 30, &PositionControllerWithBebopNode::OdomCallback, this);

    motor_velocity_reference_pub_ = nh.advertise<geometry_msgs::Twist>(bebop_msgs::default_topics::COMMAND_VEL, 1);

    takeoff_pub_ = nh.advertise<std_msgs::Empty>(bebop_msgs::default_topics::TAKE_OFF, 1);

    odometry_filtered_pub_ = nh.advertise<nav_msgs::Odometry>(bebop_simulator_msgs::default_topics::FILTERED_OUTPUT, 1);

    reference_angles_pub_ = nh.advertise<nav_msgs::Odometry>(bebop_simulator_msgs::default_topics::REFERENCE_ANGLES, 1);

    smoothed_reference_pub_  = nh.advertise<nav_msgs::Odometry>(bebop_simulator_msgs::default_topics::SMOOTHED_TRAJECTORY, 1);

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
  position_controller_.waypoint_filter_.SetTrajectoryPoint(eigen_reference);
  commands_.pop_front();

  if (n_commands >= 1) {
    waypointHasBeenPublished_ = true;
    ROS_INFO_ONCE("PositionController with AR.Drone node got first MultiDOFJointTrajectory message.");
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

  GetRosParameter(pnh, "dev_x",
                  position_controller_.filter_parameters_.dev_x_,
                  &position_controller_.filter_parameters_.dev_x_);
   
  GetRosParameter(pnh, "dev_y",
                  position_controller_.filter_parameters_.dev_y_,
                  &position_controller_.filter_parameters_.dev_y_);

  GetRosParameter(pnh, "dev_z",
                  position_controller_.filter_parameters_.dev_z_,
                  &position_controller_.filter_parameters_.dev_z_);

  GetRosParameter(pnh, "dev_vx",
                  position_controller_.filter_parameters_.dev_vx_,
                  &position_controller_.filter_parameters_.dev_vx_);

  GetRosParameter(pnh, "dev_vy",
                  position_controller_.filter_parameters_.dev_vy_,
                  &position_controller_.filter_parameters_.dev_vy_);

  GetRosParameter(pnh, "dev_vz",
                  position_controller_.filter_parameters_.dev_vz_,
                  &position_controller_.filter_parameters_.dev_vz_);

  GetRosParameter(pnh, "Qp/aa",
                  position_controller_.filter_parameters_.Qp_x_,
                  &position_controller_.filter_parameters_.Qp_x_);
   
  GetRosParameter(pnh, "Qp/bb",
                  position_controller_.filter_parameters_.Qp_y_,
                  &position_controller_.filter_parameters_.Qp_y_);

  GetRosParameter(pnh, "Qp/cc",
                  position_controller_.filter_parameters_.Qp_z_,
                  &position_controller_.filter_parameters_.Qp_z_);

  GetRosParameter(pnh, "Qp/dd",
                  position_controller_.filter_parameters_.Qp_vx_,
                  &position_controller_.filter_parameters_.Qp_vx_);

  GetRosParameter(pnh, "Qp/ee",
                  position_controller_.filter_parameters_.Qp_vy_,
                  &position_controller_.filter_parameters_.Qp_vy_);

  GetRosParameter(pnh, "Qp/ff",
                  position_controller_.filter_parameters_.Qp_vz_,
                  &position_controller_.filter_parameters_.Qp_vz_);

  position_controller_.filter_parameters_.Rp_(0,0) = position_controller_.filter_parameters_.dev_x_; 
  position_controller_.filter_parameters_.Rp_(1,1) = position_controller_.filter_parameters_.dev_y_; 
  position_controller_.filter_parameters_.Rp_(2,2) = position_controller_.filter_parameters_.dev_z_;
  position_controller_.filter_parameters_.Rp_(3,3) = position_controller_.filter_parameters_.dev_vx_; 
  position_controller_.filter_parameters_.Rp_(4,4) = position_controller_.filter_parameters_.dev_vy_;                     
  position_controller_.filter_parameters_.Rp_(5,5) = position_controller_.filter_parameters_.dev_vz_;

  position_controller_.filter_parameters_.Qp_(0,0) = position_controller_.filter_parameters_.Qp_x_; 
  position_controller_.filter_parameters_.Qp_(1,1) = position_controller_.filter_parameters_.Qp_y_; 
  position_controller_.filter_parameters_.Qp_(2,2) = position_controller_.filter_parameters_.Qp_z_;
  position_controller_.filter_parameters_.Qp_(3,3) = position_controller_.filter_parameters_.Qp_vx_; 
  position_controller_.filter_parameters_.Qp_(4,4) = position_controller_.filter_parameters_.Qp_vy_;                     
  position_controller_.filter_parameters_.Qp_(5,5) = position_controller_.filter_parameters_.Qp_vz_;


  position_controller_.SetControllerGains();
  position_controller_.SetVehicleParameters();
  position_controller_.SetFilterParameters();

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

      nav_msgs::Odometry odometry_filtered;
      position_controller_.GetOdometry(&odometry_filtered);
      odometry_filtered.header.stamp = odom_msg->header.stamp;
      odometry_filtered_pub_.publish(odometry_filtered);

      nav_msgs::Odometry reference_angles;
      position_controller_.GetReferenceAngles(&reference_angles);
      reference_angles.header.stamp = odom_msg->header.stamp;
      reference_angles_pub_.publish(reference_angles);

      nav_msgs::Odometry smoothed_reference;
      position_controller_.GetTrajectory(&smoothed_reference);
      smoothed_reference.header.stamp = odom_msg->header.stamp;
      smoothed_reference_pub_.publish(smoothed_reference);

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
    
    bebop_simulator::PositionControllerWithBebopNode position_controller_with_bebop_node;

    ros::spin();

    return 0;
}

