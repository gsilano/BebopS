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

#include "bebop_simulator/parameters_ros.h"
#include "bebop_simulator_msgs/default_topics.h"

namespace bebop_simulator {

//Constructor
PositionControllerNode::PositionControllerNode() {

    ROS_INFO_ONCE("Started position controller");

    //The vehicle and controller parameters are initialized
    InitializeParams();

    ros::NodeHandle nh;

    //To get the trajectory to follow
    cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,  &PositionControllerNode::MultiDofJointTrajectoryCallback, this);

    //To get data coming from the the virtual odometry sensor
    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &PositionControllerNode::OdometryCallback, this, ros::TransportHints().tcpNoDelay(true));

    //To publish the propellers angular speed
    motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    //Useful to compare the results obtained by using the noised and biased virtual odometry sensor
    odometry_sub_gt_ = nh.subscribe(bebop_simulator_msgs::default_topics::ODOMETRY_GT, 1, &PositionControllerNode::OdometryGTCallback, this);

    //Need to represent the variables into the plots
    odometry_filtered_pub_ = nh.advertise<nav_msgs::Odometry>(bebop_simulator_msgs::default_topics::FILTERED_OUTPUT, 1);

    //Just for data plotting
    filtered_errors_pub_ = nh.advertise<nav_msgs::Odometry>(bebop_simulator_msgs::default_topics::STATE_ERRORS, 1);

    //Just for data plotting
    reference_angles_pub_ = nh.advertise<nav_msgs::Odometry>(bebop_simulator_msgs::default_topics::REFERENCE_ANGLES, 1);

    //Just for data plotting
    smoothed_reference_pub_  = nh.advertise<nav_msgs::Odometry>(bebop_simulator_msgs::default_topics::SMOOTHED_TRAJECTORY, 1);

    //Just for data plotting
    uTerr_components_pub_  = nh.advertise<nav_msgs::Odometry>(bebop_simulator_msgs::default_topics::U_TERR_COMPONENTS, 1);

    //Just for data plotting
    zVelocity_components_pub_ = nh.advertise<nav_msgs::Odometry>(bebop_simulator_msgs::default_topics::Z_VELOCITY_COMPONENTS, 1);

    //Just for data plotting
    positionAndVelocityErrors_pub_= nh.advertise<nav_msgs::Odometry>(bebop_simulator_msgs::default_topics::POSITION_AND_VELOCITY_ERRORS, 1);

    //Just for data plotting
    angularAndAngularVelocityErrors_pub_= nh.advertise<nav_msgs::Odometry>(bebop_simulator_msgs::default_topics::ANGULAR_AND_ANGULAR_VELOCITY_ERRORS, 1);

}

//Destructor
PositionControllerNode::~PositionControllerNode(){}

void PositionControllerNode::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);

  // We can trigger the first command immediately.
  position_controller_.SetTrajectoryPoint(eigen_reference);

  if (n_commands >= 1) {
    waypointHasBeenPublished_ = true;
    ROS_INFO_ONCE("PositionController got first MultiDOFJointTrajectory message.");
  }
}

void PositionControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Read parameters from rosparam. The parameters are read by the YAML file and they
  // are used to create the "controller_parameters_" object
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
				  
  GetRosParameter(pnh, "U_xyz/U_x",
                  position_controller_.controller_parameters_.U_q_.x(),
                  &position_controller_.controller_parameters_.U_q_.x());
  GetRosParameter(pnh, "U_xyz/U_y",
                  position_controller_.controller_parameters_.U_q_.y(),
                  &position_controller_.controller_parameters_.U_q_.y());
  GetRosParameter(pnh, "U_xyz/U_z",
                  position_controller_.controller_parameters_.U_q_.z(),
                  &position_controller_.controller_parameters_.U_q_.z());

  //Analogously, the object "vehicle_parameters_" is created
  GetVehicleParameters(pnh, &position_controller_.vehicle_parameters_);

  //Waypoint Filter parameters
  GetRosParameter(pnh, "Tsf",
                  position_controller_.waypoint_filter_parameters_.tsf_,
                  &position_controller_.waypoint_filter_parameters_.tsf_);

  GetRosParameter(pnh, "H",
                  position_controller_.waypoint_filter_parameters_.h_,
                  &position_controller_.waypoint_filter_parameters_.h_);

  //The object "filter_parameters_"
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

  // The controller gains, vehicle, filter and waypoint paramters are set
  position_controller_.SetControllerGains();
  position_controller_.SetVehicleParameters();
  position_controller_.SetFilterParameters();
  position_controller_.SetWaypointFilterParameters();

  //Reading the parameters come from the launch file
  bool waypointFilterActive;
  bool dataStoringActive;
  bool EKFActive;
  double dataStoringTime;
  std::string user;

  if (pnh.getParam("user_account", user)){
	  ROS_INFO("Got param 'user_account': %s", user.c_str());
	  position_controller_.user_ = user;
  }
  else
      ROS_ERROR("Failed to get param 'user'");

  if (pnh.getParam("waypoint_filter", waypointFilterActive)){
    ROS_INFO("Got param 'waypoint_filter': %d", waypointFilterActive);
    position_controller_.waypointFilter_active_ = waypointFilterActive;
  }
  else
      ROS_ERROR("Failed to get param 'waypoint_filter'");

  if (pnh.getParam("csvFilesStoring", dataStoringActive)){
	  ROS_INFO("Got param 'csvFilesStoring': %d", dataStoringActive);
	  position_controller_.dataStoring_active_ = dataStoringActive;
  }
  else
      ROS_ERROR("Failed to get param 'csvFilesStoring'");

  if (pnh.getParam("EKFActive", EKFActive)){
    ROS_INFO("Got param 'EKFActive': %d", EKFActive);
    position_controller_.EKF_active_ = EKFActive;
  }
  else
      ROS_ERROR("Failed to get param 'EKFActive'");

  if (pnh.getParam("csvFilesStoringTime", dataStoringTime)){
	  ROS_INFO("Got param 'csvFilesStoringTime': %f", dataStoringTime);
	  position_controller_.dataStoringTime_ = dataStoringTime;
  }
  else
      ROS_ERROR("Failed to get param 'csvFilesStoringTime'");

  position_controller_.SetLaunchFileParameters();


}

void PositionControllerNode::Publish(){
}

// Odometry ground truth callback
void PositionControllerNode::OdometryGTCallback(const nav_msgs::OdometryConstPtr& odometry_msg_gt) {

    ROS_INFO_ONCE("PositionController got first odometry ground truth message.");

    if (waypointHasBeenPublished_){

       EigenOdometry odometry_gt;
       eigenOdometryFromMsg(odometry_msg_gt, &odometry_gt);

       odometry_gt_.pose.pose.position.x = odometry_gt.position[0];
       odometry_gt_.pose.pose.position.y = odometry_gt.position[1];
       odometry_gt_.pose.pose.position.z = odometry_gt.position[2];
       odometry_gt_.twist.twist.linear.x = odometry_gt.velocity[0];
       odometry_gt_.twist.twist.linear.y = odometry_gt.velocity[1];
       odometry_gt_.twist.twist.linear.z = odometry_gt.velocity[2];

    }
}

// Odometry callback
void PositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("PositionController got first odometry message.");

    if (waypointHasBeenPublished_){

	      //These functions allow to put the odometry message into the odometry variable --> _position, _orientation,_velocit_body,
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
	      //for all propellers, we put them into actuator_msg so they will later be used to control the drone.
	      for (int i = 0; i < ref_rotor_velocities.size(); i++)
	        actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
              
        actuator_msg->header.stamp = odometry_msg->header.stamp;
        motor_velocity_reference_pub_.publish(actuator_msg);

	      //The code reported below is used to plot the data when the simulation is running
	      nav_msgs::Odometry odometry_filtered;
	      position_controller_.GetOdometry(&odometry_filtered);
	      odometry_filtered.header.stamp = odometry_msg->header.stamp;
	      odometry_filtered_pub_.publish(odometry_filtered);

	      // Just for data plotting
	      nav_msgs::Odometry reference_angles;
	      position_controller_.GetReferenceAngles(&reference_angles);
	      reference_angles.header.stamp = odometry_msg->header.stamp;
	      reference_angles_pub_.publish(reference_angles);

	      // Just for data plotting
	      nav_msgs::Odometry smoothed_reference;
	      position_controller_.GetTrajectory(&smoothed_reference);
	      smoothed_reference.header.stamp = odometry_msg->header.stamp;
	      smoothed_reference_pub_.publish(smoothed_reference);

	      // Just for data plotting
	      nav_msgs::Odometry uTerr_components;
	      position_controller_.GetUTerrComponents(&uTerr_components);
	      uTerr_components.header.stamp = odometry_msg->header.stamp;
	      uTerr_components_pub_.publish(uTerr_components);

	      // Just for data plotting
	      nav_msgs::Odometry zVelocity_components;
	      position_controller_.GetVelocityAlongZComponents(&zVelocity_components);
	      zVelocity_components.header.stamp = odometry_msg->header.stamp;
	      zVelocity_components_pub_.publish(zVelocity_components);

	      // Just for data plotting
	      nav_msgs::Odometry positionAndVelocityErrors;
	      position_controller_.GetPositionAndVelocityErrors(&positionAndVelocityErrors);
	      positionAndVelocityErrors.header.stamp = odometry_msg->header.stamp;
	      positionAndVelocityErrors_pub_.publish(positionAndVelocityErrors);

	      // Just for data plotting
	      nav_msgs::Odometry angularAndAngularVelocityErrors;
	      position_controller_.GetAngularAndAngularVelocityErrors(&angularAndAngularVelocityErrors);
	      angularAndAngularVelocityErrors.header.stamp = odometry_msg->header.stamp;
	      angularAndAngularVelocityErrors_pub_.publish(angularAndAngularVelocityErrors);

	      // Just for data plotting
	      nav_msgs::Odometry filtered_errors;
	      filtered_errors.pose.pose.position.x = odometry_filtered.pose.pose.position.x - odometry_gt_.pose.pose.position.x;
	      filtered_errors.pose.pose.position.y = odometry_filtered.pose.pose.position.y - odometry_gt_.pose.pose.position.y;
	      filtered_errors.pose.pose.position.z = odometry_filtered.pose.pose.position.z - odometry_gt_.pose.pose.position.z;
	      filtered_errors.twist.twist.linear.x = odometry_filtered.twist.twist.linear.x - odometry_gt_.twist.twist.linear.x;
	      filtered_errors.twist.twist.linear.y = odometry_filtered.twist.twist.linear.y - odometry_gt_.twist.twist.linear.y;
	      filtered_errors.twist.twist.linear.z = odometry_filtered.twist.twist.linear.z - odometry_gt_.twist.twist.linear.z;

	      filtered_errors.header.stamp = odometry_msg->header.stamp;
	      filtered_errors_pub_.publish(filtered_errors);

    }	 
}


}

int main(int argc, char** argv){
    ros::init(argc, argv, "position_controller_node");

    ros::NodeHandle nh2;
    
    bebop_simulator::PositionControllerNode position_controller_node;

    ros::spin();

    return 0;
}

