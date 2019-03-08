/*
 * Copyright 2019 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copytight 2019 Peter Griggs, MIT, USA
 * Copyright 2019 Pasquale Oppido, University of Sannio in Benevento, Italy
 * Copyright 2019 Luigi Iannelli, University of Sannio in Benevento, Italy
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

#include "position_controller_with_sphinx_node.h"

#include "bebopS/parameters_ros.h"
#include "bebopS_msgs/default_topics.h"
#include "bebopS/Sphinx_msgs.h"

#include "bebop_msgs/default_topics.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Empty.h"

#define MIN_RANGE_HOVER   0.90 //Minimum hovering interval before starting the timer
#define MAX_RANGE_HOVER   1.10 //Maximum hovering interval before starting the timer
#define WAITING_RANGE        3    //Waiting range before starting the controller

namespace bebopS {

PositionControllerWithSphinxNode::PositionControllerWithSphinxNode() {

    ROS_INFO_ONCE("Started position controller with Bebop");

    //The vehicle and controller parameters are initialized
    InitializeParams();

    ros::NodeHandle nh;

    //To get the trajectory to follow
    cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,  &PositionControllerWithSphinxNode::MultiDofJointTrajectoryCallback, this);

    //To get data coming from the Parrot-Sphinx data logger
    logger_sub_ = nh.subscribe(bebopS_msgs::default_topics::PARROT_SPHINX_LOGGER, 30, &PositionControllerWithSphinxNode::LoggerCallback, this);

    //To get data coming from the bebop_autonomy package odometry topic
    odom_sub_ = nh.subscribe(bebop_msgs::default_topics::ODOM, 30, &PositionControllerWithSphinxNode::OdomCallback, this);

    //To publish the command signals
    motor_velocity_reference_pub_ = nh.advertise<geometry_msgs::Twist>(bebop_msgs::default_topics::COMMAND_VEL, 1);
    
    //Needed for taking off the drone
    takeoff_pub_ = nh.advertise<std_msgs::Empty>(bebop_msgs::default_topics::TAKE_OFF, 1);

    //Needed for representing the variables into plots
    odometry_filtered_pub_ = nh.advertise<nav_msgs::Odometry>(bebopS_msgs::default_topics::FILTERED_OUTPUT, 1);

    //Just for data plotting
    reference_angles_pub_ = nh.advertise<nav_msgs::Odometry>(bebopS_msgs::default_topics::REFERENCE_ANGLES, 1);

    //Just for data plotting
    smoothed_reference_pub_  = nh.advertise<nav_msgs::Odometry>(bebopS_msgs::default_topics::SMOOTHED_TRAJECTORY, 1);

    //Just for data plotting
    uTerr_components_pub_  = nh.advertise<nav_msgs::Odometry>(bebopS_msgs::default_topics::U_TERR_COMPONENTS, 1);

    //Just for data plotting
    zVelocity_components_pub_ = nh.advertise<nav_msgs::Odometry>(bebopS_msgs::default_topics::Z_VELOCITY_COMPONENTS, 1);

    //Just for data plotting
    positionAndVelocityErrors_pub_= nh.advertise<nav_msgs::Odometry>(bebopS_msgs::default_topics::POSITION_AND_VELOCITY_ERRORS, 1);

    //Just for data plotting
    angularAndAngularVelocityErrors_pub_= nh.advertise<nav_msgs::Odometry>(bebopS_msgs::default_topics::ANGULAR_AND_ANGULAR_VELOCITY_ERRORS, 1);

}

//Destructor
PositionControllerWithSphinxNode::~PositionControllerWithSphinxNode(){}

void PositionControllerWithSphinxNode::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg){
  
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

void PositionControllerWithSphinxNode::InitializeParams() {
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

void PositionControllerWithSphinxNode::Publish(){
}

void PositionControllerWithSphinxNode::LoggerCallback(const bebopS::Sphinx_msgs& logger_msg) {

    ROS_INFO_ONCE("PositionController with Bebop got first logger odometry message.");

    if (waypointHasBeenPublished_ && bebop_autonomy_package_activated_){

	    // The data come from the Parrot-Sphinx logger are used to build a new odometry message, later used
	    // by the position controller libray
	    EigenOdometry odometry_logger;
      EigenOdometry attitude_logger;

	    // Drone position in inertial reference system
	    odometry_logger.position[0] = logger_msg.posX;
	    odometry_logger.position[1] = logger_msg.posY;
	    odometry_logger.position[2] = logger_msg.posZ;

	    // Drone orientation in radians (Euler angles)
	    attitude_logger.position[0] = logger_msg.attitudeX;
	    attitude_logger.position[1] = logger_msg.attitudeY;
	    attitude_logger.position[2] = logger_msg.attitudeZ;

	    // Drone linear velocity in the inertial reference system
	    odometry_logger.velocity[0] = logger_msg.velXENU;
	    odometry_logger.velocity[1] = logger_msg.velYENU;
	    odometry_logger.velocity[2] = logger_msg.velZENU;

	    // Drone angular velocity in the aircraft body center reference system
	    odometry_logger.angular_velocity[0] = logger_msg.angVelXABC;
	    odometry_logger.angular_velocity[1] = logger_msg.angVelYABC;
	    odometry_logger.angular_velocity[2] = logger_msg.angVelZABC;

      ROS_DEBUG("Roll: %f Pitch: %f Yaw: %f", attitude_logger.position[1], attitude_logger.position[2], attitude_logger.position[3]);
      ROS_DEBUG("AngX: %f AngY: %f AngZ: %f", odometry_logger.angular_velocity[0], odometry_logger.angular_velocity[1], odometry_logger.angular_velocity[2]);
      ROS_DEBUG("PosX: %f PosY: %f PosZ: %f", odometry_logger.position[0], odometry_logger.position[1], odometry_logger.position[2]);
      ROS_DEBUG("LinX: %f LinY: %f LinZ: %f", odometry_logger.velocity[0], odometry_logger.velocity[1], odometry_logger.velocity[2]);

      //For taking off the drone if it is not
      if (!takeOffMsgHasBeenSent_)        
          TakeOff();

      //Check drone hovers one meter from the ground
      if(odometry_logger.position[2] > MIN_RANGE_HOVER && odometry_logger.position[2] < MAX_RANGE_HOVER && takeOffMsgHasBeenSent_){

         if(first_time_ == 0)
          first_time_ = ros::Time::now().toSec();

         if((ros::Time::now().toSec() - first_time_) > WAITING_RANGE )
          isHovering_ = true;
      }
      else{
        first_time_ = 0;
      }
      
      //creating a new twist message. twist_msg is used to send the command signals. The Bebop command signals
      //are computed later the drone took off.
	    if (takeOffMsgHasBeenSent_ && isHovering_){
         
          position_controller_.SetOdomFromLogger(odometry_logger, attitude_logger);

          geometry_msgs::Twist ref_command_signals;
	        position_controller_.CalculateCommandSignals(&ref_command_signals);
	        motor_velocity_reference_pub_.publish(ref_command_signals);          
      }

      //The code reported below is used to plot the data when the simulation is running
      nav_msgs::Odometry odometry_filtered;
      ros::Time headerStamp = ros::Time::now();
      position_controller_.GetOdometry(&odometry_filtered);
      odometry_filtered.header.stamp = headerStamp;
      odometry_filtered_pub_.publish(odometry_filtered);

      // Just for data plotting
      nav_msgs::Odometry reference_angles;
      position_controller_.GetReferenceAngles(&reference_angles);
      reference_angles.header.stamp = headerStamp;
      reference_angles_pub_.publish(reference_angles);

      // Just for data plotting
      nav_msgs::Odometry smoothed_reference;
      position_controller_.GetTrajectory(&smoothed_reference);
      smoothed_reference.header.stamp = headerStamp;
      smoothed_reference_pub_.publish(smoothed_reference);

	    // Just for data plotting
	    nav_msgs::Odometry uTerr_components;
	    position_controller_.GetUTerrComponents(&uTerr_components);
	    uTerr_components.header.stamp = headerStamp;
	    uTerr_components_pub_.publish(uTerr_components);

	    // Just for data plotting
	    nav_msgs::Odometry zVelocity_components;
	    position_controller_.GetVelocityAlongZComponents(&zVelocity_components);
	    zVelocity_components.header.stamp = headerStamp;
	    zVelocity_components_pub_.publish(zVelocity_components);

      // Just for data plotting
	    nav_msgs::Odometry positionAndVelocityErrors;
	    position_controller_.GetPositionAndVelocityErrors(&positionAndVelocityErrors);
	    positionAndVelocityErrors.header.stamp = headerStamp;
	    positionAndVelocityErrors_pub_.publish(positionAndVelocityErrors);

	    // Just for data plotting
	    nav_msgs::Odometry angularAndAngularVelocityErrors;
	    position_controller_.GetAngularAndAngularVelocityErrors(&angularAndAngularVelocityErrors);
	    angularAndAngularVelocityErrors.header.stamp = headerStamp;
	    angularAndAngularVelocityErrors_pub_.publish(angularAndAngularVelocityErrors);

    }

}

void PositionControllerWithSphinxNode::OdomCallback(const nav_msgs::OdometryConstPtr& odom_msg) {

    ROS_INFO_ONCE("PositionController with Bebop got first odometry message.");

    //The boolean variable allows to send the TakeOff message until the connection with the bebop autonomy package is fine 
    if(!bebop_autonomy_package_activated_)
       bebop_autonomy_package_activated_ = true;

    if (waypointHasBeenPublished_){

        //These functions allow to put the odometry message into the odometry variable --> _position, _orientation,_velocity_body,
        //_angular_velocity
	      EigenOdometry odom;
	      eigenOdometryFromMsg(odom_msg, &odom);
	      position_controller_.SetOdom(odom);

    }	 
}

void PositionControllerWithSphinxNode::TakeOff(){

    ROS_INFO_ONCE("PositionController with Bebop sent take off message.");

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
    
    bebopS::PositionControllerWithSphinxNode position_controller_with_bebop_node;

    ros::spin();

    return 0;
}

