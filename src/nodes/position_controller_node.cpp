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
#include <math.h>

#include "position_controller_node.h"

#include "teamsannio_med_control/parameters_ros.h"
#include "teamsannio_msgs/default_topics.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Empty.h"

#include "teamsannio_med_control/position_controller.h"

namespace teamsannio_med_control {

//Constructor
PositionControllerNode::PositionControllerNode() {

    ROS_INFO_ONCE("Started position controller");
    //The vehicle and controller parameters are initialized
    InitializeParams();
    ros::NodeHandle nh;
    //To get the trajectory to follow
    cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,  &PositionControllerNode::MultiDofJointTrajectoryCallback, this);
    //To get data coming from the the virtual odometry sensor
    takeoff_pub_ = nh.advertise<std_msgs::Empty>("/bebop/takeoff", 1);
    piloting_pub_ = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
    ROS_INFO("initialized publishers!");
    odometry_sub_ = nh.subscribe("/bebop/odom", 1, &PositionControllerNode::OdometryCallback, this, ros::TransportHints().tcpNoDelay(true));
    ROS_INFO("initialized subscribers");

}

//Destructor
PositionControllerNode::~PositionControllerNode(){}

void PositionControllerNode::SendTakeoffMsg() {
    // TODO: callback logic for takeoff
    ROS_INFO("making the drone takeoff");
    std_msgs::Empty takeoffMsg;
    takeoff_pub_.publish(takeoffMsg);
    hasTakenOff_ = true;
}

void PositionControllerNode::SendPilotMsg() {
    // TODO: callback logic for piloting (cmd_vel topic)
    // publish to cmd_vel with uT, phiR, thetaR, and psiR (0)
    // &control_.uT, &control_.phiR, &control_.thetaR, &u_x, &u_y, &u_z, &u_Terr
    geometry_msgs::Twist pilotMsg;
    geometry_msgs::Vector3 linearVector;
    geometry_msgs::Vector3 angularVector;

    // TODO: move these constants to other file
    double maxTiltAngle = M_PI / 2;
    // todo: change this speed according to pasquale
    double maxVerticalSpeed = 148.788;

    double* values;
    values = position_controller_.GetControllerOuputs();

    ROS_INFO("the expected phi is: [%lf], expected theta is [%lf], expected thrust is [%lf]", values[0], values[1], values[2]);

    // all values for vector fields need to be in [-1,...,1]
    // roll angle, phi
    //linearVector.y = values[0] / maxTiltAngle;
    linearVector.y = values[0];
    // pitch angle, theta
    //linearVector.x = values[1] / maxTiltAngle;
    linearVector.x = values[1];
    // vertical velocity, 
    linearVector.z = values[2] / maxVerticalSpeed;
    // yaw angle, should just be 0
    angularVector.z = 0.0;

    pilotMsg.linear = linearVector;
    pilotMsg.angular = angularVector;
    piloting_pub_.publish(pilotMsg);
}

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


}

void PositionControllerNode::Publish(){
}


/*
    Main callback function for the 
*/
void PositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("PositionController got first odometry message.");

    if (waypointHasBeenPublished_){
        ROS_INFO("Waypoint has been published");
        if (!hasTakenOff_) {
            SendTakeoffMsg();
        }

	    //These functions allow to put the odometry message into the odometry variable --> _position, _orientation,_velocit_body,
        //_angular_velocity
	    EigenOdometry odometry;
	    eigenOdometryFromMsg(odometry_msg, &odometry);
	    position_controller_.SetOdometry(odometry);

	    Eigen::Vector4d ref_rotor_velocities;
	    // position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

	    position_controller_.CallPosController();
        SendPilotMsg();

    }	 
}


}

int main(int argc, char** argv){
    ros::init(argc, argv, "position_controller_node");

    ros::NodeHandle nh2;
    
    teamsannio_med_control::PositionControllerNode position_controller_node;

    ros::spin();

    return 0;
}

