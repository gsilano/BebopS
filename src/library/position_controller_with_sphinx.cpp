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

#include "bebopS/position_controller_with_sphinx.h"
#include "bebopS/transform_datatypes.h"
#include "bebopS/Matrix3x3.h"
#include "bebopS/Quaternion.h" 
#include "bebopS/stabilizer_types.h"

#include "bebop_msgs/default_topics.h"

#include <math.h> 
#include <time.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iterator>

#include <ros/ros.h>
#include <chrono>
#include <inttypes.h>

#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <std_msgs/Empty.h>


#define M_PI                      3.14159265358979323846  /* pi */
#define TsP                       10e-3  /* Position control sampling time */
#define TsA                       5e-3 /* Attitude control sampling time */
#define TsE                       5 /* Refresh landing time*/

#define MAX_TILT_ANGLE            30  /* Current tilt max in degree */
#define MAX_VERT_SPEED            1  /* Current max vertical speed in m/s */
#define MAX_ROT_SPEED             100 /* Current max rotation speed in degree/s */

#define MAX_POS_X                 3 /* Max position before emergency state along x-axis */
#define MAX_POS_Y                 3 /* Max position before emergency state along y-axis */
#define MAX_POS_Z                 3 /* Max position before emergency state along z-axis */
#define MAX_VEL_ERR               3 /* Max velocity error before emergency state */

namespace bebopS {

PositionControllerWithSphinx::PositionControllerWithSphinx()
    : controller_active_(false),
      dataStoring_active_(false),
      waypointFilter_active_(true),
      EKF_active_(false),
      dataStoringTime_(0),
      stateEmergency_(false),
      linearZ_(0),
      u_z_(0),
      e_x_(0),
      e_y_(0),
      e_z_(0),
      u_z_sum_(0),
      vel_command_(0),
      e_psi_sum_(0),
      yawRate_command_(0),
      dot_e_x_(0),
      dot_e_y_(0), 
      dot_e_z_(0),
      e_phi_(0),
      e_theta_(0),
      e_psi_(0),
      dot_e_phi_(0),
      dot_e_theta_(0), 
      dot_e_psi_(0),
      bf_(0),
      l_(0),
      bm_(0),
      m_(0),
      g_(0),
      Ix_(0),
      Iy_(0),
      Iz_(0),
      beta_x_(0),
      beta_y_(0),
      beta_z_(0),
      beta_phi_(0),
      beta_theta_(0),
      beta_psi_(0),
      alpha_x_(0),
      alpha_y_(0),
      alpha_z_(0),
      alpha_phi_(0),
      alpha_theta_(0),
      alpha_psi_(0),
      mu_x_(0),
      mu_y_(0),
      mu_z_(0),
      mu_phi_(0),
      mu_theta_(0),
      mu_psi_(0),
      K_x_1_(0),
      K_y_1_(0),
      K_z_1_(0),
      K_x_2_(0),
      K_y_2_(0),
      K_z_2_(0),
      lambda_x_(0),
      lambda_y_(0),
      lambda_z_(0),
      control_({0,0,0,0}), //pitch, roll, yaw rate, thrust
      state_({0,  //Position.x 
              0,  //Position.y
              0,  //Position.z
              0,  //Linear velocity x
              0,  //Linear velocity y
              0,  //Linear velocity z
              0,  //Quaternion x
              0,  //Quaternion y
              0,  //Quaternion z
              0,  //Quaternion w
              0,  //Angular velocity x
              0,  //Angular velocity y
              0}) //Angular velocity z)
              {  

            // Command signals initialization
            command_trajectory_.setFromYaw(0);
            command_trajectory_.position_W[0] = 0;
            command_trajectory_.position_W[1] = 0;
            command_trajectory_.position_W[2] = 0;

            //Odometry logger initialization
            odometry_from_logger_.position[0] = 0;
            odometry_from_logger_.position[1] = 0;
            odometry_from_logger_.position[2] = 0;

            odometry_from_logger_.velocity[0] = 0;
            odometry_from_logger_.velocity[1] = 0;
            odometry_from_logger_.velocity[2] = 0;

            odometry_from_logger_.angular_velocity[0] = 0;
            odometry_from_logger_.angular_velocity[1] = 0;
            odometry_from_logger_.angular_velocity[2] = 0;

            attitude_from_logger_.position[0] = 0;
            attitude_from_logger_.position[1] = 0;
            attitude_from_logger_.position[2] = 0;

            // Kalman filter's parameters initialization
            filter_parameters_.dev_x_ = 0;
            filter_parameters_.dev_y_ = 0;
            filter_parameters_.dev_z_ = 0;
            filter_parameters_.dev_vx_ = 0;
            filter_parameters_.dev_vy_ = 0;
            filter_parameters_.dev_vz_ = 0;
            filter_parameters_.Qp_x_ = 0;
            filter_parameters_.Qp_y_ = 0;
            filter_parameters_.Qp_z_ = 0;
            filter_parameters_.Qp_vx_ = 0;
            filter_parameters_.Qp_vy_ = 0;
            filter_parameters_.Qp_vz_ = 0;
            filter_parameters_.Rp_ = Eigen::MatrixXf::Zero(6,6);
            filter_parameters_.Qp_ = Eigen::MatrixXf::Identity(6,6);

            //For publishing landing and reset commands
            land_pub_ = n4_.advertise<std_msgs::Empty>(bebop_msgs::default_topics::LAND, 1);
            reset_pub_ = n4_.advertise<std_msgs::Empty>(bebop_msgs::default_topics::RESET, 1);

            // Timers set the outer and inner loops working frequency
            timer1_ = n1_.createTimer(ros::Duration(TsA), &PositionControllerWithSphinx::CallbackAttitude, this, false, true);
            timer2_ = n2_.createTimer(ros::Duration(TsP), &PositionControllerWithSphinx::CallbackPosition, this, false, true); 

}

//The library Destructor
PositionControllerWithSphinx::~PositionControllerWithSphinx() {}

//The callback saves data come from simulation into csv files
void PositionControllerWithSphinx::CallbackSaveData(const ros::TimerEvent& event){

   if(!dataStoring_active_){
      return;
   }

   ofstream fileControllerGains;
   ofstream fileVehicleParameters;
   ofstream fileControlSignals;
   ofstream fileReferenceAngles;
   ofstream fileVelocityErrors;
   ofstream fileDroneAttitude;
   ofstream fileTrajectoryErrors;
   ofstream fileAttitudeErrors;
   ofstream fileDerivativeAttitudeErrors;
   ofstream fileDroneAngularVelocitiesABC;
   ofstream fileDroneTrajectoryReference;
   ofstream fileDroneLinearVelocities;
   ofstream fileDronePosition;
   ofstream fileCommandSignalsBefore;
   ofstream fileCommandSignalsAfter;
   ofstream fileOdometryBebopAutonomyPackage;

   ROS_INFO("CallbackSaveData function is working. Time: %f seconds, %f nanoseconds", odometry_.timeStampSec, odometry_.timeStampNsec);
    
   fileControllerGains.open("/home/" + user_ + "/controllerGains.csv", std::ios_base::app);
   fileVehicleParameters.open("/home/" + user_ + "/vehicleParameters.csv", std::ios_base::app);
   fileControlSignals.open("/home/" + user_ + "/controlSignals.csv", std::ios_base::app);
   fileReferenceAngles.open("/home/" + user_ + "/referenceAngles.csv", std::ios_base::app);
   fileVelocityErrors.open("/home/" + user_ + "/velocityErrors.csv", std::ios_base::app);
   fileDroneAttitude.open("/home/" + user_ + "/droneAttitude.csv", std::ios_base::app);
   fileTrajectoryErrors.open("/home/" + user_ + "/trajectoryErrors.csv", std::ios_base::app);
   fileAttitudeErrors.open("/home/" + user_ + "/attitudeErrors.csv", std::ios_base::app);
   fileDerivativeAttitudeErrors.open("/home/" + user_ + "/derivativeAttitudeErrors.csv", std::ios_base::app);
   fileDroneAngularVelocitiesABC.open("/home/" + user_ + "/droneAngularVelocitiesABC.csv", std::ios_base::app);
   fileDroneTrajectoryReference.open("/home/" + user_ + "/droneTrajectoryReferences.csv", std::ios_base::app);
   fileDroneLinearVelocities.open("/home/" + user_ + "/droneLinearVelocities.csv", std::ios_base::app);
   fileDronePosition.open("/home/" + user_ + "/dronePosition.csv", std::ios_base::app);
   fileCommandSignalsBefore.open("/home/" + user_ + "/commandSignalsBeforeSaturation.csv", std::ios_base::app);
   fileCommandSignalsAfter.open("/home/" + user_ + "/commandSignalsAfterSaturation.csv", std::ios_base::app);
   fileOdometryBebopAutonomyPackage.open("/home/" + user_ + "/odometryFromBebopAutonomyPackage.csv", std::ios_base::app);

      // Saving vehicle parameters in a file
   fileControllerGains << beta_x_ << "," << beta_y_ << "," << beta_z_ << "," << alpha_x_ << "," << alpha_y_ << "," << alpha_z_ << "," << beta_phi_ << ","
    		  << beta_theta_ << "," << beta_psi_ << "," << alpha_phi_ << "," << alpha_theta_ << "," << alpha_psi_ << "," << mu_x_ << "," << mu_y_ << ","
			  << mu_z_ << "," << mu_phi_ << "," << mu_theta_ << "," << mu_psi_ << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

   // Saving vehicle parameters in a file
   fileVehicleParameters << bf_ << "," << l_ << "," << bm_ << "," << m_ << "," << g_ << "," << Ix_ << "," << Iy_ << "," << Iz_ << ","
    		  << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

   // Saving control signals in a file
   for (unsigned n=0; n < listControlSignals_.size(); ++n) {
       fileControlSignals << listControlSignals_.at( n );
   }

   // Saving the reference angles in a file
   for (unsigned n=0; n < listReferenceAngles_.size(); ++n) {
       fileReferenceAngles << listReferenceAngles_.at( n );
   }

   // Saving the velocity errors in a file
   for (unsigned n=0; n < listVelocityErrors_.size(); ++n) {
       fileVelocityErrors << listVelocityErrors_.at( n );
   }

   // Saving the drone attitude in a file
   for (unsigned n=0; n < listDroneAttitude_.size(); ++n) {
       fileDroneAttitude << listDroneAttitude_.at( n );
   }
 
   // Saving the trajectory errors in a file
   for (unsigned n=0; n < listTrajectoryErrors_.size(); ++n) {
       fileTrajectoryErrors << listTrajectoryErrors_.at( n );
   }

   // Saving the attitude errors in a file
   for (unsigned n=0; n < listAttitudeErrors_.size(); ++n) {
       fileAttitudeErrors << listAttitudeErrors_.at( n );
   }

   // Saving the derivative attitude errors in a file
   for (unsigned n=0; n < listDerivativeAttitudeErrors_.size(); ++n) {
       fileDerivativeAttitudeErrors << listDerivativeAttitudeErrors_.at( n );
   }

   // Saving the drone angular velocity in the aircraft body reference system
   for (unsigned n=0; n < listDroneAngularVelocitiesABC_.size(); ++n) {
       fileDroneAngularVelocitiesABC << listDroneAngularVelocitiesABC_.at( n );
   }

   // Saving the drone trajectory references (them coming from the waypoint filter)
   for (unsigned n=0; n < listDroneTrajectoryReference_.size(); ++n) {
       fileDroneTrajectoryReference << listDroneTrajectoryReference_.at( n );
   }

   // Saving drone linear velocity in the aircraft body center reference system
   for (unsigned n=0; n < listDroneLinearVelocities_.size(); ++n) {
       fileDroneLinearVelocities << listDroneLinearVelocities_.at( n );
   }

   // Saving the drone position along axes
   for (unsigned n=0; n < listDronePosition_.size(); ++n) {
       fileDronePosition << listDronePosition_.at( n );
   }

   // Saving command signals before
   for (unsigned n=0; n < listCommandSinglasBefore_.size(); ++n) {
       fileCommandSignalsBefore << listCommandSinglasBefore_.at( n );
   }

   // Saving command signals after
   for (unsigned n=0; n < listCommandSinglasAfter_.size(); ++n) {
       fileCommandSignalsAfter << listCommandSinglasAfter_.at( n );
   }

   // Saving control signals in a file
   for (unsigned n=0; n < listOdometryFromBebopAutonomyPackage_.size(); ++n) {
       fileOdometryBebopAutonomyPackage << listOdometryFromBebopAutonomyPackage_.at( n );
   }


   // Closing all opened files
   fileControllerGains.close ();
   fileVehicleParameters.close ();
   fileControlSignals.close ();
   fileReferenceAngles.close();
   fileVelocityErrors.close();
   fileDroneAttitude.close();
   fileTrajectoryErrors.close();
   fileAttitudeErrors.close();
   fileDerivativeAttitudeErrors.close();
   fileDroneAngularVelocitiesABC.close();
   fileDroneTrajectoryReference.close();
   fileDroneLinearVelocities.close();
   fileDronePosition.close();
   fileCommandSignalsBefore.close();
   fileCommandSignalsAfter.close();
   fileOdometryBebopAutonomyPackage.close();

   // To have a one shot storing
   dataStoring_active_ = false;

}

// The function moves the controller gains read from controller_bebop.yaml file to private variables of the class.
// These variables will be employed during the simulation
void PositionControllerWithSphinx::SetControllerGains(){

   beta_x_ = controller_parameters_.beta_xy_.x();
   beta_y_ = controller_parameters_.beta_xy_.y();
   beta_z_ = controller_parameters_.beta_z_;

   beta_phi_ = controller_parameters_.beta_phi_;
   beta_theta_ = controller_parameters_.beta_theta_;
   beta_psi_ = controller_parameters_.beta_psi_;

   alpha_x_ = 1 - beta_x_;
   alpha_y_ = 1 - beta_y_;
   alpha_z_ = 1 - beta_z_;

   alpha_phi_ = 1 - beta_phi_;
   alpha_theta_ = 1 - beta_theta_;
   alpha_psi_ = 1 - beta_psi_;

   mu_x_ = controller_parameters_.mu_xy_.x();
   mu_y_ = controller_parameters_.mu_xy_.y();
   mu_z_ = controller_parameters_.mu_z_;

   mu_phi_ = controller_parameters_.mu_phi_;
   mu_theta_ = controller_parameters_.mu_theta_;
   mu_psi_ = controller_parameters_.mu_psi_;  

   lambda_x_ = controller_parameters_.U_q_.x();
   lambda_y_ = controller_parameters_.U_q_.y();
   lambda_z_ = controller_parameters_.U_q_.z();
	  
   K_x_1_ = 1/mu_x_;
   K_x_2_ = -2 * (beta_x_/mu_x_);
	  
   K_y_1_ = 1/mu_y_;
   K_y_2_ = -2 * (beta_y_/mu_y_);
	  
   K_z_1_ = 1/mu_z_;
   K_z_2_ = -2 * (beta_z_/mu_z_);

}

// As SetControllerGains, the function is used to set the vehicle parameters into private variables of the class
void PositionControllerWithSphinx::SetVehicleParameters(){

   bf_ = vehicle_parameters_.bf_;
   l_ = vehicle_parameters_.armLength_;
   bm_ = vehicle_parameters_.bm_;
   m_ = vehicle_parameters_.mass_;
   g_ = vehicle_parameters_.gravity_;
   Ix_ = vehicle_parameters_.inertia_(0,0);
   Iy_ = vehicle_parameters_.inertia_(1,1);
   Iz_ = vehicle_parameters_.inertia_(2,2);

   // On the EKF object is invoked the method SeVehicleParameters. Such function allows to send the vehicle parameters to the EKF class. Then, they are employed to set the filter matrices
   extended_kalman_filter_bebop_.SetVehicleParameters(m_, g_);

}

// Reading parameters come frame launch file
void PositionControllerWithSphinx::SetLaunchFileParameters(){

   // The boolean variable is used to inactive the logging if it is not useful
   if(dataStoring_active_){

      // Time after which the data storing function is turned on
      timer3_ = n3_.createTimer(ros::Duration(dataStoringTime_), &PositionControllerWithSphinx::CallbackSaveData, this, false, true);

      // Cleaning the string vector contents
      listControlSignals_.clear();
      listReferenceAngles_.clear();
      listVelocityErrors_.clear();
      listDroneAttitude_.clear();
      listTrajectoryErrors_.clear();
      listAttitudeErrors_.clear();
      listDerivativeAttitudeErrors_.clear();
      listDroneAngularVelocitiesABC_.clear();
      listDroneTrajectoryReference_.clear();
      listDroneLinearVelocities_.clear();
      listDronePosition_.clear();
      listCommandSinglasBefore_.clear();
      listCommandSinglasAfter_.clear();
      listOdometryFromBebopAutonomyPackage_.clear();
	  
   }

}

void PositionControllerWithSphinx::SetFilterParameters(){

   // The function is used to move the filter parameters from the YAML file to the filter class
   extended_kalman_filter_bebop_.SetFilterParameters(&filter_parameters_);

}

// The functions is used to convert the drone attitude from quaternion to Euler angles
void PositionControllerWithSphinx::Quaternion2Euler(double* roll, double* pitch, double* yaw) const {
    assert(roll);
    assert(pitch);
    assert(yaw);

    double x, y, z, w;
    x = odometry_.orientation.x();
    y = odometry_.orientation.y();
    z = odometry_.orientation.z();
    w = odometry_.orientation.w();
    
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(*roll, *pitch, *yaw);
	
}

// When a new odometry message comes, the content of the message is stored in private variable. At the same time, the controller is going to be active.
// The attitude of the aircraft is computer (as we said before it move from quaterninon to Euler angles) and also the angular velocity is stored.
void PositionControllerWithSphinx::SetOdom(const EigenOdometry& odometry) {

   //+x forward, +y left, +z up, +yaw CCW
   odometry_ = odometry; 

   double roll, pitch, yaw;
   Quaternion2Euler(&roll, &pitch, &yaw);

   // Saving drone attitude in a file
   std::stringstream tempOdometryFromBebopAutonomyPackage;
   tempOdometryFromBebopAutonomyPackage << odometry_.position[0] << "," << odometry_.position[1] << "," << odometry_.position[2] << "," << roll << "," << pitch << "," << yaw << "," << odometry_.velocity[0] << "," << odometry_.velocity[1] << "," << odometry_.velocity[2] << "," << odometry_.angular_velocity[0] << "," << odometry_.angular_velocity[1] << "," << odometry_.angular_velocity[2] << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

   listOdometryFromBebopAutonomyPackage_.push_back(tempOdometryFromBebopAutonomyPackage.str());

}

// The function pass the odometry message built from the logger data
void PositionControllerWithSphinx::SetOdomFromLogger(const EigenOdometry& odometry_from_logger, const EigenOdometry& attitude_from_logger) {

    //+x forward, +y left, +z up, +yaw CCW
    odometry_from_logger_ = odometry_from_logger; 
    attitude_from_logger_ = attitude_from_logger; 

    // Drone attitude
    state_.attitude.roll = attitude_from_logger_.position[0]; 
    state_.attitude.pitch = attitude_from_logger_.position[1]; 
    state_.attitude.yaw = attitude_from_logger_.position[2];

    // Angular velocities are expressed in aircraft body frame reference system
    state_.angularVelocity.x = odometry_from_logger_.angular_velocity[0];
    state_.angularVelocity.y = odometry_from_logger_.angular_velocity[1];
    state_.angularVelocity.z = odometry_from_logger_.angular_velocity[2];

    //Enabling the controller and the waypoint filter initialization
    controller_active_= true;

}

// The function allows to set the waypoint filter parameters
void PositionControllerWithSphinx::SetWaypointFilterParameters(){

  waypoint_filter_.SetParameters(&waypoint_filter_parameters_);

}

// The function sets the filter trajectory points
void PositionControllerWithSphinx::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory_positionControllerNode) {

  // If the waypoint has been activated or not
  if(waypointFilter_active_){
    waypoint_filter_.SetTrajectoryPoint(command_trajectory_positionControllerNode);
  }
  else
    command_trajectory_ = command_trajectory_positionControllerNode;


}

// Just to plot the data during the simulation
void PositionControllerWithSphinx::GetTrajectory(nav_msgs::Odometry* smoothed_trajectory){
   assert(smoothed_trajectory);

   smoothed_trajectory->pose.pose.position.x = command_trajectory_.position_W[0];
   smoothed_trajectory->pose.pose.position.y = command_trajectory_.position_W[1];
   smoothed_trajectory->pose.pose.position.z = command_trajectory_.position_W[2];

}

// Just to plot the data during the simulation
void PositionControllerWithSphinx::GetOdometry(nav_msgs::Odometry* odometry_filtered){
   assert(odometry_filtered);

   *odometry_filtered = odometry_filtered_private_;

}

// Just to analyze the components that get uTerr variable
void PositionControllerWithSphinx::GetUTerrComponents(nav_msgs::Odometry* uTerrComponents){
  assert(uTerrComponents);

  uTerrComponents->pose.pose.position.x = ( (alpha_z_/mu_z_) * dot_e_z_);
  uTerrComponents->pose.pose.position.y = - ( (beta_z_/pow(mu_z_,2)) * e_z_);
  uTerrComponents->pose.pose.position.z = ( g_ + ( (alpha_z_/mu_z_) * dot_e_z_) - ( (beta_z_/pow(mu_z_,2)) * e_z_) );

}

// Just to analyze the position and velocity errors
void PositionControllerWithSphinx::GetPositionAndVelocityErrors(nav_msgs::Odometry* positionAndVelocityErrors){
   assert(positionAndVelocityErrors);

   positionAndVelocityErrors->pose.pose.position.x = e_x_;
   positionAndVelocityErrors->pose.pose.position.y = e_y_;
   positionAndVelocityErrors->pose.pose.position.z = e_z_;

   positionAndVelocityErrors->twist.twist.linear.x = dot_e_x_;
   positionAndVelocityErrors->twist.twist.linear.y = dot_e_y_;
   positionAndVelocityErrors->twist.twist.linear.z = dot_e_z_;

}

// Just to analyze the attitude and angular velocity errors
void PositionControllerWithSphinx::GetAngularAndAngularVelocityErrors(nav_msgs::Odometry* angularAndAngularVelocityErrors){
   assert(angularAndAngularVelocityErrors);

   angularAndAngularVelocityErrors->pose.pose.position.x = e_phi_;
   angularAndAngularVelocityErrors->pose.pose.position.y = e_theta_;
   angularAndAngularVelocityErrors->pose.pose.position.z = e_psi_;

   angularAndAngularVelocityErrors->twist.twist.linear.x = dot_e_phi_;
   angularAndAngularVelocityErrors->twist.twist.linear.y = dot_e_theta_;
   angularAndAngularVelocityErrors->twist.twist.linear.z = dot_e_psi_;

}

// Just to plot the data during the simulation
void PositionControllerWithSphinx::GetReferenceAngles(nav_msgs::Odometry* reference_angles){
   assert(reference_angles);

   double u_x, u_y, u_z, u_Terr;
   PosController(&control_.uT, &control_.phiR, &control_.thetaR, &u_x, &u_y, &u_z, &u_Terr);

   reference_angles->pose.pose.position.x = control_.phiR*180/M_PI;
   reference_angles->pose.pose.position.y = control_.thetaR*180/M_PI;
   reference_angles->pose.pose.position.z = control_.uT;

   reference_angles->twist.twist.linear.x = u_x;
   reference_angles->twist.twist.linear.y = u_y;
   reference_angles->twist.twist.linear.z = u_Terr;

}

// Just to plot the components make able to compute dot_e_z
void PositionControllerWithSphinx::GetVelocityAlongZComponents(nav_msgs::Odometry* zVelocity_components){
  assert(zVelocity_components);

   zVelocity_components->pose.pose.position.x = state_.linearVelocity.x;
   zVelocity_components->pose.pose.position.y = state_.linearVelocity.y;
   zVelocity_components->pose.pose.position.z = state_.linearVelocity.z;

   double phi, theta, psi;
   Quaternion2Euler(&phi, &theta, &psi);

   zVelocity_components->twist.twist.linear.x = (-sin(theta) * state_.linearVelocity.x);
   zVelocity_components->twist.twist.linear.y = ( sin(phi) * cos(theta) * state_.linearVelocity.y);
   zVelocity_components->twist.twist.linear.z = (cos(phi) * cos(theta) * state_.linearVelocity.z);

}

// The functions is used to get information about the estimated state when bias and noise affect both the accelerometer and angular rate measurements
void PositionControllerWithSphinx::SetOdometryEstimated() {

    extended_kalman_filter_bebop_.SetThrustCommand(control_.uT);

    //TODO: The EKF on data come from the Parrot-Sphinx data logger to be implemented
    extended_kalman_filter_bebop_.EstimatorSphinxLogger(&state_, &odometry_from_logger_, &attitude_from_logger_);

    ROS_DEBUG("PosX: %f PosY: %f PosZ: %f", state_.position.x, state_.position.y, state_.position.z);

}


// The function computes the command signals
void PositionControllerWithSphinx::CalculateCommandSignals(geometry_msgs::Twist* ref_command_signals) {
    assert(ref_command_signals);
    
    //this serves to inactivate the controller if we don't recieve a trajectory
    if(!controller_active_){
        ref_command_signals->linear.x = 0;
        ref_command_signals->linear.y = 0;
        ref_command_signals->linear.z = 0;
        ref_command_signals->angular.z = 0; 
        return;
    }
    
    double u_phi, u_theta, u_psi;
    double u_x, u_y, u_Terr;
    AttitudeController(&u_phi, &u_theta, &u_psi);
    PosController(&control_.uT, &control_.phiR, &control_.thetaR, &u_x, &u_y, &u_z_, &u_Terr);
 
    // Data storing section. It is activated if necessary
    if(dataStoring_active_){
      
      // Saving drone attitude in a file
      std::stringstream tempDroneAttitude;
      tempDroneAttitude << state_.attitude.roll << "," << state_.attitude.pitch << "," << state_.attitude.yaw << ","
          <<odometry_from_logger_.timeStampSec << "," << odometry_from_logger_.timeStampNsec << "\n";

      listDroneAttitude_.push_back(tempDroneAttitude.str());

      // Saving the drone angular velocity in the aircraft body reference system
      std:stringstream tempDroneAngularVelocitiesABC;
      tempDroneAngularVelocitiesABC << state_.angularVelocity.x << "," << state_.angularVelocity.y << "," << state_.angularVelocity.z
        << "," << odometry_from_logger_.timeStampSec << "," << odometry_from_logger_.timeStampNsec << "\n";

      listDroneAngularVelocitiesABC_.push_back(tempDroneAngularVelocitiesABC.str());
    }
    
    //The commands are normalized to take into account the real commands that can be send to the drone
    //Them range is between -1 and 1.
    double theta_ref_degree, phi_ref_degree;
    theta_ref_degree = control_.thetaR * (180/M_PI);
    phi_ref_degree = control_.phiR * (180/M_PI);
    
    double linearX, linearY, angularZ;
    linearX = theta_ref_degree/MAX_TILT_ANGLE;
    linearY = phi_ref_degree/MAX_TILT_ANGLE;
    CommandYawRate(&angularZ);
    
    // Data storing section. It is activated if necessary
    if(dataStoring_active_){
      
      // Saving command signals before saturating in a file
      std::stringstream tempCommandSignalsBefore;
      tempCommandSignalsBefore << linearX << "," << linearY << "," << linearZ_ << "," << angularZ << "," << odometry_from_logger_.timeStampSec << "," << odometry_from_logger_.timeStampNsec << "\n";

      listCommandSinglasBefore_.push_back(tempCommandSignalsBefore.str());

    }

    //The command signals are saturated to take into the SDK constrains in sending commands
    if(!(linearZ_ > -1 && linearZ_ < 1))
        if(linearZ_ > 1)
           linearZ_ = 1;
        else
           linearZ_ = -1;

    if(!(linearX > -1 && linearX < 1))
        if(linearX > 1)
           linearX = 1;
        else
           linearX = -1;

    if(!(linearY > -1 && linearY < 1))
        if(linearY > 1)
           linearY = 1;
        else
           linearY = -1;

    if(!(angularZ > -1 && angularZ < 1))
        if(angularZ > 1)
           angularZ = 1;
        else
           angularZ = -1;

    // Data storing section. It is activated if necessary
    if(dataStoring_active_){
      
      // Saving command signals in a file
      std::stringstream tempCommandSignalsAfter;
      tempCommandSignalsAfter << linearX << "," << linearY << "," << linearZ_ << "," << angularZ << "," << odometry_from_logger_.timeStampSec << "," << odometry_from_logger_.timeStampNsec << "\n";

      listCommandSinglasAfter_.push_back(tempCommandSignalsAfter.str());

    }

    ref_command_signals->linear.x = linearX;
    ref_command_signals->linear.y = linearY;
    ref_command_signals->linear.z = linearZ_;
    ref_command_signals->angular.z = angularZ; 

}

//The function integrates thrust to obtain the velocity command signal
void PositionControllerWithSphinx::CommandVelocity(double* linearZ){
    assert(linearZ);

    //Since u_z is a force, it has to be divided by the mas to get the vertical acceleration
    double u_z_internal = u_z_/m_;

    u_z_sum_ = u_z_sum_ + u_z_internal * TsP;
	
    *linearZ = u_z_sum_/MAX_VERT_SPEED;

    ROS_INFO("z_r %f, e_z: %f, u_z: %f, linearZ: %f ", command_trajectory_.position_W[2], e_z_, u_z_internal, *linearZ);

}

//The function integrates the angular acceleration along the z-axis to obtain the angular velocity 
void PositionControllerWithSphinx::CommandYawRate(double* yawRate_command){
    assert(yawRate_command);

    e_psi_sum_ = e_psi_sum_ + e_psi_ * TsA;

    *yawRate_command = (( (alpha_psi_/mu_psi_) * e_psi_) - ( (beta_psi_/pow(mu_psi_,2)) * e_psi_sum_))/(MAX_ROT_SPEED * M_PI/180);

}

//The function handles the landing in emergency case
void PositionControllerWithSphinx::LandEmergency(){

    if(stateEmergency_){
       ROS_INFO_ONCE("PositionController with Bebop starts landing.");
       std_msgs::Empty empty_msg;
       land_pub_.publish(empty_msg);
    }
}

//The function handles the emergency
void PositionControllerWithSphinx::Emergency(){

    ROS_INFO_ONCE("PositionController with Bebop goes in Emergency mode.");

    stateEmergency_ = true;
    timer3_ = n3_.createTimer(ros::Duration(TsE), &PositionControllerWithSphinx::CallbackLand, this, false, true); 
    std_msgs::Empty empty_msg;
    reset_pub_.publish(empty_msg);

}

// The function computes the velocity errors
void PositionControllerWithSphinx::VelocityErrors(double* dot_e_x, double* dot_e_y, double* dot_e_z){
   assert(dot_e_x);
   assert(dot_e_y);
   assert(dot_e_z);

   // WITH THE EXTENDED KALMAN FILTER
   if(EKF_active_){
	*dot_e_x = - state_.linearVelocity.x;
	*dot_e_y = - state_.linearVelocity.y;
	*dot_e_z = - state_.linearVelocity.z;
   }
   else{
	   
       //The linear velocities are already expressed in the ENU reference system
       *dot_e_x = - state_.linearVelocity.x;
       *dot_e_y = - state_.linearVelocity.y;
       *dot_e_z = - state_.linearVelocity.z;

   }

   if(dataStoring_active_){
     // Saving drone linear velocity in the aircraft body center reference system
     std::stringstream tempDroneLinearVelocities;
     tempDroneLinearVelocities << state_.linearVelocity.x << "," << state_.linearVelocity.y << "," << state_.linearVelocity.z << ","
         << odometry_from_logger_.timeStampSec << "," << odometry_from_logger_.timeStampNsec << "\n";

     listDroneLinearVelocities_.push_back(tempDroneLinearVelocities.str());
   }

}

// The function computes the position errors
void PositionControllerWithSphinx::PositionErrors(double* e_x, double* e_y, double* e_z){
   assert(e_x);
   assert(e_y); 
   assert(e_z);
   
   double x_r, y_r, z_r;
   x_r = command_trajectory_.position_W[0];
   y_r = command_trajectory_.position_W[1]; 
   z_r = command_trajectory_.position_W[2];

   *e_x = x_r - state_.position.x;
   *e_y = y_r - state_.position.y;
   *e_z = z_r - state_.position.z;

   //Every Tsp the Bebop vertical command is updated
   CommandVelocity(&linearZ_);

   ROS_DEBUG("x_r: %f, y_r: %f, z_r: %f", x_r, y_r, z_r);
   ROS_DEBUG("x_d: %f y_d: %f z_d: %f", state_.position.x, state_.position.y, state_.position.z);
   ROS_DEBUG("e_x: %f, e_y: %f, e_z: %f", *e_x, *e_y, *e_z);

}

// The function computes the position controller outputs
void PositionControllerWithSphinx::PosController(double* u_T, double* phi_r, double* theta_r, double* u_x, double* u_y, double* u_z, double* u_Terr){
   assert(u_T);
   assert(phi_r);
   assert(theta_r);
   assert(u_x);
   assert(u_y);
   assert(u_z);
   assert(u_Terr);
   
   //u_x computing
   *u_x = ( (e_x_ * K_x_1_ * K_x_2_)/lambda_x_ ) + ( (dot_e_x_ * K_x_2_)/lambda_x_ );
   
   if (*u_x > 1 || *u_x <-1)
	   if (*u_x > 1)
		   *u_x = 1;
	   else
		   *u_x = -1;
	   
   *u_x = (*u_x * 1/2) + ( (K_x_1_/lambda_x_) * dot_e_x_ );
   
   if (*u_x > 1 || *u_x <-1)
	   if (*u_x > 1)
		   *u_x = 1;
	   else
		   *u_x = -1;
	   
   *u_x = m_ * (*u_x * lambda_x_);
   
   //u_y computing
   *u_y = ( (e_y_ * K_y_1_ * K_y_2_)/lambda_y_ ) + ( (dot_e_y_ * K_y_2_)/lambda_y_ );
   
   if (*u_y > 1 || *u_y <-1)
	   if (*u_y > 1)
		   *u_y = 1;
	   else
		   *u_y = -1;
	   
   *u_y = (*u_y * 1/2) + ( (K_y_1_/lambda_y_) * dot_e_y_ );
   
   if (*u_y > 1 || *u_y <-1)
	   if (*u_y > 1)
		   *u_y = 1;
	   else
		   *u_y = -1;
	   
   *u_y = m_* ( *u_y * lambda_y_);
   
   //u_z computing
   *u_z = ( (e_z_ * K_z_1_ * K_z_2_)/lambda_z_ ) + ( (dot_e_z_ * K_z_2_)/lambda_z_ );
   
   if (*u_z > 1 || *u_z <-1)
	   if (*u_z > 1)
		   *u_z = 1;
	   else
		   *u_z = -1;
	   
   *u_z = (*u_z * 1/2) + ( (K_z_1_/lambda_z_) * dot_e_z_ );
   
   if (*u_z > 1 || *u_z <-1)
	   if (*u_z > 1)
		   *u_z = 1;
	   else
		   *u_z = -1;
	   
   *u_z = m_* ( *u_z * lambda_z_);
   
   //u_Terr computing
   *u_Terr = *u_z + (m_ * g_);
   
   //u_T computing
   *u_T = sqrt( pow(*u_x,2) + pow(*u_y,2) + pow(*u_Terr,2) );
   
   double psi_r;
   psi_r = command_trajectory_.getYaw();

   *theta_r = atan( ( (*u_x * cos(psi_r) ) + ( *u_y * sin(psi_r) ) )  / *u_Terr );

   *phi_r = atan( cos(*theta_r) * ( ( (*u_x * sin(psi_r)) - (*u_y * cos(psi_r)) ) / (*u_Terr) ) );

   if(dataStoring_active_){
      //Saving reference angles in a file
      std::stringstream tempReferenceAngles;
      tempReferenceAngles << *theta_r << "," << *phi_r << "," << odometry_from_logger_.timeStampSec << "," << odometry_from_logger_.timeStampNsec << "\n";

      listReferenceAngles_.push_back(tempReferenceAngles.str());
	  
      double u_phi, u_theta, u_psi;
      AttitudeController(&u_phi, &u_theta, &u_psi);
	  
      //Saving control signals in a file
      std::stringstream tempControlSignals;
      tempControlSignals << *u_T << "," << u_phi << "," << u_theta << "," << u_psi << "," << *u_x << "," << *u_y << ","
          << *u_Terr << "," << *u_z << "," <<odometry_from_logger_.timeStampSec << "," << odometry_from_logger_.timeStampNsec << "\n";

      listControlSignals_.push_back(tempControlSignals.str());

    }

}

//The function computes the attitude errors
void PositionControllerWithSphinx::AttitudeErrors(double* e_phi, double* e_theta, double* e_psi){
   assert(e_phi);
   assert(e_theta);
   assert(e_psi);
   
   double psi_r;
   psi_r = command_trajectory_.getYaw();
   
   double u_T, u_x, u_y, u_z, u_Terr;
   PosController(&u_T, &control_.phiR, &control_.thetaR, &u_x, &u_y, &u_z, &u_Terr);

   *e_phi = control_.phiR - state_.attitude.roll;
   *e_theta = control_.thetaR - state_.attitude.pitch;
   *e_psi = psi_r - state_.attitude.yaw;

}

//The angular velocity errors
void PositionControllerWithSphinx::AngularVelocityErrors(double* dot_e_phi, double* dot_e_theta, double* dot_e_psi){
   assert(dot_e_phi);
   assert(dot_e_theta);
   assert(dot_e_psi);

   double psi_r;
   psi_r = command_trajectory_.getYaw();
   
   double dot_phi, dot_theta, dot_psi;

   dot_phi = state_.angularVelocity.x + (sin(state_.attitude.roll) * tan(state_.attitude.pitch) * state_.angularVelocity.y)
                + (cos(state_.attitude.roll) * tan(state_.attitude.pitch) * state_.angularVelocity.z);
    
   dot_theta = (cos(state_.attitude.roll) * state_.angularVelocity.y) - (sin(state_.attitude.roll) * state_.angularVelocity.z);

   dot_psi = ( ( sin(state_.attitude.roll) * state_.angularVelocity.y) / cos(state_.attitude.pitch) ) +
		         ( ( cos(state_.attitude.roll) * state_.angularVelocity.z) / cos(state_.attitude.pitch) );

   *dot_e_phi =  - dot_phi;
   *dot_e_theta = - dot_theta;
   *dot_e_psi = - dot_psi;


}

//The function computes the attitude controller outputs
void PositionControllerWithSphinx::AttitudeController(double* u_phi, double* u_theta, double* u_psi){
   assert(u_phi);
   assert(u_theta);
   assert(u_psi);

   *u_phi = Ix_ *   ( ( ( (alpha_phi_/mu_phi_    ) * dot_e_phi_  ) - ( (beta_phi_/pow(mu_phi_,2)    ) * e_phi_  ) ) - ( ( (Iy_ - Iz_)/(Ix_ * mu_theta_ * mu_psi_) ) * e_theta_ * e_psi_) );
   *u_theta = Iy_ * ( ( ( (alpha_theta_/mu_theta_) * dot_e_theta_) - ( (beta_theta_/pow(mu_theta_,2)) * e_theta_) ) - ( ( (Iz_ - Ix_)/(Iy_ * mu_phi_ * mu_psi_  ) ) * e_phi_   * e_psi_) );
   *u_psi = Iz_ *   ( ( ( (alpha_psi_/mu_psi_    ) * dot_e_psi_  ) - ( (beta_psi_/pow(mu_psi_,2)    ) * e_psi_  ) ) - ( ( (Ix_ - Iy_)/(Iz_ * mu_theta_ * mu_phi_) ) * e_theta_ * e_phi_) );

}

//The function every TsA computes the attitude and angular velocity errors. When the data storing is active, the data are saved into csv files
void PositionControllerWithSphinx::CallbackAttitude(const ros::TimerEvent& event){
     
     AttitudeErrors(&e_phi_, &e_theta_, &e_psi_);
     AngularVelocityErrors(&dot_e_phi_, &dot_e_theta_, &dot_e_psi_);

     //Saving the time instant when the attitude errors are computed
     if(dataStoring_active_){	

        ros::Time begin = ros::Time::now();
        double secs = begin.toSec();

        //Saving attitude derivate errors in a file
        std::stringstream tempDerivativeAttitudeErrors;
        tempDerivativeAttitudeErrors << dot_e_phi_ << "," << dot_e_theta_ << "," << dot_e_psi_ << "," <<odometry_from_logger_.timeStampSec << "," << odometry_from_logger_.timeStampNsec << "," << secs << "\n";

        listDerivativeAttitudeErrors_.push_back(tempDerivativeAttitudeErrors.str());

        //Saving attitude errors in a file
        std::stringstream tempAttitudeErrors;
        tempAttitudeErrors << e_phi_ << "," << e_theta_ << "," << e_psi_ << "," <<odometry_from_logger_.timeStampSec << "," << odometry_from_logger_.timeStampNsec << secs << "\n";

        listAttitudeErrors_.push_back(tempAttitudeErrors.str());

        //Saving the drone position along axes
        std::stringstream tempDronePosition;
        tempDronePosition << state_.position.x << "," << state_.position.y << "," << state_.position.z << ","
            << odometry_from_logger_.timeStampSec << "," << odometry_from_logger_.timeStampNsec << "," << secs << "\n";

        listDronePosition_.push_back(tempDronePosition.str());

      }
}

//The function every TsP:
//  * the next point to follow has generated (the output of the waypoint filter)
//  * the output of the waypoint filter is put into the command_trajectory_ data structure
//  * the EKF is used to estimate the drone attitude and linear velocity
//  * the position and velocity errors are computed
//  * the last part is used to store the data into csv files if the data storing is active
void PositionControllerWithSphinx::CallbackPosition(const ros::TimerEvent& event){
  
     SetOdometryEstimated();

     // The function is used to invoke the waypoint filter employs to reduce the error dimension along the axes when the drone stars to follow the trajectory.
     // The waypoint filter works with an update time of Tsp
     if(controller_active_)
        waypoint_filter_.Initialize(state_);

     if(waypointFilter_active_ && controller_active_){
        waypoint_filter_.TrajectoryGeneration();
        waypoint_filter_.GetTrajectoryPoint(&command_trajectory_);
     }

     PositionErrors(&e_x_, &e_y_, &e_z_);
     VelocityErrors(&dot_e_x_, &dot_e_y_, &dot_e_z_);

     if(abs(state_.position.x) > MAX_POS_X || abs(state_.position.y) > MAX_POS_Y || state_.position.z > MAX_POS_Z || abs(dot_e_z_) > MAX_VEL_ERR || abs(dot_e_y_) > MAX_VEL_ERR || abs(dot_e_x_) > MAX_VEL_ERR)
        Emergency();

     //Saving the time instant when the position errors are computed
     if(dataStoring_active_){

        ros::Time begin = ros::Time::now();
        double secs = begin.toSec();

        //Saving velocity errors in a file
        std::stringstream tempVelocityErrors;
        tempVelocityErrors << dot_e_x_ << "," << dot_e_y_ << "," << dot_e_z_ << "," <<odometry_from_logger_.timeStampSec << "," << odometry_from_logger_.timeStampNsec << "," << secs << "\n";

        listVelocityErrors_.push_back(tempVelocityErrors.str());

        //Saving trajectory errors in a file
        std::stringstream tempTrajectoryErrors;
        tempTrajectoryErrors << e_x_ << "," << e_y_ << "," << e_z_ << "," <<odometry_from_logger_.timeStampSec << "," << odometry_from_logger_.timeStampNsec << "," << secs << "\n";

        listTrajectoryErrors_.push_back(tempTrajectoryErrors.str());

        //Saving the drone trajectory references (them coming from the waypoint filter)
        std::stringstream tempTrajectoryReferences;
        tempTrajectoryReferences << command_trajectory_.position_W[0] << "," << command_trajectory_.position_W[1] << "," << command_trajectory_.position_W[2] << ","
            <<odometry_from_logger_.timeStampSec << "," << odometry_from_logger_.timeStampNsec << "," << secs << "\n";

        listDroneTrajectoryReference_.push_back(tempTrajectoryReferences.str());

     }
}


//Callback handling the emergency status
void PositionControllerWithSphinx::CallbackLand(const ros::TimerEvent& event){

    ROS_INFO_ONCE("PositionController with Bebop activated the call back for landing.");  
    
    LandEmergency();
   
}


}
