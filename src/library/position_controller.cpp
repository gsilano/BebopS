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

#include "teamsannio_med_control/position_controller.h"
#include "teamsannio_med_control/transform_datatypes.h"
#include "teamsannio_med_control/Matrix3x3.h"
#include "teamsannio_med_control/Quaternion.h" 
#include "teamsannio_med_control/stabilizer_types.h"

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


#define M_PI                      3.14159265358979323846  /* pi */
#define TsP                       10e-3  /* Position control sampling time */
#define TsA                       5e-3 /* Attitude control sampling time */
#define storeTime                 15  /* Store time*/
#define MAX_ROTOR_VELOCITY        1475 /* Max rotors velocity [rad/s] */

using namespace std;

namespace teamsannio_med_control {

PositionController::PositionController()
    : controller_active_(false),
      dataStoring_active_(false),
	  EKF_active_(false),
      e_x_(0),
      e_y_(0),
      e_z_(0),
      dot_e_x_(0),
      dot_e_y_(0), 
      dot_e_z_(0),
      e_phi_(0),
      e_theta_(0),
      e_psi_(0),
      dot_e_phi_(0),
      dot_e_theta_(0), 
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
      dot_e_psi_(0),
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

					//Initializing the structure employed to set the command signals
            		command_trajectory_.setFromYaw(0);
            		command_trajectory_.position_W[0] = 0;
            		command_trajectory_.position_W[1] = 0;
            		command_trajectory_.position_W[2] = 0;

            		//Initializing the structured employed to set the filter parameters
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

            	    //The timers are used to fix the working frequency of the Outer and Inner loop
            		timer1_ = n1_.createTimer(ros::Duration(TsA), &PositionController::CallbackAttitude, this, false, true);
            		timer2_ = n2_.createTimer(ros::Duration(TsP), &PositionController::CallbackPosition, this, false, true);
			    

            		//the boolean variable is used to inactive the logging if it is not useful
            		if(dataStoring_active_){
            			timer3_ = n3_.createTimer(ros::Duration(storeTime), &PositionController::CallbackSaveData, this, false, true);

            			//Cleaning the string vector contents
            			listControlSignals_.clear();
            			listControlSignals_.clear();
						listControlMixerTerms_.clear();
						listPropellersAngularVelocities_.clear();
						listReferenceAngles_.clear();
						listVelocityErrors_.clear();
						listDroneAttitude_.clear();
						listTrajectoryErrors_.clear();
						listAttitudeErrors_.clear();
						listDerivativeAttitudeErrors_.clear();
						listTimeAttitudeErrors_.clear();
						listTimePositionErrors_.clear();

						//the client needed to get information about the Gazebo simulation environment both the attitude and position errors
						clientAttitude_ = clientHandleAttitude_.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
						clientPosition_ = clientHandlePosition_.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");

						ros::WallTime beginWallOffset = ros::WallTime::now();
						wallSecsOffset_ = beginWallOffset.toSec();
         
            }
			
            

}

//The library Destructor
PositionController::~PositionController() {}

//The callback is used to store data about the simulation into csv files
void PositionController::CallbackSaveData(const ros::TimerEvent& event){

      ofstream fileControllerGains;
      ofstream fileVehicleParameters;
      ofstream fileControlSignals;
      ofstream fileControlMixerTerms;
      ofstream filePropellersAngularVelocities;
      ofstream fileReferenceAngles;
      ofstream fileVelocityErrors;
      ofstream fileDroneAttiude;
      ofstream fileTrajectoryErrors;
      ofstream fileAttitudeErrors;
      ofstream fileDerivativeAttitudeErrors;
      ofstream fileTimeAttitudeErrors;
      ofstream fileTimePositionErrors;

      ROS_INFO("CallbackSavaData function is working. Time: %f seconds, %f nanoseconds", odometry_.timeStampSec, odometry_.timeStampNsec);
    
      fileControllerGains.open ("/home/controllerGains.csv", std::ios_base::app);
      fileVehicleParameters.open ("/home/vehicleParamters.csv", std::ios_base::app);
      fileControlSignals.open ("/home/controlSignals.csv", std::ios_base::app);
      fileControlMixerTerms.open ("/home/controlMixer.csv", std::ios_base::app);
      filePropellersAngularVelocities.open ("/home/propellersAngularVelocities.csv", std::ios_base::app);
      fileReferenceAngles.open ("/home/referenceAngles.csv", std::ios_base::app);
      fileVelocityErrors.open ("/home/velocityErrors.csv", std::ios_base::app);
      fileDroneAttiude.open ("/home/droneAttitude.csv", std::ios_base::app);
      fileTrajectoryErrors.open ("/home/trajectoryErrors.csv", std::ios_base::app);
      fileAttitudeErrors.open ("/home/attitudeErrors.csv", std::ios_base::app);
      fileDerivativeAttitudeErrors.open ("/home/derivativeAttitudeErrors.csv", std::ios_base::app);
      fileTimeAttitudeErrors.open ("/home/timeAttitudeErrors.csv", std::ios_base::app);
      fileTimePositionErrors.open ("/home/timePositionErrors.csv", std::ios_base::app);

      //Saving vehicle parameters in a file
      fileControllerGains << beta_x_ << "," << beta_y_ << "," << beta_z_ << "," << alpha_x_ << "," << alpha_y_ << "," << alpha_z_ << "," << beta_phi_ << "," << beta_theta_ << "," << beta_psi_ << "," << alpha_phi_ << "," << alpha_theta_ << "," << alpha_psi_ << "," << mu_x_ << "," << mu_y_ << "," << mu_z_ << "," << mu_phi_ << "," << mu_theta_ << "," << mu_psi_ << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      //Saving vehicle parameters in a file
      fileVehicleParameters << bf_ << "," << l_ << "," << bm_ << "," << m_ << "," << g_ << "," << Ix_ << "," << Iy_ << "," << Iz_ << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      //Saving control signals in a file
      for (unsigned n=0; n < listControlSignals_.size(); ++n) {
          fileControlSignals << listControlSignals_.at( n );
      }

      //Saving the control mixer terms in a file
      for (unsigned n=0; n < listControlMixerTerms_.size(); ++n) {
          fileControlMixerTerms << listControlMixerTerms_.at( n );
      }

      //Saving the propellers angular velocities in a file
      for (unsigned n=0; n < listPropellersAngularVelocities_.size(); ++n) {
          filePropellersAngularVelocities << listPropellersAngularVelocities_.at( n );
      }

      //Saving the reference angles in a file
      for (unsigned n=0; n < listReferenceAngles_.size(); ++n) {
          fileReferenceAngles << listReferenceAngles_.at( n );
      }

      //Saving the velocity errors in a file
      for (unsigned n=0; n < listVelocityErrors_.size(); ++n) {
          fileVelocityErrors << listVelocityErrors_.at( n );
      }

      //Saving the drone attitude in a file
      for (unsigned n=0; n < listDroneAttitude_.size(); ++n) {
          fileDroneAttiude << listDroneAttitude_.at( n );
      }
 
      //Saving the trajectory errors in a file
      for (unsigned n=0; n < listTrajectoryErrors_.size(); ++n) {
          fileTrajectoryErrors << listTrajectoryErrors_.at( n );
      }

      //Saving the attitude errors in a file
      for (unsigned n=0; n < listAttitudeErrors_.size(); ++n) {
          fileAttitudeErrors << listAttitudeErrors_.at( n );
      }

      //Saving the derivative attitude errors in a file
      for (unsigned n=0; n < listDerivativeAttitudeErrors_.size(); ++n) {
          fileDerivativeAttitudeErrors << listDerivativeAttitudeErrors_.at( n );
      }

      //Saving the position and attitude errors along the time
      for (unsigned n=0; n < listTimeAttitudeErrors_.size(); ++n) {
          fileTimeAttitudeErrors << listTimeAttitudeErrors_.at( n );
      }

      for (unsigned n=0; n < listTimePositionErrors_.size(); ++n) {
          fileTimePositionErrors << listTimePositionErrors_.at( n );
      }

      //Closing all opened files
      fileControllerGains.close ();
      fileVehicleParameters.close ();
      fileControlSignals.close ();
      fileControlMixerTerms.close();
      filePropellersAngularVelocities.close();
      fileReferenceAngles.close();
      fileVelocityErrors.close();
      fileDroneAttiude.close();
      fileTrajectoryErrors.close();
      fileAttitudeErrors.close();
      fileDerivativeAttitudeErrors.close();
      fileTimeAttitudeErrors.close();
      fileTimePositionErrors.close();

}

//The function is used to move the controller gains read from file (controller_bebop.yaml) to the private variables of the class.
//These variables will be employed during the simulation
void PositionController::SetControllerGains(){

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

}

//As SetControllerGains, the function is used to set the vehicle paramteters into private variables of the class
void PositionController::SetVehicleParameters(){

      bf_ = vehicle_parameters_.bf_;
      l_ = vehicle_parameters_.armLength_;
      bm_ = vehicle_parameters_.bm_;
      m_ = vehicle_parameters_.mass_;
      g_ = vehicle_parameters_.gravity_;
      Ix_ = vehicle_parameters_.inertia_(0,0);
      Iy_ = vehicle_parameters_.inertia_(1,1);
      Iz_ = vehicle_parameters_.inertia_(2,2);

      //On the EKF object is invoked the method SeVehicleParamters. Such function allows to send the vehicle paramter to the EKF class.
      //Then, they are employed to set the filter matrices
      extended_kalman_filter_bebop_.SetVehicleParameters(m_, g_);

}

void PositionController::SetFilterParameters(){

	  //The function is used to move the filter parameters from the YAML file to the filter class
      extended_kalman_filter_bebop_.SetFilterParameters(&filter_parameters_);

}

//The functions is used to convert the drone attitude from quaternion to Euler angles
void PositionController::Quaternion2Euler(double* roll, double* pitch, double* yaw) const {
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

//When a new odometry message comes, the content of the message is stored in private variable. At the same time, the controller is going to be active.
//The attitude of the aircraft is computer (as we said before it move from quaterninon to Euler angles) and also the angulary velocity is stored.
void PositionController::SetOdometry(const EigenOdometry& odometry) {
    
    odometry_ = odometry; 
    controller_active_= true;

    Quaternion2Euler(&state_.attitude.roll, &state_.attitude.pitch, &state_.attitude.yaw);

    state_.angularVelocity.x = odometry_.angular_velocity[0];
    state_.angularVelocity.y = odometry_.angular_velocity[1];
    state_.angularVelocity.z = odometry_.angular_velocity[2];

}

//The function is used to invoke the waypoint filter emploies to reduce the error dimension along the axes when the drone stars to follow the trajectory
void PositionController::SetTrajectoryPoint(){

    waypoint_filter_.GetTrajectoryPoint(&command_trajectory_);

}

//Just to plot the data during the simulation
void PositionController::GetTrajectory(nav_msgs::Odometry* smoothed_trajectory){

   smoothed_trajectory->pose.pose.position.x = command_trajectory_.position_W[0];
   smoothed_trajectory->pose.pose.position.y = command_trajectory_.position_W[1];
   smoothed_trajectory->pose.pose.position.z = command_trajectory_.position_W[2];

}

//Just to plot the data during the simulation
void PositionController::GetOdometry(nav_msgs::Odometry* odometry_filtered){

   *odometry_filtered = odometry_filtered_private_;

}

//Just to plot the data during the simulation
void PositionController::GetReferenceAngles(nav_msgs::Odometry* reference_angles){
    assert(reference_angles);

   reference_angles->pose.pose.position.x = control_.roll*180/M_PI;
   reference_angles->pose.pose.position.y = control_.pitch*180/M_PI;
   reference_angles->pose.pose.position.z = control_.thrust;

   double u_x, u_y, u_T, u_Terr;
   PosController(&u_x, &u_y, &u_T, &u_Terr);

   reference_angles->twist.twist.linear.x = u_x;
   reference_angles->twist.twist.linear.y = u_y;
   reference_angles->twist.twist.linear.z = u_Terr;  

}

//The functions is used to get information about the estimated state when bias and noise affect both the accelerometer and angular rate measurements
void PositionController::SetOdometryEstimated() {

    extended_kalman_filter_bebop_.SetThrustCommand(control_.thrust);

    //The EKF works or not in according to the value of the EKF_active_ variables
    if(EKF_active_)
    	extended_kalman_filter_bebop_.EstimatorWithNoise(&state_, &odometry_, &odometry_filtered_private_);
    else
    	extended_kalman_filter_bebop_.Estimator(&state_, &odometry_);

}

void PositionController::CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities) {
    assert(rotor_velocities);
    
    //The controller is inactive if a point to reach is not coming
    if(!controller_active_){
       *rotor_velocities = Eigen::Vector4d::Zero(rotor_velocities->rows());
    return;
    }

    double u_phi, u_theta, u_psi;
    double u_x, u_y, u_Terr;
    AttitudeController(&u_phi, &u_theta, &u_psi);
    PosController(&u_x, &u_y, &control_.thrust, &u_Terr);
    
    if(dataStoring_active_){
		//Saving control signals in a file
		std::stringstream tempControlSignals;
		tempControlSignals << control_.thrust << "," << u_phi << "," << u_theta << "," << u_psi << "," << u_x << "," << u_y << "," << u_Terr << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

		listControlSignals_.push_back(tempControlSignals.str());

		//Saving drone attitude in a file
		std::stringstream tempDroneAttitude;
		tempDroneAttitude << state_.attitude.roll << "," << state_.attitude.pitch << "," << state_.attitude.yaw << "," <<odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

		listDroneAttitude_.push_back(tempDroneAttitude.str());

    }
    
    double first, second, third, fourth;
    first = (1 / ( 4 * bf_ )) * control_.thrust;
    second = (1 / (4 * bf_ * l_ * cos(M_PI/4) ) ) * u_phi;
    third = (1 / (4 * bf_ * l_ * cos(M_PI/4) ) ) * u_theta;
    fourth = (1 / ( 4 * bf_ * bm_)) * u_psi;

	
    if(dataStoring_active_){
		//Saving the control mixer terms in a file
		std::stringstream tempControlMixerTerms;
		tempControlMixerTerms << first << "," << second << "," << third << "," << fourth << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

		listControlMixerTerms_.push_back(tempControlMixerTerms.str());
    }

    double not_satured_1, not_satured_2, not_satured_3, not_satured_4;
    not_satured_1 = first - second - third - fourth;
    not_satured_2 = first + second - third + fourth;
    not_satured_3 = first + second + third - fourth;
    not_satured_4 = first - second + third + fourth;

    //The values have been saturated to avoid the root square of negative values
    double satured_1, satured_2, satured_3, satured_4;
    if(not_satured_1 < 0)
       satured_1 = 0;
    else
       satured_1 = not_satured_1;

    if(not_satured_2 < 0)
       satured_2 = 0;
    else
       satured_2 = not_satured_2;

    if(not_satured_3 < 0)
       satured_3 = 0;
    else
       satured_3 = not_satured_3;

    if(not_satured_4 < 0)
       satured_4 = 0;
    else
       satured_4 = not_satured_4;

    double omega_1, omega_2, omega_3, omega_4;
    omega_1 = sqrt(satured_1);
    omega_2 = sqrt(satured_2);
    omega_3 = sqrt(satured_3);
    omega_4 = sqrt(satured_4);
    
    //The propellers velocities is limited by taking into account the physical constrains
    if(omega_1 > MAX_ROTOR_VELOCITY)
       omega_1 = MAX_ROTOR_VELOCITY;
	
    if(omega_2 > MAX_ROTOR_VELOCITY)
       omega_2 = MAX_ROTOR_VELOCITY;
	
    if(omega_3 > MAX_ROTOR_VELOCITY)
       omega_3 = MAX_ROTOR_VELOCITY;
	
    if(omega_4 > MAX_ROTOR_VELOCITY)
       omega_4 = MAX_ROTOR_VELOCITY;

    if(dataStoring_active_){
		//Saving propellers angular velocities in a file
		std::stringstream tempPropellersAngularVelocities;
		tempPropellersAngularVelocities << omega_1 << "," << omega_2 << "," << omega_3 << "," << omega_4 << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

		listPropellersAngularVelocities_.push_back(tempPropellersAngularVelocities.str());
    }
    
    *rotor_velocities = Eigen::Vector4d(omega_1, omega_2, omega_3, omega_4);
}


//The functions computes the reference angles limited to 45° (equal to M_PI)
void PositionController::ReferenceAngles(double* phi_r, double* theta_r){
   assert(phi_r);
   assert(theta_r);

   double psi_r;
   psi_r = command_trajectory_.getYaw();

   double u_x, u_y, u_T, u_Terr;
   PosController(&u_x, &u_y, &u_T, &u_Terr);

   *theta_r = atan( ( (u_x * cos(psi_r) ) + ( u_y * sin(psi_r) ) )  / u_Terr );
   *phi_r = atan( cos(*theta_r) * ( ( (u_x * sin(psi_r)) - (u_y * cos(psi_r)) ) / (u_Terr) ) );

   if(*theta_r > M_PI/4 || *theta_r < -M_PI/4)
	   if(*theta_r > M_PI/4)
		   *theta_r = M_PI/4;
	   else
		   *theta_r = -M_PI/4;

   if(*phi_r > M_PI/4 || *phi_r < -M_PI/4)
	   if(*phi_r > M_PI/4)
		   *phi_r = M_PI/4;
	   else
		   *phi_r = -M_PI/4;


   if(dataStoring_active_){
	  //Saving reference angles in a file
	  std::stringstream tempReferenceAngles;
	  tempReferenceAngles << *theta_r << "," << *phi_r << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

	  listReferenceAngles_.push_back(tempReferenceAngles.str());
    }
    
}

//The function computes the velocity errors
void PositionController::VelocityErrors(double* dot_e_x, double* dot_e_y, double* dot_e_z){
   assert(dot_e_x);
   assert(dot_e_y);
   assert(dot_e_z);

   double x_r, y_r, z_r;
   x_r = command_trajectory_.position_W[0];
   y_r = command_trajectory_.position_W[1]; 
   z_r = command_trajectory_.position_W[2];

   //WITH THE EXTENDED KALMAN FILTER
   if(EKF_active_){
	   *dot_e_x = - state_.linearVelocity.x;
	   *dot_e_y = - state_.linearVelocity.y;
	   *dot_e_z = - state_.linearVelocity.z;
   }
   else{
	   //The linear velocities are expressed in the inertial body frame.
	   double dot_x, dot_y, dot_z, theta, psi, phi;

	   theta = state_.attitude.pitch;
	   psi = state_.attitude.yaw;
	   phi = state_.attitude.roll;

	   dot_x = (cos(theta) * cos(psi) * state_.linearVelocity.x) +
	           ( ( (sin(phi) * sin(theta) * cos(psi) ) - ( cos(phi) * sin(psi) ) ) * state_.linearVelocity.y) +
	           ( ( (cos(phi) * sin(theta) * cos(psi) ) + ( sin(phi) * sin(psi) ) ) *  state_.linearVelocity.z);

	   dot_y = (cos(theta) * sin(psi) * state_.linearVelocity.x) +
	           ( ( (sin(phi) * sin(theta) * sin(psi) ) + ( cos(phi) * cos(psi) ) ) * state_.linearVelocity.y) +
	           ( ( (cos(phi) * sin(theta) * sin(psi) ) - ( sin(phi) * cos(psi) ) ) *  state_.linearVelocity.z);

	   dot_z = (-sin(theta) * state_.linearVelocity.x) + ( sin(phi) * cos(theta) * state_.linearVelocity.y) +
	           (cos(phi) * cos(theta) * state_.linearVelocity.z);

	   *dot_e_x = - dot_x;
   	   *dot_e_y = - dot_y;
   	   *dot_e_z = - dot_z;
   }

}

//The function computes the position errors
void PositionController::PositionErrors(double* e_x, double* e_y, double* e_z){
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

}

//The function computes the position controller outputs
void PositionController::PosController(double* u_x, double* u_y, double* u_T, double* u_Terr){
   assert(u_x);
   assert(u_y);
   assert(u_T);
   assert(u_Terr);

   *u_x = m_ * ( (alpha_x_/mu_x_) * dot_e_x_) - ( (beta_x_/pow(mu_x_,2)) * e_x_);
   *u_y = m_ * ( (alpha_y_/mu_y_) * dot_e_y_) - ( (beta_y_/pow(mu_y_,2)) * e_y_);
   *u_Terr = m_ * ( g_ + ( (alpha_z_/mu_z_) * dot_e_z_) - ( (beta_z_/pow(mu_z_,2)) * e_z_) );
   *u_T = sqrt( pow(*u_x,2) + pow(*u_y,2) + pow(*u_Terr,2) );
   
}

//The function computes the attitude errors
void PositionController::AttitudeErrors(double* e_phi, double* e_theta, double* e_psi){
   assert(e_phi);
   assert(e_theta);
   assert(e_psi);
   
   double psi_r;
   psi_r = command_trajectory_.getYaw();
   
   ReferenceAngles(&control_.roll, &control_.pitch);

   *e_phi = control_.roll - state_.attitude.roll;
   *e_theta = control_.pitch - state_.attitude.pitch;
   *e_psi = psi_r - state_.attitude.yaw;

}

//The angular velocity errors
void PositionController::AngularVelocityErrors(double* dot_e_phi, double* dot_e_theta, double* dot_e_psi){
   assert(dot_e_phi);
   assert(dot_e_theta);
   assert(dot_e_psi);

   double psi_r;
   psi_r = command_trajectory_.getYaw();
   
   double dot_phi, dot_theta, dot_psi;

   dot_phi = state_.angularVelocity.x + (sin(state_.attitude.roll)*tan(state_.attitude.pitch)*state_.angularVelocity.y)
                + (cos(state_.attitude.roll)*tan(state_.attitude.pitch)*state_.angularVelocity.z);
    
   dot_theta = (cos(state_.attitude.roll)*state_.angularVelocity.y) - (sin(state_.attitude.roll)*state_.angularVelocity.z);    

   dot_psi = ((sin(state_.attitude.roll)*state_.angularVelocity.y)/cos(state_.attitude.pitch)) +
		        ((cos(state_.attitude.roll)*state_.angularVelocity.z)/cos(state_.attitude.pitch));

   *dot_e_phi =  - dot_phi;
   *dot_e_theta = - dot_theta;
   *dot_e_psi = - dot_psi;

}

//The function computes the attitude controller outputs
void PositionController::AttitudeController(double* u_phi, double* u_theta, double* u_psi){
   assert(u_phi);
   assert(u_theta);
   assert(u_psi);

   *u_phi = Ix_ *   ( ( ( (alpha_phi_/mu_phi_    ) * dot_e_phi_  ) - ( (beta_phi_/pow(mu_phi_,2)    ) * e_phi_  ) ) - ( ( (Iy_ - Iz_)/(Ix_ * mu_theta_ * mu_psi_) ) * e_theta_ * e_psi_) );
   *u_theta = Iy_ * ( ( ( (alpha_theta_/mu_theta_) * dot_e_theta_) - ( (beta_theta_/pow(mu_theta_,2)) * e_theta_) ) - ( ( (Iz_ - Ix_)/(Iy_ * mu_phi_ * mu_psi_  ) ) * e_phi_   * e_psi_) );
   *u_psi = Iz_ *   ( ( ( (alpha_psi_/mu_psi_    ) * dot_e_psi_  ) - ( (beta_psi_/pow(mu_psi_,2)    ) * e_psi_  ) ) - ( ( (Ix_ - Iy_)/(Iz_ * mu_theta_ * mu_phi_) ) * e_theta_ * e_phi_) );
}


//The function every TsA computes the attidue and angular velocity errors. When the data storing is active, the data are saved into csv files
void PositionController::CallbackAttitude(const ros::TimerEvent& event){
     
     AttitudeErrors(&e_phi_, &e_theta_, &e_psi_);
     AngularVelocityErrors(&dot_e_phi_, &dot_e_theta_, &dot_e_psi_);
     
     //Saving the time instant when the attitude errors are computed
     if(dataStoring_active_){	
		clientAttitude_.call(my_messageAttitude_);

		std::stringstream tempTimeAttitudeErrors;
		tempTimeAttitudeErrors << my_messageAttitude_.response.sim_time << "\n";
		listTimeAttitudeErrors_.push_back(tempTimeAttitudeErrors.str());

		ros::WallTime beginWall = ros::WallTime::now();
		double wallSecs = beginWall.toSec() - wallSecsOffset_;

		ros::Time begin = ros::Time::now();
		double secs = begin.toSec();

		//Saving attitude derivate errors in a file
		std::stringstream tempDerivativeAttitudeErrors;
		tempDerivativeAttitudeErrors << dot_e_phi_ << "," << dot_e_theta_ << "," << dot_e_psi_ << "," <<odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << my_messageAttitude_.response.sim_time << "," << wallSecs << "," << secs << "\n";

		listDerivativeAttitudeErrors_.push_back(tempDerivativeAttitudeErrors.str());

		//Saving attitude errors in a file
		std::stringstream tempAttitudeErrors;
		tempAttitudeErrors << e_phi_ << "," << e_theta_ << "," << e_psi_ << "," <<odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << my_messageAttitude_.response.sim_time << "," << wallSecs << "," << secs << "\n";

		listAttitudeErrors_.push_back(tempAttitudeErrors.str());

      }
}

//The function every TsP:
//	* the next point to follow has generated (the output of the waypoint filter)
//	* the output of the waypoint filter is put into the command_trajectory_ data structure
//  * the EKF is used to estimate the drone attitude and linear velocity
//  * the position and velocity errors are computed
//  * the last part is used to store the data into csv files if the data storing is active
void PositionController::CallbackPosition(const ros::TimerEvent& event){
  
     waypoint_filter_.TrajectoryGeneration();
     SetTrajectoryPoint();
     SetOdometryEstimated();
     PositionErrors(&e_x_, &e_y_, &e_z_);
     VelocityErrors(&dot_e_x_, &dot_e_y_, &dot_e_z_);
     
     //Saving the time instant when the position errors are computed
     if(dataStoring_active_){
		clientPosition_.call(my_messagePosition_);

		std::stringstream tempTimePositionErrors;
		tempTimePositionErrors << my_messagePosition_.response.sim_time << "\n";
		listTimePositionErrors_.push_back(tempTimePositionErrors.str());

		ros::WallTime beginWall = ros::WallTime::now();
		double wallSecs = beginWall.toSec() - wallSecsOffset_;

		ros::Time begin = ros::Time::now();
		double secs = begin.toSec();

		//Saving velocity errors in a file
		std::stringstream tempVelocityErrors;
		tempVelocityErrors << dot_e_x_ << "," << dot_e_y_ << "," << dot_e_z_ << "," <<odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << my_messagePosition_.response.sim_time << "," << wallSecs << "," << secs << "\n";

		listVelocityErrors_.push_back(tempVelocityErrors.str());

		//Saving trajectory errors in a file
		std::stringstream tempTrajectoryErrors;
		tempTrajectoryErrors << e_x_ << "," << e_y_ << "," << e_z_ << "," <<odometry_.timeStampSec << "," << odometry_.timeStampNsec << "," << my_messagePosition_.response.sim_time << "," << wallSecs << "," << secs << "\n";

		listTrajectoryErrors_.push_back(tempTrajectoryErrors.str());

     }
}


}
