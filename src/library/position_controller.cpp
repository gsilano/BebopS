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

using namespace std;

namespace teamsannio_med_control {

PositionController::PositionController()
    : controller_active_(false),
      dataStoring_active_(false),
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

            timer1_ = n1_.createTimer(ros::Duration(TsA), &PositionController::CallbackAttitude, this, false, true);
            timer2_ = n2_.createTimer(ros::Duration(TsP), &PositionController::CallbackPosition, this, false, true); 
			    

            //this serves to inactivate the logging if we wan't to save the simulated data     
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

PositionController::~PositionController() {}

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
    
      fileControllerGains.open ("/home/giuseppe/controllerGains.csv", std::ios_base::app);
      fileVehicleParameters.open ("/home/giuseppe/vehicleParamters.csv", std::ios_base::app);
      fileControlSignals.open ("/home/giuseppe/controlSignals.csv", std::ios_base::app);
      fileControlMixerTerms.open ("/home/giuseppe/controlMixer.csv", std::ios_base::app);
      filePropellersAngularVelocities.open ("/home/giuseppe/propellersAngularVelocities.csv", std::ios_base::app);
      fileReferenceAngles.open ("/home/giuseppe/referenceAngles.csv", std::ios_base::app);
      fileVelocityErrors.open ("/home/giuseppe/velocityErrors.csv", std::ios_base::app);
      fileDroneAttiude.open ("/home/giuseppe/droneAttitude.csv", std::ios_base::app);
      fileTrajectoryErrors.open ("/home/giuseppe/trajectoryErrors.csv", std::ios_base::app);
      fileAttitudeErrors.open ("/home/giuseppe/attitudeErrors.csv", std::ios_base::app);
      fileDerivativeAttitudeErrors.open ("/home/giuseppe/derivativeAttitudeErrors.csv", std::ios_base::app);
      fileTimeAttitudeErrors.open ("/home/giuseppe/timeAttitudeErrors.csv", std::ios_base::app);
      fileTimePositionErrors.open ("/home/giuseppe/timePositionErrors.csv", std::ios_base::app);

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

void PositionController::SetVehicleParameters(){

      bf_ = vehicle_parameters_.bf_;
      l_ = vehicle_parameters_.armLength_;
      bm_ = vehicle_parameters_.bm_;
      m_ = vehicle_parameters_.mass_;
      g_ = vehicle_parameters_.gravity_;
      Ix_ = vehicle_parameters_.inertia_(0,0);
      Iy_ = vehicle_parameters_.inertia_(1,1);
      Iz_ = vehicle_parameters_.inertia_(2,2);

      extended_kalman_filter_bebop_.SetVehicleParameters(m_, g_);

}

void PositionController::SetFilterParameters(){

      extended_kalman_filter_bebop_.SetFilterParameters(&filter_parameters_);

}

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

void PositionController::SetOdometry(const EigenOdometry& odometry) {
    
    odometry_ = odometry; 

    Quaternion2Euler(&state_.attitude.roll, &state_.attitude.pitch, &state_.attitude.yaw);

    state_.angularVelocity.x = odometry_.angular_velocity[0];
    state_.angularVelocity.y = odometry_.angular_velocity[1];
    state_.angularVelocity.z = odometry_.angular_velocity[2];

}

void PositionController::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {

    command_trajectory_= command_trajectory;
    controller_active_= true;

}

void PositionController::SetOdometryEstimated() {

    extended_kalman_filter_bebop_.SetThrustCommand(control_.thrust);
    extended_kalman_filter_bebop_.Estimator(&state_, &odometry_);

}

void PositionController::CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities) {
    assert(rotor_velocities);
    
    //this serves to inactivate the controller if we don't recieve a trajectory
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
    first = (1/ ( 4 * bf_ )) * control_.thrust;
    second = (1/ (4 * bf_ * l_ * cos(M_PI/4) ) ) * u_phi;
    third = (1/ (4 * bf_ * l_ * cos(M_PI/4) ) ) * u_theta;
    fourth = (1/ ( 4 * bf_ * bm_)) * u_psi;

	
    if(dataStoring_active_){
	//Saving the control mixer terms in a file
	std::stringstream tempControlMixerTerms;
	tempControlMixerTerms << first << "," << second << "," << third << "," << fourth << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

	listControlMixerTerms_.push_back(tempControlMixerTerms.str());
    }

    double not_sat1, not_sat2, not_sat3, not_sat4;
    not_sat1 = first - second - third - fourth;
    not_sat2 = first + second - third + fourth;
    not_sat3 = first + second + third - fourth;
    not_sat4 = first - second + third + fourth;

    //The values have been saturated to avoid the root square of negative values
    double sat1, sat2, sat3, sat4;
    if(not_sat1 < 0)
       sat1 = 0;
    else
       sat1 = not_sat1;

    if(not_sat2 < 0)
       sat2 = 0;
    else
       sat2 = not_sat2;

    if(not_sat3 < 0)
       sat3 = 0;
    else
       sat3 = not_sat3;

    if(not_sat4 < 0)
       sat4 = 0;
    else
       sat4 = not_sat4;

    double omega_1, omega_2, omega_3, omega_4;
    omega_1 = sqrt(sat1);
    omega_2 = sqrt(sat2);
    omega_3 = sqrt(sat3);
    omega_4 = sqrt(sat4);
    
    //The propellers velocities is limited by taking into account the physical constrains
    double maxRotorsVelocity = 1475;
    if(omega_1 > maxRotorsVelocity)
       omega_1 = maxRotorsVelocity;
	
    if(omega_2 > maxRotorsVelocity)
       omega_2 = maxRotorsVelocity;
	
    if(omega_3 > maxRotorsVelocity)
       omega_3 = maxRotorsVelocity;
	
    if(omega_4 > maxRotorsVelocity)
       omega_4 = maxRotorsVelocity;

    if(dataStoring_active_){
		//Saving propellers angular velocities in a file
		std::stringstream tempPropellersAngularVelocities;
		tempPropellersAngularVelocities << omega_1 << "," << omega_2 << "," << omega_3 << "," << omega_4 << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

		listPropellersAngularVelocities_.push_back(tempPropellersAngularVelocities.str());
    }
    
    *rotor_velocities = Eigen::Vector4d(omega_1, omega_2, omega_3, omega_4);
}


void PositionController::ReferenceAngles(double* phi_r, double* theta_r){
   assert(phi_r);
   assert(theta_r);

   double psi_r;
   psi_r = command_trajectory_.getYaw();

   double u_x, u_y, u_T, u_Terr;
   PosController(&u_x, &u_y, &u_T, &u_Terr);

   *theta_r = atan( ( (u_x * cos(psi_r) ) + ( u_y * sin(psi_r) ) )  / u_Terr );
   *phi_r = atan( cos(*theta_r) * ( ( (u_x * sin(psi_r)) - (u_y * cos(psi_r)) ) / (u_Terr) ) );

   	if(dataStoring_active_){
		//Saving reference angles in a file
		std::stringstream tempReferenceAngles;
		tempReferenceAngles << *theta_r << "," << *phi_r << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

		listReferenceAngles_.push_back(tempReferenceAngles.str());
	}
    
}

void PositionController::VelocityErrors(double* dot_e_x, double* dot_e_y, double* dot_e_z){
   assert(dot_e_x);
   assert(dot_e_y);
   assert(dot_e_z);

   double x_r, y_r, z_r;
   x_r = command_trajectory_.position_W[0];
   y_r = command_trajectory_.position_W[1]; 
   z_r = command_trajectory_.position_W[2];

   *dot_e_x = - state_.linearVelocity.x;
   *dot_e_y = - state_.linearVelocity.y;
   *dot_e_z = - state_.linearVelocity.z;
   
}

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

void PositionController::PosController(double* u_x, double* u_y, double* u_T, double* u_Terr){
   assert(u_x);
   assert(u_y);
   assert(u_T);
   assert(u_Terr);

   *u_x = m_ * ( (alpha_x_/mu_x_) * dot_e_x_) - ( (beta_x_/pow(mu_x_,2)) * e_x_);
   *u_y = m_ * ( (alpha_y_/mu_y_) * dot_e_y_) -  ( (beta_y_/pow(mu_y_,2)) *  e_y_);
   *u_Terr = m_ * ( g_ + ( (alpha_z_/mu_z_) * dot_e_z_) - ( (beta_z_/pow(mu_z_,2)) * e_z_) );
   *u_T = sqrt( pow(*u_x,2) + pow(*u_y,2) + pow(*u_Terr,2) );
   
}

void PositionController::AttitudeErrors(double* e_phi, double* e_theta, double* e_psi){
   assert(e_phi);
   assert(e_theta);
   assert(e_psi);
   
   double psi_r;
   psi_r = command_trajectory_.getYaw();
   
   double phi_r, theta_r;
   ReferenceAngles(&phi_r, &theta_r);

   *e_phi = phi_r - state_.attitude.roll;
   *e_theta = theta_r - state_.attitude.pitch;
   *e_psi = psi_r - state_.attitude.yaw;

}

void PositionController::AngularVelocityErrors(double* dot_e_phi, double* dot_e_theta, double* dot_e_psi){
   assert(dot_e_phi);
   assert(dot_e_theta);
   assert(dot_e_psi);

   double psi_r;
   psi_r = command_trajectory_.getYaw();
   
   double phi_r, theta_r;
   ReferenceAngles(&phi_r, &theta_r);
   
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

void PositionController::AttitudeController(double* u_phi, double* u_theta, double* u_psi){
   assert(u_phi);
   assert(u_theta);
   assert(u_psi);

   *u_phi = Ix_ * ( ( ( (alpha_phi_/mu_phi_) * dot_e_phi_) - ( (beta_phi_/pow(mu_phi_,2)) * e_phi_) ) - ( ( (Iy_ - Iz_)/(Ix_ * mu_theta_ * mu_psi_) ) * e_theta_ * e_psi_) );
   *u_theta = Iy_ * ( ( ( (alpha_theta_/mu_theta_) * dot_e_theta_) - ( (beta_theta_/pow(mu_theta_,2)) * e_theta_) ) - ( ( (Iz_ - Ix_)/(Iy_ * mu_phi_ * mu_psi_) ) * e_phi_ * e_psi_) );
   *u_psi = Iz_ * ( ( ( (alpha_psi_/mu_psi_) * dot_e_psi_) - ( (beta_psi_/pow(mu_psi_,2)) * e_psi_) ) - ( ( (Ix_ - Iy_)/(Iz_ * mu_theta_ * mu_phi_) ) * e_theta_ * e_phi_) );
}

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

void PositionController::CallbackPosition(const ros::TimerEvent& event){
  
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
