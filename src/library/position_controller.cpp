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
#define MAX_ROTOR_VELOCITY        1475 /* Max rotors velocity [rad/s] */
#define MIN_ROTOR_VELOCITY        0 /* Min rotors velocity [rad/s] */
#define POW_MAX_ROTOR_VELOCITY    MAX_ROTOR_VELOCITY*MAX_ROTOR_VELOCITY /* Squared max rotors velocity [rad/s] */

using namespace std;

namespace teamsannio_med_control {

PositionController::PositionController()
    : controller_active_(false),
      dataStoring_active_(false),
      waypointFilter_active_(true),
      EKF_active_(false),
      dataStoringTime_(0),
      wallSecsOffset_(0),
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
      control_({0,0,0,0}), //roll, pitch, yaw rate, thrust
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
            		//timer1_ = n1_.createTimer(ros::Duration(TsA), &PositionController::CallbackAttitude, this, false, true);
            		timer2_ = n2_.createTimer(ros::Duration(TsP), &PositionController::CallbackPosition, this, false, true);


}

//The library Destructor
PositionController::~PositionController() {}

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

//As SetControllerGains, the function is used to set the vehicle parameteters into private variables of the class
void PositionController::SetVehicleParameters(){

      bf_ = vehicle_parameters_.bf_;
      l_ = vehicle_parameters_.armLength_;
      bm_ = vehicle_parameters_.bm_;
      m_ = vehicle_parameters_.mass_;
      g_ = vehicle_parameters_.gravity_;
      Ix_ = vehicle_parameters_.inertia_(0,0);
      Iy_ = vehicle_parameters_.inertia_(1,1);
      Iz_ = vehicle_parameters_.inertia_(2,2);

      //On the EKF object is invoked the method SeVehicleParameters. Such function allows to send the vehicle parameter to the EKF class.
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
//The attitude of the aircraft is computed (as we said before it move from quaterninon to Euler angles) and also the angular velocity is stored.
void PositionController::SetOdometry(const EigenOdometry& odometry) {
    
    odometry_ = odometry; 
    controller_active_= true;

    Quaternion2Euler(&state_.attitude.roll, &state_.attitude.pitch, &state_.attitude.yaw);

    state_.angularVelocity.x = odometry_.angular_velocity[0];
    state_.angularVelocity.y = odometry_.angular_velocity[1];
    state_.angularVelocity.z = odometry_.angular_velocity[2];

}

// The function allows to set the waypoint filter parameters
void PositionController::SetWaypointFilterParameters(){

  waypoint_filter_.SetParameters(&waypoint_filter_parameters_);

}

void PositionController::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory_positionControllerNode) {

  // If the waypoint has been activated or not
  if(waypointFilter_active_){
    waypoint_filter_.SetTrajectoryPoint(command_trajectory_positionControllerNode);
  }
  else
    command_trajectory_ = command_trajectory_positionControllerNode;


}


//The functions is used to get information about the estimated state when bias and noise affect both the accelerometer and angular rate measurements
void PositionController::SetOdometryEstimated() {

    extended_kalman_filter_bebop_.SetThrustCommand(control_.uT);

    //The EKF works or not in according to the value of the EKF_active_ variables
    if(EKF_active_)
    	extended_kalman_filter_bebop_.EstimatorWithNoise(&state_, &odometry_, &odometry_filtered_private_);
    else
    	extended_kalman_filter_bebop_.Estimator(&state_, &odometry_);
}

double* PositionController::GetControllerOuputs() {
  static double r[3];
  r[0] = control_.phiR;
  r[1] = control_.thetaR;
  r[2] = control_.uT;
  return r;
}

void PositionController::CallPosController() {
    double u_phi, u_theta, u_psi;
    double u_x, u_y, u_z, u_Terr;
    ROS_INFO("[CallPosController] before calling pos controller, control thrust: [%lf], phi: [%lf], theta: [%lf]", control_.uT, control_.phiR, control_.thetaR);
	  PosController(&control_.uT, &control_.phiR, &control_.thetaR, &u_x, &u_y, &u_z, &u_Terr);
    // ROS_INFO("command signals: x= [%lf], y= [%lf], z= [%lf]", u_x, u_y, u_z);
}

//The function computes the velocity errors
void PositionController::VelocityErrors(double* dot_e_x, double* dot_e_y, double* dot_e_z){
   assert(dot_e_x);
   assert(dot_e_y);
   assert(dot_e_z);

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

   if(dataStoring_active_){
     //Saving drone linear velocity in the aircraft body center reference system
     std::stringstream tempDroneLinearVelocitiesABC;
     tempDroneLinearVelocitiesABC << state_.linearVelocity.x << "," << state_.linearVelocity.y << "," << state_.linearVelocity.z << ","
         << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

     listDroneLinearVelocitiesABC_.push_back(tempDroneLinearVelocitiesABC.str());
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

  // ROS_INFO("reference position is ([%lf], [%lf], [%lf])", x_r, y_r, z_r );
  // ROS_INFO("state position is ([%lf], [%lf], [%lf]", state_.position.x, state_.position.y, state_.position.z );

}

//The function computes the position controller outputs
void PositionController::PosController(double* u_T, double* phi_r, double* theta_r, double* u_x, double* u_y, double* u_z, double* u_Terr){
   assert(u_T);
   assert(phi_r);
   assert(theta_r);
   assert(u_x);
   assert(u_y);
   assert(u_z);
   assert(u_Terr);

   ROS_INFO("[PosController] control thrust: [%lf], phi: [%lf], theta: [%lf]", control_.uT, control_.phiR, control_.thetaR);
   ROS_INFO("[PosController] thrust: [%lf], phi: [%lf], theta: [%lf]", &u_T, &phi_r, &theta_r);
   
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
   
   *u_Terr = *u_z + (m_ * g_);
   
   *u_T = sqrt( pow(*u_x,2) + pow(*u_y,2) + pow(*u_Terr,2) );
   
   double psi_r;
   psi_r = command_trajectory_.getYaw();

   *theta_r = atan( ( (*u_x * cos(psi_r) ) + ( *u_y * sin(psi_r) ) )  / *u_Terr );

   *phi_r = atan( cos(*theta_r) * ( ( (*u_x * sin(psi_r)) - (*u_y * cos(psi_r)) ) / (*u_Terr) ) );

}


//The function every TsP:
//	* the next point to follow has generated (the output of the waypoint filter)
//	* the output of the waypoint filter is put into the command_trajectory_ data structure
//  * the EKF is used to estimate the drone attitude and linear velocity
//  * the position and velocity errors are computed
//  * the last part is used to store the data into csv files if the data storing is active
void PositionController::CallbackPosition(const ros::TimerEvent& event){
  
     // The function is used to invoke the waypoint filter emploies to reduce the error dimension along the axes when the drone stars to follow the trajectory.
     // The waypoint filter works with an update time of Tsp
     if(controller_active_)
       waypoint_filter_.Initialize(state_);

     if(waypointFilter_active_ && controller_active_){
        waypoint_filter_.TrajectoryGeneration();
        // we set command_trajectory to command_trajectory_toSend to the value of the next waypoint
        waypoint_filter_.GetTrajectoryPoint(&command_trajectory_);
     }

     SetOdometryEstimated();
     PositionErrors(&e_x_, &e_y_, &e_z_);
     VelocityErrors(&dot_e_x_, &dot_e_y_, &dot_e_z_);
}


}
