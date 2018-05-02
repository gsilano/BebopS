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
#include <ros/ros.h>
#include <chrono>
#include <inttypes.h>

#include <nav_msgs/Odometry.h>
#include <ros/console.h>


#define M_PI                      3.14159265358979323846  /* pi */
#define TsP                       1e7  /* Position control sampling time */
#define TsA                       5e6 /* Attitude control sampling time */

namespace teamsannio_med_control {

PositionController::PositionController()
    : controller_active_(false),
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
      x_r_pre_(0),
      y_r_pre_(0),
      z_r_pre_(0),
      phi_r_pre_(0),
      theta_r_pre_(0),
      psi_r_pre_(0){  

            timer1_ = n1_.createTimer(ros::Duration(0.005), &PositionController::CallbackAttitude, this, false, true);
            timer2_ = n2_.createTimer(ros::Duration(0.01), &PositionController::CallbackPosition, this, false, true); 

}

PositionController::~PositionController() {}

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

}

void PositionController::SetOdometry(const EigenOdometry& odometry) {
    
    odometry_ = odometry; 
    SetOdometryEstimated();

}

void PositionController::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {

    command_trajectory_= command_trajectory;
    controller_active_= true;

}

void PositionController::SetOdometryEstimated() {

    extended_kalman_filter_bebop_.Estimator(&state_, &odometry_);
}

void PositionController::CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities) {
    assert(rotor_velocities);
    
    //this serves to inactivate the controller if we don't recieve a trajectory
    if(!controller_active_){
       *rotor_velocities = Eigen::Vector4d::Zero(rotor_velocities->rows());
    return;
    }

    double u_T, u_phi, u_theta, u_psi;
    double u_x, u_y, u_Terr;
    AttitudeController(&u_phi, &u_theta, &u_psi);
    PosController(&u_x, &u_y, &u_T, &u_Terr);
    
    double first, second, third, fourth;
    first = (1/ ( 4 * bf_ )) * u_T;
    second = (1/ (4 * bf_ * l_ * cos(M_PI/4) ) ) * u_phi;
    third = (1/ (4 * bf_ * l_ * cos(M_PI/4) ) ) * u_theta;
    fourth = (1/ ( 4 * bf_ * bm_)) * u_psi;

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
    if(omega_1 > 838)
       omega_1 = 838;
	
    if(omega_2 > 838)
       omega_2 = 838;
	
    if(omega_3 > 838)
       omega_3 = 838;
	
    if(omega_4 > 838)
       omega_4 = 838;

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

}

void PositionController::VelocityErrors(double* dot_e_x, double* dot_e_y, double* dot_e_z){
   assert(dot_e_x);
   assert(dot_e_y);
   assert(dot_e_z);

   double x_r, y_r, z_r;
   x_r = command_trajectory_.position_W[0];
   y_r = command_trajectory_.position_W[1]; 
   z_r = command_trajectory_.position_W[2];

  // calculate delta_t
  if (!prev_time_pos_.isZero()) // Not first time through the program  
  {
    delta_t_pos_ = ros::Time::now() - prev_time_pos_;
    prev_time_pos_ = ros::Time::now();
    if (0 == delta_t_pos_.toSec())
    {
      ROS_ERROR("delta_t is 0, skipping this loop. Possible overloaded cpu at time: %f", ros::Time::now().toSec());
      return;
    }
  }
  else
  {
    ROS_INFO("prev_time is 0, doing nothing");
    prev_time_pos_ = ros::Time::now();
    return;
  }
   
   double dot_x_r, dot_y_r, dot_z_r;
   dot_x_r = (x_r - x_r_pre_)/delta_t_pos_.toSec();
   dot_y_r = (y_r - y_r_pre_)/delta_t_pos_.toSec();
   dot_z_r = (z_r - z_r_pre_)/delta_t_pos_.toSec();

   x_r_pre_ = x_r;
   y_r_pre_ = y_r;
   z_r_pre_ = z_r;
   
   *dot_e_x = dot_x_r - state_.linearVelocity.x;
   *dot_e_y = dot_y_r - state_.linearVelocity.y; 
   *dot_e_z = dot_z_r - state_.linearVelocity.z;
   
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

  // calculate delta_t
  if (!prev_time_att_.isZero()) // Not first time through the program  
  {
    delta_t_att_ = ros::Time::now() - prev_time_att_;
    prev_time_att_ = ros::Time::now();
    if (0 == delta_t_att_.toSec())
    {
      ROS_ERROR("delta_t is 0, skipping this loop. Possible overloaded cpu at time: %f", ros::Time::now().toSec());
      return;
    }
  }
  else
  {
    ROS_INFO("prev_time is 0, doing nothing");
    prev_time_att_ = ros::Time::now();
    return;
  }

   double psi_r;
   psi_r = command_trajectory_.getYaw();
   
   double phi_r, theta_r;
   ReferenceAngles(&phi_r, &theta_r);
   
   double dot_phi_r, dot_theta_r, dot_psi_r;
   dot_phi_r = (phi_r - phi_r_pre_)/delta_t_att_.toSec();
   dot_theta_r = (theta_r - theta_r_pre_)/delta_t_att_.toSec();
   dot_psi_r = (psi_r - psi_r_pre_)/delta_t_att_.toSec();

   phi_r_pre_ = phi_r;
   theta_r_pre_ = theta_r;
   psi_r_pre_ = psi_r;

   *dot_e_phi =  dot_phi_r - state_.angularVelocity.x;
   *dot_e_theta = dot_theta_r - state_.angularVelocity.y;
   *dot_e_psi = dot_psi_r - state_.angularVelocity.z;

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
}

void PositionController::CallbackPosition(const ros::TimerEvent& event){
 
     PositionErrors(&e_x_, &e_y_, &e_z_);
     VelocityErrors(&dot_e_x_, &dot_e_y_, &dot_e_z_);
}


}
