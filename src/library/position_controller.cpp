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

#include <nav_msgs/Odometry.h>
#include <ros/console.h>


#define M_PI                      3.14159265358979323846  /* pi */
#define Ts                        0.01f  /* TIME STEP */

namespace teamsannio_med_control {

PositionController::PositionController()
    : controller_active_(false),
      first_time_derivate_computing_XY(false),
      first_time_derivate_computing_Z(false),
      first_time_derivate_computing_Angles(false){

}

PositionController::~PositionController() {}

void PositionController::SetOdometry(const EigenOdometry& odometry) {
    
    odometry_ = odometry; 
    SetOdometryEstimated(); 

}

void PositionController::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
    command_trajectory_= command_trajectory;
    controller_active_= true;
}

void PositionController::CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities) {
    assert(rotor_velocities);
    
    //this serves to inactivate the controller if we don't recieve a trajectory
    if(!controller_active_){
       *rotor_velocities = Eigen::Vector4d::Zero(rotor_velocities->rows());
    return;
    }

    double bf, l, bm;
    bf = vehicle_parameters_.bf_;
    bm  = vehicle_parameters_.bm_;
    l  = vehicle_parameters_.armLength_;

    double u_T, u_phi, u_theta, u_psi;
    AltitudeControl(&u_T);
    RollPitchYawControl(&u_phi, &u_theta, &u_psi);

    double first, second, third, fourth;
    first = (1/( 4 * bf )) * u_T;
    second = (1/ (4 * bf * l * cos(M_PI/4) ) ) * u_phi;
    third = (1/ (4 * bf * l * cos(M_PI/4) ) ) * u_theta;
    fourth = (1/( 4 * bf * bm)) * u_psi;

    double not_sat1, not_sat2, not_sat3, not_sat4;
    not_sat1 = first - second - third - fourth;
    not_sat2 = first + second - third + fourth;
    not_sat3 = first + second + third - fourth;
    not_sat4 = first - second + third + fourth;

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

void PositionController::SetOdometryEstimated() {

    extended_kalman_filter_bebop_.Estimator(&state_, &odometry_);
}

void PositionController::ErrorsXY(double* e_x, double* dot_ex, double* e_y, double* dot_ey){

    assert(e_x);
    assert(e_y);
    assert(dot_ex);
    assert(dot_ey);

    //Trajectory references
    double x_r, y_r;
    x_r = command_trajectory_.position_W[0];
    y_r = command_trajectory_.position_W[1];

    //Trajectory derivates
    double x_r_pre, y_r_pre;
    double dot_x_r, dot_y_r;

    if(!first_time_derivate_computing_XY){
       x_r_pre = 0;
       y_r_pre = 0;

       first_time_derivate_computing_XY = true;
    }

    dot_x_r = (x_r - x_r_pre)/Ts;
    x_r_pre = x_r;
    
    dot_y_r = (y_r - y_r_pre)/Ts;
    y_r_pre = y_r;
    
    //Errors computed
    *e_x = x_r - state_.position.x;
    *e_y = y_r - state_.position.y;

    *dot_ex = dot_x_r - state_.linearVelocity.x;
    *dot_ey = dot_y_r - state_.linearVelocity.y;

}

void PositionController::ErrorsZ(double* e_z, double* dot_ez){

    assert(e_z);
    assert(dot_ez);

    //Trajectory references
    double z_r;
    z_r = command_trajectory_.position_W[2];

    //Trajectory derivates
    double z_r_pre;
    double dot_z_r;

    if(!first_time_derivate_computing_Z){
       z_r_pre = 0;

       first_time_derivate_computing_Z = true;
    }

    dot_z_r = (z_r - z_r_pre)/Ts;
    z_r_pre = z_r;
    
    //Errors computed
    *e_z = z_r - state_.position.z;

    *dot_ez = dot_z_r - state_.linearVelocity.z;

}

void PositionController::ErrorsAngles(double* e_phi, double* dot_ephi, double* e_theta, double* dot_etheta, double* e_psi, double* dot_epsi){

    assert(e_phi);
    assert(e_theta);
    assert(e_psi);
    assert(dot_ephi);
    assert(dot_etheta);
    assert(dot_epsi);

    //Trajectory references
    double psi_r;
    psi_r = command_trajectory_.getYaw();

    //Virtual references
    double phi_r, theta_r;
    AttitudePlanner(&phi_r, &theta_r);

    //Trajectory derivates
    double phi_r_pre, theta_r_pre, psi_r_pre;
    double dot_phi_r, dot_theta_r, dot_psi_r;

    if(!first_time_derivate_computing_Angles){
       phi_r_pre = 0;
       theta_r_pre = 0;
       psi_r_pre = 0;
       first_time_derivate_computing_Angles = true;
    }

    dot_phi_r = (phi_r - phi_r_pre)/Ts;
    phi_r_pre = phi_r;

    dot_theta_r = (theta_r - theta_r_pre)/Ts; 
    theta_r_pre = theta_r;

    dot_psi_r = (psi_r - psi_r_pre)/Ts;
    psi_r_pre = psi_r;

    //Errors computed
    *e_phi = phi_r - state_.attitude.roll;
    *e_theta = theta_r - state_.attitude.pitch;
    *e_psi = psi_r - state_.attitude.yaw; 

    *dot_ephi = dot_phi_r - state_.angularVelocity.x;
    *dot_etheta = dot_theta_r - state_.angularVelocity.y;
    *dot_epsi = dot_psi_r - state_.angularVelocity.z;

}

void PositionController::PositionControl(double* u_x, double* u_y){
   assert(u_x);
   assert(u_y);

   double u_T;
   AltitudeControl(&u_T);

   double tilde_ux, tilde_uy;
   DesiredActuation(&tilde_ux, &tilde_uy);

   double den = sqrt (pow(tilde_ux,2) + pow(tilde_uy,2) + 0.01);
  
   *u_x = (tilde_ux * abs(u_T) )/den;
   *u_y = (tilde_uy * abs(u_T) )/den;

}

void PositionController::DesiredActuation(double* tilde_ux, double* tilde_uy){
   assert(tilde_ux);
   assert(tilde_uy);

   double e_x, dot_ex, e_y, dot_ey;
   ErrorsXY(&e_x, &dot_ex, &e_y, &dot_ey);

   double beta_x, beta_y, alpha_x, alpha_y;
   beta_x = controller_parameters_.xy_gain_kp_.x();
   beta_y = controller_parameters_.xy_gain_kp_.y();

   alpha_x = 1 - beta_x;
   alpha_y = 1 - beta_y;

   double mu_x, mu_y;
   mu_x = controller_parameters_.mu_xy_.x();
   mu_y = controller_parameters_.mu_xy_.y();

   double m;
   m = vehicle_parameters_.mass_;

   *tilde_ux = ( ((alpha_x/mu_x) * dot_ex) - ((beta_x/pow(mu_x,2)) * e_x) )/ m;
   *tilde_uy = ( ((alpha_y/mu_y) * dot_ey) - ((beta_y/pow(mu_y,2)) * e_y) )/ m;

}

void PositionController::AttitudePlanner(double* phi_r, double* theta_r){
   assert(phi_r);
   assert(theta_r);

   double psi_r;
   psi_r = command_trajectory_.getYaw();

   double e_z, dot_ez;
   ErrorsZ(&e_z, &dot_ez);

   double u_x, u_y;
   PositionControl(&u_x, &u_y);

   double beta_z, alpha_z;
   beta_z = controller_parameters_.z_gain_kp_;
   alpha_z = 1 - beta_z;

   double mu_z;
   mu_z = controller_parameters_.mu_z_;

   double g;
   g = vehicle_parameters_.gravity_;

   double common;
   common = 1/( (dot_ez * (alpha_z/mu_z)) - (e_z * (beta_z/pow(mu_z,2))) + g );

   *theta_r = atan( common * ( (sin(psi_r) * u_y) + (cos(psi_r) * u_x) ) );
   *phi_r = atan(common * cos(*theta_r) * ( (sin(psi_r) * u_x) - (cos(psi_r)* u_y) ));

}

void PositionController::AltitudeControl(double* u_T){
   assert(u_T);

   double phi, theta;
   phi = state_.attitude.roll;
   theta = state_.attitude.pitch;

   double e_z, dot_ez;
   ErrorsZ(&e_z, &dot_ez);

   double m, g;
   m = vehicle_parameters_.mass_;
   g = vehicle_parameters_.gravity_;

   double beta_z, alpha_z;
   beta_z = controller_parameters_.z_gain_kp_;
   alpha_z = 1 - beta_z;

   double mu_z;
   mu_z = controller_parameters_.mu_z_;

   *u_T = (m / (cos(phi) * cos(theta)) ) * ( ((alpha_z/mu_z) * dot_ez) - ((beta_z/pow(mu_z,2)) * e_z) + g);
}

void PositionController::RollPitchYawControl(double* u_phi, double* u_theta, double* u_psi){
   assert(u_phi);
   assert(u_theta);
   assert(u_psi);

   double e_phi, dot_ephi, e_theta, dot_etheta, e_psi, dot_epsi;
   ErrorsAngles(&e_phi, &dot_ephi, &e_theta, &dot_etheta, &e_psi, &dot_epsi);

   double Ix, Iy, Iz;
   Ix = vehicle_parameters_.inertia_(0,0);
   Iy = vehicle_parameters_.inertia_(1,1);
   Iz = vehicle_parameters_.inertia_(2,2);

   double beta_phi, beta_theta, beta_psi, alpha_phi, alpha_theta, alpha_psi;
   beta_phi = controller_parameters_.roll_gain_kp_;
   beta_theta = controller_parameters_.pitch_gain_kp_;
   beta_psi = controller_parameters_.yaw_rate_gain_kp_;

   alpha_phi = 1 - beta_phi;
   alpha_theta = 1 - beta_theta;
   alpha_psi = 1 - beta_psi;

   double mu_phi, mu_theta, mu_psi;
   mu_phi = controller_parameters_.mu_roll_;
   mu_theta = controller_parameters_.mu_pitch_;
   mu_psi = controller_parameters_.mu_yaw_;

   *u_phi = Ix * ( (dot_ephi * (alpha_phi/mu_phi) ) - (e_phi * (beta_phi/pow(mu_phi,2)) ) - ( e_theta * e_psi * ((Iy-Iz)/(Ix * mu_theta * mu_psi))) );

   *u_theta = Iy * ( (dot_etheta * (alpha_theta/mu_theta) ) - (e_theta * (beta_theta/pow(mu_theta,2)) ) - ( e_phi * e_psi * ((Iz-Ix)/(Iy * mu_phi * mu_psi))) );

   *u_psi = Iz * ( (dot_epsi * (alpha_psi/mu_psi) ) - (e_psi * (beta_psi/pow(mu_psi,2)) ) - ( e_phi * e_theta * ((Ix-Iy)/(Iz * mu_phi * mu_theta))) );;

}

}
