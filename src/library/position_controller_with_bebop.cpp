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

#include "teamsannio_med_control/position_controller_with_bebop.h"
#include "teamsannio_med_control/transform_datatypes.h"
#include "teamsannio_med_control/Matrix3x3.h"
#include "teamsannio_med_control/Quaternion.h" 
#include "teamsannio_med_control/stabilizer_types.h"
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
#define MAX_THRUST                2175625 /* It is equal to pow(max_rotor_velocity,2) */

#define MAX_TILT_ANGLE            180 /* Current tilt max in degree */
#define MAX_VERT_SPEED            10  /* Current max vertical speed in m/s */
#define MAX_ROT_SPEED             900 /* Current max rotation speed in degree/s */

namespace teamsannio_med_control {

PositionControllerWithBebop::PositionControllerWithBebop()
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
      dot_e_psi_(0){  

            land_pub_ = n3_.advertise<std_msgs::Empty>(bebop_msgs::default_topics::LAND, 1);
            reset_pub_ = n3_.advertise<std_msgs::Empty>(bebop_msgs::default_topics::RESET, 1);

            timer1_ = n1_.createTimer(ros::Duration(TsA), &PositionControllerWithBebop::CallbackAttitude, this, false, true);
            timer2_ = n2_.createTimer(ros::Duration(TsP), &PositionControllerWithBebop::CallbackPosition, this, false, true); 

}

PositionControllerWithBebop::~PositionControllerWithBebop() {}

void PositionControllerWithBebop::SetControllerGains(){

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

void PositionControllerWithBebop::SetVehicleParameters(){

      bf_ = vehicle_parameters_.bf_;
      l_ = vehicle_parameters_.armLength_;
      bm_ = vehicle_parameters_.bm_;
      m_ = vehicle_parameters_.mass_;
      g_ = vehicle_parameters_.gravity_;
      Ix_ = vehicle_parameters_.inertia_(0,0);
      Iy_ = vehicle_parameters_.inertia_(1,1);
      Iz_ = vehicle_parameters_.inertia_(2,2);

}

void PositionControllerWithBebop::SetOdom(const EigenOdometry& odometry) {

   //+x forward, +y left, +z up, +yaw CCW
    odometry_ = odometry; 
    SetOdometryEstimated();

}

void PositionControllerWithBebop::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {

    command_trajectory_= command_trajectory;
    controller_active_= true;

}

void PositionControllerWithBebop::SetOdometryEstimated() {

    extended_kalman_filter_ardrone_.Estimator(&state_, &odometry_);
}

void PositionControllerWithBebop::ReferenceAngles(double* phi_r, double* theta_r){
   assert(phi_r);
   assert(theta_r);

   double psi_r;
   psi_r = command_trajectory_.getYaw();

   double u_x, u_y, u_Terr;
   PosController(&u_x, &u_y, &control_.thrust, &u_Terr);

   *theta_r = atan( ( (u_x * cos(psi_r) ) + ( u_y * sin(psi_r) ) )  / u_Terr );
   *phi_r = atan( cos(*theta_r) * ( ( (u_x * sin(psi_r)) - (u_y * cos(psi_r)) ) / (u_Terr) ) );
    
}

void PositionControllerWithBebop::PosController(double* u_x, double* u_y, double* u_T, double* u_Terr){
   assert(u_x);
   assert(u_y);
   assert(u_T);

   *u_x = m_ * ( (alpha_x_/mu_x_) * dot_e_x_) - ( (beta_x_/pow(mu_x_,2)) * e_x_);
   *u_y = m_ * ( (alpha_y_/mu_y_) * dot_e_y_) -  ( (beta_y_/pow(mu_y_,2)) *  e_y_);
   *u_Terr = m_ * ( g_ + ( (alpha_z_/mu_z_) * dot_e_z_) - ( (beta_z_/pow(mu_z_,2)) * e_z_) );
   *u_T = sqrt( pow(*u_x,2) + pow(*u_y,2) + pow(*u_Terr,2) );
   
}

void PositionControllerWithBebop::AttitudeController(double* u_phi, double* u_theta, double* u_psi){
   assert(u_phi);
   assert(u_theta);
   assert(u_psi);

   *u_phi = Ix_ * ( ( ( (alpha_phi_/mu_phi_) * dot_e_phi_) - ( (beta_phi_/pow(mu_phi_,2)) * e_phi_) ) - ( ( (Iy_ - Iz_)/(Ix_ * mu_theta_ * mu_psi_) ) * e_theta_ * e_psi_) );
   *u_theta = Iy_ * ( ( ( (alpha_theta_/mu_theta_) * dot_e_theta_) - ( (beta_theta_/pow(mu_theta_,2)) * e_theta_) ) - ( ( (Iz_ - Ix_)/(Iy_ * mu_phi_ * mu_psi_) ) * e_phi_ * e_psi_) );
   *u_psi = Iz_ * ( ( ( (alpha_psi_/mu_psi_) * dot_e_psi_) - ( (beta_psi_/pow(mu_psi_,2)) * e_psi_) ) - ( ( (Ix_ - Iy_)/(Iz_ * mu_theta_ * mu_phi_) ) * e_theta_ * e_phi_) );

}

void PositionControllerWithBebop::AngularVelocityErrors(double* dot_e_phi, double* dot_e_theta, double* dot_e_psi){
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

void PositionControllerWithBebop::CalculateCommandSignals(geometry_msgs::Twist* ref_command_signals) {
    assert(ref_command_signals);
    
    //this serves to inactivate the controller if we don't recieve a trajectory
    if(!controller_active_){
        ref_command_signals->linear.x = 0;
        ref_command_signals->linear.y = 0;
        ref_command_signals->linear.z = 0;
        ref_command_signals->angular.z = 0; 
        return;
    }
    
    double u_phi, u_theta;
    ReferenceAngles(&control_.roll, &control_.pitch);
    AttitudeController(&u_phi, &u_theta, &control_.yawRate);

    //The commands are normalized to take into account the real commands that can be send to the drone
    //Them range is between -1 and 1.
    double thrustNormalized, rollNormalized, pitchNormalized, yawRateNormalized;
    thrustNormalized = control_.thrust/MAX_VERT_SPEED;
    rollNormalized = control_.roll/MAX_TILT_ANGLE;
    pitchNormalized = control_.pitch/MAX_TILT_ANGLE;
    yawRateNormalized = control_.yawRate/MAX_ROT_SPEED;

    //The command signals are saturated to take into the SDK constrains in sending commands
    if(!(thrustNormalized > -1 && thrustNormalized < 1))
        if(thrustNormalized > 1)
           thrustNormalized = 1;
        else
           thrustNormalized = -1;

    if(!(rollNormalized > -1 && rollNormalized < 1))
        if(rollNormalized > 1)
           rollNormalized = 1;
        else
           rollNormalized = -1;

    if(!(pitchNormalized > -1 && pitchNormalized < 1))
        if(pitchNormalized > 1)
           pitchNormalized = 1;
        else
           pitchNormalized = -1;

    if(!(yawRateNormalized > -1 && yawRateNormalized < 1))
        if(yawRateNormalized > 1)
           yawRateNormalized = 1;
        else
           yawRateNormalized = -1;

	/*
	Tieni a mente per strutturazione legge di controllo
	roll_degree       = linear.y  * max_tilt_angle
	pitch_degree      = linear.x  * max_tilt_angle
	ver_vel_m_per_s   = linear.z  * max_vert_speed
	rot_vel_deg_per_s = angular.z * max_rot_speed
	*/
    ref_command_signals->linear.x = pitchNormalized;
    ref_command_signals->linear.y = rollNormalized;
    ref_command_signals->linear.z = thrustNormalized;
    ref_command_signals->angular.z = yawRateNormalized; 

}

void PositionControllerWithBebop::Land(){

    //DEVI GESTIRE QUANDO ATTERRARE, SOTTO QUALE CONDIZIONE
    
    std_msgs::Empty empty_msg;
    land_pub_.publish(empty_msg);

}

void PositionControllerWithBebop::Emergency(){

    //DEVI REALIZZARE UNA CONDIZIONE CHE, SUPERATO UN DETERMINATO VALORE DI VELOCITA' DELL'ERRORE LO PORTA IN PROTEZIONE

    std_msgs::Empty empty_msg;
    reset_pub_.publish(empty_msg);

}

void PositionControllerWithBebop::VelocityErrors(double* dot_e_x, double* dot_e_y, double* dot_e_z){
   assert(dot_e_x);
   assert(dot_e_y);
   assert(dot_e_z);

   double x_r, y_r, z_r;
   x_r = command_trajectory_.position_W[0];
   y_r = command_trajectory_.position_W[1]; 
   z_r = command_trajectory_.position_W[2];
   
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

void PositionControllerWithBebop::PositionErrors(double* e_x, double* e_y, double* e_z){
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

void PositionControllerWithBebop::AttitudeErrors(double* e_phi, double* e_theta, double* e_psi){
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

void PositionControllerWithBebop::CallbackAttitude(const ros::TimerEvent& event){
     
     AttitudeErrors(&e_phi_, &e_theta_, &e_psi_);
     AngularVelocityErrors(&dot_e_phi_, &dot_e_theta_, &dot_e_psi_);
     
}

void PositionControllerWithBebop::CallbackPosition(const ros::TimerEvent& event){
 
     PositionErrors(&e_x_, &e_y_, &e_z_);
     VelocityErrors(&dot_e_x_, &dot_e_y_, &dot_e_z_);
     
}


}
