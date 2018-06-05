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

#include "teamsannio_med_control/extendedKalmanFilter.h"
#include "teamsannio_med_control/transform_datatypes.h"
#include "teamsannio_med_control/Matrix3x3.h"
#include "teamsannio_med_control/Quaternion.h" 
#include "teamsannio_med_control/stabilizer_types.h"
#include "teamsannio_med_control/common.h"

#include <math.h> 
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <mav_msgs/conversions.h>
#include <ros/console.h>

#define TsP                       10e-3  /* Position control sampling time */



namespace teamsannio_med_control {

ExtendedKalmanFilter::ExtendedKalmanFilter() {
	
	timer11_ = n11_.createTimer(ros::Duration(TsP), &ExtendedKalmanFilter::CallbackEstimate, this, false, true); 
	
}

ExtendedKalmanFilter::~ExtendedKalmanFilter() {}

void ExtendedKalmanFilter::SetOdometry(const EigenOdometry& odometry) {
    
    odometry_private_ = odometry;    
}

void ExtendedKalmanFilter::Estimator(state_t *state_, EigenOdometry* odometry_){
   assert(state_);
   assert(odometry_);

   SetOdometry(*odometry_);

   state_->position.x = odometry_private_.position[0];
   state_->position.y = odometry_private_.position[1];
   state_->position.z = odometry_private_.position[2];
  
   state_->linearVelocity.x = odometry_private_.velocity[0];
   state_->linearVelocity.y = odometry_private_.velocity[1];
   state_->linearVelocity.z = odometry_private_.velocity[2];

   state_->attitude.roll = roll;
   state_->attitude.pitch = pitch;
   state_->attitude.yaw = yaw;
 
   state_->angularVelocity.x = odometry_private_.angular_velocity[0];
   state_->angularVelocity.y = odometry_private_.angular_velocity[1];
   state_->angularVelocity.z = odometry_private_.angular_velocity[2];
 
}

void ExtendedKalmanFilter::SetVehicleParameters(){

      m_ = vehicle_parameters_.mass_;
      g_ = vehicle_parameters_.gravity_;
      Ix_ = vehicle_parameters_.inertia_(0,0);
      Iy_ = vehicle_parameters_.inertia_(1,1);
      Iz_ = vehicle_parameters_.inertia_(2,2);

}

void ExtendedKalmanFilter::Quaternion2Euler(double* roll, double* pitch, double* yaw) const {
    assert(roll);
    assert(pitch);
    assert(yaw);

    double x, y, z, w;
    x = odometry_private_.orientation.x();
    y = odometry_private_.orientation.y();
    z = odometry_private_.orientation.z();
    w = odometry_private_.orientation.w();
    
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(*roll, *pitch, *yaw);
	
}

void ExtendedKalmanFilter::predict(Eigen::Vector6f* xp, Eigen::Matrix6f* P){
    assert(xp);
    assert(P);

    double x, y, z, dx, dy, dz;
    x  = odometry_private_.position[0];
    y  = odometry_private_.position[1];
    z  = odometry_private_.position[2];
	dx = odometry_private_.velocity[0];
	dy = odometry_private_.velocity[1];
	dz = odometry_private_.velocity[2];
	
	phi = state.attitude.roll;
	theta = state.attitude.pitch;
	psi = state.attitude.yaw;
	
	
	dot_x = (cos(theta) * cos(psi) * dx) + 
           ( ( (sin(phi) * sin(theta) * cos(psi) ) - ( cos(phi) * sin(psi) ) ) * dy) + 
           ( ( (cos(phi) * sin(theta) * cos(psi) ) + ( sin(phi) * sin(psi) ) ) * dz); 

    dot_y = (cos(theta) * sin(psi) * dx) +
           ( ( (sin(phi) * sin(theta) * sin(psi) ) + ( cos(phi) * cos(psi) ) ) * dy) +
           ( ( (cos(phi) * sin(theta) * sin(psi) ) - ( sin(phi) * cos(psi) ) ) * dz);

   dot_z = (-sin(theta) * dx) + ( sin(phi) * cos(theta) * dy) +
           (cos(phi) * cos(theta) * dz);
   
   
   double u_x, u_y, u_T, u_Terr;
   PositionController::PosController(&u_x, &u_y, &u_T, &u_Terr);
	
	// Non linear Position model discretized with forward Euler
	x = x + Tsp_ * dot_x;
	y = y + Tsp_ * dot_y;
	z = z + Tsp_ * dot_z;
	dot_x = dot_x + Tsp_ * (*u_T/m_ * (cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(theta)));
	dot_y = dot_y + Tsp_ * (*u_T/m_ * (sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)));
	dot_z = dot_z + Tsp_ * (-g_ + *u_T/m_ * (cos(theta) * cos(phi));
	
	// Jacobian Matrix
	Eigen::Matrix6f A;
	
	A <<  1, 0, 0,  Tsp_,    0,    0,
		  0, 1, 0,     0, Tsp_,    0,
		  0, 0, 1, 	   0,    0, Tsp_,
		  0, 0, 0,     1,    0,    0,
		  0, 0, 0,     0,    1,    0,
		  0, 0, 0,     0,    0,    1;
		  
	Eigen::Matrix6f Qp;
	
	Qp <<     1e-6,			0,			0,			0,			0,			0,
				 0,	     1e-6,			0,			0,			0,			0,
				 0,			0,       1e-6,			0,			0,			0,
				 0,			0,			0,       1e-6,			0,			0,
				 0,			0,			0,			0,       1e-6, 			0,
				 0,			0,			0,			0,			0,	     1e-6;
				 
	Qp = pow(Qp,2);
	
	// Prediction error Matrix
	Eigen::MatrixXd P = A * P * A.transpose() + Qp;
	
	Eigen::VectorXd xp(6);
	
	xp(1) = x;
	xp(2) = y;
	xp(3) = z;
	xp(4) = dot_x;
	xp(5) = dot_y;
	xp(6) = dot_z;
				 
}	


void ExtendedKalmanFilter::correct(Eigen::Matrix6f* xe, Eigen::Matrix6f* pe){
	
	Eigen::Matrix6f P;
	Eigen::Vector6f xp;
	predict( &xp,  &P);
	
	Eigen::Matrix6f Hp;
	
	Hp << 1, 0, 0, 0, 0, 0,
		  0, 1, 0, 0, 0, 0,
		  0, 0, 1, 0, 0, 0,
		  0, 0, 0, 1, 0, 0,
		  0, 0, 0, 0, 1, 0,
		  0, 0, 0, 0, 0, 1;
	
	Eigen::Matrix6f Rp;
	
	Rp << 0.01,    0,    0,    0,    0,    0,
			 0, 0.01,    0,    0,    0,    0,
			 0,    0, 0.01,    0,    0,    0,
			 0,    0,    0, 0.01,    0,    0,
			 0,    0,    0,    0, 0.01,    0,
			 0,    0,    0,    0,    0, 0.01;
	
	Rp = pow(Rp,2);
	
	Eigen::Matrix6f K = P_ * Hp * (Hp.transpose() * P_ * Hp + Rp).inverse();
	
	Eigen::Matrix6f *pe = P - K * Hp.transpose() * P_;
	Eigen::Matrix6f *xe = xp_ + K * (y - Hp * xp_);
	
	
}

void ExtendedKalmanFilter::CallbackEstimate(const ros::TimerEvent& event){
     
     predict(&xp, &P);
     correct(&xp, &pe);
}


}