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


namespace teamsannio_med_control {

ExtendedKalmanFilter::ExtendedKalmanFilter() {}

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

   double roll, pitch, yaw;
   Quaternion2Euler(&roll, &pitch, &yaw);

   state_->attitude.roll = roll;
   state_->attitude.pitch = pitch;
   state_->attitude.yaw = yaw;
 
   state_->angularVelocity.x = odometry_private_.angular_velocity[0];
   state_->angularVelocity.y = odometry_private_.angular_velocity[1];
   state_->angularVelocity.z = odometry_private_.angular_velocity[2];
 
}

void ExtendedKalmanFilter::Quaternion2Euler(double* roll, double* pitch, double* yaw) const {
    
	
}



  void Ekf::Predict(double* u_T, double* hatx, double* Pp)
  {
    assert(roll);
    assert(pitch);
    assert(yaw);

    double x, y, z, dx, dy, dz, w;
    x = odometry_private_.orientation.x();
    y = odometry_private_.orientation.y();
    z = odometry_private_.orientation.z();
	dx = odometry_private_.velocity[0];
	dy = odometry_private_.velocity[1];
	dz = odometry_private_.velocity[2];
	
    w = odometry_private_.orientation.w();
    
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(*roll, *pitch, *yaw);
	
	// Non linear Position model discretized with forward Euler
	x = x + Tsp_ * dx;
	y = y + Tsp_ * dy;
	z = z + Tsp_ * dz;
	dx = dx + Tsp_ * (*u_T/m_ * (cos(state_.attitude.yaw) * sin(state_.attitude.pitch) * cos(state_.attitude.roll) + sin(state_.attitude.yaw) * sin(state_.attitude.pitch)));
	dy = dy + Tsp_ * (*u_T/m_ * (sin(state_.attitude.yaw) * sin(state_.attitude.pitch) * cos(state_.attitude.roll) - cos(state_.attitude.yaw) * sin(state_.attitude.roll)));
	dx = dx + Tsp_ * (-g_ + *u_T/m_ * (cos(state_.attitude.pitch) * cos(state_.attitude.roll));
	
	// Jacobian Matrix
	Matrix6f A;
	
	A <<  1, 0, 0,  Tsp_,    0,    0,
		  0, 1, 0,     0, Tsp_,    0,
		  0, 0, 1, 	  0,    0, Tsp_,
		  0, 0, 0,     1,    0,    0,
		  0, 0, 0,     0,    1,    0,
		  0, 0, 0,     0,    0,    1;
		  
	Matrix6f Qp;
	
	Qp << 0.000001,			0,			0,			0,			0,			0,
				 0,	 0.000001,			0,			0,			0,			0,
				 0,			0,   0.000001,			0,			0,			0,
				 0,			0,			0,   0.000001,			0,			0,
				 0,			0,			0,			0,   0.000001, 			0,
				 0,			0,			0,			0,			0,	 0.000001;
				 
	Qp = pow(Qp,2);
	
	// Prediction error Matrix
	P = A * Pp * A.transpose() + Qp;
	
	VectorXd xp(6);
	
	xp(1) = x;
	xp(2) = y;
	xp(3) = z;
	xp(4) = dx;
	xp(5) = dy;
	xp(6) = dz;
				 
}	


void EKF::Correct(double* xp, double* P){
	
	double x, y, z, dx, dy, dz, w;
    x = odometry_private_.orientation.x();
    y = odometry_private_.orientation.y();
    z = odometry_private_.orientation.z();
	dx = odometry_private_.velocity[0];
	dy = odometry_private_.velocity[1];
	dz = odometry_private_.velocity[2];
	
	Matrix6f Hp;
	
	Hp << 1, 0, 0, 0, 0, 0,
		  0, 1, 0, 0, 0, 0,
		  0, 0, 1, 0, 0, 0,
		  0, 0, 0, 1, 0, 0,
		  0, 0, 0, 0, 1, 0,
		  0, 0, 0, 0, 0, 1;
	
	Matrix6f Rp;
	
	Rp << 0.01,    0,    0,    0,    0,    0,
			 0, 0.01,    0,    0,    0,    0,
			 0,    0, 0.01,    0,    0,    0,
			 0,    0,    0, 0.01,    0,    0,
			 0,    0,    0,    0, 0.01,    0,
			 0,    0,    0,    0,    0, 0.01;
	
	Rp = pow(Rp,2);
	
	K = P * Hp * (Hp.transpose() * P * Hp + Rp).inverse();
	
	pe = P - K * Hp.transpose() * P;
	xe = xp + K * (y - Hp * xp);
	
	
}