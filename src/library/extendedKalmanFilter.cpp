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

ExtendedKalmanFilter::ExtendedKalmanFilter()
       :Xp_(Eigen::VectorXf::Zero(6)), 
        Xe_(Eigen::VectorXf::Zero(6)),
	P_(Eigen::MatrixXf::Zero(6,6)),
        Pe_(Eigen::MatrixXf::Zero(6,6)),
        Hatx_(Eigen::VectorXf::Zero(6)),
        u_T_private_(0),
        m_private_(0),
        g_private_(0),
        A_private_(Eigen::MatrixXf::Zero(6,6)),
        Hp_(Eigen::MatrixXf::Identity(6,6)),
        Rp_private_(Eigen::MatrixXf::Zero(6,6)),
        Qp_private_(Eigen::MatrixXf::Identity(6,6)),
        Qp_std_(Eigen::MatrixXf::Zero(6,6)),
        Rp_std_(Eigen::MatrixXf::Zero(6,6)){

              	
		A_private_ <<   1, 0, 0,  TsP,    0,    0,
				0, 1, 0,     0, TsP,    0,
				0, 0, 1, 	   0,    0, TsP,
				0, 0, 0,     1,    0,    0,
				0, 0, 0,     0,    1,    0,
				0, 0, 0,     0,    0,    1;

}

ExtendedKalmanFilter::~ExtendedKalmanFilter() {}

void ExtendedKalmanFilter::SetOdometry(const EigenOdometry& odometry) {
    
    odometry_private_ = odometry;    
}

void ExtendedKalmanFilter::SetFilterParameters(FilterParameters *filter_parameters_){

     Rp_private_ = filter_parameters_->Rp_; 
     Qp_private_ = filter_parameters_->Qp_;

     Qp_std_ = Qp_private_.transpose()*Qp_private_;
                 	
     Rp_std_ = Rp_private_.transpose()*Rp_private_;
}

void ExtendedKalmanFilter::Estimator(state_t *state_, EigenOdometry* odometry_){
   assert(state_);
   assert(odometry_);

   SetOdometry(*odometry_);

   Predict();
   Correct();

   state_->position.x = Xe_(0);
   state_->position.y = Xe_(1);
   state_->position.z = Xe_(2);

   state_->linearVelocity.x = Xe_(3);
   state_->linearVelocity.x = Xe_(4);
   state_->linearVelocity.x = Xe_(5);   
 
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


void ExtendedKalmanFilter::SetThrustCommand(double u_T){

    u_T_private_ = u_T;

}

void ExtendedKalmanFilter::SetVehicleParameters(double m, double g){

      m_private_ = m;
      g_private_ = g;
      
}

void ExtendedKalmanFilter::Predict(){

    double phi, theta, psi;
    Quaternion2Euler(&phi, &theta, &psi);

    double x, y, z, dx, dy, dz;
    x  = Hatx_(0);
    y  = Hatx_(1);
    z  = Hatx_(2);

    dx = Hatx_(3);
    dy = Hatx_(4);
    dz = Hatx_(5);

    double dx_ENU, dy_ENU, dz_ENU;
	
    dx_ENU = (cos(theta) * cos(psi) * dx) + 
	     ( ( (sin(phi) * sin(theta) * cos(psi) ) - ( cos(phi) * sin(psi) ) ) * dy) + 
	     ( ( (cos(phi) * sin(theta) * cos(psi) ) + ( sin(phi) * sin(psi) ) ) * dz); 

    dy_ENU = (cos(theta) * sin(psi) * dx) +
	     ( ( (sin(phi) * sin(theta) * sin(psi) ) + ( cos(phi) * cos(psi) ) ) * dy) +
	     ( ( (cos(phi) * sin(theta) * sin(psi) ) - ( sin(phi) * cos(psi) ) ) * dz);

    dz_ENU = (-sin(theta) * dx) + ( sin(phi) * cos(theta) * dy) +
	     (cos(phi) * cos(theta) * dz);


     //Nonlinear state propagation 
     x = x + TsP * dx_ENU;
     y = y + TsP * dy_ENU;
     z = z + TsP * dz_ENU;
     dx_ENU = dx_ENU + TsP * (u_T_private_ * (1/m_private_) * (cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(theta)));
     dy_ENU = dy_ENU + TsP * (u_T_private_ * (1/m_private_) * (sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)));
     dz_ENU = dz_ENU + TsP * (-g_private_ + u_T_private_ * (1/m_private_) * (cos(theta) * cos(phi)));
	
		 				 
     // Prediction error Matrix
     P_ = A_private_*(P_)*A_private_.transpose() + Qp_std_;
		
     //The predicted state
     Xp_ << x, y, z, dx, dy, dz;

}	


void ExtendedKalmanFilter::Correct(){
	
      Eigen::VectorXf Meas(6);

      Meas<< odometry_private_.position[0],
	     odometry_private_.position[1],
	     odometry_private_.position[2],
	     odometry_private_.velocity[0],
	     odometry_private_.velocity[1],
	     odometry_private_.velocity[2];

	
       Eigen::MatrixXf K(6,6);
       K = P_ * Hp_ * (Hp_.transpose() * P_ * Hp_ + Rp_std_).inverse();
	
       Pe_ = P_ - K * Hp_.transpose() * P_;
		
       Xe_ = Xp_ + K * (Meas - Hp_ * Xp_);
	
}


}
