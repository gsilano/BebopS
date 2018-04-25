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

#ifndef BEBOP_CONTROL_POSITION_CONTROLLER_H
#define BEBOP_CONTROL_POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "extendedKalmanFilter.h"
#include "stabilizer_types.h"
#include "parameters.h"
#include "common.h"


namespace teamsannio_med_control {

// Default values for the position controller of the Bebop. XYController [x,y], Roll Control [phi],
// Pitch Control [theta], Altitude Control [z], Yaw Control  [psi] 
static const Eigen::Vector2d kPDefaultXYController = Eigen::Vector2d(-1.3351, -1.1307);
static const double kPDefaultAltitudeController = -1.5994;

static const double kPDefaultPitchController = -2.7457;
static const double kPDefaultRollController = -2.2616;
static const double kPDefaultYawRateController = -1.8249;

static const Eigen::Vector2d MuDefaultXYController = Eigen::Vector2d(1, 1);
static const double MuDefaultAltitudeController = 0.12;

static const double MuDefaultPitchController = 0.26;
static const double MuDefaultYawRateController = 0.04;
static const double MuDefaultRollController = 0.09;


class PositionControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PositionControllerParameters()
      : xy_gain_kp_(kPDefaultXYController), 
        z_gain_kp_(kPDefaultAltitudeController), 
        roll_gain_kp_(kPDefaultRollController), 
        pitch_gain_kp_(kPDefaultPitchController),  
        yaw_rate_gain_kp_(kPDefaultYawRateController),
        mu_xy_(MuDefaultXYController),
        mu_z_(MuDefaultAltitudeController),
        mu_pitch_(MuDefaultPitchController),
        mu_roll_(MuDefaultRollController),
        mu_yaw_(MuDefaultYawRateController){
  }

  Eigen::Vector2d xy_gain_kp_;
  double z_gain_kp_;

  double roll_gain_kp_;
  double pitch_gain_kp_;
  double yaw_rate_gain_kp_;

  Eigen::Vector2d mu_xy_;
  double mu_z_;

  double mu_pitch_;
  double mu_yaw_;
  double mu_roll_;
};
    
    class PositionController{
        public:
            PositionController();
            ~PositionController();
            void CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities);

            void SetOdometry(const EigenOdometry& odometry);
            void SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);
            
            PositionControllerParameters controller_parameters_;
            ExtendedKalmanFilter extended_kalman_filter_bebop_;
            VehicleParameters vehicle_parameters_;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        private:
            bool controller_active_;
            bool first_time_derivate_computing_XY;
            bool first_time_derivate_computing_Z;
            bool first_time_derivate_computing_Angles;

	    state_t state_;
            mav_msgs::EigenTrajectoryPoint command_trajectory_;
            EigenOdometry odometry_;

            void SetOdometryEstimated();
            void ErrorsXY(double* e_x, double* dot_ex, double* e_y, double* dot_ey);
            void ErrorsZ(double* e_z, double* dot_ez);
            void ErrorsAngles(double* e_phi, double* dot_ephi, double* e_theta, double* dot_etheta, double* e_psi, double* dot_epsi);
            void PositionControl(double* u_x, double* u_y);
            void DesiredActuation(double* tilde_ux, double* tilde_uy);
            void AttitudePlanner(double* phi_r, double* theta_r);
            void AltitudeControl(double* u_T);
            void RollPitchYawControl(double* u_phi, double* u_theta, double* u_psi);

    };

}
#endif // BEBOP_CONTROL_POSITION_CONTROLLER_H
