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

#ifndef _ESTIMATOR_EXTENDED_KALMAN_FILTER_H_
#define _ESTIMATOR_EXTENDED_KALMAN_FILTER_H_

#include "bebop_simulator/transform_datatypes.h"
#include "bebop_simulator/Matrix3x3.h"
#include "bebop_simulator/Quaternion.h"

#include <nav_msgs/Odometry.h>
#include <mav_msgs/conversions.h>
#include <Eigen/Eigen>
#include <mav_msgs/eigen_mav_msgs.h>

#include <random>
#include <cmath>

#include "stabilizer_types.h"
#include "filter_parameters.h"
#include "common.h"
#include "parameters_ros.h"

namespace bebop_simulator {

class ExtendedKalmanFilter {
  public:
     
     ExtendedKalmanFilter();
     ~ExtendedKalmanFilter();

     void EstimatorWithoutNoise(state_t *state_, EigenOdometry* odometry_, nav_msgs::Odometry* odometry_filtered);
     void EstimatorWithNoise(state_t *state_, EigenOdometry* odometry_, nav_msgs::Odometry* odometry_filtered);
     void SetThrustCommand(double u_T);
     void SetVehicleParameters(double m, double g);
     void SetFilterParameters(FilterParameters *filter_parameters_);

     //The function is used to disable the Extended Kalman Filter
     void Estimator(state_t *state_, EigenOdometry* odometry_);

  private:

     EigenOdometry odometry_private_;

     //Filter Vectors
     Eigen::VectorXf Xp_, Xe_, Hatx_;
     Eigen::MatrixXf P_, Pe_;
     Eigen::MatrixXf Rp_private_, Qp_private_;

     Eigen::MatrixXf A_private_, Qp_std_, Rp_std_, Hp_;

     std::normal_distribution<double> distribution_;
 
     //Vehicle parameters
     double m_private_, g_private_;
     double u_T_private_;

     void Quaternion2Euler(double* roll, double* pitch, double* yaw) const;
     void AttitudeAddingNoise(double *phin, double *thetan, double* psin, double phi, double theta, double psi);
     void SetOdometry(const EigenOdometry& odometry);
     void CorrectWithoutNoise();
     void CorrectWithNoise();
     void PredictWithoutNoise();
     void PredictWithNoise();

   };

}

#endif // _ESTIMATOR_EXTENDED_KALMAN_FILTER_H_


