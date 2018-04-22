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

#include <mav_msgs/eigen_mav_msgs.h>
#include "teamsannio_med_control/transform_datatypes.h"
#include "teamsannio_med_control/Matrix3x3.h"
#include "teamsannio_med_control/Quaternion.h"
#include <nav_msgs/Odometry.h>
#include <mav_msgs/conversions.h>

#include "stabilizer_types.h"
#include "common.h"


namespace teamsannio_med_control {

class ExtendedKalmanFilter {
  public:
    ExtendedKalmanFilter();
    ~ExtendedKalmanFilter();

    void Estimator(state_t *state_, EigenOdometry* odometry_);

  private:

    EigenOdometry odometry_private_;

    void Quaternion2Euler(double* roll, double* pitch, double* yaw) const;
    void SetOdometry(const EigenOdometry& odometry);

};

}

#endif // _ESTIMATOR_EXTENDED_KALMAN_FILTER_H_


