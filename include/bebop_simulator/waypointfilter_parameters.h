/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
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

#ifndef _WAYPOINT_FILTER_PARAMETERS_H_
#define _WAYPOINT_FILTER_PARAMETERS_H_

#include <Eigen/Eigen>
#include <ros/ros.h>

namespace bebop_simulator {

static constexpr double DefaultH = 10e-3; /* Sampling time [s] */
static constexpr double DefaultTsf = 1.5; /* Waypoint filter pole [s] */

class WaypointFilterParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  WaypointFilterParameters()
      : tsf_(DefaultTsf),
        h_(DefaultH){
  }

  double tsf_;
  double h_;
};

}

#endif // _WAYPOINT_FILTER_PARAMETERS_H_
