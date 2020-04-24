/*
 * Copyright 2020 Ria Sonecha, Massachusetts Institute of Technology in Cambridge, MA, USA
 * Copyright 2020 Giuseppe Silano, University of Sannio in Benevento, Italy
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


#ifndef INCLUDE_BEBOP_SIMULATOR_PARAMETERS_H_
#define INCLUDE_BEBOP_SIMULATOR_PARAMETERS_H_

#include <mav_msgs/eigen_mav_msgs.h>

using namespace std;

namespace bebop_simulator {

// Default vehicle parameters for Asctec Firefly.
static constexpr double kDefaultMass = 1.56779;
static constexpr double kDefaultArmLength = 0.215;
static constexpr double kDefaultInertiaXx = 0.0347563;
static constexpr double kDefaultInertiaYy = 0.0458929;
static constexpr double kDefaultInertiaZz = 0.0977;
static constexpr double kDefaultRotorForceConstant = 8.54858e-6;
static constexpr double kDefaultRotorMomentConstant = 1.6e-2;

// Default physics parameters.
static constexpr double kDefaultGravity = 9.81;


class VehicleParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VehicleParameters()
      : mass_(kDefaultMass),
        gravity_(kDefaultGravity),
        inertia_(Eigen::Vector3d(kDefaultInertiaXx, kDefaultInertiaYy,
                                 kDefaultInertiaZz).asDiagonal()),
        bf_(kDefaultRotorForceConstant),
        bm_(kDefaultRotorMomentConstant),
        armLength_(kDefaultArmLength) {}
  double mass_;
  const double gravity_;
  Eigen::Matrix3d inertia_;
  double bm_;
  double bf_;
  double armLength_;
};

}

#endif /* INCLUDE_BEBOP_SIMULATOR_PARAMETERS_H_ */
