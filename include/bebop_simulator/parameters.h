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

#ifndef INCLUDE_BEBOP_CONTROL_PARAMETERS_H_
#define INCLUDE_BEBOP_CONTROL_PARAMETERS_H_

namespace bebop_simulator {

// Default Bebop 2 parameters
static constexpr double kDefaultMass = 0.5;
static constexpr double kDefaultArmLength = 0.12905;
static constexpr double kDefaultInertiaXx = 0.00389;
static constexpr double kDefaultInertiaYy = 0.00389;
static constexpr double kDefaultInertiaZz = 0.0078;
static constexpr double kDefaultRotorForceConstant = 8.54858e-6;
static constexpr double kDefaultRotorMomentConstant = 0.016;

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

#endif /* INCLUDE_BEBOP_CONTROL_PARAMETERS_H_ */
