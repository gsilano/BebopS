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

#ifndef _FILTER_PARAMETERS_H_
#define _FILTER_PARAMETERS_H_

#include <Eigen/Eigen>
#include <ros/ros.h>

namespace bebop_simulator {

static constexpr double DefaultDevX = 0.01;
static constexpr double DefaultDevY = 0.01;
static constexpr double DefaultDevZ = 0.01;

static constexpr double DefaultDevVX = 0.01;
static constexpr double DefaultDevVY = 0.01;
static constexpr double DefaultDevVZ = 0.01;

static constexpr double DefaultQpX = 1e-6;
static constexpr double DefaultQpY = 1e-6;
static constexpr double DefaultQpZ = 1e-6;

static constexpr double DefaultQpVX = 1e-6;
static constexpr double DefaultQpVY = 1e-6;
static constexpr double DefaultQpVZ = 1e-6;

class FilterParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FilterParameters(): 
	dev_x_(DefaultDevX), 
	dev_y_(DefaultDevY), 
	dev_z_(DefaultDevZ), 
	dev_vx_(DefaultDevVX),  
	dev_vy_(DefaultDevVY),
	dev_vz_(DefaultDevVZ),
	Qp_x_(DefaultQpX), 
	Qp_y_(DefaultQpY), 
	Qp_z_(DefaultQpZ), 
	Qp_vx_(DefaultQpVX),  
	Qp_vy_(DefaultQpVY),
	Qp_vz_(DefaultQpVZ),
	Rp_(Eigen::MatrixXf::Zero(6,6)),
	Qp_(Eigen::MatrixXf::Identity(6,6)){
  }

  Eigen::MatrixXf Rp_, Qp_;

  double dev_x_, Qp_x_;
  double dev_y_, Qp_y_;
  double dev_z_, Qp_z_;

  double dev_vx_, Qp_vx_;
  double dev_vy_, Qp_vy_;
  double dev_vz_, Qp_vz_;
};

}

#endif // _FILTER_PARAMETERS_H_
