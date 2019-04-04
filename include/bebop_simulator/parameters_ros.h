#ifndef INCLUDE_BEBOP_CONTROL_PARAMETERS_ROS_H_
#define INCLUDE_BEBOP_CONTROL_PARAMETERS_ROS_H_

#include <ros/ros.h>

#include "parameters.h"

namespace bebop_simulator {

template<typename T> inline void GetRosParameter(const ros::NodeHandle& nh,
                                                 const std::string& key,
                                                 const T& default_value,
                                                 T* value) {
  ROS_ASSERT(value != nullptr);
  bool have_parameter = nh.getParam(key, *value);
  if (!have_parameter) {
    ROS_WARN_STREAM("[rosparam]: could not find parameter " << nh.getNamespace()
                    << "/" << key << ", setting to default: " << default_value);
    *value = default_value;
  }
}

// The function allows to take data from the yaml file describing the quadrotor paramters
inline void GetVehicleParameters(const ros::NodeHandle& nh, VehicleParameters* vehicle_parameters) {
  GetRosParameter(nh, "mass",
                  vehicle_parameters->mass_,
                  &vehicle_parameters->mass_);
  GetRosParameter(nh, "inertia/xx",
                  vehicle_parameters->inertia_(0, 0),
                  &vehicle_parameters->inertia_(0, 0));
  GetRosParameter(nh, "inertia/xy",
                  vehicle_parameters->inertia_(0, 1),
                  &vehicle_parameters->inertia_(0, 1));
  vehicle_parameters->inertia_(1, 0) = vehicle_parameters->inertia_(0, 1);
  GetRosParameter(nh, "inertia/xz",
                  vehicle_parameters->inertia_(0, 2),
                  &vehicle_parameters->inertia_(0, 2));
  vehicle_parameters->inertia_(2, 0) = vehicle_parameters->inertia_(0, 2);
  GetRosParameter(nh, "inertia/yy",
                  vehicle_parameters->inertia_(1, 1),
                  &vehicle_parameters->inertia_(1, 1));
  GetRosParameter(nh, "inertia/yz",
                  vehicle_parameters->inertia_(1, 2),
                  &vehicle_parameters->inertia_(1, 2));
  vehicle_parameters->inertia_(2, 1) = vehicle_parameters->inertia_(1, 2);
  GetRosParameter(nh, "inertia/zz",
                  vehicle_parameters->inertia_(2, 2),
                  &vehicle_parameters->inertia_(2, 2));
  GetRosParameter(nh, "bf",
                  vehicle_parameters->bf_,
                  &vehicle_parameters->bf_);
  GetRosParameter(nh, "bm",
                  vehicle_parameters->bm_,
                  &vehicle_parameters->bm_);
  GetRosParameter(nh, "l",
                  vehicle_parameters->armLength_,
                  &vehicle_parameters->armLength_);
}

}

#endif /* INCLUDE_BEBOP_CONTROL_PARAMETERS_ROS_H_ */
