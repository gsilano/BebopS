/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#ifndef INCLUDE_BEBOP_CONTROL_COMMON_H_
#define INCLUDE_BEBOP_CONTROL_COMMON_H_

#include <assert.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>


namespace bebop_simulator {

// Default values.
static const std::string kDefaultNamespace = "";
static const std::string kDefaultCommandMotorSpeedTopic =
    mav_msgs::default_topics::COMMAND_ACTUATORS; // "command/motor_speed";
static const std::string kDefaultCommandMultiDofJointTrajectoryTopic =
    mav_msgs::default_topics::COMMAND_TRAJECTORY; // "command/trajectory"
static const std::string kDefaultCommandRollPitchYawrateThrustTopic =
    mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST;
    // "command/roll_pitch_yawrate_thrust"
static const std::string kDefaultImuTopic =
    mav_msgs::default_topics::IMU; // "imu
static const std::string kDefaultOdometryTopic =
    mav_msgs::default_topics::ODOMETRY; // "odometry"

struct EigenOdometry {
  EigenOdometry()
      : timeStampSec(-1),
        timeStampNsec(-1),
        position(0.0, 0.0, 0.0),
        orientation(Eigen::Quaterniond::Identity()),
        velocity(0.0, 0.0, 0.0),
        angular_velocity(0.0, 0.0, 0.0) {};

  EigenOdometry(const double _timeStampSec,
                const double _timeStampNsec,
                const Eigen::Vector3d& _position,
                const Eigen::Quaterniond& _orientation,
                const Eigen::Vector3d& _velocity,
                const Eigen::Vector3d& _angular_velocity) {
    
    timeStampSec = _timeStampSec;
    timeStampNsec = _timeStampNsec;
    position = _position;
    orientation = _orientation;
    velocity = _velocity;
    angular_velocity = _angular_velocity;
  };

  double timeStampSec; //nano seconds timeStamp Odometry message
  double timeStampNsec; //nano seconds timeStamp Odometry message
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d velocity; // Velocity is expressed in the Body frame!
  Eigen::Vector3d angular_velocity;
};

inline void eigenOdometryFromMsg(const nav_msgs::OdometryConstPtr& msg,
                                 EigenOdometry* odometry) {
  odometry->timeStampSec = msg->header.stamp.sec;
  odometry->timeStampNsec = msg->header.stamp.nsec;
  odometry->position = mav_msgs::vector3FromPointMsg(msg->pose.pose.position);
  odometry->orientation = mav_msgs::quaternionFromMsg(msg->pose.pose.orientation);
  odometry->velocity = mav_msgs::vector3FromMsg(msg->twist.twist.linear);
  odometry->angular_velocity = mav_msgs::vector3FromMsg(msg->twist.twist.angular);
}

}
#endif /* INCLUDE_BEBOP_CONTROL_COMMON_H_ */
