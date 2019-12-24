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

#ifndef BEBOP_CONTROL_POSITION_CONTROLLER_NODE_H
#define BEBOP_CONTROL_POSITION_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "bebop_simulator/common.h"
#include "bebop_simulator/position_controller_with_bebop.h"
#include "bebop_simulator/parameters_ros.h"
#include "bebop_simulator/parameters.h"

namespace bebop_simulator {

    class PositionControllerWithBebopNode{
        public:
            PositionControllerWithBebopNode();
            ~PositionControllerWithBebopNode();
             
            void InitializeParams();
            void Publish();

        private:

            bool waypointHasBeenPublished_ = false;
            bool takeOffMsgHasBeenSent_ = false;

            PositionControllerWithBebop position_controller_;

            //subscribers
            ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
            ros::Subscriber odom_sub_;

            //publisher
            ros::Publisher motor_velocity_reference_pub_;
            ros::Publisher takeoff_pub_;
            ros::Publisher odometry_filtered_pub_;
            ros::Publisher reference_angles_pub_;
            ros::Publisher smoothed_reference_pub_;

            mav_msgs::EigenTrajectoryPointDeque commands_;
            std::deque<ros::Duration> command_waiting_times_;
            ros::Timer command_timer_;

            void MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);
            void TakeOff();
            void OdomCallback(const nav_msgs::OdometryConstPtr& odom_msg);


    };
}

#endif // BEBOP_CONTROL_POSITION_CONTROLLER_NODE_H
