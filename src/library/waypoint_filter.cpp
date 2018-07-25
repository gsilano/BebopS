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

#include "teamsannio_med_control/waypoint_filter.h"

#include <Eigen/Eigen>
#include <chrono>
#include <time.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#define T 1.5 /* Waypoint filter pole [s] */
#define H 10e-3 /* Sampling time [s] */

using namespace std;

namespace teamsannio_med_control {

WaypointFilter::WaypointFilter(){

         command_trajectory_private_.setFromYaw(0);
         command_trajectory_toSend_.setFromYaw(0);

         command_trajectory_private_.position_W[0] = 0;
         command_trajectory_private_.position_W[1] = 0;
         command_trajectory_private_.position_W[2] = 0;

         command_trajectory_toSend_.position_W[0] = 0;
         command_trajectory_toSend_.position_W[1] = 0;
         command_trajectory_toSend_.position_W[2] = 0;


}

WaypointFilter::~WaypointFilter() {}

void WaypointFilter::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory_positionControllerNode){

    command_trajectory_private_ = command_trajectory_positionControllerNode;

}

void WaypointFilter::GetTrajectoryPoint(mav_msgs::EigenTrajectoryPoint* command_trajectory_positionController){

    *command_trajectory_positionController = command_trajectory_toSend_;

}

void WaypointFilter::TrajectoryGeneration(){

    command_trajectory_toSend_.position_W[0] = (T/(T+H)) * command_trajectory_toSend_.position_W[0] + (H/(T+H)) * command_trajectory_private_.position_W[0];
    command_trajectory_toSend_.position_W[1] = (T/(T+H)) * command_trajectory_toSend_.position_W[1] + (H/(T+H)) * command_trajectory_private_.position_W[1];
    command_trajectory_toSend_.position_W[2] = (T/(T+H)) * command_trajectory_toSend_.position_W[2] + (H/(T+H)) * command_trajectory_private_.position_W[2];

    double yaw = (T/(T+H)) * command_trajectory_toSend_.getYaw() + (H/(T+H)) * command_trajectory_private_.getYaw();

    command_trajectory_toSend_.setFromYaw(yaw);

}


}
