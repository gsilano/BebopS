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

#include "bebop_simulator/waypoint_filter.h"

#include <Eigen/Eigen>
#include <chrono>
#include <time.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>


using namespace std;

namespace bebop_simulator {

WaypointFilter::WaypointFilter()
         :filter_initialized_(false){

         command_trajectory_private_.setFromYaw(0);
         command_trajectory_toSend_.setFromYaw(0);

         command_trajectory_private_.position_W[0] = 0;
         command_trajectory_private_.position_W[1] = 0;
         command_trajectory_private_.position_W[2] = 0;

         command_trajectory_toSend_.position_W[0] = 0;
         command_trajectory_toSend_.position_W[1] = 0;
         command_trajectory_toSend_.position_W[2] = 0;

         Tsf_ = 0;
         H_ = 0;


}

WaypointFilter::~WaypointFilter() {}

// Set the filter parameters
void WaypointFilter::SetParameters(WaypointFilterParameters *waypointFilter_parameters_){


     Tsf_ = waypointFilter_parameters_->tsf_;
     H_ = waypointFilter_parameters_->h_;

}

// Set the trajectory point
void WaypointFilter::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory_positionControllerNode){

    command_trajectory_private_ = command_trajectory_positionControllerNode;

}

// Get the trajectory point from the control library
void WaypointFilter::GetTrajectoryPoint(mav_msgs::EigenTrajectoryPoint* command_trajectory_positionController){

    *command_trajectory_positionController = command_trajectory_toSend_;

}

// Filter initialization
void WaypointFilter::Initialize(state_t state_){

   if(!filter_initialized_){

     command_trajectory_toSend_.position_W[0] = state_.position.x;
     command_trajectory_toSend_.position_W[1] = state_.position.y;
     command_trajectory_toSend_.position_W[2] = state_.position.z;

     filter_initialized_ = true;

   }
}

// Trajectory generation
void WaypointFilter::TrajectoryGeneration(){

    command_trajectory_toSend_.position_W[0] = (Tsf_/(Tsf_+H_)) * command_trajectory_toSend_.position_W[0] + (H_/(Tsf_+H_)) * command_trajectory_private_.position_W[0];
    command_trajectory_toSend_.position_W[1] = (Tsf_/(Tsf_+H_)) * command_trajectory_toSend_.position_W[1] + (H_/(Tsf_+H_)) * command_trajectory_private_.position_W[1];
    command_trajectory_toSend_.position_W[2] = (Tsf_/(Tsf_+H_)) * command_trajectory_toSend_.position_W[2] + (H_/(Tsf_+H_)) * command_trajectory_private_.position_W[2];

    double yaw = (Tsf_/(Tsf_+H_)) * command_trajectory_toSend_.getYaw() + (H_/(Tsf_+H_)) * command_trajectory_private_.getYaw();
    command_trajectory_toSend_.setFromYaw(yaw);
}


}
