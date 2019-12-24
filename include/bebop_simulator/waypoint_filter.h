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

#ifndef WAYPOINT_FILTER_H
#define WAYPOINT_FILTER_H

#include <ros/time.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include "stabilizer_types.h"

#include "waypointfilter_parameters.h"

using namespace std;

namespace bebop_simulator {

	class WaypointFilter{

	      public:

		    WaypointFilter();
		    ~WaypointFilter();

		    void SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory_positionControllerNode);
		    void GetTrajectoryPoint(mav_msgs::EigenTrajectoryPoint* command_trajectory_positionController);
            void TrajectoryGeneration();
            void SetParameters(WaypointFilterParameters *waypointFilter_parameters_);
            void Initialize(state_t state_);

		    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	      private:

		    double Tsf_, H_;
		    bool filter_initialized_;

		    mav_msgs::EigenTrajectoryPoint command_trajectory_private_;
		    mav_msgs::EigenTrajectoryPoint command_trajectory_toSend_;

	};

}
#endif // WAYPOINT_FILTER_H
