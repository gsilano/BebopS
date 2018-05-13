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

#ifndef BEBOP_CONTROL_POSITION_CONTROLLER_H
#define BEBOP_CONTROL_POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>


#include <string>

#include <ros/time.h>

#include "extendedKalmanFilter.h"
#include "stabilizer_types.h"
#include "parameters.h"
#include "common.h"

using namespace std;



namespace teamsannio_med_control {

// Default values for the position controller of the Bebop. XYController [x,y], Roll Control [phi],
// Pitch Control [theta], Altitude Control [z], Yaw Control  [psi] 
static const Eigen::Vector2d kPDefaultXYController = Eigen::Vector2d(-1.3351, -1.1307);
static const double kPDefaultAltitudeController = -1.5994;

static const double kPDefaultRollController = -2.2616;
static const double kPDefaultPitchController = -2.7457;
static const double kPDefaultYawRateController = -1.8249;

static const Eigen::Vector2d MuDefaultXYController = Eigen::Vector2d(0.08, 0.03);
static const double MuDefaultAltitudeController = 0.12;

static const double MuDefaultRollController = 0.09;
static const double MuDefaultPitchController = 0.26;
static const double MuDefaultYawRateController = 0.04;



class PositionControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PositionControllerParameters()
      : beta_xy_(kPDefaultXYController), 
        beta_z_(kPDefaultAltitudeController), 
        beta_phi_(kPDefaultRollController), 
        beta_theta_(kPDefaultPitchController),  
        beta_psi_(kPDefaultYawRateController),
        mu_xy_(MuDefaultXYController),
        mu_z_(MuDefaultAltitudeController),
        mu_theta_(MuDefaultPitchController),
        mu_phi_(MuDefaultRollController),
        mu_psi_(MuDefaultYawRateController){
  }

  Eigen::Vector2d beta_xy_;
  double beta_z_;

  double beta_phi_;
  double beta_theta_;
  double beta_psi_;

  Eigen::Vector2d mu_xy_;
  double mu_z_;

  double mu_phi_;
  double mu_theta_;
  double mu_psi_;
};
    
    class PositionController{
        public:
            PositionController();
            ~PositionController();
            void CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities);

            void SetOdometry(const EigenOdometry& odometry);
            void SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);
            void SetControllerGains();
            void SetVehicleParameters();
            
            PositionControllerParameters controller_parameters_;
            ExtendedKalmanFilter extended_kalman_filter_bebop_;
            VehicleParameters vehicle_parameters_;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        private:
            bool controller_active_;
			bool dataStoring_active_;

            //Sting vectors used to stare data
            std::vector<string> listControlSignals_;
            std::vector<string> listControlMixerTerms_;
            std::vector<string> listPropellersAngularVelocities_;
            std::vector<string> listReferenceAngles_;
            std::vector<string> listVelocityErrors_;
            std::vector<string> listDroneAttitude_;
            std::vector<string> listTrajectoryErrors_;
            std::vector<string> listAttitudeErrors_;
            std::vector<string> listDerivativeAttitudeErrors_;
          
            //Controller gains
            double beta_x_, beta_y_, beta_z_;
            double beta_phi_, beta_theta_, beta_psi_;

            double alpha_x_, alpha_y_, alpha_z_;
            double alpha_phi_, alpha_theta_, alpha_psi_;

            double mu_x_, mu_y_, mu_z_;
            double mu_phi_, mu_theta_, mu_psi_;

            //Position and linear velocity errors
            double e_x_;
            double e_y_;
            double e_z_;
            double dot_e_x_;
            double dot_e_y_; 
            double dot_e_z_;
 
            //Attitude and angular velocity errors
            double e_phi_;
            double e_theta_;
            double e_psi_;
            double dot_e_phi_;
            double dot_e_theta_; 
            double dot_e_psi_;

            //Vehicle parameters
            double bf_, m_, g_;
            double l_, bm_;
            double Ix_, Iy_, Iz_;
            
            ros::NodeHandle n1_;
            ros::NodeHandle n2_;
            ros::NodeHandle n3_;
            ros::Timer timer1_;
            ros::Timer timer2_;
            ros::Timer timer3_;

            //Callback functions to compute the errors among axis and angles
            void CallbackAttitude(const ros::TimerEvent& event);
            void CallbackPosition(const ros::TimerEvent& event);
            void CallbackSaveData(const ros::TimerEvent& event);

	    state_t state_;
            mav_msgs::EigenTrajectoryPoint command_trajectory_;
            EigenOdometry odometry_;

            void SetOdometryEstimated();
            void AttitudeController(double* u_phi, double* u_theta, double* u_psi);
            void AngularVelocityErrors(double* dot_e_phi_, double* dot_e_theta_, double* dot_e_psi_);
            void AttitudeErrors(double* e_phi_, double* e_theta_, double* e_psi_);
            void PosController(double* u_x, double* u_y, double* u_T, double* u_Terr);
            void PositionErrors(double* e_x, double* e_y, double* e_z);
            void VelocityErrors(double* dot_e_x, double* dot_e_y, double* dot_e_z);
            void ReferenceAngles(double* phi_r, double* theta_r);

    };

}
#endif // BEBOP_CONTROL_POSITION_CONTROLLER_H
