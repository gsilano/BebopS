#ifndef INCLUDE_BEBOP_CONTROL_PARAMETERS_H_
#define INCLUDE_BEBOP_CONTROL_PARAMETERS_H_

namespace teamsannio_med_control {

// Default vehicle parameters
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
