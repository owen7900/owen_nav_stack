#pragma once

#include <chrono>
#include <eigen3/Eigen/Core>
#include <owen_common/Pose2D.hpp>
#include <rclcpp/rclcpp.hpp>

namespace Localization::Filters {

class UKF {
 public:
  struct OdomUpdate {
    double leftWheelSpeed;
    double rightWheelSpeed;
    Eigen::Matrix<double, 2, 2> v;
  };
  struct PositionUpdate {
    owen_common::types::Pose2D pose;
    Eigen::Matrix<double, 3, 3> v;
  };
  struct VelocityUpdate {
    double linearVel;
    double angVel;
    Eigen::Matrix<double, 2, 2> v;
  };

  using Pose = owen_common::types::Pose2D;
  static constexpr int NumStates = 5;
  /// x,y,yaw,vel,angVel
  using StateType = Eigen::Vector<double, NumStates>;
  using CovarianceType = Eigen::Matrix<double, NumStates, NumStates>;
  using TimePoint = rclcpp::Time;

 public:
  void Predict(const TimePoint &t);
  void Update(const TimePoint &t, const OdomUpdate &update);
  void Update(const TimePoint &t, const PositionUpdate &update);
  void Update(const TimePoint &t, const VelocityUpdate &update);

  Pose GetPose() const;
  StateType GetState() const;

 private:
  double getDt(const TimePoint &t) const;

 private:
  StateType x;
  CovarianceType cov;
  TimePoint lastTime;
};

}  // namespace Localization::Filters
