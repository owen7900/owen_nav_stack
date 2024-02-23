#include "owen_navigation/path_followers/PurePursuit.hpp"

namespace Navigation::PathFollowers {
namespace Constants {
const std::string Name = "pure_pursuit";
constexpr double Pi = 3.14159;
constexpr double HalfPi = Pi / 2.0;
constexpr double TwoPi = Pi * 2.0;

const std::string LookaheadDistanceName = Name + "_lookahead_distance";
const std::string SuccessRadiusName = Name + "_success_radius";
const std::string TurnGainName = Name + "_turn_gain";
const std::string MinObstacleDistanceName = Name + "_min_obstacle_distance";
const std::string ObstacleTurnLimitName = Name + "_obstacle_turn_limit";
const std::string ForwardGainName = Name + "_forward_gain";
const std::string AngularLinearWeightName = Name + "_angular_linear_weight";
const std::string MaxForwardVelName = Name + "_max_forward_vel";
const std::vector<std::string> AllParameters{
    LookaheadDistanceName,   SuccessRadiusName,     TurnGainName,
    MinObstacleDistanceName, ObstacleTurnLimitName, ForwardGainName,
    AngularLinearWeightName, MaxForwardVelName};

constexpr double DefaultLookaheadDistance = 0.5;
constexpr double DefaultSuccessRadius = 0.1;
constexpr double DefaultTurnGain = 0.5;
constexpr double DefaultMinObstacleDistance = 0.25;
constexpr double DefaultObstacleTurnLimit = 0.5;
constexpr double DefaultForwardGain = 0.5;
constexpr double DefaultAngularLinearWeight = 0.2;
constexpr double DefaultMaxForwardVel = 0.2;

}  // namespace Constants

PurePursuit::PurePursuit(rclcpp::Node& n,
                         const std::shared_ptr<Mapping::MapManager>& map)
    : BasePathFollower(map), params{}, pose{} {
  this->setDefaultParamValues(n);
  this->paramsCallbackHandle = n.add_on_set_parameters_callback(
      [this](const auto& params) { return updateParams(params); });
  this->updateParams(n.get_parameters(Constants::AllParameters));
}

void PurePursuit::setDefaultParamValues(rclcpp::Node& n) {
  n.declare_parameter(Constants::LookaheadDistanceName,
                      Constants::DefaultLookaheadDistance);
  n.declare_parameter(Constants::SuccessRadiusName,
                      Constants::DefaultSuccessRadius);
  n.declare_parameter(Constants::MinObstacleDistanceName,
                      Constants::DefaultMinObstacleDistance);
  n.declare_parameter(Constants::TurnGainName, Constants::DefaultTurnGain);
  n.declare_parameter(Constants::ObstacleTurnLimitName,
                      Constants::DefaultObstacleTurnLimit);
  n.declare_parameter(Constants::ForwardGainName,
                      Constants::DefaultForwardGain);
  n.declare_parameter(Constants::AngularLinearWeightName,
                      Constants::DefaultAngularLinearWeight);
  n.declare_parameter(Constants::MaxForwardVelName,
                      Constants::DefaultMaxForwardVel);
}

rcl_interfaces::msg::SetParametersResult PurePursuit::updateParams(
    const std::vector<rclcpp::Parameter>& inParams) {
  for (const auto& param : inParams) {
    if (param.get_name() == Constants::LookaheadDistanceName) {
      params.lookaheadDistance = param.as_double();
      continue;
    }
    if (param.get_name() == Constants::SuccessRadiusName) {
      params.successRadius = param.as_double();
      continue;
    }
    if (param.get_name() == Constants::TurnGainName) {
      params.turnGain = param.as_double();
      continue;
    }
    if (param.get_name() == Constants::MinObstacleDistanceName) {
      params.minObstacleDistance = param.as_double();
      continue;
    }
    if (param.get_name() == Constants::ObstacleTurnLimitName) {
      params.obstacleTurnLimit = param.as_double();
      continue;
    }
    if (param.get_name() == Constants::ForwardGainName) {
      params.forwardGain = param.as_double();
      continue;
    }
    if (param.get_name() == Constants::AngularLinearWeightName) {
      params.angularLinearWeight = param.as_double();
      continue;
    }
    if (param.get_name() == Constants::MaxForwardVelName) {
      params.maxForwardVel = param.as_double();
      continue;
    }
  }
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  return res;
}

std::optional<BasePathFollower::Command> PurePursuit::CalculateCommand(
    const owen_common::types::Pose2D& pose) {
  this->pose = pose;
  BasePathFollower::Command ret;
  this->isArrived = false;
  if (this->path.PeekDataRef().empty()) {
    RCLCPP_ERROR(rclcpp::get_logger(Constants::Name),
                 "The path provided to the controller was empty.");
    return {};
  }

  const auto targetPoint = this->getTargetPoint();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("pure_pursuit"),
                     "CurrPt: " << pose << " targetPoint: " << targetPoint);
  const double deltaX = targetPoint.x - this->pose.x;
  const double deltaY = targetPoint.y - this->pose.y;
  const double targetHeading = std::atan2(deltaY, deltaX);
  const double deltaDist = std::sqrt(std::pow(deltaX, 2) + std::pow(deltaY, 2));
  double deltaHeading =
      std::fmod(targetHeading - this->pose.yaw + Constants::Pi,
                Constants::TwoPi) -
      Constants::Pi;

  if (deltaHeading < -Constants::Pi) {
    deltaHeading += Constants::Pi * 2;
  }

  if (deltaDist <= params.successRadius) {
    isArrived = true;
    RCLCPP_INFO_STREAM(rclcpp::get_logger(Constants::Name),
                       "Arrived, delta dist: " << deltaDist << " sucRad: "
                                               << params.successRadius);
    return {ret};
  }
  // if (deltaDist > params.lookaheadDistance + 1.0) {
  //   RCLCPP_ERROR_STREAM(rclcpp::get_logger(Constants::Name),
  //                       "Lookingahead too far");
  //   return ret;
  // }

  const double turnCommand = deltaHeading * params.turnGain;
  double forwardCommand = 0.0;

  const double closestObstacle = this->map->GetMap().GetClosestObstacleDistance(
      owen_common::types::Point2D{this->pose.x, this->pose.y},
      params.minObstacleDistance);

  if (std::abs(deltaHeading) < Constants::HalfPi &&
      (closestObstacle > params.minObstacleDistance ||
       std::abs(deltaHeading) < params.obstacleTurnLimit)) {
    forwardCommand = deltaDist * params.forwardGain -
                     params.angularLinearWeight * deltaHeading;
    if (forwardCommand > params.maxForwardVel) {
      forwardCommand = params.maxForwardVel;
    }
  }

  ret.linear.x = forwardCommand;
  ret.angular.z = turnCommand;

  return ret;
}

owen_common::types::Point2D PurePursuit::getTargetPoint() const {
  const size_t pathLen = this->path.PeekDataRef().size();
  const size_t roverIdx = this->getClosestPointAlongPath();
  double dist = 0;
  for (size_t i = roverIdx + 2; i < pathLen; ++i) {
    const owen_common::types::BaseLine2D<double> lastLineSegment{
        this->path.PeekDataRef()[i - 1], this->path.PeekDataRef()[i]};
    const double lastSegmentDistance = lastLineSegment.GetLength();
    if (dist + lastSegmentDistance >= params.lookaheadDistance) {
      double projectionDistance = params.lookaheadDistance - dist;
      return lastLineSegment.ProjectFromP1(projectionDistance);
    }
    dist += lastSegmentDistance;
  }
  return this->path.PeekDataRef().back();
}

size_t PurePursuit::getClosestPointAlongPath() const {
  const size_t pathLen = this->path.PeekDataRef().size();
  double minDist = std::numeric_limits<double>::max();
  size_t minIdx = 0;
  for (size_t i = 0; i < pathLen; ++i) {
    const double dist = this->path.PeekDataRef()[i].distanceFromPoint(
        {this->pose.x, this->pose.y});
    if (dist < minDist) {
      minIdx = i;
      minDist = dist;
    }
  }
  return minIdx;
}

}  // namespace Navigation::PathFollowers
