#include "owen_localization/UKF.hpp"

namespace Localization::Filters {

void UKF::Predict(const TimePoint &t) { const auto dt = getDt(t); }
void UKF::Update(const TimePoint &t, const OdomUpdate &update) {}
void UKF::Update(const TimePoint &t, const PositionUpdate &update) {}
void UKF::Update(const TimePoint &t, const VelocityUpdate &update) {}

UKF::Pose UKF::GetPose() const { return Pose{x[0], x[1], x[2]}; }
UKF::StateType UKF::GetState() const { return x; }

double UKF::getDt(const TimePoint &t) const { return (t - lastTime).seconds(); }

}  // namespace Localization::Filters
