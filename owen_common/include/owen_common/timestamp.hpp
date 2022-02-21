#ifndef OWEN_COMMON__TIMESTAMP_HPP_
#define OWEN_COMMON__TIMESTAMP_HPP_

#include "owen_common/visibility_control.h"
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>

namespace owen_common
{
template <typename T>
class Timestamp
{
public:
  Timestamp() : _timeout(rclcpp::Duration::from_seconds(0.0)){};

  inline void SetTimeout(rclcpp::Duration timeout)
  {
    this->_timeout = timeout;
  };
  inline void SetData(T data)
  {
    this->_data = data;
    this->_lastTime = rclcpp::Clock().now();
  };
  inline bool IsDataTimeout() const
  {
    return (rclcpp::Clock().now() - this->_lastTime) > this->_timeout;
  };
  inline T GetData() const
  {
    return this->_data;
  };
  inline void Reset()
  {
    this->_lastTime = rclcpp::Time(0.0);
  }

private:
  T _data;
  rclcpp::Duration _timeout;
  rclcpp::Time _lastTime;
};

}  // namespace owen_common

#endif  // OWEN_COMMON__OWEN_COMMON_HPP_
