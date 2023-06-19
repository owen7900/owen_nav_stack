#pragma once

#include <chrono>

namespace owen_common {

template <typename T>
class TimeoutData {
 public:
  void SetData(const T &t) {
    data = t;
    updateTime = std::chrono::system_clock::now();
  }
  void SetData(T &&t) {
    data = t;
    updateTime = std::chrono::system_clock::now();
  }
  double GetDataAge() const {
    return std::chrono::duration<double>{std::chrono::system_clock::now() -
                                         updateTime}
        .count();
  }

  T GetData() const { return data; };
  const T &GetDataRef() const { return data; };

 private:
  T data;
  std::chrono::time_point<std::chrono::system_clock> updateTime;
};

};  // namespace owen_common
