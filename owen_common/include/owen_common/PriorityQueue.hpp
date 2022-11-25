#pragma once

#include <algorithm>
#include <vector>
#include <unordered_map>

template <typename T>
class PriorityQueue
{
public:
  inline void update()
  {
    std::sort(queue.begin(), queue.end(),
              [this](const T& a, const T& b) { return this->weights.at(a) > this->weights.at(b); });
  }

  void insert(T id, double weight)
  {
    weights[id] = weight;
    queue.push_back(id);
  }

  void update_weight(const T& id, double weight)
  {
    weights[id] = weight;
  }

  int top()
  {
    return queue.empty() ? -1 : queue.back();
  };

  double get_weight(const T& id)
  {
    return this->weights.at(id);
  }

  void pop_back()
  {
    queue.pop_back();
  }

  bool empty()
  {
    return queue.empty();
  }

private:
  std::unordered_map<T, double> weights;
  std::vector<T> queue;
};
