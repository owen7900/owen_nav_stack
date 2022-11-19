#pragma once

#include <string>
#include <unordered_map>
#include <vector>

struct Elevator
{
  std::vector<int32_t> door_tag_ids;
  std::unordered_map<int32_t, std::string> floor_tag_ids;
  int32_t back_id;
  int32_t right_id;
  int32_t left_id;
  int32_t front_id;
  int32_t door_id;
};

class Configuration
{
public:
  bool load(const std::string& filename);

  const Elevator& get_elevator_from_door_tag(int32_t id) const;
  bool is_elevator_door_tag(int32_t id) const;

private:
  std::vector<Elevator> elevators;
};
