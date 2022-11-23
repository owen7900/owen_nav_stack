#pragma once

#include <string>
#include <unordered_map>
#include <vector>

struct Elevator
{
  std::vector<int32_t> doorTagIds;
  std::unordered_map<int32_t, std::string> floorTagIds;
  int32_t backId;
  int32_t rightId;
  int32_t leftId;
  int32_t frontId;
  int32_t doorId;
};

class Configuration
{
public:
  bool load(const std::string& filename);

  const Elevator& GetElevatorFromDoorTag(int32_t id) const;
  bool IsElevatorDoorTag(int32_t id) const;

private:
  std::vector<Elevator> _elevators;
};
