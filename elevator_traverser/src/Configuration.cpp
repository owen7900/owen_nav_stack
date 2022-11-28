#include "elevator_traverser/Configuration.hpp"

#include <yaml-cpp/yaml.h>
#include <iostream>

bool Configuration::load(const std::string& filename)
{
  try
  {
    YAML::Node top = YAML::LoadFile(filename);

    for (const auto& e : top)
    {
      Elevator el;
      for (const auto& id : e.second["door_tag_ids"])
      {
        std::cout << "got a door tag" << id.as<int32_t>() << std::endl;
        el.doorTagIds.push_back(id.as<int32_t>());
      }

      for (const auto& id : e.second["floor_tag_ids"])
      {
        el.floorTagIds[id.first.as<int32_t>()] = id.second.as<std::string>();
      }

      el.backId = e.second["back_id"].as<int32_t>();
      el.rightId = e.second["right_id"].as<int32_t>();
      el.leftId = e.second["left_id"].as<int32_t>();
      el.frontId = e.second["front_id"].as<int32_t>();
      el.doorId = e.second["door_id"].as<int32_t>();
      this->_elevators.emplace_back(std::move(el));
    }
  }
  catch (YAML::Exception& e)
  {
    std::cout << "YAML PARSE ERROR " << e.what() << std::endl;
    return false;
  }

  std::cout << "Parsed " << this->_elevators.size() << std::endl;
  return true;
}

const Elevator& Configuration::GetElevatorFromDoorTag(int32_t tag_id) const
{
  for (const auto& elevator : this->_elevators)
  {
    for (const auto& id : elevator.doorTagIds)
    {
      if (id == tag_id)
      {
        return elevator;
      }
    }
  }

  std::cout << "Unknown tag id " << tag_id << std::endl;
  throw -1;
}

bool Configuration::IsElevatorDoorTag(int32_t tag_id) const
{
  for (const auto& elevator : this->_elevators)
  {
    std::cout << "Elevator doo" << std::endl;
    for (const auto& id : elevator.doorTagIds)
    {
      std::cout << "checking " << id << " want " << tag_id << std::endl;
      if (id == tag_id)
      {
        return true;
      }
    }
  }
  return false;
}
