#include "elevator_traverser/Configuration.hpp"

#include <yaml-cpp/yaml.h>
#include <iostream>

bool Configuration::load(const std::string& filename)
{
  try
  {
    YAML::Node top = YAML::LoadFile(filename);
  }
  catch (YAML::Exception& e)
  {
    std::cout << "YAML PARSE ERROR " << e.what() << std::endl;
    return false;
  }

  return true;
}

const Elevator& Configuration::get_elevator_from_door_tag(int32_t tag_id) const
{
  for (const auto& elevator : this->elevators)
  {
    for (const auto& id : elevator.door_tag_ids)
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

bool Configuration::is_elevator_door_tag(int32_t tag_id) const
{
  for (const auto& elevator : this->elevators)
  {
    for (const auto& id : elevator.door_tag_ids)
    {
      if (id == tag_id)
      {
        return true;
      }
    }
  }
  return false;
}
