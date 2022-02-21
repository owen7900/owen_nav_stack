#include "expand_to_goal_costmap_plugin/expand_to_goal_costmap_plugin.hpp"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <rclcpp/logging.hpp>

namespace expand_to_goal_costmap_plugin
{
using std::placeholders::_1;

ExpandToGoal::ExpandToGoal()
{
}

void
ExpandToGoal::onInitialize()
{
global_frame_ = layered_costmap_->getGlobalFrameID();

  getParameters();

  rclcpp::QoS map_qos(10);  // initialize to default
  if (map_subscribe_transient_local_) {
    map_qos.transient_local();
    map_qos.reliable();
    map_qos.keep_last(1);
  }

  RCLCPP_INFO(
    logger_,
    "Subscribing to the map topic (%s) with %s durability",
    map_topic_.c_str(),
    map_subscribe_transient_local_ ? "transient local" : "volatile");

    auto node = node_.lock();
    if(!node)
    {
        return;
    }

    map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, map_qos,
    std::bind(&ExpandToGoal::incomingMap, this, std::placeholders::_1));

  if (subscribe_to_updates_) {
    RCLCPP_INFO(logger_, "Subscribing to updates");
    map_update_sub_ = node->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
      map_topic_ + "_updates",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&ExpandToGoal::incomingUpdate, this, std::placeholders::_1));
  }
    this->goalSubscriber = node->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&ExpandToGoal::goalCallback, this, _1));
    this->hasData = false;
    this->current_ = true;
}

void
ExpandToGoal::activate()
{
}

void
ExpandToGoal::deactivate()
{
}

void
ExpandToGoal::reset()
{
  has_updated_data_ = true;
  current_ = false;
  hasData = false;
}

void
ExpandToGoal::getParameters()
{
  int temp_lethal_threshold = 0;
  double temp_tf_tol = 0.0;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("subscribe_to_updates", rclcpp::ParameterValue(false));
  declareParameter("map_subscribe_transient_local", rclcpp::ParameterValue(true));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.0));
  declareParameter("map_topic", rclcpp::ParameterValue(""));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "subscribe_to_updates", subscribe_to_updates_);
  std::string private_map_topic, global_map_topic;
  node->get_parameter(name_ + "." + "map_topic", private_map_topic);
  node->get_parameter("map_topic", global_map_topic);
  if (!private_map_topic.empty()) {
    map_topic_ = private_map_topic;
  } else {
    map_topic_ = global_map_topic;
  }
  node->get_parameter(
    name_ + "." + "map_subscribe_transient_local",
    map_subscribe_transient_local_);
  node->get_parameter("track_unknown_space", track_unknown_space_);
  node->get_parameter("use_maximum", use_maximum_);
  node->get_parameter("lethal_cost_threshold", temp_lethal_threshold);
  node->get_parameter("unknown_cost_value", unknown_cost_value_);
  node->get_parameter("trinary_costmap", trinary_costmap_);
  node->get_parameter("transform_tolerance", temp_tf_tol);

  // Enforce bounds
  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  map_received_ = false;
  update_in_progress_.store(false);

  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);
}

void
ExpandToGoal::processMap(const nav_msgs::msg::OccupancyGrid & new_map)
{
  RCLCPP_DEBUG(logger_, "ExpandToGoal: Process map");

  unsigned int size_x = new_map.info.width;
  unsigned int size_y = new_map.info.height;

  RCLCPP_DEBUG(
    logger_,
    "ExpandToGoal: Received a %d X %d map at %f m/pix", size_x, size_y,
    new_map.info.resolution);

  // resize costmap if size, resolution or origin do not match
  Costmap2D * master = layered_costmap_->getCostmap();
  /*if (!layered_costmap_->isRolling() && (master->getSizeInCellsX() != size_x ||
    master->getSizeInCellsY() != size_y ||
    master->getResolution() != new_map.info.resolution ||
    master->getOriginX() != new_map.info.origin.position.x ||
    master->getOriginY() != new_map.info.origin.position.y ||
    !layered_costmap_->isSizeLocked()))
  {
    // Update the size of the layered costmap (and all layers, including this one)
    RCLCPP_INFO(
      logger_,
      "ExpandToGoal: Resizing costmap to %d X %d at %f m/pix", size_x, size_y,
      new_map.info.resolution);
    layered_costmap_->resizeMap(
      std::max(size_x,newSizeX), std::max(size_y,newSizeY), new_map.info.resolution,
      std::min(new_map.info.origin.position.x,newOriginX),
      std::min(new_map.info.origin.position.y,newOriginY),
      true);
  } else*/ if (size_x_ != size_x || size_y_ != size_y ||  // NOLINT
    resolution_ != new_map.info.resolution ||
    origin_x_ != new_map.info.origin.position.x ||
    origin_y_ != new_map.info.origin.position.y)
  {
    // only update the size of the costmap stored locally in this layer
    RCLCPP_INFO(
      logger_,
      "ExpandToGoal: Resizing static layer to %d X %d at %f m/pix", size_x, size_y,
      new_map.info.resolution);
    resizeMapWithGoal(
      size_x, size_y, new_map.info.resolution,
      new_map.info.origin.position.x, new_map.info.origin.position.y);
  }

  unsigned int newMapIndex = 0;
  unsigned int costMapIndex = 0;

    unsigned int index = 0;
    const auto currentSizeX = master->getSizeInCellsX();
    const auto currentSizeY = master->getSizeInCellsY();
    unsigned int originMx, originMy;
    master->worldToMap(new_map.info.origin.orientation.x, new_map.info.origin.position.y, originMx, originMy);

  // we have a new map, update full size of map
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  // initialize the costmap with static data
 /* for (unsigned int i = originMy; i < size_y + originMy; ++i) {
    costMapIndex = i * currentSizeY;
    newMapIndex = (i - originMy) * size_x;
    for (unsigned int j = originMx; j < size_x + originMx; ++j) {
      unsigned char value = new_map.data[newMapIndex];
      costmap_[costMapIndex] = interpretValue(value);
      ++costMapIndex;
      ++newMapIndex;
    }
  }*/
  for (unsigned int i = 0; i < size_y; ++i) {
    for (unsigned int j = 0; j < size_x ; ++j) {
      unsigned char value = new_map.data[index];
      costmap_[index] = interpretValue(value);
        ++index;
    }
  }
  map_frame_ = new_map.header.frame_id;

  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  has_updated_data_ = true;

  current_ = true;
}

void
ExpandToGoal::resizeMapWithGoal(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y)
{
    resizeMap(size_x, size_y, resolution, origin_x, origin_y);
/*    resizeMap(std::max(size_x, static_cast<unsigned int>(this->newSizeX)),
              std::max(size_y, static_cast<unsigned int>(this->newSizeY)),
              resolution,
              std::min(origin_x, this->newOriginX),
              std::min(origin_y, this->newOriginY));*/
}

void
ExpandToGoal::matchSize()
{
  // If we are using rolling costmap, the static map size is
  //   unrelated to the size of the layered costmap
    //   
/*
  if (!layered_costmap_->isRolling()) {
    Costmap2D * master = layered_costmap_->getCostmap();
    resizeMapWithGoal(
      master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
      master->getOriginX(), master->getOriginY());
  }
  */
}

unsigned char
ExpandToGoal::interpretValue(unsigned char value)
{
  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_) {
    return NO_INFORMATION;
  } else if (!track_unknown_space_ && value == unknown_cost_value_) {
    return FREE_SPACE;
  } else if (value >= lethal_threshold_) {
    return LETHAL_OBSTACLE;
  } else if (trinary_costmap_) {
    return FREE_SPACE;
  }

  double scale = static_cast<double>(value) / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void
ExpandToGoal::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!map_received_) {
    map_received_ = true;
    processMap(*new_map);
  }
  if (update_in_progress_.load()) {
    map_buffer_ = new_map;
  } else {
    processMap(*new_map);
    map_buffer_ = nullptr;
  }
}

void
ExpandToGoal::incomingUpdate(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (update->y < static_cast<int32_t>(y_) ||
    y_ + height_ < update->y + update->height ||
    update->x < static_cast<int32_t>(x_) ||
    x_ + width_ < update->x + update->width)
  {
    RCLCPP_WARN(
      logger_,
      "ExpandToGoal: Map update ignored. Exceeds bounds of static layer.\n"
      "Static layer origin: %d, %d   bounds: %d X %d\n"
      "Update origin: %d, %d   bounds: %d X %d",
      x_, y_, width_, height_, update->x, update->y, update->width,
      update->height);
    return;
  }

  if (update->header.frame_id != map_frame_) {
    RCLCPP_WARN(
      logger_,
      "ExpandToGoal: Map update ignored. Current map is in frame %s "
      "but update was in frame %s",
      map_frame_.c_str(), update->header.frame_id.c_str());
  }

  unsigned int di = 0;
  for (unsigned int y = 0; y < update->height; y++) {
    unsigned int index_base = (update->y + y) * size_x_;
    for (unsigned int x = 0; x < update->width; x++) {
      unsigned int index = index_base + x + update->x;
      costmap_[index] = interpretValue(update->data[di++]);
    }
  }

  x_ = update->x;
  y_ = update->y;
  width_ = update->width;
  height_ = update->height;
  has_updated_data_ = true;
}

void
ExpandToGoal::goalCallback(geometry_msgs::msg::PoseStamped::UniquePtr msg)
{
    if(msg)
    {
        this->x = msg->pose.position.x +((msg->pose.position.x > 0) ? 5 : -5);
        this->y = msg->pose.position.y +((msg->pose.position.y > 0) ? 5 : -5);
        this->hasData = true;

        int mx, my;
        const auto costMap = this->layered_costmap_->getCostmap();
        costMap->worldToMapNoBounds(this->x, this->y, mx, my);
        RCLCPP_INFO_STREAM(this->logger_, "ExpandToGoal got mxy" << mx << ", " << my );
        if(mx > 0 && my > 0)
        {
            newSizeX = std::max(mx, static_cast<int>(costMap->getSizeInCellsX()));
            newSizeY = std::max(my, static_cast<int>(costMap->getSizeInCellsY()));
            newOriginX = costMap->getOriginX();
            newOriginY = costMap->getOriginY();
            RCLCPP_INFO_STREAM(this->logger_, "ExpandToGoal resizing map to " << mx << ", " << my );
        }
        else
        {
            newSizeX = costMap->getSizeInCellsX();
            newSizeY = costMap->getSizeInCellsY();
            newOriginX = costMap->getOriginX();
            newOriginY = costMap->getOriginY();
            if(mx > newSizeX)
            {
                newSizeX = mx;
            }
            else if(mx < 0)
            {
                newOriginX = this->x;
                newSizeX += std::abs(mx);
            }

            if(my > newSizeY)
            {
                newSizeY = my;
            }
            else if(my < 0)
            {
                newOriginY = this->y;
                newSizeY += std::abs(my);
            }
            RCLCPP_INFO_STREAM(this->logger_, "ExpandToGoal resizing map to " << newSizeX << ", " << newSizeY << " with origin " << newOriginX << ", " << newOriginY);
        }
//        resizeMapWithGoal(newSizeX, newSizeY, costMap->getResolution(), newOriginX, newOriginY);
        layered_costmap_->resizeMap(newSizeX, newSizeY, costMap->getResolution(), newOriginX, newOriginY);

    }
}
void 
ExpandToGoal::updateBounds(

    double /*robot_x*/, double/* robot_y*/, double /*robot_yaw*/,  double * min_x,
    double *min_y,
    double *max_x,
    double *max_y)
{
if (!map_received_ && !hasData) {
    return;
  }

  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  update_in_progress_.store(true);

  // If there is a new available map, load it.
  if (map_buffer_) {
    processMap(*map_buffer_);
    map_buffer_ = nullptr;
  }
/*
  if (!layered_costmap_->isRolling() ) {
    if (!(has_updated_data_ || has_extra_bounds_)) {
      return;
    }
  }
*/
  useExtraBounds(min_x, min_y, max_x, max_y);

  double wx, wy;

  mapToWorld(x_, y_, wx, wy);
  *min_x = std::min(std::min(wx, *min_x), x);
  *min_y = std::min(std::min(wy, *min_y), y);

  mapToWorld(x_ + width_, y_ + height_, wx, wy);
  *max_x = std::max(std::max(wx, *max_x), x);
  *max_y = std::max(std::max(wy, *max_y), y);

  has_updated_data_ = false;
}

void
ExpandToGoal::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    update_in_progress_.store(false);
    return;
  }
  if (!map_received_) {
    static int count = 0;
    // throttle warning down to only 1/10 message rate
    if (++count == 10) {
      RCLCPP_WARN(logger_, "Can't update static costmap layer, no map received");
      count = 0;
    }
    update_in_progress_.store(false);
    return;
  }

  /*if (!layered_costmap_->isRolling()) {
    // if not rolling, the layered costmap (master_grid) has same coordinates as this layer
    if (!use_maximum_) {
      updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
    } else {
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    }
  } else*/ {
    // If rolling window, the master_grid is unlikely to have same coordinates as this layer
    unsigned int mx, my;
    double wx, wy;
    // Might even be in a different frame
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(
        map_frame_, global_frame_, tf2::TimePointZero,
        transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(logger_, "ExpandToGoal: %s", ex.what());
      update_in_progress_.store(false);
      return;
    }
    // Copy map data given proper transformations
    tf2::Transform tf2_transform;
    tf2::fromMsg(transform.transform, tf2_transform);

    for (int i = min_i; i < max_i; ++i) {
      for (int j = min_j; j < max_j; ++j) {
        // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
        // Transform from global_frame_ to map_frame_
        tf2::Vector3 p(wx, wy, 0);
        p = tf2_transform * p;
        // Set master_grid with cell from map
        if (worldToMap(p.x(), p.y(), mx, my)) {
          if (!use_maximum_) {
            master_grid.setCost(i, j, getCost(mx, my));
          } else {
            master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
          }
        }
      }
    }
  }
  update_in_progress_.store(false);
  current_ = true;
}

}  

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(expand_to_goal_costmap_plugin::ExpandToGoal,  nav2_costmap_2d::Layer)
