//
// Created by kevin on 10/27/22.
//

#ifndef ROOMBA_RVIZ_PLUGINS_TERRAIN_LABELLING_TOOL_HPP
#define ROOMBA_RVIZ_PLUGINS_TERRAIN_LABELLING_TOOL_HPP

#include <QObject>
#include <memory>
#include <string>
#include <utility>
#include <OgreVector.h>
#include <QCursor>  // NOLINT cpplint cannot handle include order here

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

#include "rviz_common/tool.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include "roomba_msgs/msg/multifloor_rectangle.hpp"
#include "roomba_msgs/msg/multifloor_point.hpp"
#include "roomba_msgs/srv/load_config.hpp"

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace rviz_rendering
{
    class Shape;
}  // namespace rviz_rendering

namespace roomba_rviz_plugins{

    class TerrainLabellingTool : public rviz_common::Tool {
        Q_OBJECT

    public:
        TerrainLabellingTool();
        void onInitialize() override;
        void activate() override;
        void deactivate() override;
        int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

    private:
        void onMouseUp(const std::pair<bool, Ogre::Vector3> &xy_plane_intersection);
        int onMouseDown(const std::pair<bool, Ogre::Vector3> &xy_plane_intersection);
        void labellingTypeCallback(std_msgs::msg::Int64 msg);
        void addLabelCallback(std_msgs::msg::String msg);
        void floorCallback(std_msgs::msg::String msg);
        void createObstacleRect(const std::pair<bool, Ogre::Vector3> &xy_plane_intersection);
        void createFeatureRect(const std::pair<bool, Ogre::Vector3> &xy_plane_intersection);
        void createDestinationRect(const std::pair<bool, Ogre::Vector3> &xy_plane_intersection);
        void drawRect(std::pair<std::shared_ptr<rviz_rendering::Shape>, Ogre::Vector3> &pair, const std::pair<bool, Ogre::Vector3> &xy_plane_intersection);
        void clearLabels(std::shared_ptr<std_srvs::srv::Empty::Request> request,
                         std::shared_ptr<std_srvs::srv::Empty::Response> response);
        void loadConfig(std::shared_ptr<roomba_msgs::srv::LoadConfig::Request> request,
                        std::shared_ptr<roomba_msgs::srv::LoadConfig::Response> response);

    private:
        int _currentLabelType;
        bool _lastSaved;
        std::string _currentFloor;
        std::vector<std::pair<std::shared_ptr<rviz_rendering::Shape>, Ogre::Vector3>> _obstacles;
        std::vector<std::pair<std::shared_ptr<rviz_rendering::Shape>, Ogre::Vector3>> _features;
        std::vector<std::pair<std::shared_ptr<rviz_rendering::Shape>, Ogre::Vector3>> _destinations;
        std::shared_ptr<rviz_rendering::ViewportProjectionFinder> _projection_finder;

        // TODO: Make this a service
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr _labelTypeSub;
        // TODO Make this a service
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _labelNameSub;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _floorSub;

        // TODO: Make these service clients (addObs, addFeat, addDest)
        rclcpp::Publisher<roomba_msgs::msg::MultifloorRectangle>::SharedPtr _obsPub;
        rclcpp::Publisher<roomba_msgs::msg::MultifloorRectangle>::SharedPtr _featPub;
        rclcpp::Publisher<roomba_msgs::msg::MultifloorPoint>::SharedPtr _destPub;

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _clearLabelsService;

        rclcpp::Service<roomba_msgs::srv::LoadConfig>::SharedPtr _loadConfigService;

        visualization_msgs::msg::MarkerArray _nameMarkerArray;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _nameMarkerPub;
    };

} // namespace roomba_rviz_plugins



#endif //ROOMBA_RVIZ_PLUGINS_TERRAIN_LABELLING_TOOL_HPP
