//
// Created by kevin on 10/27/22.
//

#ifndef ROOMBA_RVIZ_PLUGINS_TERRAIN_LABELLING_TOOL_HPP
#define ROOMBA_RVIZ_PLUGINS_TERRAIN_LABELLING_TOOL_HPP

#include <memory>
#include <string>
#include <utility>

#include <OgreVector.h>

#include <QCursor>  // NOLINT cpplint cannot handle include order here

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "rviz_common/tool.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include "roomba_msgs/srv/switch_labelling_type.hpp"
#include "roomba_msgs/srv/add_label.hpp"
#include "roomba_rviz_plugins/constants.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <std_srvs/srv/empty.hpp>

namespace rviz_rendering
{
    class Shape;
}  // namespace rviz_rendering

namespace roomba_rviz_plugins{

    class TerrainLabellingTool : public rviz_common::Tool {
        Q_OBJECT

    public:
        TerrainLabellingTool();
        ~TerrainLabellingTool() override;
        void onInitialize() override;
        void activate() override;
        void deactivate() override;
        int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

    private:
        rclcpp::Service<roomba_msgs::srv::SwitchLabellingType>::SharedPtr _switchLabelTypeService;
        rclcpp::Service<roomba_msgs::srv::AddLabel>::SharedPtr _addLabelService;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _clearLabelsService;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _nameMarkerPub;
        visualization_msgs::msg::MarkerArray  _nameMarkerArray;

        std::vector<std::pair<std::shared_ptr<rviz_rendering::Shape>, Ogre::Vector3>> _obstacles;
        std::vector<std::pair<std::shared_ptr<rviz_rendering::Shape>, Ogre::Vector3>> _features;
        std::vector<std::shared_ptr<rviz_rendering::Shape>> _destinations;

        int _currentLabellingType;
        int _currentFloor;
        bool _lastSaved;
        std::shared_ptr<rviz_rendering::ViewportProjectionFinder> _projectionFinder;
        Ogre::Vector3 _middleDownPos;

    private:
        void changeLabelType(std::shared_ptr<roomba_msgs::srv::SwitchLabellingType::Request> request,
                             std::shared_ptr<roomba_msgs::srv::SwitchLabellingType::Response> response);
        void addLabel(std::shared_ptr<roomba_msgs::srv::AddLabel::Request> request,
                             std::shared_ptr<roomba_msgs::srv::AddLabel::Response> response);
        void clearLabels(std::shared_ptr<std_srvs::srv::Empty::Request> request,
                      std::shared_ptr<std_srvs::srv::Empty::Response> response);
        int processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
        int processMouseMovedRightDown(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
        int processMouseLeftButtonReleased(Ogre::Vector3 xy_plane_intersection);
        int processMouseMiddleDown(Ogre::Vector3 xy_plane_intersection);
        int processMouseMiddleReleased(const Ogre::Vector3 xy_plane_intersection);
        double calculateAngle(Ogre::Vector3 start_point, Ogre::Vector3 end_point);
        void createObstacle(Ogre::Vector3 position);
        void createFeature(Ogre::Vector3 position);
        void createDestination(Ogre::Vector3 position);
        void editPosAndSize(std::pair<std::shared_ptr<rviz_rendering::Shape>, Ogre::Vector3> &pair, Ogre::Vector3 xy_plane_intersection);
        void moveRect(std::pair<std::shared_ptr<rviz_rendering::Shape>, Ogre::Vector3> &pair, Ogre::Vector3 xy_plane_intersection);
        Ogre::Vector3 rotatePoint(Ogre::Vector3 originalPoint, Ogre::Vector3 origin, double angle);
    };

} // namespace roomba_rviz_plugins



#endif //ROOMBA_RVIZ_PLUGINS_TERRAIN_LABELLING_TOOL_HPP
