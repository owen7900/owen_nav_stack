//
// Created by kevin on 11/26/22.
//

#ifndef CAPSTON_WS_MAP_NODE_LABELLING_TOOL_HPP
#define CAPSTON_WS_MAP_NODE_LABELLING_TOOL_HPP

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

#include "roomba_msgs/msg/multifloor_point.hpp"
#include "roomba_msgs/srv/load_config.hpp"
#include "roomba_msgs/srv/add_map_node.hpp"

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"


namespace rviz_rendering
{
    class Shape;
}  // namespace rviz_rendering

namespace roomba_rviz_plugins{

    class MapNodeLabellingTool : public rviz_common::Tool {
        Q_OBJECT

    public:
        MapNodeLabellingTool();
        void onInitialize() override;
        void activate() override;
        void deactivate() override;
        int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

    private:
        int onMouseDown(const std::pair<bool, Ogre::Vector3> &xy_plane_intersection);
        void floorCallback(std_msgs::msg::String msg);
        void addMapNode(std::shared_ptr<roomba_msgs::srv::AddMapNode::Request> request,
                        std::shared_ptr<roomba_msgs::srv::AddMapNode::Response> response);
        void clearNodes(std::shared_ptr<std_srvs::srv::Empty::Request> request,
                         std::shared_ptr<std_srvs::srv::Empty::Response> response);
        void loadNodes(std::shared_ptr<roomba_msgs::srv::LoadConfig::Request> request,
                        std::shared_ptr<roomba_msgs::srv::LoadConfig::Response> response);
        std::shared_ptr<rviz_rendering::Shape> createCircle(double x, double y);

    private:
        bool _lastSaved;
        std::string _currentFloor;
        int _markerCount;
        std::vector<std::shared_ptr<rviz_rendering::Shape>> _mapNodes;
        std::vector<std::shared_ptr<rviz_rendering::Shape>> _connections;
        std::shared_ptr<rviz_rendering::ViewportProjectionFinder> _projection_finder;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _floorSub;
        rclcpp::Service<roomba_msgs::srv::AddMapNode>::SharedPtr _addMapNodeService;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _clearNodesService;
        rclcpp::Service<roomba_msgs::srv::LoadConfig>::SharedPtr _loadNodesService;

        visualization_msgs::msg::MarkerArray _nameMarkerArray;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _nameMarkerPub;
        visualization_msgs::msg::MarkerArray _connectionsMarkerArray;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _connectionsMarkerPub;
    };

} // namespace roomba_rviz_plugins


#endif //CAPSTON_WS_MAP_NODE_LABELLING_TOOL_HPP
