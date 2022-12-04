//
// Created by kevin on 11/26/22.
//

#include "roomba_rviz_plugins/map_node_labelling_tool.hpp"

#include <memory>
#include <utility>
#include <iostream>

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include <yaml-cpp/yaml.h>

#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/display_context.hpp"

namespace roomba_rviz_plugins {
    MapNodeLabellingTool::MapNodeLabellingTool()
            : rviz_common::Tool(), _lastSaved(true), _currentFloor("1"), _markerCount(0) {
        shortcut_key_ = 'n';
        _projection_finder = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
    }

    void MapNodeLabellingTool::onInitialize() {
        rviz_common::Tool::onInitialize();
        setName("Map Node Labelling");

        auto raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();

        _floorSub = raw_node->create_subscription<std_msgs::msg::String>("/floor_id",
                                                                         10,
                                                                         std::bind(
                                                                                 &MapNodeLabellingTool::floorCallback,
                                                                                 this, std::placeholders::_1));


        _nameMarkerPub = raw_node->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array",
                                                                                          10);
        _connectionsMarkerPub = raw_node->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/visualization_marker_array", 10);


        _clearNodesService = raw_node->create_service<std_srvs::srv::Empty>("clearNodes",
                                                                            std::bind(&MapNodeLabellingTool::clearNodes,
                                                                                      this,
                                                                                      std::placeholders::_1,
                                                                                      std::placeholders::_2));

        _loadNodesService = raw_node->create_service<roomba_msgs::srv::LoadConfig>("loadNodes", std::bind(
                &MapNodeLabellingTool::loadNodes,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        _addMapNodeService = raw_node->create_service<roomba_msgs::srv::AddMapNode>("addMapNode", std::bind(
                &MapNodeLabellingTool::addMapNode,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
    }

    void MapNodeLabellingTool::activate() {}

    void MapNodeLabellingTool::deactivate() {}

    int MapNodeLabellingTool::processMouseEvent(rviz_common::ViewportMouseEvent &event) {
        const auto point_projection_on_xy_plane = _projection_finder->getViewportPointProjectionOnXYPlane(
                event.panel->getRenderWindow(), event.x, event.y);

        if (event.leftDown()) {
            return onMouseDown(point_projection_on_xy_plane);
        }
        return 0;
    }

    int MapNodeLabellingTool::onMouseDown(const std::pair<bool, Ogre::Vector3> &xy_plane_intersection) {
        int flags = 0;
        flags |= Render;
        if (!_lastSaved) {
            if (_mapNodes.size() > 0) {
                _mapNodes.pop_back();
                _lastSaved = true;
            }
        }

        _mapNodes.push_back(createCircle(xy_plane_intersection.second.x, xy_plane_intersection.second.y));
        _lastSaved = false;

        return flags;
    }

    void MapNodeLabellingTool::floorCallback(std_msgs::msg::String msg) {
        _currentFloor = msg.data;
    }


    void MapNodeLabellingTool::clearNodes(std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                          std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        for (auto &node: _mapNodes) {
            node->getRootNode()->setVisible(false);
        }
        _mapNodes.clear();

        for (auto &marker: _nameMarkerArray.markers) {
            marker.action = visualization_msgs::msg::Marker::DELETE;
        }
        _nameMarkerPub->publish(_nameMarkerArray);

        for (auto &line: _connectionsMarkerArray.markers) {
            line.action = visualization_msgs::msg::Marker::DELETE;
        }
        _connectionsMarkerPub->publish(_connectionsMarkerArray);
        _markerCount = 0;
    }


    void MapNodeLabellingTool::loadNodes(std::shared_ptr<roomba_msgs::srv::LoadConfig::Request> request,
                                         std::shared_ptr<roomba_msgs::srv::LoadConfig::Response> response) {
        auto configPath = request->path.data;
        YAML::Node top = YAML::LoadFile(configPath);

        for (const auto &node: top) {
            auto floorId = node.second["floor_id"].as<std::string>();
            if (floorId != _currentFloor) {
                _mapNodes.push_back(nullptr);
                continue;
            }

            auto x = node.second["x"].as<double>();
            auto y = node.second["y"].as<double>();
            auto circle = createCircle(x, y);
            _mapNodes.push_back(circle);

            visualization_msgs::msg::Marker marker_msg;
            marker_msg.header.frame_id = "/map";
            marker_msg.id = ++_markerCount;
            marker_msg.text = node.first.as<std::string>();

            marker_msg.color.a = 1.0f;
            marker_msg.color.r = 1.0f;
            marker_msg.color.g = 1.0f;
            marker_msg.color.b = 1.0f;
            marker_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker_msg.pose.position.x = circle->getPosition().x;
            marker_msg.pose.position.y = circle->getPosition().y;
            marker_msg.pose.position.z = 0.2;
            marker_msg.scale.z = 0.2;
            _nameMarkerArray.markers.push_back(marker_msg);
        }



        for (const auto &node: top) {
            auto connections = node.second["connections"].as<std::vector<YAML::Node>>();
            std::string currentId = node.first.as<std::string>();
            for (const auto &connection: connections) {
                std::string conn_id = connection["id"].as<std::string>();
                if (conn_id == "") {
                    continue;
                }

                std::string current = currentId;
                std::string conn = conn_id;

                current.erase(0, 1);
                conn.erase(0, 1);

                int currentIdInt = std::stoi(current);
                int connIdInt = std::stoi(conn);

                auto currentNode = _mapNodes[currentIdInt - 1];
                if (currentNode == nullptr) {
                    continue;
                }
                auto otherNode = _mapNodes[connIdInt- 1];
                if (otherNode == nullptr) {
                    continue;
                }

                visualization_msgs::msg::Marker marker_msg;
                marker_msg.header.frame_id = "/map";
                marker_msg.id = ++_markerCount;
                marker_msg.color.a = 1.0f;
                marker_msg.color.r = 0.5f;
                marker_msg.color.g = 0.0f;
                marker_msg.color.b = 0.5f;
                marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;

                geometry_msgs::msg::Point otherNodePoint;
                otherNodePoint.x = otherNode->getPosition().x;
                otherNodePoint.y = otherNode->getPosition().y;
                marker_msg.points.push_back(otherNodePoint);

                geometry_msgs::msg::Point currentNodePoint;
                currentNodePoint.x = currentNode->getPosition().x;
                currentNodePoint.y = currentNode->getPosition().y;
                marker_msg.points.push_back(currentNodePoint);

                marker_msg.scale.x = 0.05;
                _connectionsMarkerArray.markers.push_back(marker_msg);

                auto cost = connection["cost"];
                if (cost.as<std::string>() == "") {
                    continue;
                }
                // add a cost marker
            }

            _connectionsMarkerPub->publish(_connectionsMarkerArray);
            _nameMarkerPub->publish(_nameMarkerArray);
            _lastSaved = true;
        }
    }

        void MapNodeLabellingTool::addMapNode(std::shared_ptr<roomba_msgs::srv::AddMapNode::Request> request,
                                              std::shared_ptr<roomba_msgs::srv::AddMapNode::Response> response) {
            auto lastNode = _mapNodes.back();
            response->node_pos.point.x = lastNode->getPosition().x;
            response->node_pos.point.y = lastNode->getPosition().y;
            response->node_pos.floor_id.data = _currentFloor;
            response->node_pos.label.data = _currentFloor + request->node_name.data;
            _lastSaved = true;

            visualization_msgs::msg::Marker marker_msg;
            marker_msg.header.frame_id = "/map";
            marker_msg.id = ++_markerCount;
            marker_msg.text = response->node_pos.label.data;
            marker_msg.color.a = 1.0f;
            marker_msg.color.r = 1.0f;
            marker_msg.color.g = 1.0f;
            marker_msg.color.b = 1.0f;
            marker_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker_msg.pose.position.x = lastNode->getPosition().x;
            marker_msg.pose.position.y = lastNode->getPosition().y;
            marker_msg.pose.position.z = 0.2;
            marker_msg.scale.z = 0.2;
            _nameMarkerArray.markers.push_back(marker_msg);
            _nameMarkerPub->publish(_nameMarkerArray);

            if (_mapNodes.size() > 1) {
                visualization_msgs::msg::Marker marker_msg;
                marker_msg.header.frame_id = "/map";
                marker_msg.id = ++_markerCount;
                marker_msg.color.a = 1.0f;
                marker_msg.color.r = 0.5f;
                marker_msg.color.g = 0.0f;
                marker_msg.color.b = 0.5f;
                marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;

                marker_msg.points.push_back(response->node_pos.point);
                auto nodeBefore = _mapNodes.end()[-2];
                geometry_msgs::msg::Point pointBefore;
                pointBefore.x = nodeBefore->getPosition().x;
                pointBefore.y = nodeBefore->getPosition().y;
                marker_msg.points.push_back(pointBefore);

                marker_msg.scale.x = 0.05;
                _connectionsMarkerArray.markers.push_back(marker_msg);
                _connectionsMarkerPub->publish(_connectionsMarkerArray);
            }
        }

        std::shared_ptr<rviz_rendering::Shape> MapNodeLabellingTool::createCircle(double x, double y) {
            auto circle = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Sphere, scene_manager_,
                                                                  nullptr);
            circle->setColor(0.5f, 0.0f, 0.5f, 1.0f);
            circle->setPosition(Ogre::Vector3(x, y, 0));
            circle->setScale(Ogre::Vector3(0.2, 0.2, 0.0001));
            circle->getRootNode()->setVisible(true);
            return circle;
        }
    }


#include <pluginlib/class_list_macros.hpp>  // NOLINT
    PLUGINLIB_EXPORT_CLASS(roomba_rviz_plugins::MapNodeLabellingTool, rviz_common::Tool)
