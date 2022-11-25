//
// Created by kevin on 10/27/22.
//

#include "roomba_rviz_plugins/terrain_labelling_tool.hpp"

#include <memory>
#include <utility>
#include <iostream>

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/display_context.hpp"

#include "roomba_rviz_plugins/constants.hpp"


namespace roomba_rviz_plugins {
    TerrainLabellingTool::TerrainLabellingTool() : rviz_common::Tool(), _currentLabelType(0), _lastSaved(true), _currentFloor("1") {
        shortcut_key_ = 'l';
        _projection_finder = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
    }

    void TerrainLabellingTool::onInitialize() {
        rviz_common::Tool::onInitialize();
        setName("Terrain Labelling");

        auto raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();

        _labelTypeSub = raw_node->create_subscription<std_msgs::msg::Int64>("/labelType",
                                                                            10,
                                                                            std::bind(
                                                                                    &TerrainLabellingTool::labellingTypeCallback,
                                                                                    this, std::placeholders::_1));

        _labelNameSub = raw_node->create_subscription<std_msgs::msg::String>("/labelName",
                                                                             10,
                                                                             std::bind(
                                                                                     &TerrainLabellingTool::addLabelCallback,
                                                                                     this, std::placeholders::_1));

        _floorSub = raw_node->create_subscription<std_msgs::msg::String>("/floor_id",
                                                                             10,
                                                                             std::bind(
                                                                                     &TerrainLabellingTool::floorCallback,
                                                                                     this, std::placeholders::_1));

        _obsPub = raw_node->create_publisher<roomba_msgs::msg::MultifloorRectangle>("/obstacles", 10);
        _featPub = raw_node->create_publisher<roomba_msgs::msg::MultifloorRectangle>("/features", 10);
        _destPub = raw_node->create_publisher<roomba_msgs::msg::MultifloorPoint>("/destinations", 10);

        _nameMarkerPub = raw_node->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 10);


        _clearLabelsService = raw_node->create_service<std_srvs::srv::Empty>("clearLabels", std::bind(&TerrainLabellingTool::clearLabels,
                                                                                                      this,
                                                                                                      std::placeholders::_1,
                                                                                                      std::placeholders::_2));
    }

    void TerrainLabellingTool::activate() {}

    void TerrainLabellingTool::deactivate() {}

    int TerrainLabellingTool::processMouseEvent(rviz_common::ViewportMouseEvent &event) {
        const auto point_projection_on_xy_plane = _projection_finder->getViewportPointProjectionOnXYPlane(
                event.panel->getRenderWindow(), event.x, event.y);

        if (event.leftDown()) {
            return onMouseDown(point_projection_on_xy_plane);
        } else if (event.leftUp()) {
            onMouseUp((point_projection_on_xy_plane));
        }
        return 0;
    }

    int TerrainLabellingTool::onMouseDown(const std::pair<bool, Ogre::Vector3> &xy_plane_intersection) {
        int flags = 0;
        if(!_lastSaved){
            switch(_currentLabelType){
                case CONSTANTS::OBSTACLE:
                    _obstacles.pop_back();
                    break;
                case CONSTANTS::FEATURE:
                    _features.pop_back();
                    break;
                case CONSTANTS::DESTINATION:
                    _destinations.pop_back();
                    break;
            }
        }
        if (xy_plane_intersection.first) {
            switch (_currentLabelType) {
                case CONSTANTS::OBSTACLE:
                    createObstacleRect(xy_plane_intersection);
                    break;
                case CONSTANTS::FEATURE:
                    createFeatureRect(xy_plane_intersection);
                    break;
                case CONSTANTS::DESTINATION:
                    createDestinationRect(xy_plane_intersection);
                    break;
            }


            flags |= Render;
        }

        return flags;
    }

    void TerrainLabellingTool::onMouseUp(const std::pair<bool, Ogre::Vector3> &xy_plane_intersection) {
        std::pair<std::shared_ptr<rviz_rendering::Shape>, Ogre::Vector3> pair;

        switch (_currentLabelType) {
            case CONSTANTS::OBSTACLE:
                drawRect(_obstacles.back(), xy_plane_intersection);
                break;
            case CONSTANTS::FEATURE:
                drawRect(_features.back(), xy_plane_intersection);
                break;
            case CONSTANTS::DESTINATION:
                drawRect(_destinations.back(), xy_plane_intersection);
                break;
        }
    }

    void TerrainLabellingTool::drawRect(std::pair<std::shared_ptr<rviz_rendering::Shape>, Ogre::Vector3> &pair,
                                        const std::pair<bool, Ogre::Vector3> &xy_plane_intersection) {
        auto &rect = pair.first;

        auto diffx = xy_plane_intersection.second.x - rect->getPosition().x;
        auto diffy = xy_plane_intersection.second.y - rect->getPosition().y;
        auto size = Ogre::Vector3(diffx, diffy, 0.0001);
        pair.second = size;
        rect->setScale(size);

        auto og_x = rect->getPosition().x;
        auto og_y = rect->getPosition().y;

        rect->setPosition(Ogre::Vector3(0.5 * diffx + og_x, 0.5 * diffy + og_y, 0.00));
        rect->getRootNode()->setVisible(true);

        std::cout << rect->getPosition().xy() << std::endl;
    }

    void TerrainLabellingTool::labellingTypeCallback(std_msgs::msg::Int64 msg) {
        if(!_lastSaved){
            switch(_currentLabelType){
                case CONSTANTS::OBSTACLE:
                    _obstacles.pop_back();
                    break;
                case CONSTANTS::FEATURE:
                    _features.pop_back();
                    break;
                case CONSTANTS::DESTINATION:
                    _destinations.pop_back();
                    break;
            }
            _lastSaved = true;
        }
        _currentLabelType = msg.data;
    }

    void TerrainLabellingTool::addLabelCallback(std_msgs::msg::String msg) {
        switch (_currentLabelType) {
            case CONSTANTS::OBSTACLE: {
                roomba_msgs::msg::MultifloorRectangle message;
                message.floor_id.data = _currentFloor;
                message.label = msg;
                auto pair = _obstacles.back();
                auto rect = pair.first;
                auto size = pair.second;
                message.p1.x = rect->getPosition().x - 0.5 * size.x;
                message.p2.x = rect->getPosition().x + 0.5 * size.x;
                message.p1.y = rect->getPosition().y - 0.5 * size.y;
                message.p2.y = rect->getPosition().y + 0.5 * size.y;
                _obsPub->publish(message);

                visualization_msgs::msg::Marker marker_msg;
                marker_msg.header.frame_id = "/map";
                marker_msg.id = _nameMarkerArray.markers.size();
                marker_msg.text = msg.data;
                marker_msg.color.a = 1.0f;
                marker_msg.color.r = 1.0f;
                marker_msg.color.g = 1.0f;
                marker_msg.color.b = 1.0f;
                marker_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                marker_msg.pose.position.x = rect->getPosition().x;
                marker_msg.pose.position.y = rect->getPosition().y;
                marker_msg.pose.position.z = 0.2;
                marker_msg.scale.z = 0.2;
                _nameMarkerArray.markers.push_back(marker_msg);
                _nameMarkerPub->publish(_nameMarkerArray);

                break;
            }
            case CONSTANTS::FEATURE: {
                roomba_msgs::msg::MultifloorRectangle message;
                message.floor_id.data = _currentFloor;
                message.label = msg;
                auto pair = _features.back();
                auto rect = pair.first;
                auto size = pair.second;
                message.p1.x = rect->getPosition().x - 0.5 * size.x;
                message.p2.x = rect->getPosition().x + 0.5 * size.x;
                message.p1.y = rect->getPosition().y - 0.5 * size.y;
                message.p2.y = rect->getPosition().y + 0.5 * size.y;
                _featPub->publish(message);

                visualization_msgs::msg::Marker marker_msg;
                marker_msg.header.frame_id = "/map";
                marker_msg.id = _nameMarkerArray.markers.size();
                marker_msg.text = msg.data;
                marker_msg.color.a = 1.0f;
                marker_msg.color.r = 1.0f;
                marker_msg.color.g = 1.0f;
                marker_msg.color.b = 1.0f;
                marker_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                marker_msg.pose.position.x = rect->getPosition().x;
                marker_msg.pose.position.y = rect->getPosition().y;
                marker_msg.pose.position.z = 0.2;
                marker_msg.scale.z = 0.2;
                _nameMarkerArray.markers.push_back(marker_msg);
                _nameMarkerPub->publish(_nameMarkerArray);
                break;
            }
            case CONSTANTS::DESTINATION: {
                roomba_msgs::msg::MultifloorPoint message;
                message.floor_id.data = _currentFloor;
                message.label = msg;
                auto pair = _destinations.back();
                auto rect = pair.first;
                message.point.x = rect->getPosition().x;
                message.point.y = rect->getPosition().y;
                _destPub->publish(message);

                visualization_msgs::msg::Marker marker_msg;
                marker_msg.header.frame_id = "/map";
                marker_msg.id = _nameMarkerArray.markers.size();
                marker_msg.text = msg.data;
                marker_msg.color.a = 1.0f;
                marker_msg.color.r = 1.0f;
                marker_msg.color.g = 1.0f;
                marker_msg.color.b = 1.0f;
                marker_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                marker_msg.pose.position.x = rect->getPosition().x;
                marker_msg.pose.position.y = rect->getPosition().y;
                marker_msg.pose.position.z = 0.2;
                marker_msg.scale.z = 0.2;
                _nameMarkerArray.markers.push_back(marker_msg);
                _nameMarkerPub->publish(_nameMarkerArray);
                break;
            }
        }
        _lastSaved = true;
    }

    void TerrainLabellingTool::floorCallback(std_msgs::msg::String msg) {
        _currentFloor = msg.data;
    }

    void TerrainLabellingTool::createObstacleRect(const std::pair<bool, Ogre::Vector3> &xy_plane_intersection) {
        auto rect = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Cube, scene_manager_, nullptr);

        rect->setColor(1.0f, 0.0f, 0.0f, 1.0f);
        rect->getRootNode()->setVisible(false);
        rect->setPosition(xy_plane_intersection.second);

        _obstacles.push_back(std::pair<std::shared_ptr<rviz_rendering::Shape>, Ogre::Vector3>(rect, Ogre::Vector3()));

        _lastSaved = false;
    }

    void TerrainLabellingTool::createFeatureRect(const std::pair<bool, Ogre::Vector3> &xy_plane_intersection) {
        auto rect = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Cube, scene_manager_, nullptr);

        rect->setColor(0.0f, 1.0f, 0.0f, 1.0f);
        rect->getRootNode()->setVisible(false);
        rect->setPosition(xy_plane_intersection.second);

        _features.push_back(std::pair<std::shared_ptr<rviz_rendering::Shape>, Ogre::Vector3>(rect, Ogre::Vector3()));

        _lastSaved = false;
    }

    void TerrainLabellingTool::createDestinationRect(const std::pair<bool, Ogre::Vector3> &xy_plane_intersection) {
        auto rect = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Cube, scene_manager_, nullptr);

        rect->setColor(0.0f, 0.0f, 1.0f, 1.0f);
        rect->getRootNode()->setVisible(false);
        rect->setPosition(xy_plane_intersection.second);

        _destinations.push_back(std::pair<std::shared_ptr<rviz_rendering::Shape>, Ogre::Vector3>(rect, Ogre::Vector3()));

        _lastSaved = false;
    }

    void TerrainLabellingTool::clearLabels(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                           std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        for (auto &obs : _obstacles){
            obs.first->getRootNode()->setVisible(false);
        }
        _obstacles.clear();

        for (auto &feat : _features){
            feat.first->getRootNode()->setVisible(false);
        }
        _features.clear();

        for (auto &dest : _destinations){
            dest.first->getRootNode()->setVisible(false);
        }
        _destinations.clear();

        for(auto &marker : _nameMarkerArray.markers){
            marker.action = visualization_msgs::msg::Marker::DELETE;
        }
        _nameMarkerPub->publish(_nameMarkerArray);
    }
}


#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(roomba_rviz_plugins::TerrainLabellingTool, rviz_common::Tool)