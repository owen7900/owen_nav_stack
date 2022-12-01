//
// Created by kevin on 10/27/22.
//

#include "roomba_rviz_plugins/terrain_labelling_tool.hpp"

#include <memory>
#include <string>
#include <utility>

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/display_context.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace roomba_rviz_plugins {
    TerrainLabellingTool::TerrainLabellingTool()
            : rviz_common::Tool(), _currentLabellingType(0), _lastSaved(true), _currentFloor(1) {
        _projectionFinder = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
    }

    TerrainLabellingTool::~TerrainLabellingTool() = default;

    void TerrainLabellingTool::onInitialize() {
        auto raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
        _switchLabelTypeService = raw_node->create_service<roomba_msgs::srv::SwitchLabellingType>("switchLabelType",
                                                                                                  std::bind(
                                                                                                          &TerrainLabellingTool::changeLabelType,
                                                                                                          this,
                                                                                                          std::placeholders::_1,
                                                                                                          std::placeholders::_2));

        _addLabelService = raw_node->create_service<roomba_msgs::srv::AddLabel>("addLabel",
                                                                                                  std::bind(
                                                                                                          &TerrainLabellingTool::addLabel,
                                                                                                          this,
                                                                                                          std::placeholders::_1,
                                                                                                          std::placeholders::_2));

        _clearLabelsService = raw_node->create_service<std_srvs::srv::Empty>("clearLabels",
                                                                                std::bind(
                                                                                        &TerrainLabellingTool::clearLabels,
                                                                                        this,
                                                                                        std::placeholders::_1,
                                                                                        std::placeholders::_2));


        _nameMarkerPub = raw_node->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 10);
    }

    void TerrainLabellingTool::activate() {}

    void TerrainLabellingTool::deactivate() {}

    int TerrainLabellingTool::processMouseEvent(rviz_common::ViewportMouseEvent &event) {
        auto point_projection_on_xy_plane = _projectionFinder->getViewportPointProjectionOnXYPlane(
                event.panel->getRenderWindow(), event.x, event.y);

        if (event.leftDown()) {
            return processMouseLeftButtonPressed(point_projection_on_xy_plane);
        } else if (event.type == QEvent::MouseMove && event.right()) {
            return processMouseMovedRightDown(point_projection_on_xy_plane);
        } else if (event.leftUp()) {
            return processMouseLeftButtonReleased(point_projection_on_xy_plane.second);
        } else if(event.middleDown()){
            return processMouseMiddleDown(point_projection_on_xy_plane.second);
        } else if(event.middleUp()){
            return processMouseMiddleReleased(point_projection_on_xy_plane.second);
        }

        return 0;
    }

    int TerrainLabellingTool::processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection) {
        int flags = 0;
        if (!_lastSaved) {
            if (_currentLabellingType == CONSTANTS::OBSTACLE)
                _obstacles.pop_back();
            if (_currentLabellingType == CONSTANTS::FEATURE)
                _features.pop_back();
            if (_currentLabellingType == CONSTANTS::DESTINATION)
                _destinations.pop_back();
        }

        if (xy_plane_intersection.first) {
            if (_currentLabellingType == CONSTANTS::OBSTACLE)
                createObstacle(xy_plane_intersection.second);
            if (_currentLabellingType == CONSTANTS::FEATURE)
                createFeature(xy_plane_intersection.second);
            if (_currentLabellingType == CONSTANTS::DESTINATION)
                createDestination(xy_plane_intersection.second);

            _lastSaved = false;
            flags |= Render;
        }
        return flags;
    }

    int TerrainLabellingTool::processMouseMiddleDown(Ogre::Vector3 xy_plane_intersection) {
        int flags = 0;
        if(_lastSaved || _currentLabellingType == CONSTANTS::DESTINATION)
            return flags;

        _middleDownPos = xy_plane_intersection;
        return flags |= Render;
    }

    int TerrainLabellingTool::processMouseMiddleReleased(const Ogre::Vector3 xy_plane_intersection) {
        int flags = 0;
        if(_lastSaved || _currentLabellingType == CONSTANTS::DESTINATION)
            return flags;
        if(_currentLabellingType == CONSTANTS::OBSTACLE)
            moveRect(_obstacles.back(), xy_plane_intersection);
        if(_currentLabellingType == CONSTANTS::FEATURE)
            moveRect(_features.back(), xy_plane_intersection);

        return flags;
    }

    int TerrainLabellingTool::processMouseMovedRightDown(std::pair<bool, Ogre::Vector3> xy_plane_intersection) {
        int flags = 0;
        if(_lastSaved)
            return flags;
        // compute angle in x-y plane
        if (xy_plane_intersection.first) {
            std::pair<std::shared_ptr<rviz_rendering::Shape>, Ogre::Vector3> pair;
            if (_currentLabellingType == CONSTANTS::OBSTACLE)
                pair = _obstacles.back();
            if (_currentLabellingType == CONSTANTS::FEATURE)
                pair = _features.back();
            if(_currentLabellingType == CONSTANTS::DESTINATION)
                return flags;

            double angle  = calculateAngle(xy_plane_intersection.second, pair.first->getPosition());
            pair.first->setOrientation(Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z));

            flags |= Render;
        }
        return flags;
    }

    int TerrainLabellingTool::processMouseLeftButtonReleased(Ogre::Vector3 xy_plane_intersection) {
        int flags = 0;
        if (_currentLabellingType == CONSTANTS::OBSTACLE)
            editPosAndSize(_obstacles.back(), xy_plane_intersection);
        if (_currentLabellingType == CONSTANTS::FEATURE)
            editPosAndSize(_features.back(), xy_plane_intersection);
        if(_currentLabellingType == CONSTANTS::DESTINATION)
            return flags | Render;

        flags |= Render;
        return flags;
    }

    void TerrainLabellingTool::moveRect(std::pair<std::shared_ptr<rviz_rendering::Shape>, Ogre::Vector3> &pair, Ogre::Vector3 xy_plane_intersection){
        double diffX =  xy_plane_intersection.x - _middleDownPos.x;
        double diffY = xy_plane_intersection.y - _middleDownPos.y;
        auto oldPos = pair.first->getPosition();
        Ogre::Vector3 newPos;
        newPos.x = oldPos.x + diffX;
        newPos.y = oldPos.y + diffY;

        pair.first->setPosition(newPos);
        pair.first->getRootNode()->setVisible(true);
    }

    void TerrainLabellingTool::editPosAndSize(std::pair<std::shared_ptr<rviz_rendering::Shape>, Ogre::Vector3> &pair, Ogre::Vector3 xy_plane_intersection){
        auto &rect = pair.first;
        double diffx = xy_plane_intersection.x - rect->getPosition().x;
        double diffy = xy_plane_intersection.y - rect->getPosition().y;
        auto size = Ogre::Vector3(diffx, diffy, 0.001);
        rect->setScale(size);
        pair.second = size;

        double og_x = rect->getPosition().x;
        double og_y = rect->getPosition().y;
        rect->setPosition(Ogre::Vector3((0.5 * diffx) + og_x, (0.5 * diffy) + og_y, 0.00));
    }

    double TerrainLabellingTool::calculateAngle(Ogre::Vector3 start_point, Ogre::Vector3 end_point) {
        return atan2(start_point.y - end_point.y, start_point.x - end_point.x);
    }

    void TerrainLabellingTool::changeLabelType(std::shared_ptr<roomba_msgs::srv::SwitchLabellingType::Request> request,
                                               std::shared_ptr<roomba_msgs::srv::SwitchLabellingType::Response> response) {
        if (!_lastSaved) {
            if (_currentLabellingType == CONSTANTS::OBSTACLE)
                _obstacles.pop_back();
            if (_currentLabellingType == CONSTANTS::FEATURE)
                _features.pop_back();
            if (_currentLabellingType == CONSTANTS::DESTINATION)
                _destinations.pop_back();
            _lastSaved = true;
        }
        _currentLabellingType = request->type.data;
    }

    void TerrainLabellingTool::addLabel(std::shared_ptr<roomba_msgs::srv::AddLabel::Request> request,
                                        std::shared_ptr<roomba_msgs::srv::AddLabel::Response> response) {
        if(_currentLabellingType == CONSTANTS::DESTINATION){
            auto point = _destinations.back()->getPosition();
            roomba_msgs::msg::MultifloorPoint p;
            p.point.x = point.x;
            p.point.y = point.y;
            p.floor_id.data = std::to_string(_currentFloor);
            p.label = request->label;
            response->points.push_back(p);

            visualization_msgs::msg::Marker marker_msg;
            marker_msg.header.frame_id = "/map";
            marker_msg.id = _nameMarkerArray.markers.size();
            marker_msg.text = request->label.data;
            marker_msg.color.a = 1.0f;
            marker_msg.color.r = 1.0f;
            marker_msg.color.g = 1.0f;
            marker_msg.color.b = 1.0f;
            marker_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker_msg.pose.position.x = p.point.x;
            marker_msg.pose.position.y = p.point.y;
            marker_msg.pose.position.z = 0.2;
            marker_msg.scale.z = 0.2;
            _nameMarkerArray.markers.push_back(marker_msg);
            _nameMarkerPub->publish(_nameMarkerArray);

            _lastSaved = true;
            return;
        }

        std::pair<std::shared_ptr<rviz_rendering::Shape>, Ogre::Vector3> pair;
        if(_currentLabellingType == CONSTANTS::OBSTACLE){
            pair = _obstacles.back();
        }
        if(_currentLabellingType == CONSTANTS::FEATURE){
            pair = _features.back();
        }

        auto size = pair.second;

        auto center = pair.first->getPosition();
        double theta = pair.first->getOrientation().getRoll().valueRadians();

        double p = center.x;
        double q = center.y;
        roomba_msgs::msg::MultifloorPoint og_p1;
        og_p1.point.x = center.x - (0.5*size.x);
        og_p1.point.y = center.y + (0.5*size.y);
        roomba_msgs::msg::MultifloorPoint og_p2;
        og_p2.point.x = center.x + (0.5*size.x);
        og_p2.point.y = center.y + (0.5*size.y);
        roomba_msgs::msg::MultifloorPoint og_p3;
        og_p3.point.x = center.x + (0.5*size.x);
        og_p3.point.y = center.y - (0.5*size.y);
        roomba_msgs::msg::MultifloorPoint og_p4;
        og_p4.point.x = center.x - (0.5*size.x);
        og_p4.point.y = center.y - (0.5*size.y);

        std::vector<roomba_msgs::msg::MultifloorPoint> points;

        roomba_msgs::msg::MultifloorPoint p1;
        p1.point.x = rotatePoint(Ogre::Vector3(og_p1.point.x, og_p1.point.y, 0), center, theta).x;
        p1.point.y = rotatePoint(Ogre::Vector3(og_p1.point.x, og_p1.point.y, 0), center, theta).y;
        p1.floor_id.data = std::to_string(_currentFloor);
        p1.label = request->label;
        points.push_back(p1);

        roomba_msgs::msg::MultifloorPoint p2;
        p2.point.x = rotatePoint(Ogre::Vector3(og_p2.point.x, og_p2.point.y, 0), center, theta).x;
        p2.point.y = rotatePoint(Ogre::Vector3(og_p2.point.x, og_p2.point.y, 0), center, theta).y;
        p2.floor_id.data = std::to_string(_currentFloor);
        p2.label = request->label;
        points.push_back(p2);

        roomba_msgs::msg::MultifloorPoint p3;
        p3.point.x = rotatePoint(Ogre::Vector3(og_p3.point.x, og_p3.point.y, 0), center, theta).x;
        p3.point.y = rotatePoint(Ogre::Vector3(og_p3.point.x, og_p3.point.y, 0), center, theta).y;
        p3.floor_id.data = std::to_string(_currentFloor);
        p3.label = request->label;
        points.push_back(p3);

        roomba_msgs::msg::MultifloorPoint p4;
        p4.point.x = rotatePoint(Ogre::Vector3(og_p4.point.x, og_p4.point.y, 0), center, theta).x;
        p4.point.y = rotatePoint(Ogre::Vector3(og_p4.point.x, og_p4.point.y, 0), center, theta).y;
        p4.floor_id.data = std::to_string(_currentFloor);
        p4.label = request->label;
        points.push_back(p4);

        response->points = points;

        visualization_msgs::msg::Marker marker_msg;
        marker_msg.header.frame_id = "/map";
        marker_msg.id = _nameMarkerArray.markers.size();
        marker_msg.text = request->label.data;
        marker_msg.color.a = 1.0f;
        marker_msg.color.r = 1.0f;
        marker_msg.color.g = 1.0f;
        marker_msg.color.b = 1.0f;
        marker_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker_msg.pose.position.x = center.x;
        marker_msg.pose.position.y = center.y;
        marker_msg.pose.position.z = 0.2;
        marker_msg.scale.z = 0.2;
        _nameMarkerArray.markers.push_back(marker_msg);
        _nameMarkerPub->publish(_nameMarkerArray);

        _lastSaved = true;
    }

    Ogre::Vector3 TerrainLabellingTool::rotatePoint(Ogre::Vector3 originalPoint, Ogre::Vector3 origin, double angle){
        Ogre::Vector3 rotatedPoint;
        rotatedPoint.x = cos(angle)*(originalPoint.x - origin.x) - sin(angle)*(originalPoint.y - origin.y) + origin.x;
        rotatedPoint.y = sin(angle)*(originalPoint.x - origin.x) + cos(angle)*(originalPoint.y - origin.y) + origin.y;
        return rotatedPoint;
    }

    void TerrainLabellingTool::createObstacle(Ogre::Vector3 position) {
        auto rect = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Cube, scene_manager_,
                                                            nullptr);
        rect->setColor(1.0f, 0.0f, 0.0f, 1.0f);
        rect->setPosition(position);
        rect->setScale(Ogre::Vector3(0, 0, 0));
        rect->getRootNode()->setVisible(true);
        _obstacles.push_back(std::pair<std::shared_ptr<rviz_rendering::Shape>, Ogre::Vector3>(rect, Ogre::Vector3()));
    }

    void TerrainLabellingTool::createFeature(Ogre::Vector3 position) {
        auto rect = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Cube, scene_manager_,
                                                            nullptr);
        rect->setColor(0.0f, 1.0f, 0.0f, 1.0f);
        rect->setPosition(position);
        rect->setScale(Ogre::Vector3(0, 0, 0));
        rect->getRootNode()->setVisible(true);
        _features.push_back(std::pair<std::shared_ptr<rviz_rendering::Shape>, Ogre::Vector3>(rect, Ogre::Vector3()));
    }

    void TerrainLabellingTool::createDestination(Ogre::Vector3 position) {
        auto circle = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Sphere, scene_manager_,
                                                              nullptr);
        circle->setColor(0.0f, 0.0f, 1.0f, 1.0f);
        circle->setPosition(position);
        circle->setScale(Ogre::Vector3(0.2, 0.2, 0.001));
        circle->getRootNode()->setVisible(true);
        _destinations.push_back(circle);
    }

    void TerrainLabellingTool::clearLabels(std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                           std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        _obstacles.clear();
        _features.clear();
        _destinations.clear();

        for (auto &marker: _nameMarkerArray.markers) {
            marker.action = visualization_msgs::msg::Marker::DELETE;
        }
        _nameMarkerPub->publish(_nameMarkerArray);
    }
}


#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(roomba_rviz_plugins::TerrainLabellingTool, rviz_common::Tool)