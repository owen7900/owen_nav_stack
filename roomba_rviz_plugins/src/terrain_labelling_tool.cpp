//
// Created by kevin on 10/27/22.
//

#include "../include/roomba_rviz_plugins/terrain_labelling_tool.hpp"

#include <memory>
#include <string>
#include <utility>

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"


namespace roomba_rviz_plugins{
    TerrainLabellingTool::TerrainLabellingTool() : rviz_common::Tool(), _arrow(nullptr), _angle(0){
        shortcut_key_ = 'l';
        _projection_finder = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
    }

    void TerrainLabellingTool::onInitialize(){
        rviz_common::Tool::onInitialize();
        setName("Terrain Labelling");

        _arrow = std::make_shared<rviz_rendering::Arrow>(
                scene_manager_, nullptr, 2.0f, 0.2f, 0.5f, 0.35f);
        _arrow->setColor(0.0f, 1.0f, 0.0f, 1.0f);
        _arrow->getSceneNode()->setVisible(false);
    }

    void TerrainLabellingTool::activate(){
        _state = Position;
    }

    void TerrainLabellingTool::deactivate(){
        _arrow->getSceneNode()->setVisible(false);
    }

    int TerrainLabellingTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
    {
        auto point_projection_on_xy_plane = _projection_finder->getViewportPointProjectionOnXYPlane(
                event.panel->getRenderWindow(), event.x, event.y);

        if (event.leftDown()) {
            return processMouseLeftButtonPressed(point_projection_on_xy_plane);
        } else if (event.type == QEvent::MouseMove && event.left()) {
            return processMouseMoved(point_projection_on_xy_plane);
        } else if (event.leftUp()) {
            return processMouseLeftButtonReleased();
        }
        return 0;
    }

    int TerrainLabellingTool::processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection)
    {
        int flags = 0;
        assert(_state == Position);
        if (xy_plane_intersection.first) {
            _arrow_position = xy_plane_intersection.second;
            _arrow->setPosition(_arrow_position);

            _state = Orientation;
            flags |= Render;
        }
        return flags;
    }

    int TerrainLabellingTool::processMouseMoved(std::pair<bool, Ogre::Vector3> xy_plane_intersection)
    {
        int flags = 0;
        if (_state == Orientation) {
            // compute angle in x-y plane
            if (xy_plane_intersection.first) {
                _angle = calculateAngle(xy_plane_intersection.second, _arrow_position);
                makeArrowVisibleAndSetOrientation(_angle);

                flags |= Render;
            }
        }

        return flags;
    }

    void TerrainLabellingTool::makeArrowVisibleAndSetOrientation(double angle)
    {
        _arrow->getSceneNode()->setVisible(true);

        // we need base_orient, since the arrow goes along the -z axis by default
        // (for historical reasons)
        Ogre::Quaternion orient_x = Ogre::Quaternion(
                Ogre::Radian(-Ogre::Math::HALF_PI),
                Ogre::Vector3::UNIT_Y);

        _arrow->setOrientation(Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z) * orient_x);
    }

    int TerrainLabellingTool::processMouseLeftButtonReleased()
    {
        int flags = 0;
        if (_state == Orientation) {
            onThingFinished(_arrow_position.x, _arrow_position.y, _arrow_position.x, _arrow_position.y);
            flags |= (Finished | Render);
        }

        return flags;
    }

    double TerrainLabellingTool::calculateAngle(Ogre::Vector3 start_point, Ogre::Vector3 end_point)
    {
        return atan2(start_point.y - end_point.y, start_point.x - end_point.x);
    }


    void TerrainLabellingTool::onThingFinished(double x0, double y0, double x1, double y1) {

    }


}


#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(roomba_rviz_plugins::TerrainLabellingTool, rviz_common::Tool)