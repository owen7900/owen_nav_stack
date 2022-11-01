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

#include "rviz_common/tool.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_rendering
{
    class Arrow;
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
        void onThingFinished(double x0, double y0, double x1, double y1);
        int processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
        int processMouseMoved(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
        int processMouseLeftButtonReleased();
        void makeArrowVisibleAndSetOrientation(double angle);
        double calculateAngle(Ogre::Vector3 start_point, Ogre::Vector3 end_point);

    private:
        std::shared_ptr<rviz_rendering::Arrow> _arrow;
        Ogre::Vector3 _arrow_position;
        std::shared_ptr<rviz_rendering::ViewportProjectionFinder> _projection_finder;
        enum State
        {
            Position,
            Orientation
        };
        State _state;
        double _angle;

    };

} // namespace roomba_rviz_plugins



#endif //ROOMBA_RVIZ_PLUGINS_TERRAIN_LABELLING_TOOL_HPP
