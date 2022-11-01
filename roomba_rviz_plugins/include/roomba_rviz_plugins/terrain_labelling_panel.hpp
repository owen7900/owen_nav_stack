//
// Created by kevin on 10/26/22.
//

#ifndef ROOMBA_RVIZ_PLUGINS_TERRAIN_LABELLING_PANEL_HPP
#define ROOMBA_RVIZ_PLUGINS_TERRAIN_LABELLING_PANEL_HPP


#include <memory>
#include <string>
#include <vector>

// QT
#include <QPushButton>
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QtGui>
#include <QLabel>
#include <QFrame>
#include <QRadioButton>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class QLineEdit;
class QSpinBox;
class QComboBox;

namespace roomba_rviz_plugins{
class TerrainLabelling : public rviz_common::Panel{
    Q_OBJECT

public:
    explicit TerrainLabelling(QWidget *parent = 0);
    ~TerrainLabelling();

private Q_SLOTS:
    void SwitchTerrainLabel(bool passable);
    void SaveFeatures();
    void ClearFeatures();

protected:
    QVBoxLayout * _verticalBox;
    QHBoxLayout * _hbox1;
    QHBoxLayout * _hbox2;

    QLabel * _terrainLabel;

    QRadioButton * _passableRB;
    QRadioButton * _impassableRB;

    QPushButton * _saveButton;
    QPushButton * _clearButton;

    QFrame * _line;
};
}



#endif //ROOMBA_RVIZ_PLUGINS_TERRAIN_LABELLING_PANEL_HPP
