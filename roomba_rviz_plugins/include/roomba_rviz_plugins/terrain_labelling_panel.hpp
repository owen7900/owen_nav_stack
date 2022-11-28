//
// Created by kevin on 10/26/22.
//

#ifndef ROOMBA_RVIZ_PLUGINS_TERRAIN_LABELLING_PANEL_HPP
#define ROOMBA_RVIZ_PLUGINS_TERRAIN_LABELLING_PANEL_HPP


#include <memory>
#include <string>
#include <vector>

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
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "roomba_msgs/msg/multifloor_rectangle.hpp"
#include "roomba_msgs/msg/multifloor_point.hpp"
#include "roomba_msgs/srv/load_config.hpp"
#include "std_srvs/srv/empty.hpp"


namespace roomba_rviz_plugins{
class TerrainLabelling : public rviz_common::Panel{
    Q_OBJECT

public:
    explicit TerrainLabelling(QWidget *parent = 0);
    ~TerrainLabelling();
    void onInitialize() override;

private Q_SLOTS:
    void SwitchLabelType();
    void SaveFeatures();
    void ClearFeatures();
    void AddLabel();
    void LoadConfig();

private:
    void obstacleCallback(roomba_msgs::msg::MultifloorRectangle msg);
    void featureCallback(roomba_msgs::msg::MultifloorRectangle msg);
    void destinationCallback(roomba_msgs::msg::MultifloorPoint msg);

protected:
    QVBoxLayout * _verticalBox;
    QHBoxLayout * _hbox1;
    QHBoxLayout * _hbox2;
    QHBoxLayout * _hbox3;
    QHBoxLayout * _hbox4;

    QLabel * _labelType;

    QLineEdit * _label;
    QPushButton * _addLabelButton;

    QRadioButton * _obstacleRB;
    QRadioButton * _featureRB;
    QRadioButton * _destinationRB;

    QLineEdit * _loadConfigPath;
    QPushButton * _loadConfigButton;

    QLineEdit * _outputPath;
    QPushButton * _saveButton;
    QPushButton * _clearButton;

    QFrame * _line1;
    QFrame * _line2;
    QFrame * _line3;

private:
    int _currentLabelType;
    std::vector<roomba_msgs::msg::MultifloorRectangle> _obstacles;
    std::vector<roomba_msgs::msg::MultifloorRectangle> _features;
    std::vector<roomba_msgs::msg::MultifloorPoint> _destinations;


    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _clearLabelsClient;
    rclcpp::Client<roomba_msgs::srv::LoadConfig>::SharedPtr _loadConfigClient;

    // TODO: make this a service client
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr _labelTypePub;
    // TODO: make this a service client
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _labelNamePub;
    // TODO: make these each a service (addObs, addFeat, addDest)
    rclcpp::Subscription<roomba_msgs::msg::MultifloorRectangle>::SharedPtr _obsSub;
    rclcpp::Subscription<roomba_msgs::msg::MultifloorRectangle>::SharedPtr _featSub;
    rclcpp::Subscription<roomba_msgs::msg::MultifloorPoint>::SharedPtr _destSub;
};
}



#endif //ROOMBA_RVIZ_PLUGINS_TERRAIN_LABELLING_PANEL_HPP
