//
// Created by kevin on 11/26/22.
//

#ifndef CAPSTON_WS_MAP_NODE_LABELLING_PANEL_HPP
#define CAPSTON_WS_MAP_NODE_LABELLING_PANEL_HPP

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
#include <std_srvs/srv/empty.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "roomba_msgs/msg/multifloor_point.hpp"
#include "roomba_msgs/srv/load_config.hpp"
#include "roomba_msgs/srv/add_map_node.hpp"

namespace roomba_rviz_plugins{
    class MapNodeLabellingPanel : public rviz_common::Panel{
        Q_OBJECT

    public:
        explicit MapNodeLabellingPanel(QWidget *parent = 0);
        void onInitialize() override;

    private Q_SLOTS:
        void ExportNodes();
        void ClearMapNodes();
        void AddNode();
        void LoadNodes();

    private:
        void saveMapNode(rclcpp::Client<roomba_msgs::srv::AddMapNode>::SharedFuture future);
//        void saveNodes(rclcpp::Client<roomba_msgs::srv::LoadConfig>::SharedFuture future);

    protected:
        QVBoxLayout * _verticalBox;
        QHBoxLayout * _hbox1;
        QHBoxLayout * _hbox2;
        QHBoxLayout * _hbox3;

        QPushButton * _addNodeButton;

        QLineEdit * _loadNodesPath;
        QPushButton * _loadNodesButton;

        QLineEdit * _outputPath;
        QPushButton * _saveButton;
        QPushButton * _clearButton;

        QFrame * _line1;
        QFrame * _line2;

    private:
        std::vector<roomba_msgs::msg::MultifloorPoint> _mapNodes;
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _clearNodesClient;
        rclcpp::Client<roomba_msgs::srv::LoadConfig>::SharedPtr _loadNodesClient;
        rclcpp::Client<roomba_msgs::srv::AddMapNode>::SharedPtr _addMapNodeClient;
    };
}


#endif //CAPSTON_WS_MAP_NODE_LABELLING_PANEL_HPP
