//
// Created by kevin on 11/26/22.
//
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QLabel>
#include <QFrame>

#include <fstream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <memory>

#include "rviz_common/display_context.hpp"
#include "roomba_rviz_plugins/map_node_labelling_panel.hpp"

using namespace std::chrono_literals;

namespace roomba_rviz_plugins {
    MapNodeLabellingPanel::MapNodeLabellingPanel(QWidget *parent) : Panel(parent) {
        _verticalBox = new QVBoxLayout();
        _hbox1 = new QHBoxLayout();
        _hbox2 = new QHBoxLayout();
        _hbox3 = new QHBoxLayout();

        _line1 = new QFrame();
        _line1->setFrameShape(QFrame::HLine);
        _line1->setFrameShadow(QFrame::Sunken);

        _line2 = new QFrame();
        _line2->setFrameShape(QFrame::HLine);
        _line2->setFrameShadow(QFrame::Sunken);

        _addNodeButton = new QPushButton(this);
        _addNodeButton->setText("Add node");
        connect(_addNodeButton, SIGNAL(clicked()), this, SLOT(AddNode()));

        _loadNodesPath = new QLineEdit();
        _loadNodesPath->setPlaceholderText("Existing config file path");

        _loadNodesButton = new QPushButton(this);
        _loadNodesButton->setText("Load existing config");
        connect(_loadNodesButton, SIGNAL(clicked()), this, SLOT(LoadNodes()));;

        _outputPath = new QLineEdit();
        _outputPath->setPlaceholderText("Output file path");

        _saveButton = new QPushButton(this);
        _saveButton->setText("Save nodes to file");
        connect(_saveButton, SIGNAL(clicked()), this, SLOT(ExportNodes()));

        _clearButton = new QPushButton(this);
        _clearButton->setText("Clear all nodes");
        connect(_clearButton, SIGNAL(clicked()), this, SLOT(ClearMapNodes()));

        _hbox1->addWidget(_loadNodesPath);
        _hbox1->addWidget(_loadNodesButton);

        _hbox2->addWidget(_addNodeButton);

        _hbox3->addWidget(_outputPath);
        _hbox3->addWidget(_saveButton);
        _hbox3->addWidget(_clearButton);

        _verticalBox->addLayout(_hbox1);
        _verticalBox->addWidget(_line1);
        _verticalBox->addLayout(_hbox2);
        _verticalBox->addWidget(_line2);
        _verticalBox->addLayout(_hbox3);

        setLayout(_verticalBox);
    }

    void MapNodeLabellingPanel::onInitialize() {
        auto context = this->getDisplayContext();
        auto lock = context->getRosNodeAbstraction().lock();
        auto raw_node = lock->get_raw_node();

        _clearNodesClient = raw_node->create_client<std_srvs::srv::Empty>("clearNodes");
        _loadNodesClient = raw_node->create_client<roomba_msgs::srv::LoadConfig>("loadNodes");
        _addMapNodeClient = raw_node->create_client<roomba_msgs::srv::AddMapNode>("addMapNode");
    }

    void MapNodeLabellingPanel::ExportNodes() {
        auto outputPath = _outputPath->text().toStdString();
        YAML::Node node;

        for(size_t i = 0; i < _mapNodes.size(); ++i){
            const auto &mapNode = _mapNodes[i];
            node[mapNode.label.data];
            YAML::Node id;
            if(i == 0){
                id["id"] = "";
                id["cost"] = "";
            } else{
                id["id"] = std::to_string(i);
                auto &lastNode = _mapNodes[i - 1];
                const auto dist = std::pow((lastNode.point.x - mapNode.point.x), 2) + std::pow((lastNode.point.y - mapNode.point.y), 2);
                id["cost"] = std::to_string(dist);
            }

            node[mapNode.label.data]["connections"].push_back(id);

            node[mapNode.label.data]["floor_id"] = mapNode.floor_id.data;
            node[mapNode.label.data]["x"] = mapNode.point.x;
            node[mapNode.label.data]["y"] = mapNode.point.y;
        }

        const auto output = YAML::Dump(node);
        std::ofstream out(outputPath);
        out << output;
        out.close();

        _outputPath->clear();
    }

    void MapNodeLabellingPanel::ClearMapNodes() {
        _mapNodes.clear();
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        _clearNodesClient->async_send_request(request);
    }

    void MapNodeLabellingPanel::LoadNodes() {
        ClearMapNodes();
        auto request = std::make_shared<roomba_msgs::srv::LoadConfig::Request>();
        request->path.data = _loadNodesPath->text().toStdString();
        _loadNodesClient->async_send_request(request);
        _loadNodesPath->clear();
    }

    void MapNodeLabellingPanel::AddNode() {
        auto request = std::make_shared<roomba_msgs::srv::AddMapNode::Request>();
        request->node_name.data = std::to_string(_mapNodes.size() + 1);
        _addMapNodeClient->async_send_request(request, std::bind(&MapNodeLabellingPanel::saveMapNode,
                                                                               this,
                                                                               std::placeholders::_1));
    }

    void MapNodeLabellingPanel::saveMapNode(rclcpp::Client<roomba_msgs::srv::AddMapNode>::SharedFuture future) {
        roomba_msgs::msg::MultifloorPoint node;
        node.point.x = future.get()->node_pos.point.x;
        node.point.y = future.get()->node_pos.point.y;
        node.floor_id = future.get()->node_pos.floor_id;
        node.label.data = std::to_string(_mapNodes.size() + 1);
        _mapNodes.push_back(node);
    }


} // namespace roomba_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(roomba_rviz_plugins::MapNodeLabellingPanel, rviz_common::Panel)