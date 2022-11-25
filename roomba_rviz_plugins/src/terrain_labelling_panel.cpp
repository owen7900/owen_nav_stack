//
// Created by kevin on 10/26/22.
//

#include "../include/roomba_rviz_plugins/terrain_labelling_panel.hpp"

#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QLabel>
#include <QFrame>

#include <fstream>
#include <string>
#include <yaml-cpp/yaml.h>

#include "roomba_rviz_plugins/constants.hpp"

#include "rviz_common/display_context.hpp"

namespace roomba_rviz_plugins{
    TerrainLabelling::TerrainLabelling(QWidget *parent)
    : Panel(parent){
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

        _label = new QLineEdit();
        _label->setPlaceholderText("Label name");

        _addLabelButton = new QPushButton(this);
        _addLabelButton->setText("Add label");
        connect(_addLabelButton, SIGNAL(clicked()), this, SLOT(AddLabel()));

        _outputPath = new QLineEdit();
        _outputPath->setPlaceholderText("Output file path");

        _saveButton = new QPushButton(this);
        _saveButton->setText("Save labels to file");
        connect(_saveButton, SIGNAL(clicked()), this, SLOT(SaveFeatures()));

        _clearButton = new QPushButton(this);
        _clearButton->setText("Clear all labels");
        connect(_clearButton, SIGNAL(clicked()), this, SLOT(ClearFeatures()));

        _labelType = new QLabel(this);
        _labelType->setText("Select label type: ");
        _labelType->setAlignment(Qt::AlignCenter);

        _obstacleRB = new QRadioButton(tr("Obstacle"));
        _featureRB = new QRadioButton(tr("Feature"));
        _destinationRB= new QRadioButton(tr("Destination"));

        connect(_obstacleRB, SIGNAL(clicked()), this, SLOT(SwitchLabelType()));
        connect(_featureRB, SIGNAL(clicked()), this, SLOT(SwitchLabelType()));
        connect(_destinationRB, SIGNAL(clicked()), this, SLOT(SwitchLabelType()));

        _hbox1->addWidget(_labelType);
        _hbox1->addWidget(_obstacleRB);
        _hbox1->addWidget(_featureRB);
        _hbox1->addWidget(_destinationRB);

        _hbox2->addWidget(_label);
        _hbox2->addWidget(_addLabelButton);

        _hbox3->addWidget(_outputPath);
        _hbox3->addWidget(_saveButton);
        _hbox3->addWidget(_clearButton);

        _verticalBox->addLayout(_hbox1);
        _verticalBox->addWidget(_line1);
        _verticalBox->addLayout(_hbox2);
        _verticalBox->addWidget(_line2);
        _verticalBox->addLayout(_hbox3);

        setLayout(_verticalBox);

        _obstacleRB->setChecked(true);
    }

    void TerrainLabelling::onInitialize() {
        auto context = this->getDisplayContext();
        auto lock = context->getRosNodeAbstraction().lock();
        auto raw_node = lock->get_raw_node();

        _labelTypePub = raw_node->create_publisher<std_msgs::msg::Int64>("/labelType", 10);
        _labelNamePub = raw_node->create_publisher<std_msgs::msg::String>("/labelName", 10);

        _obsSub = raw_node->create_subscription<roomba_msgs::msg::MultifloorRectangle>("/obstacles",
                                                                            10,
                                                                            std::bind(
                                                                                    &TerrainLabelling::obstacleCallback,
                                                                                    this, std::placeholders::_1));

        _featSub = raw_node->create_subscription<roomba_msgs::msg::MultifloorRectangle>("/features",
                                                                                      10,
                                                                                      std::bind(
                                                                                              &TerrainLabelling::featureCallback,
                                                                                              this, std::placeholders::_1));

        _destSub = raw_node->create_subscription<roomba_msgs::msg::MultifloorPoint>("/destinations",
                                                                                      10,
                                                                                      std::bind(
                                                                                              &TerrainLabelling::destinationCallback,
                                                                                              this, std::placeholders::_1));

        _clearLabelsClient = raw_node->create_client<std_srvs::srv::Empty>("clearLabels");
    }

    TerrainLabelling::~TerrainLabelling() noexcept {}

    void TerrainLabelling::obstacleCallback(roomba_msgs::msg::MultifloorRectangle msg) {
        _obstacles.push_back(msg);
    }

    void TerrainLabelling::featureCallback(roomba_msgs::msg::MultifloorRectangle msg) {
        _features.push_back(msg);
    }

    void TerrainLabelling::destinationCallback(roomba_msgs::msg::MultifloorPoint msg) {
        _destinations.push_back(msg);
    }

    void TerrainLabelling::SwitchLabelType(){
        if(_obstacleRB->isChecked()){
            _currentLabelType = CONSTANTS::OBSTACLE;
        }
        if(_featureRB->isChecked()){
            _currentLabelType = CONSTANTS::FEATURE;
        }
        if(_destinationRB->isChecked()){
            _currentLabelType = CONSTANTS::DESTINATION;
        }

        std_msgs::msg::Int64 msg = std_msgs::msg::Int64();
        msg.data = _currentLabelType;
        _labelTypePub->publish(msg);
    }

    void TerrainLabelling::AddLabel() {
        auto msg = std_msgs::msg::String();
        msg.data = _label->text().toStdString();
        _labelNamePub->publish(msg);
        _label->clear();
    }

    void TerrainLabelling::SaveFeatures() {
        auto outputPath = _outputPath->text().toStdString();
        YAML::Node node;

        node["no_pass"] = std::vector<std::string>();
        node["obstacles"];
        for(const auto &obs : _obstacles){
            node["no pass"].push_back(obs.label.data);
            node["obstacles"][obs.label.data];
            node["obstacles"][obs.label.data]["xmin"] = obs.p1.x;
            node["obstacles"][obs.label.data]["ymin"] = obs.p1.y;
            node["obstacles"][obs.label.data]["xmax"] = obs.p2.x;
            node["obstacles"][obs.label.data]["ymax"] = obs.p2.y;
            node["obstacles"][obs.label.data]["floor"] = obs.floor_id.data;
        }

        node["features"];
        for(const auto &feat : _features){
            node["features"][feat.label.data];
            node["features"][feat.label.data]["xmin"] = feat.p1.x;
            node["features"][feat.label.data]["ymin"] = feat.p1.y;
            node["features"][feat.label.data]["xmax"] = feat.p2.x;
            node["features"][feat.label.data]["ymax"] = feat.p2.y;
            node["features"][feat.label.data]["floor"] = feat.floor_id.data;
        }

        node["destinations"];
        for(const auto &dest : _destinations){
            node["destinations"][dest.label.data];
            node["destinations"][dest.label.data]["x"] = dest.point.x;
            node["destinations"][dest.label.data]["y"] = dest.point.y;
            node["destinations"][dest.label.data]["floor"] = dest.floor_id.data;
        }

        const auto output = YAML::Dump(node);
        std::ofstream out(outputPath);
        out << output;
        out.close();

        _outputPath->clear();
    }

    void TerrainLabelling::ClearFeatures() {
        _obstacles.clear();
        _features.clear();
        _destinations.clear();

        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        _clearLabelsClient->async_send_request(request);
    }

} // namespace roomba_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(roomba_rviz_plugins::TerrainLabelling, rviz_common::Panel)
