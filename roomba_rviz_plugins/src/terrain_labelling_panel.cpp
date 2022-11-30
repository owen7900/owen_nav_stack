//
// Created by kevin on 10/26/22.
//

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
#include "roomba_rviz_plugins/terrain_labelling_panel.hpp"

namespace roomba_rviz_plugins{
    TerrainLabelling::TerrainLabelling(QWidget *parent)
    : Panel(parent){
        _verticalBox = new QVBoxLayout();
        _hbox1 = new QHBoxLayout();
        _hbox2 = new QHBoxLayout();
        _hbox3 = new QHBoxLayout();
        _hbox4 = new QHBoxLayout();

        _line1 = new QFrame();
        _line1->setFrameShape(QFrame::HLine);
        _line1->setFrameShadow(QFrame::Sunken);

        _line2 = new QFrame();
        _line2->setFrameShape(QFrame::HLine);
        _line2->setFrameShadow(QFrame::Sunken);

        _line3 = new QFrame();
        _line3->setFrameShape(QFrame::HLine);
        _line3->setFrameShadow(QFrame::Sunken);

        _label = new QLineEdit();
        _label->setPlaceholderText("Label name");

        _addLabelButton = new QPushButton(this);
        _addLabelButton->setText("Add label");
        connect(_addLabelButton, SIGNAL(clicked()), this, SLOT(AddLabel()));

        _loadConfigPath = new QLineEdit();
        _loadConfigPath->setPlaceholderText("Existing config file path");

        _loadConfigButton = new QPushButton(this);
        _loadConfigButton->setText("Load existing config");
        connect(_loadConfigButton, SIGNAL(clicked()), this, SLOT(LoadConfig()));

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

        _hbox3->addWidget(_loadConfigPath);
        _hbox3->addWidget(_loadConfigButton);

        _hbox4->addWidget(_outputPath);
        _hbox4->addWidget(_saveButton);
        _hbox4->addWidget(_clearButton);

        _verticalBox->addLayout(_hbox1);
        _verticalBox->addWidget(_line1);
        _verticalBox->addLayout(_hbox2);
        _verticalBox->addWidget(_line2);
        _verticalBox->addLayout(_hbox3);
        _verticalBox->addWidget(_line3);
        _verticalBox->addLayout(_hbox4);

        setLayout(_verticalBox);

        _obstacleRB->setChecked(true);
    }

    void TerrainLabelling::onInitialize() {
        auto context = this->getDisplayContext();
        auto lock = context->getRosNodeAbstraction().lock();
        auto raw_node = lock->get_raw_node();

        _labelTypeClient = raw_node->create_client<roomba_msgs::srv::SwitchLabellingType>("switchLabelType");
        _clearLabelsClient = raw_node->create_client<std_srvs::srv::Empty>("clearLabels");
        _loadConfigClient = raw_node->create_client<roomba_msgs::srv::LoadConfig>("loadConfig");
        _addLabelClient = raw_node->create_client<roomba_msgs::srv::AddLabel>("addLabel");

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

        auto request = std::make_shared<roomba_msgs::srv::SwitchLabellingType::Request>();
        request->type.data = _currentLabelType;
        _labelTypeClient->async_send_request(request);
    }

    void TerrainLabelling::AddLabel() {
        auto request = std::make_shared<roomba_msgs::srv::AddLabel::Request>();
        request->label.data = _label->text().toStdString();
        _addLabelClient->async_send_request(request, std::bind(&TerrainLabelling::saveLabel,
                                                               this,
                                                               std::placeholders::_1));
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
            node["obstacles"][obs.label.data]["x1"] = obs.p1.x;
            node["obstacles"][obs.label.data]["y1"] = obs.p1.y;
            node["obstacles"][obs.label.data]["x2"] = obs.p2.x;
            node["obstacles"][obs.label.data]["y2"] = obs.p2.y;
            node["obstacles"][obs.label.data]["x3"] = obs.p3.y;
            node["obstacles"][obs.label.data]["y3"] = obs.p3.y;
            node["obstacles"][obs.label.data]["x4"] = obs.p4.y;
            node["obstacles"][obs.label.data]["y4"] = obs.p4.y;
            node["obstacles"][obs.label.data]["floor"] = obs.floor_id.data;
        }

        node["features"];
        for(const auto &feat : _features){
            node["features"][feat.label.data]["x1"] = feat.p1.x;
            node["features"][feat.label.data]["y1"] = feat.p1.y;
            node["features"][feat.label.data]["x2"] = feat.p2.x;
            node["features"][feat.label.data]["y2"] = feat.p2.y;
            node["features"][feat.label.data]["x3"] = feat.p3.x;
            node["features"][feat.label.data]["y3"] = feat.p3.y;
            node["features"][feat.label.data]["x4"] = feat.p4.x;
            node["features"][feat.label.data]["y4"] = feat.p4.y;
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

    void TerrainLabelling::LoadConfig() {
        ClearFeatures();
        auto request = std::make_shared<roomba_msgs::srv::LoadConfig::Request>();
        request->path.data = _loadConfigPath->text().toStdString();
        _loadConfigClient->async_send_request(request);
        _loadConfigPath->clear();
    }

    void TerrainLabelling::saveLabel(rclcpp::Client<roomba_msgs::srv::AddLabel>::SharedFuture future) {
        auto points = future.get()->points;
        if(points.size() == 1){
            _destinations.push_back(points.back());
        } else{
            roomba_msgs::msg::MultifloorRectangle rect;
            rect.label = points[0].label;
            rect.floor_id = points[0].floor_id;
            rect.p1 = points[0].point;
            rect.p2 = points[1].point;
            rect.p3 = points[2].point;
            rect.p4 = points[3].point;
            if(_currentLabelType == CONSTANTS::OBSTACLE)
                _obstacles.push_back(rect);
            if(_currentLabelType == CONSTANTS::FEATURE)
                _features.push_back(rect);
        }
    }

} // namespace roomba_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(roomba_rviz_plugins::TerrainLabelling, rviz_common::Panel)
