//
// Created by kevin on 10/26/22.
//

#include "../include/roomba_rviz_plugins/terrain_labelling_panel.hpp"

#include <QPushButton>
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QtGui>
#include <QLabel>
#include <QFrame>

namespace roomba_rviz_plugins{
    TerrainLabelling::TerrainLabelling(QWidget *parent)
    : Panel(parent){
        _verticalBox = new QVBoxLayout();
        _hbox1 = new QHBoxLayout();
        _hbox2 = new QHBoxLayout();

        _line = new QFrame();
        _line->setFrameShape(QFrame::HLine);
        _line->setFrameShadow(QFrame::Sunken);

        _saveButton = new QPushButton(this);
        _saveButton->setText("Save labels");
        connect(_saveButton, SIGNAL(clicked()), this, SLOT(SaveFeatures()));

        _clearButton = new QPushButton(this);
        _clearButton->setText("Clear labels");
        connect(_saveButton, SIGNAL(clicked()), this, SLOT(ClearFeatures()));

        _terrainLabel = new QLabel(this);
        _terrainLabel->setText("Select terrain label: ");
        _terrainLabel->setAlignment(Qt::AlignCenter);

        _passableRB = new QRadioButton(tr("Passable"));
        _impassableRB = new QRadioButton(tr("Thou shall not pass"));

        connect(_passableRB, SIGNAL(clicked()), this, SLOT(SwitchTerrainLabel(true)));
        connect(_impassableRB, SIGNAL(clicked()), this, SLOT(SwitchTerrainLabel(false)));

        _hbox1->addWidget(_terrainLabel);
        _hbox1->addWidget(_passableRB);
        _hbox1->addWidget(_impassableRB);

        _hbox2->addWidget(_saveButton);
        _hbox2->addWidget(_clearButton);

        _verticalBox->addLayout(_hbox1);
        _verticalBox->addWidget(_line);
        _verticalBox->addLayout(_hbox2);

        setLayout(_verticalBox);
    }

    TerrainLabelling::~TerrainLabelling() noexcept {

    }

    void TerrainLabelling::SwitchTerrainLabel(bool passable) {

    }

    void TerrainLabelling::SaveFeatures() {

    }

    void TerrainLabelling::ClearFeatures() {

    }

} // namespace roomba_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(roomba_rviz_plugins::TerrainLabelling, rviz_common::Panel)
