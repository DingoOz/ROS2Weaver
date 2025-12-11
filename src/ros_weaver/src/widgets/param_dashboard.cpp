#include "ros_weaver/widgets/param_dashboard.hpp"
#include "ros_weaver/canvas/package_block.hpp"

#include <QHeaderView>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>
#include <QMessageBox>
#include <QTextStream>

namespace ros_weaver {

ParamDashboard::ParamDashboard(QWidget* parent)
  : QWidget(parent)
  , paramTree_(nullptr)
  , searchEdit_(nullptr)
  , addButton_(nullptr)
  , removeButton_(nullptr)
  , resetButton_(nullptr)
  , nodeNameLabel_(nullptr)
  , currentBlock_(nullptr)
  , updatingTree_(false)
{
  setupUi();
}

ParamDashboard::~ParamDashboard() = default;

void ParamDashboard::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(4, 4, 4, 4);
  mainLayout->setSpacing(4);

  // Node name label
  nodeNameLabel_ = new QLabel(tr("No node selected"));
  nodeNameLabel_->setStyleSheet("font-weight: bold; padding: 4px;");
  mainLayout->addWidget(nodeNameLabel_);

  // Search bar
  QHBoxLayout* searchLayout = new QHBoxLayout();
  searchEdit_ = new QLineEdit();
  searchEdit_->setPlaceholderText(tr("Search parameters..."));
  searchEdit_->setClearButtonEnabled(true);
  connect(searchEdit_, &QLineEdit::textChanged, this, [this](const QString& text) {
    for (int i = 0; i < paramTree_->topLevelItemCount(); ++i) {
      QTreeWidgetItem* item = paramTree_->topLevelItem(i);
      bool matches = text.isEmpty() ||
                     item->text(0).contains(text, Qt::CaseInsensitive);
      item->setHidden(!matches);
    }
  });
  searchLayout->addWidget(searchEdit_);
  mainLayout->addLayout(searchLayout);

  // Parameter tree
  paramTree_ = new QTreeWidget();
  paramTree_->setHeaderLabels({tr("Parameter"), tr("Value"), tr("Type")});
  paramTree_->setColumnCount(3);
  paramTree_->header()->setStretchLastSection(false);
  paramTree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
  paramTree_->header()->setSectionResizeMode(1, QHeaderView::Stretch);
  paramTree_->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  paramTree_->setAlternatingRowColors(true);
  paramTree_->setRootIsDecorated(false);
  connect(paramTree_, &QTreeWidget::itemChanged,
          this, &ParamDashboard::onItemChanged);
  mainLayout->addWidget(paramTree_);

  // Buttons
  QHBoxLayout* buttonLayout = new QHBoxLayout();

  addButton_ = new QPushButton(tr("Add"));
  addButton_->setToolTip(tr("Add a new parameter"));
  connect(addButton_, &QPushButton::clicked, this, &ParamDashboard::onAddParameter);
  buttonLayout->addWidget(addButton_);

  removeButton_ = new QPushButton(tr("Remove"));
  removeButton_->setToolTip(tr("Remove selected parameter"));
  connect(removeButton_, &QPushButton::clicked, this, &ParamDashboard::onRemoveParameter);
  buttonLayout->addWidget(removeButton_);

  buttonLayout->addStretch();

  resetButton_ = new QPushButton(tr("Reset"));
  resetButton_->setToolTip(tr("Reset all parameters to defaults"));
  connect(resetButton_, &QPushButton::clicked, this, &ParamDashboard::onResetToDefaults);
  buttonLayout->addWidget(resetButton_);

  mainLayout->addLayout(buttonLayout);
}

void ParamDashboard::setCurrentBlock(PackageBlock* block) {
  currentBlock_ = block;

  if (!block) {
    nodeNameLabel_->setText(tr("No node selected"));
    params_.clear();
    populateTree();
    return;
  }

  nodeNameLabel_->setText(block->packageName());

  // Load default parameters based on node type
  loadDefaultsForNodeType(block->packageName());
  populateTree();
}

void ParamDashboard::loadDefaultsForNodeType(const QString& nodeType) {
  params_.clear();

  // Define default parameters for common node types
  if (nodeType == "publisher_node") {
    params_ = {
      {"topic_name", "string", "/chatter", "/chatter", "Topic to publish to", {}, {}, {}},
      {"publish_rate", "double", 10.0, 10.0, "Publishing rate in Hz", 0.1, 1000.0, {}},
      {"queue_size", "int", 10, 10, "Message queue size", 1, 1000, {}},
      {"use_sim_time", "bool", false, false, "Use simulation time", {}, {}, {}},
    };
  }
  else if (nodeType == "subscriber_node") {
    params_ = {
      {"topic_name", "string", "/chatter", "/chatter", "Topic to subscribe to", {}, {}, {}},
      {"queue_size", "int", 10, 10, "Message queue size", 1, 1000, {}},
      {"use_sim_time", "bool", false, false, "Use simulation time", {}, {}, {}},
    };
  }
  else if (nodeType == "slam_toolbox") {
    params_ = {
      {"base_frame", "string", "base_link", "base_link", "Base frame of the robot", {}, {}, {}},
      {"odom_frame", "string", "odom", "odom", "Odometry frame", {}, {}, {}},
      {"map_frame", "string", "map", "map", "Map frame", {}, {}, {}},
      {"resolution", "double", 0.05, 0.05, "Map resolution in meters", 0.01, 1.0, {}},
      {"max_laser_range", "double", 20.0, 20.0, "Maximum laser range", 1.0, 100.0, {}},
      {"minimum_travel_distance", "double", 0.5, 0.5, "Minimum travel distance before processing", 0.0, 10.0, {}},
      {"minimum_travel_heading", "double", 0.5, 0.5, "Minimum heading change before processing", 0.0, 3.14, {}},
      {"map_update_interval", "double", 5.0, 5.0, "Map update interval in seconds", 0.1, 60.0, {}},
      {"use_sim_time", "bool", false, false, "Use simulation time", {}, {}, {}},
    };
  }
  else if (nodeType == "nav2_planner") {
    params_ = {
      {"planner_plugin", "string", "GridBased", "GridBased", "Planner plugin to use", {}, {}, {"GridBased", "SmacPlanner", "ThetaStar"}},
      {"expected_planner_frequency", "double", 1.0, 1.0, "Expected planning frequency", 0.1, 10.0, {}},
      {"use_sim_time", "bool", false, false, "Use simulation time", {}, {}, {}},
    };
  }
  else if (nodeType == "nav2_controller") {
    params_ = {
      {"controller_plugin", "string", "DWB", "DWB", "Controller plugin to use", {}, {}, {"DWB", "TEB", "RegulatedPurePursuit"}},
      {"controller_frequency", "double", 20.0, 20.0, "Controller frequency in Hz", 1.0, 100.0, {}},
      {"min_vel_x", "double", 0.0, 0.0, "Minimum x velocity", -1.0, 1.0, {}},
      {"max_vel_x", "double", 0.5, 0.5, "Maximum x velocity", 0.0, 5.0, {}},
      {"max_vel_theta", "double", 1.0, 1.0, "Maximum angular velocity", 0.0, 10.0, {}},
      {"acc_lim_x", "double", 2.5, 2.5, "X acceleration limit", 0.0, 10.0, {}},
      {"acc_lim_theta", "double", 3.2, 3.2, "Angular acceleration limit", 0.0, 10.0, {}},
      {"use_sim_time", "bool", false, false, "Use simulation time", {}, {}, {}},
    };
  }
  else if (nodeType == "lidar_sensor") {
    params_ = {
      {"frame_id", "string", "laser_frame", "laser_frame", "TF frame for the sensor", {}, {}, {}},
      {"angle_min", "double", -3.14159, -3.14159, "Start angle of scan (radians)", -3.14159, 3.14159, {}},
      {"angle_max", "double", 3.14159, 3.14159, "End angle of scan (radians)", -3.14159, 3.14159, {}},
      {"range_min", "double", 0.1, 0.1, "Minimum range value (meters)", 0.0, 10.0, {}},
      {"range_max", "double", 30.0, 30.0, "Maximum range value (meters)", 1.0, 100.0, {}},
      {"scan_frequency", "double", 10.0, 10.0, "Scan frequency in Hz", 1.0, 100.0, {}},
    };
  }
  else if (nodeType == "diff_drive" || nodeType == "robot_controller") {
    params_ = {
      {"wheel_radius", "double", 0.033, 0.033, "Wheel radius in meters", 0.01, 1.0, {}},
      {"wheel_separation", "double", 0.16, 0.16, "Distance between wheels in meters", 0.05, 2.0, {}},
      {"max_linear_velocity", "double", 0.22, 0.22, "Maximum linear velocity (m/s)", 0.0, 5.0, {}},
      {"max_angular_velocity", "double", 2.84, 2.84, "Maximum angular velocity (rad/s)", 0.0, 10.0, {}},
      {"publish_rate", "double", 30.0, 30.0, "Odometry publish rate in Hz", 1.0, 100.0, {}},
      {"use_sim_time", "bool", false, false, "Use simulation time", {}, {}, {}},
    };
  }
  else if (nodeType == "image_processor") {
    params_ = {
      {"image_transport", "string", "raw", "raw", "Image transport type", {}, {}, {"raw", "compressed", "theora"}},
      {"queue_size", "int", 5, 5, "Message queue size", 1, 100, {}},
      {"detection_threshold", "double", 0.5, 0.5, "Detection confidence threshold", 0.0, 1.0, {}},
      {"use_gpu", "bool", false, false, "Use GPU acceleration", {}, {}, {}},
    };
  }
  else {
    // Generic node defaults
    params_ = {
      {"use_sim_time", "bool", false, false, "Use simulation time", {}, {}, {}},
    };
  }
}

void ParamDashboard::populateTree() {
  updatingTree_ = true;
  paramTree_->clear();

  for (const auto& param : params_) {
    addParamToTree(param);
  }

  updatingTree_ = false;
}

void ParamDashboard::addParamToTree(const ParamDefinition& param) {
  QTreeWidgetItem* item = createParamItem(param);
  paramTree_->addTopLevelItem(item);

  // Create custom editor widget for the value column
  QWidget* editor = createValueEditor(param, item);
  if (editor) {
    paramTree_->setItemWidget(item, 1, editor);
  }
}

QTreeWidgetItem* ParamDashboard::createParamItem(const ParamDefinition& param) {
  QTreeWidgetItem* item = new QTreeWidgetItem();
  item->setText(0, param.name);
  item->setToolTip(0, param.description);
  item->setText(2, param.type);
  item->setData(0, Qt::UserRole, param.name);

  return item;
}

QWidget* ParamDashboard::createValueEditor(const ParamDefinition& param, QTreeWidgetItem* item) {
  if (param.type == "bool") {
    QCheckBox* checkbox = new QCheckBox();
    checkbox->setChecked(param.currentValue.toBool());
    connect(checkbox, &QCheckBox::toggled, this, [this, item](bool checked) {
      QString paramName = item->data(0, Qt::UserRole).toString();
      for (auto& p : params_) {
        if (p.name == paramName) {
          p.currentValue = checked;
          break;
        }
      }
      emit parameterChanged(paramName, checked);
      emit parametersModified();
    });
    return checkbox;
  }
  else if (param.type == "int") {
    QSpinBox* spinbox = new QSpinBox();
    spinbox->setRange(param.minValue.isValid() ? param.minValue.toInt() : -999999,
                      param.maxValue.isValid() ? param.maxValue.toInt() : 999999);
    spinbox->setValue(param.currentValue.toInt());
    connect(spinbox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, [this, item](int value) {
      QString paramName = item->data(0, Qt::UserRole).toString();
      for (auto& p : params_) {
        if (p.name == paramName) {
          p.currentValue = value;
          break;
        }
      }
      emit parameterChanged(paramName, value);
      emit parametersModified();
    });
    return spinbox;
  }
  else if (param.type == "double") {
    QDoubleSpinBox* spinbox = new QDoubleSpinBox();
    spinbox->setDecimals(4);
    spinbox->setRange(param.minValue.isValid() ? param.minValue.toDouble() : -999999.0,
                      param.maxValue.isValid() ? param.maxValue.toDouble() : 999999.0);
    spinbox->setValue(param.currentValue.toDouble());
    spinbox->setSingleStep(0.1);
    connect(spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this, item](double value) {
      QString paramName = item->data(0, Qt::UserRole).toString();
      for (auto& p : params_) {
        if (p.name == paramName) {
          p.currentValue = value;
          break;
        }
      }
      emit parameterChanged(paramName, value);
      emit parametersModified();
    });
    return spinbox;
  }
  else if (param.type == "string") {
    if (!param.enumValues.isEmpty()) {
      // Use combo box for enum values
      QComboBox* combo = new QComboBox();
      combo->addItems(param.enumValues);
      combo->setCurrentText(param.currentValue.toString());
      connect(combo, &QComboBox::currentTextChanged,
              this, [this, item](const QString& value) {
        QString paramName = item->data(0, Qt::UserRole).toString();
        for (auto& p : params_) {
          if (p.name == paramName) {
            p.currentValue = value;
            break;
          }
        }
        emit parameterChanged(paramName, value);
        emit parametersModified();
      });
      return combo;
    } else {
      QLineEdit* edit = new QLineEdit();
      edit->setText(param.currentValue.toString());
      connect(edit, &QLineEdit::textChanged,
              this, [this, item](const QString& value) {
        QString paramName = item->data(0, Qt::UserRole).toString();
        for (auto& p : params_) {
          if (p.name == paramName) {
            p.currentValue = value;
            break;
          }
        }
        emit parameterChanged(paramName, value);
        emit parametersModified();
      });
      return edit;
    }
  }

  return nullptr;
}

void ParamDashboard::onItemChanged(QTreeWidgetItem* item, int column) {
  if (updatingTree_) return;
  Q_UNUSED(item)
  Q_UNUSED(column)
}

void ParamDashboard::onAddParameter() {
  // Add a new parameter with default values
  ParamDefinition newParam;
  newParam.name = QString("param_%1").arg(params_.size());
  newParam.type = "string";
  newParam.defaultValue = "";
  newParam.currentValue = "";
  newParam.description = "New parameter";

  params_.append(newParam);
  addParamToTree(newParam);
  emit parametersModified();
}

void ParamDashboard::onRemoveParameter() {
  QTreeWidgetItem* current = paramTree_->currentItem();
  if (!current) return;

  QString paramName = current->data(0, Qt::UserRole).toString();

  // Remove from params list
  for (int i = 0; i < params_.size(); ++i) {
    if (params_[i].name == paramName) {
      params_.removeAt(i);
      break;
    }
  }

  // Remove from tree
  delete current;
  emit parametersModified();
}

void ParamDashboard::onResetToDefaults() {
  for (auto& param : params_) {
    param.currentValue = param.defaultValue;
  }
  populateTree();
  emit parametersModified();
}

QList<ParamDefinition> ParamDashboard::parameters() const {
  return params_;
}

void ParamDashboard::setParameters(const QList<ParamDefinition>& params) {
  params_ = params;
  populateTree();
}

QString ParamDashboard::toYaml() const {
  QString yaml;
  QTextStream stream(&yaml);

  if (currentBlock_) {
    stream << currentBlock_->packageName() << ":\n";
    stream << "  ros__parameters:\n";

    for (const auto& param : params_) {
      stream << "    " << param.name << ": ";

      if (param.type == "string") {
        stream << "\"" << param.currentValue.toString() << "\"";
      } else if (param.type == "bool") {
        stream << (param.currentValue.toBool() ? "true" : "false");
      } else {
        stream << param.currentValue.toString();
      }
      stream << "\n";
    }
  }

  return yaml;
}

bool ParamDashboard::fromYaml(const QString& yaml) {
  // Simple YAML parsing - for production use yaml-cpp
  Q_UNUSED(yaml)
  // TODO: Implement YAML parsing
  return false;
}

}  // namespace ros_weaver
