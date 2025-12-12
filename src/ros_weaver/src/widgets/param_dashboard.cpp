#include "ros_weaver/widgets/param_dashboard.hpp"
#include "ros_weaver/canvas/package_block.hpp"
#include "ros_weaver/core/project.hpp"

#include <QHeaderView>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>
#include <QMessageBox>
#include <QTextStream>
#include <QFileDialog>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QInputDialog>
#include <QMenu>
#include <QAction>

#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

namespace ros_weaver {

ParamDashboard::ParamDashboard(QWidget* parent)
  : QWidget(parent)
  , paramTree_(nullptr)
  , searchEdit_(nullptr)
  , addButton_(nullptr)
  , removeButton_(nullptr)
  , resetButton_(nullptr)
  , importExportButton_(nullptr)
  , groupButton_(nullptr)
  , nodeNameLabel_(nullptr)
  , validationLabel_(nullptr)
  , yamlSourceCombo_(nullptr)
  , sourceLabel_(nullptr)
  , syncToBlockButton_(nullptr)
  , currentBlock_(nullptr)
  , updatingTree_(false)
  , showingYamlParams_(false)
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

  // Validation status label
  validationLabel_ = new QLabel();
  validationLabel_->setVisible(false);
  validationLabel_->setStyleSheet("color: #d9534f; padding: 2px 4px; background: #f2dede; border-radius: 2px;");
  mainLayout->addWidget(validationLabel_);

  // YAML source selector row
  QHBoxLayout* sourceLayout = new QHBoxLayout();
  sourceLabel_ = new QLabel(tr("Source:"));
  sourceLayout->addWidget(sourceLabel_);

  yamlSourceCombo_ = new QComboBox();
  yamlSourceCombo_->setToolTip(tr("Select parameter source: block parameters or YAML file"));
  yamlSourceCombo_->addItem(tr("Block Parameters"), QVariant());  // Default option
  connect(yamlSourceCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &ParamDashboard::onYamlSourceChanged);
  sourceLayout->addWidget(yamlSourceCombo_, 1);

  syncToBlockButton_ = new QPushButton(tr("Sync to Block"));
  syncToBlockButton_->setToolTip(tr("Copy current YAML parameters to the block"));
  syncToBlockButton_->setVisible(false);  // Only show when viewing YAML
  connect(syncToBlockButton_, &QPushButton::clicked, this, &ParamDashboard::onSyncToBlock);
  sourceLayout->addWidget(syncToBlockButton_);

  mainLayout->addLayout(sourceLayout);

  // Search bar
  QHBoxLayout* searchLayout = new QHBoxLayout();
  searchEdit_ = new QLineEdit();
  searchEdit_->setPlaceholderText(tr("Search parameters..."));
  searchEdit_->setClearButtonEnabled(true);
  connect(searchEdit_, &QLineEdit::textChanged, this, [this](const QString& text) {
    std::function<void(QTreeWidgetItem*, bool)> filterItem = [&](QTreeWidgetItem* item, bool parentVisible) {
      bool matches = text.isEmpty() ||
                     item->text(0).contains(text, Qt::CaseInsensitive);

      // Check children
      bool hasVisibleChild = false;
      for (int i = 0; i < item->childCount(); ++i) {
        filterItem(item->child(i), matches || parentVisible);
        if (!item->child(i)->isHidden()) {
          hasVisibleChild = true;
        }
      }

      // Show item if it matches, parent is visible, or has visible children
      item->setHidden(!matches && !parentVisible && !hasVisibleChild);

      // Expand items with visible children
      if (hasVisibleChild && !text.isEmpty()) {
        item->setExpanded(true);
      }
    };

    for (int i = 0; i < paramTree_->topLevelItemCount(); ++i) {
      filterItem(paramTree_->topLevelItem(i), false);
    }
  });
  searchLayout->addWidget(searchEdit_);
  mainLayout->addLayout(searchLayout);

  // Parameter tree
  paramTree_ = new QTreeWidget();
  paramTree_->setHeaderLabels({tr("Parameter"), tr("Value"), tr("Type")});
  paramTree_->setColumnCount(3);
  paramTree_->header()->setStretchLastSection(false);
  // Allow user to resize columns by dragging edges
  paramTree_->header()->setSectionResizeMode(0, QHeaderView::Interactive);
  paramTree_->header()->setSectionResizeMode(1, QHeaderView::Interactive);
  paramTree_->header()->setSectionResizeMode(2, QHeaderView::Interactive);
  // Set reasonable default widths
  paramTree_->setColumnWidth(0, 150);
  paramTree_->setColumnWidth(1, 150);
  paramTree_->setColumnWidth(2, 60);
  paramTree_->setAlternatingRowColors(true);
  paramTree_->setRootIsDecorated(true);
  paramTree_->setAnimated(true);
  connect(paramTree_, &QTreeWidget::itemChanged,
          this, &ParamDashboard::onItemChanged);
  mainLayout->addWidget(paramTree_);

  // Buttons row 1: Add/Remove/Group
  QHBoxLayout* buttonLayout1 = new QHBoxLayout();

  addButton_ = new QPushButton(tr("Add"));
  addButton_->setToolTip(tr("Add a new parameter"));
  connect(addButton_, &QPushButton::clicked, this, &ParamDashboard::onAddParameter);
  buttonLayout1->addWidget(addButton_);

  removeButton_ = new QPushButton(tr("Remove"));
  removeButton_->setToolTip(tr("Remove selected parameter"));
  connect(removeButton_, &QPushButton::clicked, this, &ParamDashboard::onRemoveParameter);
  buttonLayout1->addWidget(removeButton_);

  // Group button with menu
  groupButton_ = new QToolButton();
  groupButton_->setText(tr("Group"));
  groupButton_->setToolTip(tr("Add parameter group"));
  groupButton_->setPopupMode(QToolButton::InstantPopup);
  QMenu* groupMenu = new QMenu(groupButton_);
  groupMenu->addAction(tr("Add Group..."), this, &ParamDashboard::onAddGroup);
  groupMenu->addSeparator();
  groupMenu->addAction(tr("Expand All"), this, &ParamDashboard::onExpandAll);
  groupMenu->addAction(tr("Collapse All"), this, &ParamDashboard::onCollapseAll);
  groupButton_->setMenu(groupMenu);
  buttonLayout1->addWidget(groupButton_);

  buttonLayout1->addStretch();
  mainLayout->addLayout(buttonLayout1);

  // Buttons row 2: Reset/Import/Export
  QHBoxLayout* buttonLayout2 = new QHBoxLayout();

  resetButton_ = new QPushButton(tr("Reset"));
  resetButton_->setToolTip(tr("Reset all parameters to defaults"));
  connect(resetButton_, &QPushButton::clicked, this, &ParamDashboard::onResetToDefaults);
  buttonLayout2->addWidget(resetButton_);

  buttonLayout2->addStretch();

  // Import/Export button with menu
  importExportButton_ = new QToolButton();
  importExportButton_->setText(tr("YAML"));
  importExportButton_->setToolTip(tr("Import/Export YAML parameters"));
  importExportButton_->setPopupMode(QToolButton::InstantPopup);
  QMenu* yamlMenu = new QMenu(importExportButton_);
  yamlMenu->addAction(tr("Import from YAML..."), this, &ParamDashboard::onImportYaml);
  yamlMenu->addAction(tr("Export to YAML..."), this, &ParamDashboard::onExportYaml);
  importExportButton_->setMenu(yamlMenu);
  buttonLayout2->addWidget(importExportButton_);

  mainLayout->addLayout(buttonLayout2);

  // Auto-save parameters to block when modified
  connect(this, &ParamDashboard::parametersModified, this, &ParamDashboard::saveParametersToBlock);
}

void ParamDashboard::setCurrentBlock(PackageBlock* block) {
  // Save current parameters to the previous block before switching
  // Only save if we're viewing block params (not YAML file params)
  if (currentBlock_ && !params_.isEmpty() && !showingYamlParams_) {
    saveParametersToBlock();
  }

  currentBlock_ = block;
  groupItems_.clear();
  showingYamlParams_ = false;

  if (!block) {
    nodeNameLabel_->setText(tr("No node selected"));
    params_.clear();
    populateTree();
    validationLabel_->setVisible(false);
    syncToBlockButton_->setVisible(false);
    yamlSourceCombo_->setCurrentIndex(0);
    return;
  }

  nodeNameLabel_->setText(block->packageName());

  // Check for user-specified preferred source first
  QString preferredSource = block->preferredYamlSource();

  if (preferredSource == "block") {
    // User explicitly wants block parameters
    yamlSourceCombo_->blockSignals(true);
    yamlSourceCombo_->setCurrentIndex(0);
    yamlSourceCombo_->blockSignals(false);
    syncToBlockButton_->setVisible(false);
    loadBlockParamsDefault(block);
  } else if (!preferredSource.isEmpty()) {
    // User specified a YAML file
    int index = yamlSourceCombo_->findData(preferredSource);
    if (index >= 0) {
      yamlSourceCombo_->blockSignals(true);
      yamlSourceCombo_->setCurrentIndex(index);
      yamlSourceCombo_->blockSignals(false);
      showingYamlParams_ = true;
      syncToBlockButton_->setVisible(true);
      loadYamlFileParams(preferredSource);
    } else {
      // Preferred YAML not available, fall back to auto-detect
      selectYamlSourceAutoDetect(block);
    }
  } else {
    // Auto-detect: find YAML file matching this block
    selectYamlSourceAutoDetect(block);
  }

  populateTree();
  validateAll();
}

void ParamDashboard::loadBlockParamsDefault(PackageBlock* block) {
  // Load parameters from block if it has them, otherwise load defaults
  if (block->hasParameters()) {
    loadParametersFromBlock();
  } else {
    loadDefaultsForNodeType(block->packageName());
    // Save defaults to block
    saveParametersToBlock();
  }
}

void ParamDashboard::selectYamlSourceAutoDetect(PackageBlock* block) {
  // Auto-detect YAML file based on block name with flexible matching
  QString matchingYamlFile;
  QString blockName = block->packageName();

  // Build list of possible YAML node names for this block
  QStringList possibleNames;
  possibleNames << blockName;

  // Handle common Nav2 naming patterns (block name -> YAML node name)
  // e.g., nav2_controller -> controller_server, nav2_planner -> planner_server
  if (blockName.startsWith("nav2_")) {
    QString baseName = blockName.mid(5);  // Remove "nav2_" prefix
    possibleNames << baseName + "_server";  // e.g., controller -> controller_server
    possibleNames << baseName;               // e.g., controller
  }

  for (const YamlFileInfo& yamlFile : yamlFiles_) {
    for (const QString& possibleName : possibleNames) {
      if (yamlFile.nodeNames.contains(possibleName)) {
        matchingYamlFile = yamlFile.filePath;
        break;
      }
    }
    if (!matchingYamlFile.isEmpty()) break;
  }

  // Auto-select YAML file if one matches, otherwise use block params
  if (!matchingYamlFile.isEmpty()) {
    int index = yamlSourceCombo_->findData(matchingYamlFile);
    if (index >= 0) {
      yamlSourceCombo_->blockSignals(true);
      yamlSourceCombo_->setCurrentIndex(index);
      yamlSourceCombo_->blockSignals(false);
      showingYamlParams_ = true;
      syncToBlockButton_->setVisible(true);
      loadYamlFileParams(matchingYamlFile);
    } else {
      // Fallback to block params
      loadBlockParamsDefault(block);
    }
  } else {
    // No matching YAML file, use block params
    yamlSourceCombo_->blockSignals(true);
    yamlSourceCombo_->setCurrentIndex(0);
    yamlSourceCombo_->blockSignals(false);
    syncToBlockButton_->setVisible(false);
    loadBlockParamsDefault(block);
  }
}

void ParamDashboard::loadDefaultsForNodeType(const QString& nodeType) {
  params_.clear();

  // Define default parameters for common node types with groups
  if (nodeType == "publisher_node") {
    params_ = {
      {"topic_name", "string", "/chatter", "/chatter", "Topic to publish to", {}, {}, {}, "topics", true, ""},
      {"publish_rate", "double", 10.0, 10.0, "Publishing rate in Hz", 0.1, 1000.0, {}, "timing", true, ""},
      {"queue_size", "int", 10, 10, "Message queue size", 1, 1000, {}, "qos", true, ""},
      {"use_sim_time", "bool", false, false, "Use simulation time", {}, {}, {}, "timing", true, ""},
    };
  }
  else if (nodeType == "subscriber_node") {
    params_ = {
      {"topic_name", "string", "/chatter", "/chatter", "Topic to subscribe to", {}, {}, {}, "topics", true, ""},
      {"queue_size", "int", 10, 10, "Message queue size", 1, 1000, {}, "qos", true, ""},
      {"use_sim_time", "bool", false, false, "Use simulation time", {}, {}, {}, "timing", true, ""},
    };
  }
  else if (nodeType == "slam_toolbox") {
    params_ = {
      {"base_frame", "string", "base_link", "base_link", "Base frame of the robot", {}, {}, {}, "frames", true, ""},
      {"odom_frame", "string", "odom", "odom", "Odometry frame", {}, {}, {}, "frames", true, ""},
      {"map_frame", "string", "map", "map", "Map frame", {}, {}, {}, "frames", true, ""},
      {"resolution", "double", 0.05, 0.05, "Map resolution in meters", 0.01, 1.0, {}, "map", true, ""},
      {"max_laser_range", "double", 20.0, 20.0, "Maximum laser range", 1.0, 100.0, {}, "sensor", true, ""},
      {"minimum_travel_distance", "double", 0.5, 0.5, "Minimum travel distance before processing", 0.0, 10.0, {}, "processing", true, ""},
      {"minimum_travel_heading", "double", 0.5, 0.5, "Minimum heading change before processing", 0.0, 3.14, {}, "processing", true, ""},
      {"map_update_interval", "double", 5.0, 5.0, "Map update interval in seconds", 0.1, 60.0, {}, "timing", true, ""},
      {"use_sim_time", "bool", false, false, "Use simulation time", {}, {}, {}, "timing", true, ""},
    };
  }
  else if (nodeType == "nav2_planner") {
    params_ = {
      {"planner_plugin", "string", "GridBased", "GridBased", "Planner plugin to use", {}, {}, {"GridBased", "SmacPlanner", "ThetaStar"}, "planner", true, ""},
      {"expected_planner_frequency", "double", 1.0, 1.0, "Expected planning frequency", 0.1, 10.0, {}, "planner", true, ""},
      {"tolerance", "double", 0.5, 0.5, "Goal tolerance in meters", 0.01, 5.0, {}, "planner", true, ""},
      {"use_sim_time", "bool", false, false, "Use simulation time", {}, {}, {}, "timing", true, ""},
    };
  }
  else if (nodeType == "nav2_controller") {
    params_ = {
      {"controller_plugin", "string", "DWB", "DWB", "Controller plugin to use", {}, {}, {"DWB", "TEB", "RegulatedPurePursuit"}, "controller", true, ""},
      {"controller_frequency", "double", 20.0, 20.0, "Controller frequency in Hz", 1.0, 100.0, {}, "controller", true, ""},
      {"min_vel_x", "double", 0.0, 0.0, "Minimum x velocity", -1.0, 1.0, {}, "velocity", true, ""},
      {"max_vel_x", "double", 0.5, 0.5, "Maximum x velocity", 0.0, 5.0, {}, "velocity", true, ""},
      {"max_vel_theta", "double", 1.0, 1.0, "Maximum angular velocity", 0.0, 10.0, {}, "velocity", true, ""},
      {"acc_lim_x", "double", 2.5, 2.5, "X acceleration limit", 0.0, 10.0, {}, "acceleration", true, ""},
      {"acc_lim_theta", "double", 3.2, 3.2, "Angular acceleration limit", 0.0, 10.0, {}, "acceleration", true, ""},
      {"use_sim_time", "bool", false, false, "Use simulation time", {}, {}, {}, "timing", true, ""},
    };
  }
  else if (nodeType == "lidar_sensor") {
    params_ = {
      {"frame_id", "string", "laser_frame", "laser_frame", "TF frame for the sensor", {}, {}, {}, "frames", true, ""},
      {"angle_min", "double", -3.14159, -3.14159, "Start angle of scan (radians)", -3.14159, 3.14159, {}, "scan", true, ""},
      {"angle_max", "double", 3.14159, 3.14159, "End angle of scan (radians)", -3.14159, 3.14159, {}, "scan", true, ""},
      {"range_min", "double", 0.1, 0.1, "Minimum range value (meters)", 0.0, 10.0, {}, "range", true, ""},
      {"range_max", "double", 30.0, 30.0, "Maximum range value (meters)", 1.0, 100.0, {}, "range", true, ""},
      {"scan_frequency", "double", 10.0, 10.0, "Scan frequency in Hz", 1.0, 100.0, {}, "timing", true, ""},
    };
  }
  else if (nodeType == "diff_drive" || nodeType == "robot_controller") {
    params_ = {
      {"wheel_radius", "double", 0.033, 0.033, "Wheel radius in meters", 0.01, 1.0, {}, "geometry", true, ""},
      {"wheel_separation", "double", 0.16, 0.16, "Distance between wheels in meters", 0.05, 2.0, {}, "geometry", true, ""},
      {"max_linear_velocity", "double", 0.22, 0.22, "Maximum linear velocity (m/s)", 0.0, 5.0, {}, "limits", true, ""},
      {"max_angular_velocity", "double", 2.84, 2.84, "Maximum angular velocity (rad/s)", 0.0, 10.0, {}, "limits", true, ""},
      {"publish_rate", "double", 30.0, 30.0, "Odometry publish rate in Hz", 1.0, 100.0, {}, "timing", true, ""},
      {"use_sim_time", "bool", false, false, "Use simulation time", {}, {}, {}, "timing", true, ""},
    };
  }
  else if (nodeType == "image_processor") {
    params_ = {
      {"image_transport", "string", "raw", "raw", "Image transport type", {}, {}, {"raw", "compressed", "theora"}, "transport", true, ""},
      {"queue_size", "int", 5, 5, "Message queue size", 1, 100, {}, "qos", true, ""},
      {"detection_threshold", "double", 0.5, 0.5, "Detection confidence threshold", 0.0, 1.0, {}, "detection", true, ""},
      {"use_gpu", "bool", false, false, "Use GPU acceleration", {}, {}, {}, "hardware", true, ""},
    };
  }
  else {
    // Generic node defaults
    params_ = {
      {"use_sim_time", "bool", false, false, "Use simulation time", {}, {}, {}, "", true, ""},
    };
  }
}

void ParamDashboard::populateTree() {
  updatingTree_ = true;
  paramTree_->clear();
  groupItems_.clear();

  // First pass: create group items
  for (const auto& param : params_) {
    if (!param.group.isEmpty()) {
      findOrCreateGroupItem(param.group);
    }
  }

  // Second pass: add parameters to their groups
  for (const auto& param : params_) {
    if (param.group.isEmpty()) {
      addParamToTree(param, nullptr);
    } else {
      QTreeWidgetItem* groupItem = groupItems_.value(param.group, nullptr);
      addParamToTree(param, groupItem);
    }
  }

  // Expand all groups by default
  paramTree_->expandAll();

  updatingTree_ = false;
}

QTreeWidgetItem* ParamDashboard::findOrCreateGroupItem(const QString& groupName) {
  if (groupItems_.contains(groupName)) {
    return groupItems_[groupName];
  }

  QTreeWidgetItem* groupItem = new QTreeWidgetItem();
  groupItem->setText(0, groupName);
  groupItem->setText(2, "group");
  groupItem->setData(0, Qt::UserRole, groupName);
  groupItem->setData(0, Qt::UserRole + 1, "group");

  // Style the group item
  QFont font = groupItem->font(0);
  font.setBold(true);
  groupItem->setFont(0, font);
  groupItem->setForeground(0, QColor(50, 50, 60));
  groupItem->setForeground(1, QColor(50, 50, 60));
  groupItem->setForeground(2, QColor(50, 50, 60));
  groupItem->setBackground(0, QColor(220, 225, 235));
  groupItem->setBackground(1, QColor(220, 225, 235));
  groupItem->setBackground(2, QColor(220, 225, 235));

  paramTree_->addTopLevelItem(groupItem);
  groupItems_[groupName] = groupItem;

  return groupItem;
}

void ParamDashboard::addParamToTree(const ParamDefinition& param, QTreeWidgetItem* parentItem) {
  QTreeWidgetItem* item = createParamItem(param);

  if (parentItem) {
    parentItem->addChild(item);
  } else {
    paramTree_->addTopLevelItem(item);
  }

  // Create custom editor widget for the value column
  QWidget* editor = createValueEditor(param, item);
  if (editor) {
    paramTree_->setItemWidget(item, 1, editor);
  }

  // Update validation state
  updateValidationState(item, param.isValid, param.validationError);
}

QTreeWidgetItem* ParamDashboard::createParamItem(const ParamDefinition& param) {
  QTreeWidgetItem* item = new QTreeWidgetItem();
  item->setText(0, param.name);
  item->setToolTip(0, param.description);
  item->setText(2, param.type);
  item->setData(0, Qt::UserRole, param.name);
  item->setData(0, Qt::UserRole + 1, "param");

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
          validateParameter(p);
          updateValidationState(item, p.isValid, p.validationError);
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
          validateParameter(p);
          updateValidationState(item, p.isValid, p.validationError);
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
          validateParameter(p);
          updateValidationState(item, p.isValid, p.validationError);
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
            validateParameter(p);
            updateValidationState(item, p.isValid, p.validationError);
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
            validateParameter(p);
            updateValidationState(item, p.isValid, p.validationError);
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

void ParamDashboard::updateValidationState(QTreeWidgetItem* item, bool isValid, const QString& errorMsg) {
  if (isValid) {
    item->setBackground(0, QBrush());
    item->setToolTip(1, "");
  } else {
    item->setBackground(0, QColor(255, 220, 220));
    item->setToolTip(1, errorMsg);
  }

  // Update overall validation label
  bool allValid = true;
  for (const auto& p : params_) {
    if (!p.isValid) {
      allValid = false;
      break;
    }
  }

  if (allValid) {
    validationLabel_->setVisible(false);
  } else {
    int invalidCount = 0;
    for (const auto& p : params_) {
      if (!p.isValid) invalidCount++;
    }
    validationLabel_->setText(tr("%1 parameter(s) have validation errors").arg(invalidCount));
    validationLabel_->setVisible(true);
  }

  emit validationStateChanged(allValid);
}

bool ParamDashboard::validateParameter(ParamDefinition& param) {
  param.isValid = true;
  param.validationError.clear();

  if (param.type == "int" || param.type == "double") {
    if (!validateNumericRange(param)) {
      return false;
    }
  } else if (param.type == "string") {
    if (!validateStringValue(param)) {
      return false;
    }
  }

  return true;
}

bool ParamDashboard::validateNumericRange(const ParamDefinition& param) {
  if (param.type == "int") {
    int value = param.currentValue.toInt();
    if (param.minValue.isValid() && value < param.minValue.toInt()) {
      const_cast<ParamDefinition&>(param).isValid = false;
      const_cast<ParamDefinition&>(param).validationError =
        tr("Value %1 is below minimum %2").arg(value).arg(param.minValue.toInt());
      return false;
    }
    if (param.maxValue.isValid() && value > param.maxValue.toInt()) {
      const_cast<ParamDefinition&>(param).isValid = false;
      const_cast<ParamDefinition&>(param).validationError =
        tr("Value %1 is above maximum %2").arg(value).arg(param.maxValue.toInt());
      return false;
    }
  } else if (param.type == "double") {
    double value = param.currentValue.toDouble();
    if (param.minValue.isValid() && value < param.minValue.toDouble()) {
      const_cast<ParamDefinition&>(param).isValid = false;
      const_cast<ParamDefinition&>(param).validationError =
        tr("Value %1 is below minimum %2").arg(value).arg(param.minValue.toDouble());
      return false;
    }
    if (param.maxValue.isValid() && value > param.maxValue.toDouble()) {
      const_cast<ParamDefinition&>(param).isValid = false;
      const_cast<ParamDefinition&>(param).validationError =
        tr("Value %1 is above maximum %2").arg(value).arg(param.maxValue.toDouble());
      return false;
    }
  }
  return true;
}

bool ParamDashboard::validateStringValue(const ParamDefinition& param) {
  QString value = param.currentValue.toString();

  // If enum values are defined, check that value is in the list
  if (!param.enumValues.isEmpty() && !param.enumValues.contains(value)) {
    const_cast<ParamDefinition&>(param).isValid = false;
    const_cast<ParamDefinition&>(param).validationError =
      tr("Value '%1' is not in allowed values: %2").arg(value).arg(param.enumValues.join(", "));
    return false;
  }

  return true;
}

bool ParamDashboard::validateAll() {
  bool allValid = true;
  for (auto& param : params_) {
    if (!validateParameter(param)) {
      allValid = false;
    }
  }

  // Update tree validation states
  populateTree();

  return allValid;
}

void ParamDashboard::onItemChanged(QTreeWidgetItem* item, int column) {
  if (updatingTree_) return;
  Q_UNUSED(item)
  Q_UNUSED(column)
}

void ParamDashboard::onAddParameter() {
  // Get parameter group (optional)
  QStringList groupNames = groupItems_.keys();
  groupNames.prepend(tr("(No Group)"));

  bool ok;
  QString selectedGroup = QInputDialog::getItem(
    this, tr("Add Parameter"), tr("Select group:"),
    groupNames, 0, false, &ok
  );

  if (!ok) return;

  QString paramName = QInputDialog::getText(
    this, tr("Add Parameter"), tr("Parameter name:"),
    QLineEdit::Normal, QString("param_%1").arg(params_.size()), &ok
  );

  if (!ok || paramName.isEmpty()) return;

  // Get parameter type
  QStringList types = {"string", "int", "double", "bool"};
  QString selectedType = QInputDialog::getItem(
    this, tr("Add Parameter"), tr("Parameter type:"),
    types, 0, false, &ok
  );

  if (!ok) return;

  // Add a new parameter with default values
  ParamDefinition newParam;
  newParam.name = paramName;
  newParam.type = selectedType;
  newParam.group = (selectedGroup == tr("(No Group)")) ? QString() : selectedGroup;
  newParam.description = "User-defined parameter";
  newParam.isValid = true;

  if (selectedType == "string") {
    newParam.defaultValue = "";
    newParam.currentValue = "";
  } else if (selectedType == "int") {
    newParam.defaultValue = 0;
    newParam.currentValue = 0;
  } else if (selectedType == "double") {
    newParam.defaultValue = 0.0;
    newParam.currentValue = 0.0;
  } else if (selectedType == "bool") {
    newParam.defaultValue = false;
    newParam.currentValue = false;
  }

  params_.append(newParam);
  populateTree();
  emit parametersModified();
}

void ParamDashboard::onRemoveParameter() {
  QTreeWidgetItem* current = paramTree_->currentItem();
  if (!current) return;

  // Check if it's a group item
  QString itemType = current->data(0, Qt::UserRole + 1).toString();
  if (itemType == "group") {
    // Remove all parameters in the group
    QString groupName = current->data(0, Qt::UserRole).toString();
    params_.erase(std::remove_if(params_.begin(), params_.end(),
      [&groupName](const ParamDefinition& p) { return p.group == groupName; }),
      params_.end());
    groupItems_.remove(groupName);
  } else {
    // Remove single parameter
    QString paramName = current->data(0, Qt::UserRole).toString();
    for (int i = 0; i < params_.size(); ++i) {
      if (params_[i].name == paramName) {
        params_.removeAt(i);
        break;
      }
    }
  }

  populateTree();
  emit parametersModified();
}

void ParamDashboard::onResetToDefaults() {
  for (auto& param : params_) {
    param.currentValue = param.defaultValue;
    param.isValid = true;
    param.validationError.clear();
  }
  populateTree();
  emit parametersModified();
}

void ParamDashboard::onAddGroup() {
  bool ok;
  QString groupName = QInputDialog::getText(
    this, tr("Add Group"), tr("Group name:"),
    QLineEdit::Normal, "", &ok
  );

  if (ok && !groupName.isEmpty()) {
    findOrCreateGroupItem(groupName);
  }
}

void ParamDashboard::onExpandAll() {
  paramTree_->expandAll();
}

void ParamDashboard::onCollapseAll() {
  paramTree_->collapseAll();
}

void ParamDashboard::onImportYaml() {
  QString fileName = QFileDialog::getOpenFileName(
    this, tr("Import YAML Parameters"),
    QString(),
    tr("YAML Files (*.yaml *.yml);;All Files (*)")
  );

  if (fileName.isEmpty()) return;

  if (importYamlFile(fileName)) {
    QMessageBox::information(this, tr("Import Successful"),
      tr("Parameters imported successfully from %1").arg(fileName));
  } else {
    QMessageBox::warning(this, tr("Import Failed"),
      tr("Failed to import parameters from %1").arg(fileName));
  }
}

void ParamDashboard::onExportYaml() {
  QString fileName = QFileDialog::getSaveFileName(
    this, tr("Export YAML Parameters"),
    currentBlock_ ? currentBlock_->packageName() + "_params.yaml" : "params.yaml",
    tr("YAML Files (*.yaml *.yml);;All Files (*)")
  );

  if (fileName.isEmpty()) return;

  // Ensure .yaml extension
  if (!fileName.endsWith(".yaml", Qt::CaseInsensitive) &&
      !fileName.endsWith(".yml", Qt::CaseInsensitive)) {
    fileName += ".yaml";
  }

  if (exportYamlFile(fileName)) {
    QMessageBox::information(this, tr("Export Successful"),
      tr("Parameters exported successfully to %1").arg(fileName));
  } else {
    QMessageBox::warning(this, tr("Export Failed"),
      tr("Failed to export parameters to %1").arg(fileName));
  }
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

    // Group parameters by their group
    QMap<QString, QList<ParamDefinition>> grouped;
    QList<ParamDefinition> ungrouped;

    for (const auto& param : params_) {
      if (param.group.isEmpty()) {
        ungrouped.append(param);
      } else {
        grouped[param.group].append(param);
      }
    }

    // Write ungrouped parameters first
    for (const auto& param : ungrouped) {
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

    // Write grouped parameters
    for (auto it = grouped.begin(); it != grouped.end(); ++it) {
      stream << "    " << it.key() << ":\n";
      for (const auto& param : it.value()) {
        stream << "      " << param.name << ": ";
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
  }

  return yaml;
}

bool ParamDashboard::fromYaml(const QString& yaml) {
  try {
    YAML::Node root = YAML::Load(yaml.toStdString());

    if (root.IsNull() || !root.IsMap()) {
      return false;
    }

    params_.clear();

    // Iterate through the YAML structure
    for (auto it = root.begin(); it != root.end(); ++it) {
      QString nodeName = QString::fromStdString(it->first.as<std::string>());
      YAML::Node nodeData = it->second;

      // Look for ros__parameters
      if (nodeData["ros__parameters"]) {
        parseYamlParams(nodeData["ros__parameters"], "");
      } else if (nodeData.IsMap()) {
        // Direct parameters without ros__parameters wrapper
        parseYamlParams(nodeData, "");
      }
    }

    populateTree();
    return true;
  } catch (const YAML::Exception& e) {
    qWarning("YAML parsing error: %s", e.what());
    return false;
  }
}

void ParamDashboard::parseYamlParams(const YAML::Node& node, const QString& group) {
  for (auto it = node.begin(); it != node.end(); ++it) {
    QString key = QString::fromStdString(it->first.as<std::string>());
    YAML::Node value = it->second;

    if (value.IsMap()) {
      // This is a nested group
      parseYamlParams(value, key);
    } else {
      // This is a parameter
      ParamDefinition param;
      param.name = key;
      param.group = group;
      param.isValid = true;

      if (value.IsScalar()) {
        std::string strVal = value.as<std::string>();

        // Try to determine type
        if (strVal == "true" || strVal == "false") {
          param.type = "bool";
          param.currentValue = (strVal == "true");
          param.defaultValue = param.currentValue;
        } else {
          // Try int
          bool isInt = false;
          int intVal = QString::fromStdString(strVal).toInt(&isInt);
          if (isInt && strVal.find('.') == std::string::npos) {
            param.type = "int";
            param.currentValue = intVal;
            param.defaultValue = param.currentValue;
          } else {
            // Try double
            bool isDouble = false;
            double doubleVal = QString::fromStdString(strVal).toDouble(&isDouble);
            if (isDouble) {
              param.type = "double";
              param.currentValue = doubleVal;
              param.defaultValue = param.currentValue;
            } else {
              // String
              param.type = "string";
              param.currentValue = QString::fromStdString(strVal);
              param.defaultValue = param.currentValue;
            }
          }
        }
      } else if (value.IsSequence()) {
        param.type = "array";
        QStringList items;
        for (size_t i = 0; i < value.size(); ++i) {
          items.append(QString::fromStdString(value[i].as<std::string>()));
        }
        param.currentValue = items;
        param.defaultValue = param.currentValue;
      }

      params_.append(param);
    }
  }
}

bool ParamDashboard::importYamlFile(const QString& filePath) {
  QFile file(filePath);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return false;
  }

  QString content = QString::fromUtf8(file.readAll());
  file.close();

  return fromYaml(content);
}

bool ParamDashboard::exportYamlFile(const QString& filePath) const {
  QFile file(filePath);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    return false;
  }

  QString yaml = toYaml();
  file.write(yaml.toUtf8());
  file.close();

  return true;
}

// Conversion methods between ParamDefinition and BlockParamData
ParamDefinition ParamDashboard::fromBlockParam(const BlockParamData& blockParam) {
  ParamDefinition param;
  param.name = blockParam.name;
  param.type = blockParam.type;
  param.defaultValue = blockParam.defaultValue;
  param.currentValue = blockParam.currentValue;
  param.description = blockParam.description;
  param.minValue = blockParam.minValue;
  param.maxValue = blockParam.maxValue;
  param.enumValues = blockParam.enumValues;
  param.group = blockParam.group;
  param.isValid = true;
  param.validationError.clear();
  return param;
}

BlockParamData ParamDashboard::toBlockParam(const ParamDefinition& param) {
  BlockParamData blockParam;
  blockParam.name = param.name;
  blockParam.type = param.type;
  blockParam.defaultValue = param.defaultValue;
  blockParam.currentValue = param.currentValue;
  blockParam.description = param.description;
  blockParam.minValue = param.minValue;
  blockParam.maxValue = param.maxValue;
  blockParam.enumValues = param.enumValues;
  blockParam.group = param.group;
  return blockParam;
}

void ParamDashboard::saveParametersToBlock() {
  if (!currentBlock_) return;

  QList<BlockParamData> blockParams;
  for (const auto& param : params_) {
    blockParams.append(toBlockParam(param));
  }
  currentBlock_->setParameters(blockParams);
}

void ParamDashboard::loadParametersFromBlock() {
  if (!currentBlock_) return;

  params_.clear();
  const auto& blockParams = currentBlock_->parameters();
  for (const auto& blockParam : blockParams) {
    params_.append(fromBlockParam(blockParam));
  }
}

// YAML file management methods
void ParamDashboard::setProjectYamlFiles(const QList<YamlFileInfo>& yamlFiles) {
  yamlFiles_ = yamlFiles;
  yamlParamsCache_.clear();
  updateYamlSourceCombo();
}

void ParamDashboard::setProjectDirectory(const QString& projectDir) {
  projectDirectory_ = projectDir;
}

void ParamDashboard::clearYamlFiles() {
  yamlFiles_.clear();
  yamlParamsCache_.clear();
  projectDirectory_.clear();

  // Reset combo to just "Block Parameters"
  yamlSourceCombo_->blockSignals(true);
  yamlSourceCombo_->clear();
  yamlSourceCombo_->addItem(tr("Block Parameters"), QVariant());
  yamlSourceCombo_->blockSignals(false);

  showingYamlParams_ = false;
  syncToBlockButton_->setVisible(false);
}

void ParamDashboard::updateYamlSourceCombo() {
  yamlSourceCombo_->blockSignals(true);

  // Remember current selection
  QString currentSelection = yamlSourceCombo_->currentData().toString();

  yamlSourceCombo_->clear();
  yamlSourceCombo_->addItem(tr("Block Parameters"), QVariant());

  // Add YAML files
  for (const YamlFileInfo& yamlFile : yamlFiles_) {
    QFileInfo fileInfo(yamlFile.filePath);
    QString displayName = fileInfo.fileName();
    yamlSourceCombo_->addItem(displayName, yamlFile.filePath);
  }

  // Restore selection if possible
  if (!currentSelection.isEmpty()) {
    int index = yamlSourceCombo_->findData(currentSelection);
    if (index >= 0) {
      yamlSourceCombo_->setCurrentIndex(index);
    }
  }

  yamlSourceCombo_->blockSignals(false);
}

QString ParamDashboard::resolveYamlPath(const QString& relativePath) const {
  if (QFileInfo(relativePath).isAbsolute()) {
    return relativePath;
  }
  if (!projectDirectory_.isEmpty()) {
    return QDir(projectDirectory_).filePath(relativePath);
  }
  return relativePath;
}

void ParamDashboard::loadYamlFileParams(const QString& filePath) {
  QString resolvedPath = resolveYamlPath(filePath);

  // Don't use cache - we want params specific to current block
  // The cache key should include block name
  QString blockName = currentBlock_ ? currentBlock_->packageName() : QString();
  QString cacheKey = resolvedPath + "::" + blockName;

  if (yamlParamsCache_.contains(cacheKey)) {
    params_ = yamlParamsCache_[cacheKey];
    return;
  }

  // Parse YAML file
  QFile file(resolvedPath);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    RCLCPP_WARN(rclcpp::get_logger("ros_weaver"),
                "Could not open YAML file: %s", resolvedPath.toStdString().c_str());
    return;
  }

  QString content = QString::fromUtf8(file.readAll());
  file.close();

  // Clear and parse
  params_.clear();

  try {
    YAML::Node root = YAML::Load(content.toStdString());

    // Build list of possible YAML node names for this block (same logic as setCurrentBlock)
    QStringList possibleNames;
    possibleNames << blockName;
    if (blockName.startsWith("nav2_")) {
      QString baseName = blockName.mid(5);  // Remove "nav2_" prefix
      possibleNames << baseName + "_server";  // e.g., controller -> controller_server
      possibleNames << baseName;               // e.g., controller
    }

    bool foundMatch = false;
    QString matchedName;

    // Try each possible name
    for (const QString& possibleName : possibleNames) {
      if (!possibleName.isEmpty() && root[possibleName.toStdString()]) {
        matchedName = possibleName;
        YAML::Node nodeSection = root[possibleName.toStdString()];

        // Check for ros__parameters wrapper
        if (nodeSection["ros__parameters"]) {
          parseYamlParams(nodeSection["ros__parameters"], QString());
        } else {
          parseYamlParams(nodeSection, QString());
        }
        foundMatch = true;
        break;
      }
    }

    // If no match found, show all node sections from the file
    if (!foundMatch) {

      for (auto it = root.begin(); it != root.end(); ++it) {
        QString key = QString::fromStdString(it->first.as<std::string>());
        YAML::Node value = it->second;

        // Check if this is a node section with ros__parameters
        if (value.IsMap() && value["ros__parameters"]) {
          parseYamlParams(value["ros__parameters"], key);
        } else if (value.IsMap()) {
          parseYamlParams(value, key);
        }
      }

      RCLCPP_INFO(rclcpp::get_logger("ros_weaver"),
                  "Parsed %d parameters from all sections", static_cast<int>(params_.size()));
    }
  } catch (const YAML::Exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("ros_weaver"),
                "YAML parse error: %s", e.what());
  }

  // Cache the result with block-specific key
  yamlParamsCache_[cacheKey] = params_;
}

void ParamDashboard::onYamlSourceChanged(int index) {
  if (index < 0 || !currentBlock_) return;

  QVariant data = yamlSourceCombo_->itemData(index);

  if (!data.isValid() || data.toString().isEmpty()) {
    // "Block Parameters" selected
    showingYamlParams_ = false;
    syncToBlockButton_->setVisible(false);

    // Reload block parameters
    loadParametersFromBlock();
    populateTree();
    validateAll();
  } else {
    // YAML file selected
    QString filePath = data.toString();
    showingYamlParams_ = true;
    syncToBlockButton_->setVisible(true);

    // Load parameters from YAML file for this node
    loadYamlFileParams(filePath);
    populateTree();
    validateAll();
  }
}

void ParamDashboard::onSyncToBlock() {
  if (!currentBlock_ || !showingYamlParams_) return;

  // Copy current params to block
  QList<BlockParamData> blockParams;
  for (const auto& param : params_) {
    blockParams.append(toBlockParam(param));
  }
  currentBlock_->setParameters(blockParams);

  // Switch back to block parameters view
  yamlSourceCombo_->setCurrentIndex(0);

  QMessageBox::information(this, tr("Parameters Synced"),
    tr("YAML parameters have been copied to the block.\nThe block now uses these parameters."));
}

}  // namespace ros_weaver
