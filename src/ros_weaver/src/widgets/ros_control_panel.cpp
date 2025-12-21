// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, ROS Weaver Contributors

#include "ros_weaver/widgets/ros_control_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QMenu>
#include <QMessageBox>
#include <QStyle>
#include <QApplication>
#include <QRegularExpression>
#include <QDebug>
#include <iostream>

namespace ros_weaver {

RosControlPanel::RosControlPanel(QWidget* parent)
    : QWidget(parent)
    , autoRefreshTimer_(new QTimer(this)) {
  // Register metatypes for signal/slot
  qRegisterMetaType<ControllerInfo>("ControllerInfo");
  qRegisterMetaType<HardwareInterfaceInfo>("HardwareInterfaceInfo");
  qRegisterMetaType<ControllerState>("ControllerState");

  setupUi();
  setupConnections();

  // Initial refresh after a short delay - use async version to avoid blocking UI
  QTimer::singleShot(500, this, &RosControlPanel::refreshAllAsync);
}

RosControlPanel::~RosControlPanel() {
  std::cerr << "RosControlPanel destructor: starting" << std::endl;
  stopMonitoring();

  // Stop async refresh thread if running
  refreshThreadRunning_ = false;
  if (refreshThread_ && refreshThread_->joinable()) {
    refreshThread_->join();
  }
  refreshThread_.reset();

  if (commandProcess_) {
    commandProcess_->kill();
    commandProcess_->waitForFinished(1000);
  }
  std::cerr << "RosControlPanel destructor: complete" << std::endl;
}

void RosControlPanel::setupUi() {
  auto* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(4, 4, 4, 4);
  mainLayout->setSpacing(4);

  // Add toolbar
  mainLayout->addWidget(createToolbar());

  // Main splitter (horizontal: left trees | right details)
  mainSplitter_ = new QSplitter(Qt::Horizontal, this);

  // Left side: vertical splitter with controller tree and interface tree
  leftSplitter_ = new QSplitter(Qt::Vertical);

  setupControllerTree();
  leftSplitter_->addWidget(controllerTree_);

  setupInterfaceTree();
  leftSplitter_->addWidget(interfaceTree_);

  leftSplitter_->setStretchFactor(0, 2);
  leftSplitter_->setStretchFactor(1, 1);

  mainSplitter_->addWidget(leftSplitter_);

  // Right side: details panel
  setupDetailsPanel();
  mainSplitter_->addWidget(detailsPanel_);

  mainSplitter_->setStretchFactor(0, 2);
  mainSplitter_->setStretchFactor(1, 1);
  mainSplitter_->setSizes({400, 200});

  mainLayout->addWidget(mainSplitter_, 1);

  // Status bar
  statusLabel_ = new QLabel(tr("No controllers found"), this);
  statusLabel_->setStyleSheet("color: #888; font-size: 10px;");
  mainLayout->addWidget(statusLabel_);
}

QWidget* RosControlPanel::createToolbar() {
  auto* toolbar = new QWidget(this);
  auto* layout = new QHBoxLayout(toolbar);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(4);

  // Search filter
  searchEdit_ = new QLineEdit(this);
  searchEdit_->setPlaceholderText(tr("Filter controllers..."));
  searchEdit_->setClearButtonEnabled(true);
  searchEdit_->setMaximumWidth(200);
  layout->addWidget(searchEdit_);

  // Controller manager selector
  managerCombo_ = new QComboBox(this);
  managerCombo_->setToolTip(tr("Select controller manager"));
  managerCombo_->addItem(tr("Default"), "/controller_manager");
  managerCombo_->setMinimumWidth(150);
  layout->addWidget(managerCombo_);

  layout->addStretch();

  // Refresh button
  refreshButton_ = new QPushButton(this);
  refreshButton_->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));
  refreshButton_->setToolTip(tr("Refresh controller list"));
  refreshButton_->setFixedSize(28, 28);
  layout->addWidget(refreshButton_);

  // Live/Auto-refresh toggle
  liveButton_ = new QToolButton(this);
  liveButton_->setIcon(QIcon::fromTheme("media-playback-start",
                       style()->standardIcon(QStyle::SP_MediaPlay)));
  liveButton_->setToolTip(tr("Toggle auto-refresh (2s interval)"));
  liveButton_->setCheckable(true);
  liveButton_->setFixedSize(28, 28);
  layout->addWidget(liveButton_);

  return toolbar;
}

void RosControlPanel::setupControllerTree() {
  controllerTree_ = new QTreeWidget(this);
  controllerTree_->setHeaderLabels({tr("Controller"), tr("Type"), tr("State")});
  controllerTree_->setRootIsDecorated(false);
  controllerTree_->setAlternatingRowColors(true);
  controllerTree_->setSelectionMode(QAbstractItemView::SingleSelection);
  controllerTree_->setContextMenuPolicy(Qt::CustomContextMenu);

  // Set column widths
  controllerTree_->header()->setStretchLastSection(true);
  controllerTree_->setColumnWidth(0, 180);
  controllerTree_->setColumnWidth(1, 200);
}

void RosControlPanel::setupInterfaceTree() {
  interfaceTree_ = new QTreeWidget(this);
  interfaceTree_->setHeaderLabels({tr("Interface"), tr("Type"), tr("Status")});
  interfaceTree_->setRootIsDecorated(false);
  interfaceTree_->setAlternatingRowColors(true);
  interfaceTree_->setSelectionMode(QAbstractItemView::SingleSelection);
  interfaceTree_->setContextMenuPolicy(Qt::CustomContextMenu);

  // Set column widths
  interfaceTree_->header()->setStretchLastSection(true);
  interfaceTree_->setColumnWidth(0, 200);
  interfaceTree_->setColumnWidth(1, 80);
}

void RosControlPanel::setupDetailsPanel() {
  detailsPanel_ = new QWidget(this);
  auto* layout = new QVBoxLayout(detailsPanel_);
  layout->setContentsMargins(4, 4, 4, 4);
  layout->setSpacing(8);

  // Controller info group
  auto* infoGroup = new QGroupBox(tr("Controller Information"), detailsPanel_);
  auto* infoLayout = new QVBoxLayout(infoGroup);

  auto addInfoRow = [infoLayout](const QString& label, QLabel*& valueLabel) {
    auto* row = new QHBoxLayout();
    auto* labelWidget = new QLabel(label);
    labelWidget->setStyleSheet("font-weight: bold;");
    labelWidget->setFixedWidth(100);
    row->addWidget(labelWidget);
    valueLabel = new QLabel("-");
    valueLabel->setWordWrap(true);
    row->addWidget(valueLabel, 1);
    infoLayout->addLayout(row);
  };

  addInfoRow(tr("Name:"), controllerNameLabel_);
  addInfoRow(tr("Type:"), controllerTypeLabel_);
  addInfoRow(tr("State:"), controllerStateLabel_);
  addInfoRow(tr("Interfaces:"), claimedInterfacesLabel_);
  addInfoRow(tr("Last Change:"), lastStateChangeLabel_);

  layout->addWidget(infoGroup);

  // Actions group
  setupActionButtons();
  layout->addWidget(actionsGroupBox_);

  layout->addStretch();
}

void RosControlPanel::setupActionButtons() {
  actionsGroupBox_ = new QGroupBox(tr("Actions"), detailsPanel_);
  auto* layout = new QVBoxLayout(actionsGroupBox_);

  activateButton_ = new QPushButton(tr("Activate"), actionsGroupBox_);
  activateButton_->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
  activateButton_->setEnabled(false);
  layout->addWidget(activateButton_);

  deactivateButton_ = new QPushButton(tr("Deactivate"), actionsGroupBox_);
  deactivateButton_->setIcon(style()->standardIcon(QStyle::SP_MediaStop));
  deactivateButton_->setEnabled(false);
  layout->addWidget(deactivateButton_);

  layout->addSpacing(10);

  emergencyStopButton_ = new QPushButton(tr("EMERGENCY STOP ALL"), actionsGroupBox_);
  emergencyStopButton_->setStyleSheet(
      "QPushButton { background-color: #cc4444; color: white; font-weight: bold; }"
      "QPushButton:hover { background-color: #dd5555; }"
      "QPushButton:pressed { background-color: #bb3333; }");
  emergencyStopButton_->setEnabled(false);
  layout->addWidget(emergencyStopButton_);
}

void RosControlPanel::setupConnections() {
  // Toolbar controls
  connect(searchEdit_, &QLineEdit::textChanged,
          this, &RosControlPanel::onFilterChanged);
  connect(refreshButton_, &QPushButton::clicked,
          this, &RosControlPanel::refreshAll);
  connect(liveButton_, &QToolButton::toggled, this, [this](bool checked) {
    if (checked) startMonitoring();
    else stopMonitoring();
  });

  // Tree selection
  connect(controllerTree_, &QTreeWidget::itemClicked,
          this, &RosControlPanel::onControllerSelected);
  connect(interfaceTree_, &QTreeWidget::itemClicked,
          this, &RosControlPanel::onInterfaceSelected);

  // Context menus
  connect(controllerTree_, &QTreeWidget::customContextMenuRequested,
          this, &RosControlPanel::onControllerContextMenu);
  connect(interfaceTree_, &QTreeWidget::customContextMenuRequested,
          this, &RosControlPanel::onInterfaceContextMenu);

  // Action buttons
  connect(activateButton_, &QPushButton::clicked, this, [this]() {
    if (!selectedController_.isEmpty()) {
      activateController(selectedController_);
    }
  });
  connect(deactivateButton_, &QPushButton::clicked, this, [this]() {
    if (!selectedController_.isEmpty()) {
      deactivateController(selectedController_);
    }
  });
  connect(emergencyStopButton_, &QPushButton::clicked,
          this, &RosControlPanel::emergencyStopAll);

  // Auto-refresh timer
  connect(autoRefreshTimer_, &QTimer::timeout,
          this, &RosControlPanel::onAutoRefreshTimer);
}

QString RosControlPanel::runRos2ControlCommand(const QStringList& args, int timeoutMs) {
  QProcess process;
  QStringList fullArgs = {"control"};
  fullArgs.append(args);

  // Add controller manager if not default
  QString manager = managerCombo_->currentData().toString();
  if (!manager.isEmpty() && manager != "/controller_manager") {
    fullArgs << "-c" << manager;
  }

  process.start("ros2", fullArgs);

  if (!process.waitForStarted(2000)) {
    qWarning() << "Failed to start ros2 control command:" << fullArgs;
    return QString();
  }

  if (!process.waitForFinished(timeoutMs)) {
    qWarning() << "ros2 control command timed out:" << fullArgs;
    // Kill the process to prevent "destroyed while running" warning
    process.kill();
    process.waitForFinished(1000);
    return QString();
  }

  QString stdErr = process.readAllStandardError();

  if (process.exitCode() != 0) {
    // Check for common errors
    if (stdErr.contains("waiting for service") ||
        stdErr.contains("Could not contact service") ||
        stdErr.contains("timed out")) {
      qDebug() << "ros2_control not available (no controller_manager running)";
    } else {
      qWarning() << "ros2 control command failed:" << fullArgs << stdErr;
    }
    return QString();
  }

  return process.readAllStandardOutput();
}

QString RosControlPanel::runRos2ControlCommandForManager(
    const QStringList& args, const QString& manager, int timeoutMs) {
  // Thread-safe version that doesn't access UI elements
  QProcess process;
  QStringList fullArgs = {"control"};
  fullArgs.append(args);

  // Add controller manager if not default
  if (!manager.isEmpty() && manager != "/controller_manager") {
    fullArgs << "-c" << manager;
  }

  process.start("ros2", fullArgs);

  if (!process.waitForStarted(2000)) {
    qWarning() << "Failed to start ros2 control command:" << fullArgs;
    return QString();
  }

  if (!process.waitForFinished(timeoutMs)) {
    qWarning() << "ros2 control command timed out:" << fullArgs;
    // Kill the process to prevent "destroyed while running" warning
    process.kill();
    process.waitForFinished(1000);
    return QString();
  }

  QString stdErr = process.readAllStandardError();

  if (process.exitCode() != 0) {
    // Check for common errors
    if (stdErr.contains("waiting for service") ||
        stdErr.contains("Could not contact service") ||
        stdErr.contains("timed out")) {
      qDebug() << "ros2_control not available (no controller_manager running)";
    } else {
      qWarning() << "ros2 control command failed:" << fullArgs << stdErr;
    }
    return QString();
  }

  return process.readAllStandardOutput();
}

QList<ControllerInfo> RosControlPanel::parseControllerList(const QString& output) {
  QList<ControllerInfo> controllers;

  if (output.isEmpty()) {
    return controllers;
  }

  // Parse output format:
  // controller_name[controller_type] state
  // e.g., joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active

  QStringList lines = output.split('\n', Qt::SkipEmptyParts);

  for (const QString& line : lines) {
    QString trimmed = line.trimmed();
    if (trimmed.isEmpty()) continue;

    ControllerInfo info;

    // Extract controller name (everything before '[')
    int bracketStart = trimmed.indexOf('[');
    if (bracketStart > 0) {
      info.name = trimmed.left(bracketStart).trimmed();

      // Extract type (between '[' and ']')
      int bracketEnd = trimmed.indexOf(']', bracketStart);
      if (bracketEnd > bracketStart) {
        info.type = trimmed.mid(bracketStart + 1, bracketEnd - bracketStart - 1);

        // Extract state (after ']')
        QString stateStr = trimmed.mid(bracketEnd + 1).trimmed();
        info.state = parseControllerState(stateStr);
      }
    } else {
      // Fallback: whitespace-separated format
      QStringList parts = trimmed.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
      if (parts.size() >= 1) {
        info.name = parts[0];
        if (parts.size() >= 2) info.type = parts[1];
        if (parts.size() >= 3) info.state = parseControllerState(parts[2]);
      }
    }

    if (!info.name.isEmpty()) {
      info.lastStateChange = QDateTime::currentDateTime();
      controllers.append(info);
    }
  }

  return controllers;
}

QList<HardwareInterfaceInfo> RosControlPanel::parseHardwareInterfaces(const QString& output) {
  QList<HardwareInterfaceInfo> interfaces;

  if (output.isEmpty()) {
    return interfaces;
  }

  // Parse output format:
  // command interfaces
  //         joint1/position [available] [claimed]
  //         joint1/velocity [available] [unclaimed]
  // state interfaces
  //         joint1/position [available]

  QString currentType;
  QStringList lines = output.split('\n', Qt::SkipEmptyParts);

  for (const QString& line : lines) {
    QString trimmed = line.trimmed();
    if (trimmed.isEmpty()) continue;

    // Check for interface type headers
    if (trimmed == "command interfaces") {
      currentType = "command";
      continue;
    } else if (trimmed == "state interfaces") {
      currentType = "state";
      continue;
    }

    // Skip non-interface lines
    if (currentType.isEmpty()) continue;

    HardwareInterfaceInfo info;
    info.interfaceType = currentType;

    // Extract interface name and status
    // Format: interface_name [available] [claimed/unclaimed]
    int firstBracket = trimmed.indexOf('[');
    if (firstBracket > 0) {
      info.name = trimmed.left(firstBracket).trimmed();

      // Check for claimed status
      if (trimmed.contains("[claimed]")) {
        info.state = InterfaceState::Claimed;
      } else if (trimmed.contains("[available]")) {
        info.state = InterfaceState::Available;
      } else if (trimmed.contains("[unavailable]")) {
        info.state = InterfaceState::Unavailable;
      }
    } else {
      info.name = trimmed;
      info.state = InterfaceState::Available;
    }

    // Extract component name from interface name (e.g., "joint1" from "joint1/position")
    int slashPos = info.name.indexOf('/');
    if (slashPos > 0) {
      info.componentName = info.name.left(slashPos);
    }

    if (!info.name.isEmpty()) {
      interfaces.append(info);
    }
  }

  return interfaces;
}

ControllerState RosControlPanel::parseControllerState(const QString& stateStr) {
  QString lower = stateStr.toLower().trimmed();

  if (lower == "active") return ControllerState::Active;
  if (lower == "inactive") return ControllerState::Inactive;
  if (lower == "unconfigured") return ControllerState::Unconfigured;
  if (lower == "finalized") return ControllerState::Finalized;

  return ControllerState::Unknown;
}

void RosControlPanel::updateControllerTree(const QList<ControllerInfo>& controllers) {
  QString filter = searchEdit_->text().toLower();

  // Remember selection
  QString previousSelection = selectedController_;

  controllerTree_->clear();

  for (const ControllerInfo& info : controllers) {
    // Apply filter
    if (!filter.isEmpty() &&
        !info.name.toLower().contains(filter) &&
        !info.type.toLower().contains(filter)) {
      continue;
    }

    auto* item = new QTreeWidgetItem(controllerTree_);
    item->setText(0, info.name);
    item->setText(1, info.type);
    item->setText(2, getStateText(info.state));
    item->setData(0, Qt::UserRole, QVariant::fromValue(info));

    // Color coding
    QColor stateColor = getStateColor(info.state);
    item->setForeground(2, stateColor);

    // Bold for active controllers
    if (info.state == ControllerState::Active) {
      QFont font = item->font(0);
      font.setBold(true);
      item->setFont(0, font);
    }

    // Restore selection
    if (info.name == previousSelection) {
      item->setSelected(true);
      controllerTree_->setCurrentItem(item);
    }
  }

  // Update emergency stop button
  emergencyStopButton_->setEnabled(!controllers.isEmpty());
}

void RosControlPanel::updateInterfaceTree(const QList<HardwareInterfaceInfo>& interfaces) {
  interfaceTree_->clear();

  for (const HardwareInterfaceInfo& info : interfaces) {
    auto* item = new QTreeWidgetItem(interfaceTree_);
    item->setText(0, info.name);
    item->setText(1, info.interfaceType);
    item->setText(2, getInterfaceStateText(info.state));
    item->setData(0, Qt::UserRole, QVariant::fromValue(info));

    // Color coding
    QColor stateColor = getInterfaceStateColor(info.state);
    item->setForeground(2, stateColor);
  }
}

void RosControlPanel::updateDetailsPanel(const ControllerInfo* info) {
  if (!info) {
    controllerNameLabel_->setText("-");
    controllerTypeLabel_->setText("-");
    controllerStateLabel_->setText("-");
    controllerStateLabel_->setStyleSheet("");
    claimedInterfacesLabel_->setText("-");
    lastStateChangeLabel_->setText("-");

    activateButton_->setEnabled(false);
    deactivateButton_->setEnabled(false);
    return;
  }

  controllerNameLabel_->setText(info->name);
  controllerTypeLabel_->setText(info->type);

  QString stateText = getStateText(info->state);
  QColor stateColor = getStateColor(info->state);
  controllerStateLabel_->setText(stateText);
  controllerStateLabel_->setStyleSheet(
      QString("color: %1; font-weight: bold;").arg(stateColor.name()));

  if (info->claimedInterfaces.isEmpty()) {
    claimedInterfacesLabel_->setText(tr("(none)"));
  } else {
    claimedInterfacesLabel_->setText(info->claimedInterfaces.join(", "));
  }

  lastStateChangeLabel_->setText(info->lastStateChange.toString("hh:mm:ss"));

  // Update action buttons based on current state
  updateActionButtonsState();
}

void RosControlPanel::updateActionButtonsState() {
  if (selectedController_.isEmpty()) {
    activateButton_->setEnabled(false);
    deactivateButton_->setEnabled(false);
    return;
  }

  // Find the selected controller info
  const ControllerInfo* info = nullptr;
  for (const ControllerInfo& c : controllers_) {
    if (c.name == selectedController_) {
      info = &c;
      break;
    }
  }

  if (!info) {
    activateButton_->setEnabled(false);
    deactivateButton_->setEnabled(false);
    return;
  }

  // Enable/disable based on current state
  switch (info->state) {
    case ControllerState::Inactive:
      activateButton_->setEnabled(true);
      deactivateButton_->setEnabled(false);
      break;
    case ControllerState::Active:
      activateButton_->setEnabled(false);
      deactivateButton_->setEnabled(true);
      break;
    default:
      activateButton_->setEnabled(false);
      deactivateButton_->setEnabled(false);
      break;
  }
}

void RosControlPanel::updateStatusLabel() {
  int activeCount = 0;
  for (const ControllerInfo& c : controllers_) {
    if (c.state == ControllerState::Active) activeCount++;
  }

  QString status = tr("%1 controllers (%2 active), %3 interfaces")
      .arg(controllers_.size())
      .arg(activeCount)
      .arg(interfaces_.size());

  if (monitoring_) {
    status += tr(" | Auto-refresh ON");
  }

  statusLabel_->setText(status);
}

QColor RosControlPanel::getStateColor(ControllerState state) const {
  switch (state) {
    case ControllerState::Unconfigured: return QColor(220, 80, 80);   // Red
    case ControllerState::Inactive:     return QColor(220, 180, 60);  // Yellow/Orange
    case ControllerState::Active:       return QColor(80, 180, 80);   // Green
    case ControllerState::Finalized:    return QColor(80, 140, 200);  // Blue
    default:                            return QColor(128, 128, 128); // Gray
  }
}

QString RosControlPanel::getStateText(ControllerState state) const {
  switch (state) {
    case ControllerState::Unconfigured: return tr("Unconfigured");
    case ControllerState::Inactive:     return tr("Inactive");
    case ControllerState::Active:       return tr("Active");
    case ControllerState::Finalized:    return tr("Finalized");
    default:                            return tr("Unknown");
  }
}

QIcon RosControlPanel::getStateIcon(ControllerState state) const {
  // Could return custom icons based on state
  Q_UNUSED(state);
  return QIcon();
}

QColor RosControlPanel::getInterfaceStateColor(InterfaceState state) const {
  switch (state) {
    case InterfaceState::Available:   return QColor(80, 180, 80);   // Green
    case InterfaceState::Claimed:     return QColor(220, 180, 60);  // Yellow
    case InterfaceState::Unavailable: return QColor(220, 80, 80);   // Red
    default:                          return QColor(128, 128, 128); // Gray
  }
}

QString RosControlPanel::getInterfaceStateText(InterfaceState state) const {
  switch (state) {
    case InterfaceState::Available:   return tr("Available");
    case InterfaceState::Claimed:     return tr("Claimed");
    case InterfaceState::Unavailable: return tr("Unavailable");
    default:                          return tr("Unknown");
  }
}

// Slots

void RosControlPanel::refreshControllers() {
  QString output = runRos2ControlCommand({"list_controllers"}, 3000);

  if (output.isNull()) {
    // Command failed or timed out - likely no controller_manager
    controllers_.clear();
    updateControllerTree(controllers_);
    statusLabel_->setText(tr("No controller_manager available"));
    statusLabel_->setStyleSheet("color: #888;");
    return;
  }

  QList<ControllerInfo> oldControllers = controllers_;
  controllers_ = parseControllerList(output);

  // Detect state changes for AI notification
  for (const ControllerInfo& newInfo : controllers_) {
    for (const ControllerInfo& oldInfo : oldControllers) {
      if (newInfo.name == oldInfo.name && newInfo.state != oldInfo.state) {
        emit controllerStateChanged(
            newInfo.name,
            getStateText(oldInfo.state),
            getStateText(newInfo.state));
      }
    }
  }

  updateControllerTree(controllers_);
  updateStatusLabel();

  // Update details panel if selected controller still exists
  if (!selectedController_.isEmpty()) {
    const ControllerInfo* selected = nullptr;
    for (const ControllerInfo& c : controllers_) {
      if (c.name == selectedController_) {
        selected = &c;
        break;
      }
    }
    updateDetailsPanel(selected);
  }
}

void RosControlPanel::refreshHardwareInterfaces() {
  QString output = runRos2ControlCommand({"list_hardware_interfaces"}, 3000);

  if (output.isNull()) {
    // Command failed or timed out
    interfaces_.clear();
    updateInterfaceTree(interfaces_);
    return;
  }

  interfaces_ = parseHardwareInterfaces(output);
  updateInterfaceTree(interfaces_);
  updateStatusLabel();
}

void RosControlPanel::refreshAll() {
  refreshControllers();
  refreshHardwareInterfaces();
}

void RosControlPanel::refreshAllAsync() {
  // Don't start a new thread if one is already running
  if (refreshThreadRunning_.load()) {
    return;
  }

  // Wait for previous thread to complete if any
  if (refreshThread_ && refreshThread_->joinable()) {
    refreshThread_->join();
  }

  // Get the current manager from UI before starting thread
  QString manager = managerCombo_->currentData().toString();

  statusLabel_->setText(tr("Discovering controllers..."));
  refreshThreadRunning_ = true;
  asyncRefreshHasData_ = false;

  // Run the blocking ros2 control commands in a background thread
  refreshThread_ = std::make_unique<std::thread>([this, manager]() {
    // Use shorter timeout for async operations
    QString controllersOutput = runRos2ControlCommandForManager(
        {"list_controllers"}, manager, ASYNC_CLI_TIMEOUT_MS);

    if (!refreshThreadRunning_.load()) return;  // Cancelled

    QString interfacesOutput = runRos2ControlCommandForManager(
        {"list_hardware_interfaces"}, manager, ASYNC_CLI_TIMEOUT_MS);

    if (!refreshThreadRunning_.load()) return;  // Cancelled

    // Parse results (thread-safe, no UI access)
    pendingControllers_ = parseControllerList(controllersOutput);
    pendingInterfaces_ = parseHardwareInterfaces(interfacesOutput);
    asyncRefreshHasData_ = !controllersOutput.isNull() || !interfacesOutput.isNull();

    refreshThreadRunning_ = false;

    // Signal completion on main thread
    QMetaObject::invokeMethod(this, "onAsyncRefreshComplete", Qt::QueuedConnection);
  });
}

void RosControlPanel::onAsyncRefreshComplete() {
  // Update UI with results from background thread
  if (asyncRefreshHasData_) {
    controllers_ = pendingControllers_;
    interfaces_ = pendingInterfaces_;
    updateControllerTree(controllers_);
    updateInterfaceTree(interfaces_);
    updateStatusLabel();
  } else {
    // No data - ros2_control not available
    controllers_.clear();
    interfaces_.clear();
    updateControllerTree(controllers_);
    updateInterfaceTree(interfaces_);
    statusLabel_->setText(tr("No controller_manager available"));
    statusLabel_->setStyleSheet("color: #888;");
  }
}

void RosControlPanel::startMonitoring() {
  if (!monitoring_) {
    monitoring_ = true;
    autoRefreshTimer_->start(DEFAULT_REFRESH_INTERVAL_MS);
    liveButton_->setChecked(true);
    updateStatusLabel();
  }
}

void RosControlPanel::stopMonitoring() {
  if (monitoring_) {
    monitoring_ = false;
    autoRefreshTimer_->stop();
    liveButton_->setChecked(false);
    updateStatusLabel();
  }
}

void RosControlPanel::toggleMonitoring() {
  if (monitoring_) {
    stopMonitoring();
  } else {
    startMonitoring();
  }
}

void RosControlPanel::activateController(const QString& controllerName) {
  emit controllerActionRequested("activate", controllerName);

  QString output = runRos2ControlCommand({
      "switch_controllers",
      "--activate", controllerName
  });

  if (output.isEmpty()) {
    emit errorOccurred(tr("Failed to activate controller: %1").arg(controllerName));
  } else {
    // Refresh to see new state
    QTimer::singleShot(500, this, &RosControlPanel::refreshControllers);
  }
}

void RosControlPanel::deactivateController(const QString& controllerName) {
  emit controllerActionRequested("deactivate", controllerName);

  QString output = runRos2ControlCommand({
      "switch_controllers",
      "--deactivate", controllerName
  });

  if (output.isEmpty()) {
    emit errorOccurred(tr("Failed to deactivate controller: %1").arg(controllerName));
  } else {
    QTimer::singleShot(500, this, &RosControlPanel::refreshControllers);
  }
}

void RosControlPanel::switchControllers(const QStringList& activate,
                                         const QStringList& deactivate) {
  QStringList args = {"switch_controllers"};

  for (const QString& name : activate) {
    args << "--activate" << name;
  }
  for (const QString& name : deactivate) {
    args << "--deactivate" << name;
  }

  QString output = runRos2ControlCommand(args);

  if (output.isEmpty()) {
    emit errorOccurred(tr("Failed to switch controllers"));
  } else {
    QTimer::singleShot(500, this, &RosControlPanel::refreshControllers);
  }
}

void RosControlPanel::emergencyStopAll() {
  int result = QMessageBox::warning(
      this,
      tr("Emergency Stop"),
      tr("This will deactivate ALL active controllers.\n\n"
         "Are you sure you want to proceed?"),
      QMessageBox::Yes | QMessageBox::No,
      QMessageBox::No);

  if (result != QMessageBox::Yes) {
    return;
  }

  emit controllerActionRequested("emergency_stop", "all");

  // Collect all active controllers
  QStringList activeControllers;
  for (const ControllerInfo& c : controllers_) {
    if (c.state == ControllerState::Active) {
      activeControllers.append(c.name);
    }
  }

  if (activeControllers.isEmpty()) {
    return;
  }

  // Deactivate all
  QStringList args = {"switch_controllers"};
  for (const QString& name : activeControllers) {
    args << "--deactivate" << name;
  }

  QString output = runRos2ControlCommand(args);

  if (output.isEmpty()) {
    emit errorOccurred(tr("Emergency stop failed"));
  } else {
    QTimer::singleShot(500, this, &RosControlPanel::refreshControllers);
  }
}

void RosControlPanel::onControllerSelected(QTreeWidgetItem* item, int column) {
  Q_UNUSED(column);

  if (!item) {
    selectedController_.clear();
    updateDetailsPanel(nullptr);
    return;
  }

  QVariant data = item->data(0, Qt::UserRole);
  if (data.canConvert<ControllerInfo>()) {
    ControllerInfo info = data.value<ControllerInfo>();
    selectedController_ = info.name;
    updateDetailsPanel(&info);
  }
}

void RosControlPanel::onInterfaceSelected(QTreeWidgetItem* item, int column) {
  Q_UNUSED(item);
  Q_UNUSED(column);
  // Could show interface details in a dedicated area
}

void RosControlPanel::onAutoRefreshTimer() {
  refreshAll();
}

void RosControlPanel::onControllerContextMenu(const QPoint& pos) {
  QTreeWidgetItem* item = controllerTree_->itemAt(pos);
  if (!item) return;

  QVariant data = item->data(0, Qt::UserRole);
  if (!data.canConvert<ControllerInfo>()) return;

  ControllerInfo info = data.value<ControllerInfo>();

  QMenu menu;

  if (info.state == ControllerState::Inactive) {
    menu.addAction(tr("Activate"), [this, name = info.name]() {
      activateController(name);
    });
  } else if (info.state == ControllerState::Active) {
    menu.addAction(tr("Deactivate"), [this, name = info.name]() {
      deactivateController(name);
    });
  }

  menu.addSeparator();
  menu.addAction(tr("Refresh"), this, &RosControlPanel::refreshControllers);

  menu.exec(controllerTree_->mapToGlobal(pos));
}

void RosControlPanel::onInterfaceContextMenu(const QPoint& pos) {
  QTreeWidgetItem* item = interfaceTree_->itemAt(pos);
  if (!item) return;

  QMenu menu;
  menu.addAction(tr("Refresh Interfaces"), this, &RosControlPanel::refreshHardwareInterfaces);
  menu.exec(interfaceTree_->mapToGlobal(pos));
}

void RosControlPanel::onFilterChanged() {
  updateControllerTree(controllers_);
}

void RosControlPanel::onCommandFinished(int exitCode, QProcess::ExitStatus exitStatus) {
  Q_UNUSED(exitCode);
  Q_UNUSED(exitStatus);

  // Handle async command completion if needed
  pendingAction_.clear();
  pendingController_.clear();
}

}  // namespace ros_weaver
