#include "ros_weaver/widgets/lifecycle_panel.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QPainter>
#include <QMessageBox>

namespace ros_weaver {

LifecyclePanel::LifecyclePanel(QWidget* parent)
  : QWidget(parent)
  , discoverButton_(nullptr)
  , refreshButton_(nullptr)
  , statusLabel_(nullptr)
  , nodeTable_(nullptr)
  , stateMachineGroup_(nullptr)
  , stateMachineWidget_(nullptr)
  , selectedNodeLabel_(nullptr)
  , controlsGroup_(nullptr)
  , configureButton_(nullptr)
  , activateButton_(nullptr)
  , deactivateButton_(nullptr)
  , cleanupButton_(nullptr)
  , shutdownButton_(nullptr)
  , batchGroup_(nullptr)
  , configureAllButton_(nullptr)
  , activateAllButton_(nullptr)
  , deactivateAllButton_(nullptr)
  , lifecycleManager_(nullptr)
  , canvas_(nullptr)
  , rosNode_(nullptr)
  , spinning_(false)
  , refreshTimer_(nullptr)
{
  setupUi();
  setupConnections();
  initializeRosNode();

  // Auto-refresh timer
  refreshTimer_ = new QTimer(this);
  connect(refreshTimer_, &QTimer::timeout, this, &LifecyclePanel::refreshStates);
  refreshTimer_->start(5000);  // Refresh every 5 seconds
}

LifecyclePanel::~LifecyclePanel() {
  shutdownRosNode();
}

void LifecyclePanel::setCanvas(WeaverCanvas* canvas) {
  canvas_ = canvas;
}

void LifecyclePanel::syncNodesFromCanvas() {
  // Could sync with canvas nodes here
}

void LifecyclePanel::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(4, 4, 4, 4);
  mainLayout->setSpacing(4);

  // Controls toolbar
  QHBoxLayout* toolbarLayout = new QHBoxLayout();

  discoverButton_ = new QPushButton(tr("Discover"));
  discoverButton_->setToolTip(tr("Discover lifecycle nodes in the ROS graph"));
  toolbarLayout->addWidget(discoverButton_);

  refreshButton_ = new QPushButton(tr("Refresh"));
  refreshButton_->setToolTip(tr("Refresh state of all nodes"));
  toolbarLayout->addWidget(refreshButton_);

  toolbarLayout->addStretch();

  statusLabel_ = new QLabel(tr("Ready"));
  statusLabel_->setStyleSheet("color: gray;");
  toolbarLayout->addWidget(statusLabel_);

  mainLayout->addLayout(toolbarLayout);

  // Node table
  nodeTable_ = new QTableWidget();
  nodeTable_->setColumnCount(4);
  nodeTable_->setHorizontalHeaderLabels({tr(""), tr("Node"), tr("State"), tr("Last Change")});
  nodeTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
  nodeTable_->setSelectionMode(QAbstractItemView::ExtendedSelection);
  nodeTable_->setAlternatingRowColors(true);
  nodeTable_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  nodeTable_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  nodeTable_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  nodeTable_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
  nodeTable_->verticalHeader()->setVisible(false);
  mainLayout->addWidget(nodeTable_, 2);

  // State machine visualization
  stateMachineGroup_ = new QGroupBox(tr("State Machine"));
  QVBoxLayout* smLayout = new QVBoxLayout(stateMachineGroup_);

  selectedNodeLabel_ = new QLabel(tr("Select a node to view state machine"));
  selectedNodeLabel_->setAlignment(Qt::AlignCenter);
  selectedNodeLabel_->setStyleSheet("color: gray; font-style: italic;");
  smLayout->addWidget(selectedNodeLabel_);

  stateMachineWidget_ = new QWidget();
  stateMachineWidget_->setMinimumHeight(150);
  stateMachineWidget_->setStyleSheet("background-color: #1e1e1e; border-radius: 4px;");
  smLayout->addWidget(stateMachineWidget_);

  mainLayout->addWidget(stateMachineGroup_);

  // Transition controls
  controlsGroup_ = new QGroupBox(tr("Transitions"));
  QHBoxLayout* controlsLayout = new QHBoxLayout(controlsGroup_);

  configureButton_ = new QPushButton(tr("Configure"));
  configureButton_->setToolTip(tr("Transition to Inactive state"));
  configureButton_->setStyleSheet("QPushButton { background-color: #607D8B; color: white; }");
  controlsLayout->addWidget(configureButton_);

  activateButton_ = new QPushButton(tr("Activate"));
  activateButton_->setToolTip(tr("Transition to Active state"));
  activateButton_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }");
  controlsLayout->addWidget(activateButton_);

  deactivateButton_ = new QPushButton(tr("Deactivate"));
  deactivateButton_->setToolTip(tr("Transition to Inactive state"));
  deactivateButton_->setStyleSheet("QPushButton { background-color: #FF9800; color: white; }");
  controlsLayout->addWidget(deactivateButton_);

  cleanupButton_ = new QPushButton(tr("Cleanup"));
  cleanupButton_->setToolTip(tr("Transition to Unconfigured state"));
  cleanupButton_->setStyleSheet("QPushButton { background-color: #9E9E9E; color: white; }");
  controlsLayout->addWidget(cleanupButton_);

  shutdownButton_ = new QPushButton(tr("Shutdown"));
  shutdownButton_->setToolTip(tr("Shutdown the node"));
  shutdownButton_->setStyleSheet("QPushButton { background-color: #F44336; color: white; }");
  controlsLayout->addWidget(shutdownButton_);

  mainLayout->addWidget(controlsGroup_);

  // Batch controls
  batchGroup_ = new QGroupBox(tr("Batch Operations"));
  QHBoxLayout* batchLayout = new QHBoxLayout(batchGroup_);

  configureAllButton_ = new QPushButton(tr("Configure All"));
  batchLayout->addWidget(configureAllButton_);

  activateAllButton_ = new QPushButton(tr("Activate All"));
  batchLayout->addWidget(activateAllButton_);

  deactivateAllButton_ = new QPushButton(tr("Deactivate All"));
  batchLayout->addWidget(deactivateAllButton_);

  mainLayout->addWidget(batchGroup_);

  updateButtonStates();
}

void LifecyclePanel::setupConnections() {
  connect(discoverButton_, &QPushButton::clicked,
          this, &LifecyclePanel::onDiscoverClicked);
  connect(refreshButton_, &QPushButton::clicked,
          this, &LifecyclePanel::onRefreshClicked);

  connect(nodeTable_, &QTableWidget::itemSelectionChanged,
          this, &LifecyclePanel::onTableSelectionChanged);
  connect(nodeTable_, &QTableWidget::cellDoubleClicked,
          this, &LifecyclePanel::onTableDoubleClicked);

  connect(configureButton_, &QPushButton::clicked,
          this, &LifecyclePanel::onConfigureClicked);
  connect(activateButton_, &QPushButton::clicked,
          this, &LifecyclePanel::onActivateClicked);
  connect(deactivateButton_, &QPushButton::clicked,
          this, &LifecyclePanel::onDeactivateClicked);
  connect(cleanupButton_, &QPushButton::clicked,
          this, &LifecyclePanel::onCleanupClicked);
  connect(shutdownButton_, &QPushButton::clicked,
          this, &LifecyclePanel::onShutdownClicked);

  connect(configureAllButton_, &QPushButton::clicked, this, [this]() {
    QStringList nodes = getSelectedNodeNames();
    if (!nodes.isEmpty()) {
      lifecycleManager_->configureAll(nodes);
    }
  });

  connect(activateAllButton_, &QPushButton::clicked, this, [this]() {
    QStringList nodes = getSelectedNodeNames();
    if (!nodes.isEmpty()) {
      lifecycleManager_->activateAll(nodes);
    }
  });

  connect(deactivateAllButton_, &QPushButton::clicked, this, [this]() {
    QStringList nodes = getSelectedNodeNames();
    if (!nodes.isEmpty()) {
      lifecycleManager_->deactivateAll(nodes);
    }
  });
}

void LifecyclePanel::initializeRosNode() {
  try {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    rosNode_ = std::make_shared<rclcpp::Node>("ros_weaver_lifecycle_panel");

    lifecycleManager_ = new LifecycleManager(this);
    lifecycleManager_->setRosNode(rosNode_);

    connect(lifecycleManager_, &LifecycleManager::stateChanged,
            this, &LifecyclePanel::onStateChanged);
    connect(lifecycleManager_, &LifecycleManager::transitionFailed,
            this, &LifecyclePanel::onTransitionFailed);
    connect(lifecycleManager_, &LifecycleManager::transitionSucceeded,
            this, &LifecyclePanel::onTransitionSucceeded);
    connect(lifecycleManager_, &LifecycleManager::lifecycleNodesDiscovered,
            this, &LifecyclePanel::onLifecycleNodesDiscovered);
    connect(lifecycleManager_, &LifecycleManager::discoveryStarted, this, [this]() {
      statusLabel_->setText(tr("Discovering..."));
      discoverButton_->setEnabled(false);
    });
    connect(lifecycleManager_, &LifecycleManager::discoveryCompleted, this, [this]() {
      statusLabel_->setText(tr("Ready"));
      discoverButton_->setEnabled(true);
    });

    // Start spinner thread
    spinning_ = true;
    spinThread_ = std::thread([this]() {
      while (spinning_ && rclcpp::ok()) {
        rclcpp::spin_some(rosNode_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });

    // Initial discovery
    QTimer::singleShot(1000, this, &LifecyclePanel::discoverNodes);

  } catch (const std::exception& e) {
    std::cerr << "LifecyclePanel: Failed to init ROS: " << e.what() << std::endl;
  }
}

void LifecyclePanel::shutdownRosNode() {
  spinning_ = false;
  if (spinThread_.joinable()) {
    spinThread_.join();
  }
  rosNode_.reset();
}

void LifecyclePanel::discoverNodes() {
  if (lifecycleManager_) {
    lifecycleManager_->discoverLifecycleNodes();
  }
}

void LifecyclePanel::refreshStates() {
  if (!lifecycleManager_) return;

  for (int row = 0; row < nodeTable_->rowCount(); ++row) {
    QTableWidgetItem* nameItem = nodeTable_->item(row, COL_NAME);
    if (nameItem) {
      QString nodeName = nameItem->data(Qt::UserRole).toString();
      LifecycleState state = lifecycleManager_->getNodeState(nodeName);

      LifecycleNodeInfo info;
      info.nodeName = nodeName;
      info.currentState = state;
      info.lastStateChange = QDateTime::currentDateTime();
      updateNodeRow(nodeName, info);
    }
  }
}

void LifecyclePanel::onDiscoverClicked() {
  discoverNodes();
}

void LifecyclePanel::onRefreshClicked() {
  refreshStates();
}

void LifecyclePanel::onTableSelectionChanged() {
  selectedNode_ = getSelectedNodeName();
  updateButtonStates();
  updateStateMachineDisplay();

  if (!selectedNode_.isEmpty()) {
    selectedNodeLabel_->setText(selectedNode_);
    emit nodeSelected(selectedNode_);
  } else {
    selectedNodeLabel_->setText(tr("Select a node to view state machine"));
  }
}

void LifecyclePanel::onTableDoubleClicked(int row, int column) {
  Q_UNUSED(column);

  QTableWidgetItem* item = nodeTable_->item(row, COL_NAME);
  if (item) {
    QString nodeName = item->data(Qt::UserRole).toString();
    emit highlightNodeRequested(nodeName);
  }
}

void LifecyclePanel::onConfigureClicked() {
  QString node = getSelectedNodeName();
  if (!node.isEmpty()) {
    lifecycleManager_->requestTransition(node, LifecycleTransition::Configure);
    statusLabel_->setText(tr("Configuring %1...").arg(node));
  }
}

void LifecyclePanel::onActivateClicked() {
  QString node = getSelectedNodeName();
  if (!node.isEmpty()) {
    lifecycleManager_->requestTransition(node, LifecycleTransition::Activate);
    statusLabel_->setText(tr("Activating %1...").arg(node));
  }
}

void LifecyclePanel::onDeactivateClicked() {
  QString node = getSelectedNodeName();
  if (!node.isEmpty()) {
    lifecycleManager_->requestTransition(node, LifecycleTransition::Deactivate);
    statusLabel_->setText(tr("Deactivating %1...").arg(node));
  }
}

void LifecyclePanel::onCleanupClicked() {
  QString node = getSelectedNodeName();
  if (!node.isEmpty()) {
    lifecycleManager_->requestTransition(node, LifecycleTransition::CleanUp);
    statusLabel_->setText(tr("Cleaning up %1...").arg(node));
  }
}

void LifecyclePanel::onShutdownClicked() {
  QString node = getSelectedNodeName();
  if (!node.isEmpty()) {
    int ret = QMessageBox::warning(this, tr("Shutdown Node"),
        tr("Are you sure you want to shutdown '%1'?\nThis will finalize the node.")
            .arg(node),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);

    if (ret == QMessageBox::Yes) {
      lifecycleManager_->requestTransition(node, LifecycleTransition::InactiveShutdown);
      statusLabel_->setText(tr("Shutting down %1...").arg(node));
    }
  }
}

void LifecyclePanel::onStateChanged(const QString& nodeName, LifecycleState newState) {
  LifecycleNodeInfo info;
  info.nodeName = nodeName;
  info.currentState = newState;
  info.lastStateChange = QDateTime::currentDateTime();

  updateNodeRow(nodeName, info);
  statusLabel_->setText(tr("%1: %2").arg(nodeName, LifecycleManager::stateToString(newState)));

  if (nodeName == selectedNode_) {
    updateStateMachineDisplay();
  }
}

void LifecyclePanel::onTransitionFailed(const QString& nodeName, const QString& error) {
  statusLabel_->setText(tr("Failed: %1").arg(error));
  QMessageBox::warning(this, tr("Transition Failed"),
                       tr("Failed to transition '%1':\n%2").arg(nodeName, error));
}

void LifecyclePanel::onTransitionSucceeded(const QString& nodeName, LifecycleState newState) {
  statusLabel_->setText(tr("%1 -> %2")
                            .arg(nodeName, LifecycleManager::stateToString(newState)));
}

void LifecyclePanel::onLifecycleNodesDiscovered(const QStringList& nodes) {
  nodeTable_->setRowCount(0);

  for (const QString& nodeName : nodes) {
    LifecycleState state = lifecycleManager_->getNodeState(nodeName);

    LifecycleNodeInfo info;
    info.nodeName = nodeName;
    info.currentState = state;
    info.lastStateChange = QDateTime::currentDateTime();

    addNodeRow(nodeName, info);
    lifecycleManager_->monitorNode(nodeName);
  }

  statusLabel_->setText(tr("Found %1 lifecycle node(s)").arg(nodes.size()));
}

void LifecyclePanel::updateStateMachineDisplay() {
  stateMachineWidget_->update();
}

void LifecyclePanel::paintEvent(QPaintEvent* event) {
  QWidget::paintEvent(event);

  if (stateMachineWidget_ && !selectedNode_.isEmpty()) {
    QPainter painter(stateMachineWidget_);
    drawStateMachine(&painter, stateMachineWidget_->rect());
  }
}

void LifecyclePanel::updateNodeRow(const QString& nodeName, const LifecycleNodeInfo& info) {
  int row = findNodeRow(nodeName);
  if (row < 0) {
    addNodeRow(nodeName, info);
    return;
  }

  // Status icon
  QTableWidgetItem* statusItem = nodeTable_->item(row, COL_STATUS);
  if (statusItem) {
    statusItem->setText(getStateIcon(info.currentState));
    statusItem->setForeground(getStateColor(info.currentState));
  }

  // State
  QTableWidgetItem* stateItem = nodeTable_->item(row, COL_STATE);
  if (stateItem) {
    stateItem->setText(LifecycleManager::stateToString(info.currentState));
    stateItem->setForeground(getStateColor(info.currentState));
  }

  // Last change
  QTableWidgetItem* timeItem = nodeTable_->item(row, COL_LAST_CHANGE);
  if (timeItem) {
    timeItem->setText(info.lastStateChange.toString("HH:mm:ss"));
  }
}

int LifecyclePanel::findNodeRow(const QString& nodeName) {
  for (int row = 0; row < nodeTable_->rowCount(); ++row) {
    QTableWidgetItem* item = nodeTable_->item(row, COL_NAME);
    if (item && item->data(Qt::UserRole).toString() == nodeName) {
      return row;
    }
  }
  return -1;
}

void LifecyclePanel::addNodeRow(const QString& nodeName, const LifecycleNodeInfo& info) {
  int row = nodeTable_->rowCount();
  nodeTable_->insertRow(row);

  // Status icon
  QTableWidgetItem* statusItem = new QTableWidgetItem(getStateIcon(info.currentState));
  statusItem->setForeground(getStateColor(info.currentState));
  statusItem->setTextAlignment(Qt::AlignCenter);
  nodeTable_->setItem(row, COL_STATUS, statusItem);

  // Name
  QTableWidgetItem* nameItem = new QTableWidgetItem(nodeName);
  nameItem->setData(Qt::UserRole, nodeName);
  nodeTable_->setItem(row, COL_NAME, nameItem);

  // State
  QTableWidgetItem* stateItem = new QTableWidgetItem(
      LifecycleManager::stateToString(info.currentState));
  stateItem->setForeground(getStateColor(info.currentState));
  nodeTable_->setItem(row, COL_STATE, stateItem);

  // Last change
  QTableWidgetItem* timeItem = new QTableWidgetItem(
      info.lastStateChange.toString("HH:mm:ss"));
  nodeTable_->setItem(row, COL_LAST_CHANGE, timeItem);
}

void LifecyclePanel::updateButtonStates() {
  bool hasSelection = !selectedNode_.isEmpty();

  configureButton_->setEnabled(hasSelection);
  activateButton_->setEnabled(hasSelection);
  deactivateButton_->setEnabled(hasSelection);
  cleanupButton_->setEnabled(hasSelection);
  shutdownButton_->setEnabled(hasSelection);

  bool hasMultipleSelection = nodeTable_->selectedItems().size() > 1;
  configureAllButton_->setEnabled(hasMultipleSelection);
  activateAllButton_->setEnabled(hasMultipleSelection);
  deactivateAllButton_->setEnabled(hasMultipleSelection);
}

QString LifecyclePanel::getSelectedNodeName() const {
  QList<QTableWidgetItem*> selected = nodeTable_->selectedItems();
  for (QTableWidgetItem* item : selected) {
    if (item->column() == COL_NAME) {
      return item->data(Qt::UserRole).toString();
    }
  }
  return QString();
}

QStringList LifecyclePanel::getSelectedNodeNames() const {
  QStringList names;
  QList<QTableWidgetItem*> selected = nodeTable_->selectedItems();
  for (QTableWidgetItem* item : selected) {
    if (item->column() == COL_NAME) {
      names.append(item->data(Qt::UserRole).toString());
    }
  }
  return names;
}

QColor LifecyclePanel::getStateColor(LifecycleState state) const {
  switch (state) {
    case LifecycleState::Unconfigured:
      return QColor(150, 150, 150);  // Gray
    case LifecycleState::Inactive:
      return QColor(255, 200, 50);   // Yellow/Amber
    case LifecycleState::Active:
      return QColor(100, 200, 100);  // Green
    case LifecycleState::Finalized:
      return QColor(200, 100, 100);  // Red
    case LifecycleState::Configuring:
    case LifecycleState::Activating:
    case LifecycleState::Deactivating:
    case LifecycleState::CleaningUp:
      return QColor(100, 150, 255);  // Blue (transitioning)
    case LifecycleState::ErrorProcessing:
      return QColor(255, 100, 100);  // Bright red
    default:
      return QColor(100, 100, 100);  // Dark gray
  }
}

QString LifecyclePanel::getStateIcon(LifecycleState state) const {
  switch (state) {
    case LifecycleState::Unconfigured:
      return "U";
    case LifecycleState::Inactive:
      return "I";
    case LifecycleState::Active:
      return "A";
    case LifecycleState::Finalized:
      return "F";
    case LifecycleState::Configuring:
    case LifecycleState::Activating:
    case LifecycleState::Deactivating:
    case LifecycleState::CleaningUp:
    case LifecycleState::ShuttingDown:
      return "...";
    case LifecycleState::ErrorProcessing:
      return "!";
    default:
      return "?";
  }
}

void LifecyclePanel::drawStateMachine(QPainter* painter, const QRect& rect) {
  painter->setRenderHint(QPainter::Antialiasing);

  // Get current state for the selected node
  LifecycleState currentState = LifecycleState::Unknown;
  if (lifecycleManager_ && !selectedNode_.isEmpty()) {
    currentState = lifecycleManager_->getNodeState(selectedNode_);
  }

  // Define state positions
  struct StateInfo {
    QString name;
    LifecycleState state;
    QPointF pos;
  };

  int cx = rect.center().x();
  int cy = rect.center().y();
  int radius = 30;
  int spacing = 100;

  std::vector<StateInfo> states = {
      {"Unconfigured", LifecycleState::Unconfigured, QPointF(cx - spacing, cy)},
      {"Inactive", LifecycleState::Inactive, QPointF(cx, cy)},
      {"Active", LifecycleState::Active, QPointF(cx + spacing, cy)},
      {"Finalized", LifecycleState::Finalized, QPointF(cx, cy + 60)}
  };

  // Draw transitions (arrows)
  painter->setPen(QPen(QColor(80, 80, 80), 2));

  // Unconfigured -> Inactive
  painter->drawLine(QPointF(cx - spacing + radius, cy),
                    QPointF(cx - radius, cy));

  // Inactive -> Active
  painter->drawLine(QPointF(cx + radius, cy),
                    QPointF(cx + spacing - radius, cy));

  // Inactive -> Unconfigured (below)
  painter->drawLine(QPointF(cx - radius, cy + 10),
                    QPointF(cx - spacing + radius, cy + 10));

  // Active -> Inactive (below)
  painter->drawLine(QPointF(cx + spacing - radius, cy + 10),
                    QPointF(cx + radius, cy + 10));

  // Inactive -> Finalized
  painter->drawLine(QPointF(cx, cy + radius),
                    QPointF(cx, cy + 60 - radius));

  // Draw states
  for (const auto& info : states) {
    QColor color = getStateColor(info.state);
    bool isCurrent = (info.state == currentState);

    QRectF stateRect(info.pos.x() - radius, info.pos.y() - radius,
                     radius * 2, radius * 2);

    // Draw circle
    if (isCurrent) {
      painter->setBrush(color);
      painter->setPen(QPen(color.lighter(120), 3));
    } else {
      painter->setBrush(color.darker(150));
      painter->setPen(QPen(color.darker(120), 1));
    }
    painter->drawEllipse(stateRect);

    // Draw state name
    painter->setPen(isCurrent ? Qt::white : QColor(200, 200, 200));
    painter->setFont(QFont("Sans", 8, isCurrent ? QFont::Bold : QFont::Normal));
    painter->drawText(stateRect.adjusted(0, 0, 0, 0), Qt::AlignCenter,
                      info.name.left(1));  // Just first letter

    // Draw full name below
    QRectF labelRect(info.pos.x() - 50, info.pos.y() + radius + 5, 100, 20);
    painter->drawText(labelRect, Qt::AlignCenter, info.name);
  }
}

}  // namespace ros_weaver
