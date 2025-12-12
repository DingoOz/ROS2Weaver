#include "ros_weaver/widgets/tf_tree_panel.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/package_block.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QMenu>
#include <QClipboard>
#include <QApplication>
#include <QDateTime>
#include <QScrollArea>
#include <QFileInfo>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

namespace ros_weaver {

TFTreePanel::TFTreePanel(QWidget* parent)
  : QWidget(parent)
  , splitter_(nullptr)
  , searchEdit_(nullptr)
  , viewModeCombo_(nullptr)
  , refreshButton_(nullptr)
  , liveButton_(nullptr)
  , statusLabel_(nullptr)
  , treeWidget_(nullptr)
  , detailsPanel_(nullptr)
  , frameNameLabel_(nullptr)
  , parentLabel_(nullptr)
  , childrenLabel_(nullptr)
  , transformLabel_(nullptr)
  , statusInfoLabel_(nullptr)
  , rateLabel_(nullptr)
  , linksGroupBox_(nullptr)
  , linksLayout_(nullptr)
  , canvas_(nullptr)
  , rosNode_(nullptr)
  , spinning_(false)
  , listening_(false)
  , tfBuffer_(nullptr)
  , tfListener_(nullptr)
  , updateTimer_(nullptr)
  , liveUpdateEnabled_(true)
  , lastRateCalcTime_(0)
{
  qRegisterMetaType<TFFrameInfo>("TFFrameInfo");

  setupUi();
  setupConnections();
}

TFTreePanel::~TFTreePanel() {
  stopListening();
  shutdownRosNode();
}

void TFTreePanel::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(4, 4, 4, 4);
  mainLayout->setSpacing(4);

  // Toolbar
  mainLayout->addWidget(createToolbar());

  // Splitter for tree and details
  splitter_ = new QSplitter(Qt::Horizontal, this);

  setupTreeWidget();
  splitter_->addWidget(treeWidget_);

  setupDetailsPanel();
  splitter_->addWidget(detailsPanel_);

  splitter_->setStretchFactor(0, 2);
  splitter_->setStretchFactor(1, 1);

  mainLayout->addWidget(splitter_);

  // Status bar
  statusLabel_ = new QLabel(tr("Click 'Live' button to start listening to TF"), this);
  statusLabel_->setStyleSheet("color: gray; font-size: 10px;");
  mainLayout->addWidget(statusLabel_);

  // Update timer
  updateTimer_ = new QTimer(this);
  updateTimer_->setInterval(UPDATE_INTERVAL_MS);
  connect(updateTimer_, &QTimer::timeout, this, &TFTreePanel::onUpdateTimer);
}

QWidget* TFTreePanel::createToolbar() {
  QWidget* toolbar = new QWidget(this);
  QHBoxLayout* layout = new QHBoxLayout(toolbar);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(4);

  // Search
  searchEdit_ = new QLineEdit(toolbar);
  searchEdit_->setPlaceholderText(tr("Search frames..."));
  searchEdit_->setClearButtonEnabled(true);
  searchEdit_->setMaximumWidth(150);
  layout->addWidget(searchEdit_);

  // View mode
  viewModeCombo_ = new QComboBox(toolbar);
  viewModeCombo_->addItem(tr("Tree View"));
  viewModeCombo_->addItem(tr("Flat List"));
  viewModeCombo_->setToolTip(tr("Display mode"));
  layout->addWidget(viewModeCombo_);

  layout->addStretch();

  // Refresh button
  refreshButton_ = new QPushButton(tr("Refresh"), toolbar);
  refreshButton_->setToolTip(tr("Refresh TF tree (F5)"));
  layout->addWidget(refreshButton_);

  // Live toggle
  liveButton_ = new QToolButton(toolbar);
  liveButton_->setText(tr("Live"));
  liveButton_->setCheckable(true);
  liveButton_->setChecked(false);  // Start unchecked - user clicks to start listening
  liveButton_->setToolTip(tr("Click to start listening to TF transforms"));
  layout->addWidget(liveButton_);

  return toolbar;
}

void TFTreePanel::setupTreeWidget() {
  treeWidget_ = new QTreeWidget(this);
  treeWidget_->setHeaderLabels({tr("Frame"), tr("Rate"), tr("Status")});
  treeWidget_->setColumnCount(3);
  treeWidget_->setAlternatingRowColors(true);
  treeWidget_->setContextMenuPolicy(Qt::CustomContextMenu);
  treeWidget_->setSelectionMode(QAbstractItemView::SingleSelection);

  // Column widths
  treeWidget_->header()->setStretchLastSection(true);
  treeWidget_->setColumnWidth(0, 180);
  treeWidget_->setColumnWidth(1, 70);
  treeWidget_->setColumnWidth(2, 80);
}

void TFTreePanel::setupDetailsPanel() {
  detailsPanel_ = new QWidget(this);
  QVBoxLayout* layout = new QVBoxLayout(detailsPanel_);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(8);

  // Frame name
  frameNameLabel_ = new QLabel(tr("Select a frame"), detailsPanel_);
  frameNameLabel_->setStyleSheet("font-weight: bold; font-size: 14px;");
  layout->addWidget(frameNameLabel_);

  // Separator
  QFrame* line = new QFrame(detailsPanel_);
  line->setFrameShape(QFrame::HLine);
  line->setFrameShadow(QFrame::Sunken);
  layout->addWidget(line);

  // Hierarchy info
  QGroupBox* hierarchyBox = new QGroupBox(tr("Hierarchy"), detailsPanel_);
  QVBoxLayout* hierarchyLayout = new QVBoxLayout(hierarchyBox);
  parentLabel_ = new QLabel(tr("Parent: -"), hierarchyBox);
  childrenLabel_ = new QLabel(tr("Children: -"), hierarchyBox);
  childrenLabel_->setWordWrap(true);
  hierarchyLayout->addWidget(parentLabel_);
  hierarchyLayout->addWidget(childrenLabel_);
  layout->addWidget(hierarchyBox);

  // Transform info
  QGroupBox* transformBox = new QGroupBox(tr("Transform"), detailsPanel_);
  QVBoxLayout* transformLayout = new QVBoxLayout(transformBox);
  transformLabel_ = new QLabel(tr("No transform data"), transformBox);
  transformLabel_->setFont(QFont("Monospace", 9));
  transformLabel_->setWordWrap(true);
  transformLayout->addWidget(transformLabel_);
  layout->addWidget(transformBox);

  // Status info
  QGroupBox* statusBox = new QGroupBox(tr("Status"), detailsPanel_);
  QVBoxLayout* statusLayout = new QVBoxLayout(statusBox);
  statusInfoLabel_ = new QLabel(tr("Status: Unknown"), statusBox);
  rateLabel_ = new QLabel(tr("Rate: -"), statusBox);
  statusLayout->addWidget(statusInfoLabel_);
  statusLayout->addWidget(rateLabel_);
  layout->addWidget(statusBox);

  // Links
  linksGroupBox_ = new QGroupBox(tr("Links"), detailsPanel_);
  linksLayout_ = new QVBoxLayout(linksGroupBox_);
  QLabel* noLinks = new QLabel(tr("No links found"), linksGroupBox_);
  noLinks->setStyleSheet("color: gray;");
  linksLayout_->addWidget(noLinks);
  layout->addWidget(linksGroupBox_);

  layout->addStretch();
}

void TFTreePanel::setupConnections() {
  connect(searchEdit_, &QLineEdit::textChanged, this, &TFTreePanel::setSearchFilter);
  connect(refreshButton_, &QPushButton::clicked, this, &TFTreePanel::refreshTree);
  connect(liveButton_, &QToolButton::toggled, this, [this](bool checked) {
    liveUpdateEnabled_ = checked;
    if (checked) {
      startListening();
    } else {
      stopListening();
    }
  });

  connect(treeWidget_, &QTreeWidget::itemClicked, this, &TFTreePanel::onFrameSelected);
  connect(treeWidget_, &QTreeWidget::itemDoubleClicked, this, &TFTreePanel::onFrameDoubleClicked);
  connect(treeWidget_, &QTreeWidget::customContextMenuRequested, this, &TFTreePanel::onContextMenu);

  connect(this, &TFTreePanel::tfDataReceived, this, &TFTreePanel::onTfDataReceived, Qt::QueuedConnection);
}

void TFTreePanel::initializeRosNode() {
  if (rosNode_) return;

  try {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    rosNode_ = std::make_shared<rclcpp::Node>("ros_weaver_tf_viewer_" +
      std::to_string(QDateTime::currentMSecsSinceEpoch()));

    // Create TF buffer and listener
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(rosNode_->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, rosNode_);

    // Subscribe to /tf and /tf_static for raw access
    tfSubscription_ = rosNode_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 100,
      [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        processTfMessage(msg, false);
      });

    tfStaticSubscription_ = rosNode_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(100).transient_local(),
      [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        processTfMessage(msg, true);
      });

    // Start spinning
    spinning_ = true;
    spinThread_ = std::make_unique<std::thread>([this]() {
      while (spinning_.load() && rclcpp::ok()) {
        rclcpp::spin_some(rosNode_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });

  } catch (const std::exception& e) {
    statusLabel_->setText(tr("ROS2 initialization failed: %1").arg(e.what()));
  }
}

void TFTreePanel::shutdownRosNode() {
  spinning_ = false;
  listening_ = false;

  if (spinThread_ && spinThread_->joinable()) {
    spinThread_->join();
  }
  spinThread_.reset();

  tfSubscription_.reset();
  tfStaticSubscription_.reset();
  tfListener_.reset();
  tfBuffer_.reset();
  rosNode_.reset();
}

void TFTreePanel::startListening() {
  if (listening_.load()) return;

  initializeRosNode();

  if (rosNode_) {
    listening_ = true;
    liveUpdateEnabled_ = true;
    lastRateCalcTime_ = QDateTime::currentMSecsSinceEpoch();

    updateTimer_->start();

    statusLabel_->setText(tr("Listening to TF..."));
    liveButton_->setStyleSheet("background-color: #4a7;");

    // Sync button state (without re-triggering signal)
    liveButton_->blockSignals(true);
    liveButton_->setChecked(true);
    liveButton_->blockSignals(false);
  }
}

void TFTreePanel::stopListening() {
  listening_ = false;
  liveUpdateEnabled_ = false;
  updateTimer_->stop();
  statusLabel_->setText(tr("Stopped - Click 'Live' to start"));
  liveButton_->setStyleSheet("");

  // Sync button state (without re-triggering signal)
  liveButton_->blockSignals(true);
  liveButton_->setChecked(false);
  liveButton_->blockSignals(false);
}

void TFTreePanel::toggleListening() {
  if (listening_.load()) {
    stopListening();
  } else {
    startListening();
  }
}

void TFTreePanel::processTfMessage(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool isStatic) {
  if (!listening_.load()) return;

  std::lock_guard<std::mutex> lock(framesMutex_);
  qint64 now = QDateTime::currentMSecsSinceEpoch();

  for (const auto& transform : msg->transforms) {
    QString childFrame = QString::fromStdString(transform.child_frame_id);
    QString parentFrame = QString::fromStdString(transform.header.frame_id);

    // Remove leading slashes if present
    if (childFrame.startsWith('/')) childFrame = childFrame.mid(1);
    if (parentFrame.startsWith('/')) parentFrame = parentFrame.mid(1);

    TFFrameInfo& frame = frames_[childFrame];
    frame.name = childFrame;
    frame.parentName = parentFrame;
    frame.isStatic = isStatic;
    frame.lastUpdateTime = now;

    // Transform data
    frame.translationX = transform.transform.translation.x;
    frame.translationY = transform.transform.translation.y;
    frame.translationZ = transform.transform.translation.z;
    frame.rotationX = transform.transform.rotation.x;
    frame.rotationY = transform.transform.rotation.y;
    frame.rotationZ = transform.transform.rotation.z;
    frame.rotationW = transform.transform.rotation.w;

    // Update rate tracking
    frameUpdateCounts_[childFrame]++;

    // Update parent's children list
    if (!parentFrame.isEmpty() && frames_.contains(parentFrame)) {
      if (!frames_[parentFrame].childNames.contains(childFrame)) {
        frames_[parentFrame].childNames.append(childFrame);
      }
    }

    // Track root frames
    if (parentFrame.isEmpty()) {
      rootFrames_.insert(childFrame);
    } else {
      rootFrames_.remove(childFrame);
      // Ensure parent exists in our map
      if (!frames_.contains(parentFrame)) {
        frames_[parentFrame].name = parentFrame;
        rootFrames_.insert(parentFrame);
      }
    }
  }

  emit tfDataReceived();
}

void TFTreePanel::onTfDataReceived() {
  // Batched in timer for performance
}

void TFTreePanel::onUpdateTimer() {
  rebuildTree();
  checkFrameHealth();

  // Calculate rates
  qint64 now = QDateTime::currentMSecsSinceEpoch();
  double elapsed = (now - lastRateCalcTime_) / 1000.0;

  if (elapsed >= 1.0) {
    std::lock_guard<std::mutex> lock(framesMutex_);
    for (auto it = frameUpdateCounts_.begin(); it != frameUpdateCounts_.end(); ++it) {
      if (frames_.contains(it.key())) {
        frames_[it.key()].updateRateHz = it.value() / elapsed;
      }
    }
    frameUpdateCounts_.clear();
    lastRateCalcTime_ = now;
  }

  // Update status
  int frameCount = 0;
  int staticCount = 0;
  int orphanCount = 0;
  {
    std::lock_guard<std::mutex> lock(framesMutex_);
    frameCount = frames_.size();
    for (const auto& frame : frames_) {
      if (frame.isStatic) staticCount++;
      if (frame.status == FrameStatus::Orphan) orphanCount++;
    }
  }

  QString status = tr("Frames: %1 | Static: %2 | Dynamic: %3")
    .arg(frameCount)
    .arg(staticCount)
    .arg(frameCount - staticCount);
  if (orphanCount > 0) {
    status += tr(" | Orphans: %1").arg(orphanCount);
  }
  statusLabel_->setText(status);
}

void TFTreePanel::rebuildTree() {
  std::lock_guard<std::mutex> lock(framesMutex_);

  // Build parent-child relationships
  QMap<QString, QStringList> children;
  QSet<QString> allFrames;

  for (auto it = frames_.begin(); it != frames_.end(); ++it) {
    allFrames.insert(it.key());
    if (!it.value().parentName.isEmpty()) {
      children[it.value().parentName].append(it.key());
    }
  }

  // Find root frames (frames with no parent or parent not in set)
  QStringList roots;
  for (const QString& frame : allFrames) {
    const TFFrameInfo& info = frames_[frame];
    if (info.parentName.isEmpty() || !allFrames.contains(info.parentName)) {
      roots.append(frame);
    }
  }

  // Build tree recursively
  std::function<void(QTreeWidgetItem*, const QString&, int)> buildSubtree =
    [&](QTreeWidgetItem* parent, const QString& frameName, int depth) {
      QTreeWidgetItem* item = findOrCreateFrameItem(frameName);

      if (frames_.contains(frameName)) {
        TFFrameInfo& frame = frames_[frameName];
        frame.depth = depth;
        frame.childNames = children[frameName];
        updateFrameItem(item, frame);
      }

      // Move to correct parent
      if (parent) {
        if (item->parent() != parent) {
          if (item->parent()) {
            item->parent()->removeChild(item);
          } else {
            int idx = treeWidget_->indexOfTopLevelItem(item);
            if (idx >= 0) treeWidget_->takeTopLevelItem(idx);
          }
          parent->addChild(item);
        }
      } else {
        if (item->parent()) {
          item->parent()->removeChild(item);
          treeWidget_->addTopLevelItem(item);
        } else if (treeWidget_->indexOfTopLevelItem(item) < 0) {
          treeWidget_->addTopLevelItem(item);
        }
      }

      // Build children
      for (const QString& child : children[frameName]) {
        buildSubtree(item, child, depth + 1);
      }
    };

  // Build from roots
  for (const QString& root : roots) {
    buildSubtree(nullptr, root, 0);
  }

  // Expand all by default
  treeWidget_->expandAll();
}

QTreeWidgetItem* TFTreePanel::findOrCreateFrameItem(const QString& frameName) {
  if (frameItems_.contains(frameName)) {
    return frameItems_[frameName];
  }

  QTreeWidgetItem* item = new QTreeWidgetItem();
  item->setText(0, frameName);
  item->setData(0, Qt::UserRole, frameName);
  frameItems_[frameName] = item;
  return item;
}

void TFTreePanel::updateFrameItem(QTreeWidgetItem* item, const TFFrameInfo& frame) {
  item->setText(0, frame.name);

  // Rate column
  if (frame.isStatic) {
    item->setText(1, tr("Static"));
  } else if (frame.updateRateHz > 0) {
    item->setText(1, QString("%1 Hz").arg(frame.updateRateHz, 0, 'f', 1));
  } else {
    item->setText(1, "-");
  }

  // Status column
  item->setText(2, getStatusText(frame.status));

  // Color coding
  QColor color = getStatusColor(frame.status);
  item->setForeground(0, color);
  item->setForeground(2, color);

  // Icon based on status
  if (frame.status == FrameStatus::Orphan) {
    item->setIcon(0, QIcon()); // Could add warning icon
    item->setToolTip(0, tr("Orphan frame - no connection to main tree"));
  } else if (frame.isStatic) {
    item->setToolTip(0, tr("Static transform"));
  } else {
    item->setToolTip(0, tr("Dynamic transform at %1 Hz").arg(frame.updateRateHz, 0, 'f', 1));
  }
}

void TFTreePanel::checkFrameHealth() {
  std::lock_guard<std::mutex> lock(framesMutex_);
  qint64 now = QDateTime::currentMSecsSinceEpoch();

  for (auto it = frames_.begin(); it != frames_.end(); ++it) {
    TFFrameInfo& frame = it.value();

    if (frame.isStatic) {
      frame.status = FrameStatus::Static;
      continue;
    }

    double ageSec = (now - frame.lastUpdateTime) / 1000.0;

    if (frame.parentName.isEmpty() && !rootFrames_.contains(frame.name)) {
      frame.status = FrameStatus::Orphan;
    } else if (ageSec > CRITICAL_STALE_SEC) {
      frame.status = FrameStatus::Critical;
    } else if (ageSec > STALE_THRESHOLD_SEC) {
      frame.status = FrameStatus::Stale;
    } else {
      frame.status = FrameStatus::Healthy;
    }
  }
}

QColor TFTreePanel::getStatusColor(FrameStatus status) const {
  switch (status) {
    case FrameStatus::Healthy: return QColor(100, 200, 100);
    case FrameStatus::Stale: return QColor(200, 200, 100);
    case FrameStatus::Critical: return QColor(200, 100, 100);
    case FrameStatus::Static: return QColor(100, 150, 200);
    case FrameStatus::Orphan: return QColor(200, 150, 100);
    default: return QColor(150, 150, 150);
  }
}

QString TFTreePanel::getStatusText(FrameStatus status) const {
  switch (status) {
    case FrameStatus::Healthy: return tr("OK");
    case FrameStatus::Stale: return tr("Stale");
    case FrameStatus::Critical: return tr("Critical");
    case FrameStatus::Static: return tr("Static");
    case FrameStatus::Orphan: return tr("Orphan");
    default: return tr("Unknown");
  }
}

void TFTreePanel::refreshTree() {
  rebuildTree();
  checkFrameHealth();
}

void TFTreePanel::setSearchFilter(const QString& text) {
  QString filter = text.toLower();

  for (int i = 0; i < treeWidget_->topLevelItemCount(); ++i) {
    std::function<bool(QTreeWidgetItem*)> filterItem = [&](QTreeWidgetItem* item) -> bool {
      bool matches = item->text(0).toLower().contains(filter);
      bool childMatches = false;

      for (int j = 0; j < item->childCount(); ++j) {
        if (filterItem(item->child(j))) {
          childMatches = true;
        }
      }

      item->setHidden(!matches && !childMatches && !filter.isEmpty());
      return matches || childMatches;
    };

    filterItem(treeWidget_->topLevelItem(i));
  }
}

void TFTreePanel::onFrameSelected(QTreeWidgetItem* item, int /*column*/) {
  if (!item) return;

  QString frameName = item->data(0, Qt::UserRole).toString();
  selectedFrame_ = frameName;
  updateDetailsPanel(frameName);
  emit frameSelected(frameName);
}

void TFTreePanel::onFrameDoubleClicked(QTreeWidgetItem* item, int /*column*/) {
  if (!item) return;

  QString frameName = item->data(0, Qt::UserRole).toString();
  emit showFrameOnCanvas(frameName);
}

void TFTreePanel::updateDetailsPanel(const QString& frameName) {
  std::lock_guard<std::mutex> lock(framesMutex_);

  if (!frames_.contains(frameName)) {
    frameNameLabel_->setText(tr("Frame not found"));
    return;
  }

  const TFFrameInfo& frame = frames_[frameName];

  frameNameLabel_->setText(frame.name);
  parentLabel_->setText(tr("Parent: %1").arg(frame.parentName.isEmpty() ? tr("(root)") : frame.parentName));

  QString children = frame.childNames.isEmpty() ? tr("(none)") : frame.childNames.join(", ");
  childrenLabel_->setText(tr("Children: %1").arg(children));

  transformLabel_->setText(formatTransform(frame));

  QString statusText = getStatusText(frame.status);
  QColor statusColor = getStatusColor(frame.status);
  statusInfoLabel_->setText(tr("Status: <span style='color: %1;'>%2</span>")
    .arg(statusColor.name()).arg(statusText));
  statusInfoLabel_->setTextFormat(Qt::RichText);

  if (frame.isStatic) {
    rateLabel_->setText(tr("Type: Static"));
  } else {
    rateLabel_->setText(tr("Rate: %1 Hz").arg(frame.updateRateHz, 0, 'f', 1));
  }

  // Update links
  // Clear existing links
  while (QLayoutItem* item = linksLayout_->takeAt(0)) {
    if (item->widget()) delete item->widget();
    delete item;
  }

  // Discover and display links (simplified for now)
  FrameLinks links = discoverFrameLinks(frameName);

  if (links.canvasLinks.isEmpty() && links.yamlLinks.isEmpty()) {
    QLabel* noLinks = new QLabel(tr("No links found"), linksGroupBox_);
    noLinks->setStyleSheet("color: gray;");
    linksLayout_->addWidget(noLinks);
  } else {
    for (const auto& link : links.canvasLinks) {
      QPushButton* btn = new QPushButton(
        QString("Block: %1 (%2)").arg(link.blockName, link.parameterName), linksGroupBox_);
      btn->setFlat(true);
      btn->setCursor(Qt::PointingHandCursor);
      btn->setStyleSheet("text-align: left; color: #5af;");
      btn->setProperty("blockPtr", QVariant::fromValue(reinterpret_cast<quintptr>(link.block)));
      connect(btn, &QPushButton::clicked, this, &TFTreePanel::onLinkClicked);
      linksLayout_->addWidget(btn);
    }

    for (const auto& link : links.yamlLinks) {
      QFileInfo fi(link.filePath);
      QPushButton* btn = new QPushButton(
        QString("YAML: %1:%2").arg(fi.fileName()).arg(link.lineNumber), linksGroupBox_);
      btn->setFlat(true);
      btn->setCursor(Qt::PointingHandCursor);
      btn->setStyleSheet("text-align: left; color: #5af;");
      btn->setProperty("yamlPath", link.filePath);
      btn->setProperty("yamlLine", link.lineNumber);
      connect(btn, &QPushButton::clicked, this, &TFTreePanel::onLinkClicked);
      linksLayout_->addWidget(btn);
    }
  }
}

QString TFTreePanel::formatTransform(const TFFrameInfo& frame) const {
  QString result;

  result += tr("Translation:\n");
  result += QString("  x: %1\n").arg(frame.translationX, 0, 'f', 4);
  result += QString("  y: %1\n").arg(frame.translationY, 0, 'f', 4);
  result += QString("  z: %1\n").arg(frame.translationZ, 0, 'f', 4);
  result += "\n";

  result += tr("Rotation (quaternion):\n");
  result += QString("  x: %1\n").arg(frame.rotationX, 0, 'f', 4);
  result += QString("  y: %1\n").arg(frame.rotationY, 0, 'f', 4);
  result += QString("  z: %1\n").arg(frame.rotationZ, 0, 'f', 4);
  result += QString("  w: %1\n").arg(frame.rotationW, 0, 'f', 4);
  result += "\n";

  result += formatRPY(frame.rotationX, frame.rotationY, frame.rotationZ, frame.rotationW);

  return result;
}

QString TFTreePanel::formatRPY(double x, double y, double z, double w) const {
  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  QString result = tr("Rotation (RPY):\n");
  result += QString("  roll:  %1 rad (%2 deg)\n")
    .arg(roll, 0, 'f', 4)
    .arg(roll * 180.0 / M_PI, 0, 'f', 2);
  result += QString("  pitch: %1 rad (%2 deg)\n")
    .arg(pitch, 0, 'f', 4)
    .arg(pitch * 180.0 / M_PI, 0, 'f', 2);
  result += QString("  yaw:   %1 rad (%2 deg)")
    .arg(yaw, 0, 'f', 4)
    .arg(yaw * 180.0 / M_PI, 0, 'f', 2);

  return result;
}

FrameLinks TFTreePanel::discoverFrameLinks(const QString& frameName) {
  FrameLinks links;

  if (!canvas_) return links;

  // Search canvas blocks for parameters mentioning this frame
  QGraphicsScene* scene = canvas_->scene();
  if (!scene) return links;

  for (QGraphicsItem* item : scene->items()) {
    PackageBlock* block = dynamic_cast<PackageBlock*>(item);
    if (!block) continue;

    // Check block parameters
    for (const auto& param : block->parameters()) {
      QString paramValue = param.currentValue.toString();
      if (paramValue.contains(frameName)) {
        CanvasFrameLink link;
        link.blockName = block->packageName();
        link.parameterName = param.name;
        link.parameterValue = paramValue;
        link.block = block;
        links.canvasLinks.append(link);
      }
    }
  }

  return links;
}

void TFTreePanel::onLinkClicked() {
  QPushButton* btn = qobject_cast<QPushButton*>(sender());
  if (!btn) return;

  // Check if it's a block link
  quintptr blockPtr = btn->property("blockPtr").value<quintptr>();
  if (blockPtr) {
    PackageBlock* block = reinterpret_cast<PackageBlock*>(blockPtr);
    if (block && canvas_) {
      canvas_->centerOn(block);
      block->setBlockSelected(true);
    }
    return;
  }

  // Check if it's a YAML link
  QString yamlPath = btn->property("yamlPath").toString();
  int yamlLine = btn->property("yamlLine").toInt();
  if (!yamlPath.isEmpty()) {
    emit openYamlAtLine(yamlPath, yamlLine);
  }
}

void TFTreePanel::onContextMenu(const QPoint& pos) {
  QTreeWidgetItem* item = treeWidget_->itemAt(pos);
  if (!item) return;

  QString frameName = item->data(0, Qt::UserRole).toString();

  QMenu menu(this);

  menu.addAction(tr("Copy Frame Name"), [frameName]() {
    QApplication::clipboard()->setText(frameName);
  });

  menu.addSeparator();

  menu.addAction(tr("Show on Canvas"), [this, frameName]() {
    emit showFrameOnCanvas(frameName);
  });

  menu.addAction(tr("Find Links"), [this, frameName]() {
    updateDetailsPanel(frameName);
  });

  menu.exec(treeWidget_->viewport()->mapToGlobal(pos));
}

}  // namespace ros_weaver
