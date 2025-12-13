#include "ros_weaver/widgets/plot_panel.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QMessageBox>
#include <QDateTime>
#include <QApplication>
#include <QClipboard>
#include <QFileDialog>
#include <QInputDialog>
#include <QDialog>
#include <QTextStream>
#include <QDialogButtonBox>
#include <QTreeWidget>
#include <QScreen>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cmath>
#include <numeric>
#include <fstream>

namespace ros_weaver {

PlotPanel::PlotPanel(QWidget* parent)
  : QWidget(parent)
  , mainSplitter_(nullptr)
  , addTopicButton_(nullptr)
  , clearButton_(nullptr)
  , pauseButton_(nullptr)
  , timeWindowCombo_(nullptr)
  , exportButton_(nullptr)
  , screenshotButton_(nullptr)
  , statusLabel_(nullptr)
  , chart_(nullptr)
  , chartView_(nullptr)
  , axisX_(nullptr)
  , axisY_(nullptr)
  , seriesList_(nullptr)
  , detailsPanel_(nullptr)
  , selectedSeriesLabel_(nullptr)
  , statsLabel_(nullptr)
  , rateLabel_(nullptr)
  , canvas_(nullptr)
  , rosNode_(nullptr)
  , spinning_(false)
  , timeWindowSeconds_(30)
  , paused_(false)
  , updateTimer_(nullptr)
  , colorIndex_(0)
{
  // Register meta type for cross-thread signals
  static bool typeRegistered = false;
  if (!typeRegistered) {
    qRegisterMetaType<PlotDataPoint>("PlotDataPoint");
    typeRegistered = true;
  }

  // Initialize color palette
  colorPalette_ = {
    QColor(0, 114, 189),    // Blue
    QColor(217, 83, 25),    // Orange
    QColor(237, 177, 32),   // Yellow
    QColor(126, 47, 142),   // Purple
    QColor(119, 172, 48),   // Green
    QColor(77, 190, 238),   // Cyan
    QColor(162, 20, 47),    // Red
    QColor(0, 128, 128),    // Teal
  };

  setupUi();
  setupConnections();

  // Initialize ROS node
  initializeRosNode();

  // Start update timer
  updateTimer_ = new QTimer(this);
  connect(updateTimer_, &QTimer::timeout, this, &PlotPanel::onUpdateTimer);
  updateTimer_->start(UPDATE_INTERVAL_MS);
}

PlotPanel::~PlotPanel() {
  if (updateTimer_) {
    updateTimer_->stop();
  }
  shutdownRosNode();
}

void PlotPanel::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(4, 4, 4, 4);
  mainLayout->setSpacing(4);

  // Toolbar
  mainLayout->addWidget(createToolbar());

  // Main content area with splitter
  mainSplitter_ = new QSplitter(Qt::Horizontal, this);

  // Left: Series list
  setupSeriesList();
  mainSplitter_->addWidget(seriesList_);

  // Right: Chart and details
  QWidget* rightWidget = new QWidget();
  QVBoxLayout* rightLayout = new QVBoxLayout(rightWidget);
  rightLayout->setContentsMargins(0, 0, 0, 0);
  rightLayout->setSpacing(4);

  setupChart();
  rightLayout->addWidget(chartView_, 3);

  setupDetailsPanel();
  rightLayout->addWidget(detailsPanel_, 1);

  mainSplitter_->addWidget(rightWidget);

  // Set splitter proportions
  mainSplitter_->setStretchFactor(0, 1);  // Series list
  mainSplitter_->setStretchFactor(1, 4);  // Chart area
  mainSplitter_->setSizes({150, 600});

  mainLayout->addWidget(mainSplitter_);

  // Status bar
  statusLabel_ = new QLabel(tr("Ready. Click '+ Add Topic' to start plotting."), this);
  statusLabel_->setStyleSheet("color: gray; font-size: 10px;");
  mainLayout->addWidget(statusLabel_);
}

QWidget* PlotPanel::createToolbar() {
  QWidget* toolbar = new QWidget();
  QHBoxLayout* layout = new QHBoxLayout(toolbar);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(4);

  // Add topic button
  addTopicButton_ = new QPushButton(tr("+ Add Topic"));
  addTopicButton_->setToolTip(tr("Add a topic to plot"));
  layout->addWidget(addTopicButton_);

  // Clear button
  clearButton_ = new QPushButton(tr("Clear"));
  clearButton_->setToolTip(tr("Clear all data (keep subscriptions)"));
  layout->addWidget(clearButton_);

  layout->addStretch();

  // Time window selector
  QLabel* windowLabel = new QLabel(tr("Window:"));
  layout->addWidget(windowLabel);

  timeWindowCombo_ = new QComboBox();
  timeWindowCombo_->addItem(tr("10 sec"), 10);
  timeWindowCombo_->addItem(tr("30 sec"), 30);
  timeWindowCombo_->addItem(tr("1 min"), 60);
  timeWindowCombo_->addItem(tr("5 min"), 300);
  timeWindowCombo_->addItem(tr("10 min"), 600);
  timeWindowCombo_->setCurrentIndex(1);  // 30 sec default
  timeWindowCombo_->setToolTip(tr("Time window to display"));
  layout->addWidget(timeWindowCombo_);

  layout->addSpacing(10);

  // Pause button
  pauseButton_ = new QToolButton();
  pauseButton_->setText(tr("Pause"));
  pauseButton_->setCheckable(true);
  pauseButton_->setToolTip(tr("Pause/Resume plotting"));
  layout->addWidget(pauseButton_);

  layout->addSpacing(10);

  // Export button
  exportButton_ = new QPushButton(tr("Export CSV"));
  exportButton_->setToolTip(tr("Export data to CSV file"));
  layout->addWidget(exportButton_);

  // Screenshot button
  screenshotButton_ = new QPushButton(tr("Screenshot"));
  screenshotButton_->setToolTip(tr("Save plot as image"));
  layout->addWidget(screenshotButton_);

  return toolbar;
}

void PlotPanel::setupChart() {
  chart_ = new QChart();
  chart_->setTitle(tr("Real-Time Plot"));
  chart_->legend()->setVisible(true);
  chart_->legend()->setAlignment(Qt::AlignBottom);
  chart_->setAnimationOptions(QChart::NoAnimation);  // Disable for real-time

  // Create axes
  axisX_ = new QValueAxis();
  axisX_->setTitleText(tr("Time (seconds ago)"));
  axisX_->setRange(-timeWindowSeconds_, 0);
  axisX_->setLabelFormat("%.0f");
  chart_->addAxis(axisX_, Qt::AlignBottom);

  axisY_ = new QValueAxis();
  axisY_->setTitleText(tr("Value"));
  axisY_->setRange(-1, 1);
  chart_->addAxis(axisY_, Qt::AlignLeft);

  // Create chart view
  chartView_ = new QChartView(chart_);
  chartView_->setRenderHint(QPainter::Antialiasing);
  chartView_->setRubberBand(QChartView::RectangleRubberBand);
}

void PlotPanel::setupSeriesList() {
  seriesList_ = new QListWidget();
  seriesList_->setContextMenuPolicy(Qt::CustomContextMenu);
  seriesList_->setSelectionMode(QAbstractItemView::SingleSelection);
  seriesList_->setMinimumWidth(120);
  seriesList_->setMaximumWidth(250);
}

void PlotPanel::setupDetailsPanel() {
  detailsPanel_ = new QWidget();
  QVBoxLayout* layout = new QVBoxLayout(detailsPanel_);
  layout->setContentsMargins(4, 4, 4, 4);
  layout->setSpacing(2);

  selectedSeriesLabel_ = new QLabel(tr("Select a series to view statistics"));
  selectedSeriesLabel_->setStyleSheet("font-weight: bold; font-size: 11px;");
  layout->addWidget(selectedSeriesLabel_);

  statsLabel_ = new QLabel();
  statsLabel_->setStyleSheet("font-size: 10px; font-family: monospace;");
  layout->addWidget(statsLabel_);

  rateLabel_ = new QLabel();
  rateLabel_->setStyleSheet("font-size: 10px; color: #666;");
  layout->addWidget(rateLabel_);

  layout->addStretch();
}

void PlotPanel::setupConnections() {
  // Toolbar buttons
  connect(addTopicButton_, &QPushButton::clicked, this, &PlotPanel::onAddTopicClicked);
  connect(clearButton_, &QPushButton::clicked, this, &PlotPanel::clearData);
  connect(pauseButton_, &QToolButton::toggled, [this](bool checked) {
    if (checked) {
      pause();
      pauseButton_->setText(tr("Resume"));
    } else {
      resume();
      pauseButton_->setText(tr("Pause"));
    }
  });
  connect(timeWindowCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &PlotPanel::onTimeWindowChanged);
  connect(exportButton_, &QPushButton::clicked, this, &PlotPanel::exportToCsv);
  connect(screenshotButton_, &QPushButton::clicked, this, &PlotPanel::takeScreenshot);

  // Series list
  connect(seriesList_, &QListWidget::customContextMenuRequested,
          this, &PlotPanel::onSeriesListContextMenu);
  connect(seriesList_, &QListWidget::currentItemChanged,
          this, &PlotPanel::onSeriesSelectionChanged);

  // Cross-thread data signal
  connect(this, &PlotPanel::dataReceived,
          this, &PlotPanel::onDataReceived, Qt::QueuedConnection);
}

void PlotPanel::initializeRosNode() {
  if (rosNode_) return;

  try {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    std::string nodeName = "ros_weaver_plot_panel_" +
                           std::to_string(QDateTime::currentMSecsSinceEpoch());
    rosNode_ = std::make_shared<rclcpp::Node>(nodeName);

    spinning_ = true;
    spinThread_ = std::make_unique<std::thread>([this]() {
      while (spinning_ && rclcpp::ok()) {
        rclcpp::spin_some(rosNode_);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    });

  } catch (const std::exception& e) {
    statusLabel_->setText(tr("ROS2 Error: %1").arg(e.what()));
  }
}

void PlotPanel::shutdownRosNode() {
  spinning_ = false;

  {
    std::lock_guard<std::mutex> lock(dataMutex_);
    subscriptions_.clear();
  }

  if (spinThread_ && spinThread_->joinable()) {
    spinThread_->join();
  }
  spinThread_.reset();
  rosNode_.reset();
}

void PlotPanel::onAddTopicClicked() {
  showAddTopicDialog();
}

void PlotPanel::showAddTopicDialog() {
  if (!rosNode_) {
    initializeRosNode();
    if (!rosNode_) {
      QMessageBox::warning(this, tr("Error"), tr("Failed to initialize ROS2 node"));
      return;
    }
  }

  // Get available topics
  auto topicNamesAndTypes = rosNode_->get_topic_names_and_types();

  if (topicNamesAndTypes.empty()) {
    QMessageBox::information(this, tr("No Topics"),
      tr("No ROS2 topics found. Make sure ROS2 is running."));
    return;
  }

  // Create selection dialog
  QDialog dialog(this);
  dialog.setWindowTitle(tr("Select Topic and Field"));
  dialog.resize(500, 400);

  QVBoxLayout* layout = new QVBoxLayout(&dialog);

  QLabel* instructionLabel = new QLabel(tr("Select a topic and field to plot:"));
  layout->addWidget(instructionLabel);

  // Tree widget for topic/field selection
  QTreeWidget* tree = new QTreeWidget();
  tree->setHeaderLabels({tr("Topic / Field"), tr("Type")});
  tree->setColumnWidth(0, 300);

  // Populate tree with topics and their plottable fields
  for (const auto& [topicName, types] : topicNamesAndTypes) {
    if (types.empty()) continue;

    QString topic = QString::fromStdString(topicName);
    QString type = QString::fromStdString(types[0]);

    QTreeWidgetItem* topicItem = new QTreeWidgetItem(tree);
    topicItem->setText(0, topic);
    topicItem->setText(1, type);
    topicItem->setData(0, Qt::UserRole, topic);
    topicItem->setData(1, Qt::UserRole, type);

    // Add plottable fields based on message type
    QList<PlottableField> fields = discoverPlottableFields(type);
    for (const PlottableField& field : fields) {
      QTreeWidgetItem* fieldItem = new QTreeWidgetItem(topicItem);
      fieldItem->setText(0, field.fieldPath.isEmpty() ? "(value)" : field.fieldPath);
      fieldItem->setText(1, field.fieldType);
      fieldItem->setData(0, Qt::UserRole, field.fieldPath);
      fieldItem->setData(1, Qt::UserRole + 1, true);  // Mark as field item
    }

    if (fields.isEmpty()) {
      // If we couldn't detect fields, allow selecting the topic directly
      // (for simple numeric types)
      topicItem->setData(1, Qt::UserRole + 1, true);
    }
  }

  tree->expandAll();
  layout->addWidget(tree);

  // Buttons
  QDialogButtonBox* buttons = new QDialogButtonBox(
    QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  layout->addWidget(buttons);

  if (dialog.exec() == QDialog::Accepted) {
    QTreeWidgetItem* selectedItem = tree->currentItem();
    if (!selectedItem) return;

    QString topicName;
    QString fieldPath;
    QString msgType;

    // Check if this is a field item or topic item
    if (selectedItem->data(1, Qt::UserRole + 1).toBool()) {
      if (selectedItem->parent()) {
        // Field under topic
        topicName = selectedItem->parent()->data(0, Qt::UserRole).toString();
        msgType = selectedItem->parent()->data(1, Qt::UserRole).toString();
        fieldPath = selectedItem->data(0, Qt::UserRole).toString();
      } else {
        // Topic item directly selectable (simple numeric type)
        topicName = selectedItem->data(0, Qt::UserRole).toString();
        msgType = selectedItem->data(1, Qt::UserRole).toString();
        fieldPath = "";
      }
    } else {
      // Topic selected but not a simple type - show warning
      QMessageBox::information(this, tr("Select Field"),
        tr("Please expand the topic and select a specific field to plot."));
      return;
    }

    addPlot(topicName, fieldPath);
    subscribeToTopic(topicName, msgType, fieldPath);
  }
}

QList<PlottableField> PlotPanel::discoverPlottableFields(const QString& msgType) {
  QList<PlottableField> fields;

  // Handle common message types
  if (msgType == "std_msgs/msg/Float32" ||
      msgType == "std_msgs/msg/Float64" ||
      msgType == "std_msgs/msg/Int32" ||
      msgType == "std_msgs/msg/Int64" ||
      msgType == "std_msgs/msg/UInt32" ||
      msgType == "std_msgs/msg/UInt64" ||
      msgType == "std_msgs/msg/Bool") {
    // Simple types - the value itself is plottable
    PlottableField f;
    f.fieldPath = "";  // Direct value
    f.fieldType = msgType.split("/").last();
    fields.append(f);
  }
  else if (msgType == "geometry_msgs/msg/Twist") {
    fields.append({"linear.x", "float64", "", -1});
    fields.append({"linear.y", "float64", "", -1});
    fields.append({"linear.z", "float64", "", -1});
    fields.append({"angular.x", "float64", "", -1});
    fields.append({"angular.y", "float64", "", -1});
    fields.append({"angular.z", "float64", "", -1});
  }
  else if (msgType == "geometry_msgs/msg/TwistStamped") {
    fields.append({"twist.linear.x", "float64", "", -1});
    fields.append({"twist.linear.y", "float64", "", -1});
    fields.append({"twist.linear.z", "float64", "", -1});
    fields.append({"twist.angular.x", "float64", "", -1});
    fields.append({"twist.angular.y", "float64", "", -1});
    fields.append({"twist.angular.z", "float64", "", -1});
  }
  else if (msgType == "geometry_msgs/msg/Vector3") {
    fields.append({"x", "float64", "", -1});
    fields.append({"y", "float64", "", -1});
    fields.append({"z", "float64", "", -1});
  }
  else if (msgType == "geometry_msgs/msg/Point") {
    fields.append({"x", "float64", "", -1});
    fields.append({"y", "float64", "", -1});
    fields.append({"z", "float64", "", -1});
  }
  else if (msgType == "geometry_msgs/msg/Pose") {
    fields.append({"position.x", "float64", "", -1});
    fields.append({"position.y", "float64", "", -1});
    fields.append({"position.z", "float64", "", -1});
    fields.append({"orientation.x", "float64", "", -1});
    fields.append({"orientation.y", "float64", "", -1});
    fields.append({"orientation.z", "float64", "", -1});
    fields.append({"orientation.w", "float64", "", -1});
  }
  else if (msgType == "geometry_msgs/msg/PoseStamped") {
    fields.append({"pose.position.x", "float64", "", -1});
    fields.append({"pose.position.y", "float64", "", -1});
    fields.append({"pose.position.z", "float64", "", -1});
    fields.append({"pose.orientation.x", "float64", "", -1});
    fields.append({"pose.orientation.y", "float64", "", -1});
    fields.append({"pose.orientation.z", "float64", "", -1});
    fields.append({"pose.orientation.w", "float64", "", -1});
  }
  else if (msgType == "nav_msgs/msg/Odometry") {
    fields.append({"pose.pose.position.x", "float64", "", -1});
    fields.append({"pose.pose.position.y", "float64", "", -1});
    fields.append({"pose.pose.position.z", "float64", "", -1});
    fields.append({"twist.twist.linear.x", "float64", "", -1});
    fields.append({"twist.twist.linear.y", "float64", "", -1});
    fields.append({"twist.twist.angular.z", "float64", "", -1});
  }
  else if (msgType == "sensor_msgs/msg/Imu") {
    fields.append({"angular_velocity.x", "float64", "", -1});
    fields.append({"angular_velocity.y", "float64", "", -1});
    fields.append({"angular_velocity.z", "float64", "", -1});
    fields.append({"linear_acceleration.x", "float64", "", -1});
    fields.append({"linear_acceleration.y", "float64", "", -1});
    fields.append({"linear_acceleration.z", "float64", "", -1});
  }
  else if (msgType == "sensor_msgs/msg/BatteryState") {
    fields.append({"voltage", "float32", "", -1});
    fields.append({"current", "float32", "", -1});
    fields.append({"charge", "float32", "", -1});
    fields.append({"percentage", "float32", "", -1});
  }
  else if (msgType == "sensor_msgs/msg/Temperature") {
    fields.append({"temperature", "float64", "", -1});
  }
  else if (msgType == "sensor_msgs/msg/Range") {
    fields.append({"range", "float32", "", -1});
  }
  else if (msgType == "sensor_msgs/msg/JointState") {
    // For JointState, we'd need dynamic discovery based on actual message
    // For now, provide common indices
    for (int i = 0; i < 6; ++i) {
      fields.append({QString("position[%1]").arg(i), "float64", "", i});
      fields.append({QString("velocity[%1]").arg(i), "float64", "", i});
      fields.append({QString("effort[%1]").arg(i), "float64", "", i});
    }
  }

  return fields;
}

void PlotPanel::addPlot(const QString& topicName, const QString& fieldPath) {
  QString fullPath = fieldPath.isEmpty() ? topicName : topicName + "/" + fieldPath;

  {
    std::lock_guard<std::mutex> lock(dataMutex_);

    // Check if already exists
    if (plotSeries_.find(fullPath.toStdString()) != plotSeries_.end()) {
      statusLabel_->setText(tr("Already plotting: %1").arg(fullPath));
      return;
    }

    // Create new series
    PlotSeriesInfo info;
    info.topicName = topicName;
    info.fieldPath = fieldPath;
    info.fullPath = fullPath;
    info.color = generateSeriesColor();
    info.series = new QLineSeries();
    info.series->setName(fullPath);
    info.series->setColor(info.color);

    // Add to chart
    chart_->addSeries(info.series);
    info.series->attachAxis(axisX_);
    info.series->attachAxis(axisY_);

    plotSeries_[fullPath.toStdString()] = info;
  }

  // Add to list widget
  QListWidgetItem* item = new QListWidgetItem(fullPath);
  item->setData(Qt::UserRole, fullPath);
  item->setForeground(plotSeries_[fullPath.toStdString()].color);
  seriesList_->addItem(item);

  statusLabel_->setText(tr("Added: %1").arg(fullPath));
  emit plotAdded(fullPath);
}

void PlotPanel::removePlot(const QString& fullPath) {
  {
    std::lock_guard<std::mutex> lock(dataMutex_);

    auto it = plotSeries_.find(fullPath.toStdString());
    if (it == plotSeries_.end()) return;

    // Remove from chart
    chart_->removeSeries(it->second.series);
    delete it->second.series;

    plotSeries_.erase(it);
  }

  // Remove from list widget
  for (int i = 0; i < seriesList_->count(); ++i) {
    if (seriesList_->item(i)->data(Qt::UserRole).toString() == fullPath) {
      delete seriesList_->takeItem(i);
      break;
    }
  }

  statusLabel_->setText(tr("Removed: %1").arg(fullPath));
  emit plotRemoved(fullPath);
}

void PlotPanel::clearAllPlots() {
  {
    std::lock_guard<std::mutex> lock(dataMutex_);

    for (auto& [path, info] : plotSeries_) {
      chart_->removeSeries(info.series);
      delete info.series;
    }
    plotSeries_.clear();
    subscriptions_.clear();
  }

  seriesList_->clear();
  statusLabel_->setText(tr("All plots cleared"));
}

void PlotPanel::subscribeToTopic(const QString& topicName, const QString& msgType,
                                  const QString& fieldPath) {
  if (!rosNode_) return;

  std::string topic = topicName.toStdString();
  QString fullPath = fieldPath.isEmpty() ? topicName : topicName + "/" + fieldPath;

  // Check if already subscribed to this topic
  {
    std::lock_guard<std::mutex> lock(dataMutex_);
    if (subscriptions_.find(topic) != subscriptions_.end()) {
      // Already subscribed, just add the field extraction
      return;
    }
  }

  try {
    std::string type = msgType.toStdString();

    auto callback = [this, topicName, msgType, fieldPath, fullPath]
                    (std::shared_ptr<rclcpp::SerializedMessage> msg) {
      if (paused_) return;

      double value = 0.0;
      if (extractFieldValue(msg, msgType, fieldPath, value)) {
        qint64 timestamp = QDateTime::currentMSecsSinceEpoch();
        emit dataReceived(fullPath, value, timestamp);
      }
    };

    auto sub = rosNode_->create_generic_subscription(
      topic, type, rclcpp::QoS(10), callback);

    {
      std::lock_guard<std::mutex> lock(dataMutex_);
      subscriptions_[topic] = sub;
    }

    statusLabel_->setText(tr("Subscribed to: %1").arg(topicName));

  } catch (const std::exception& e) {
    statusLabel_->setText(tr("Subscribe error: %1").arg(e.what()));
  }
}

void PlotPanel::unsubscribeFromTopic(const QString& topicName) {
  std::lock_guard<std::mutex> lock(dataMutex_);
  subscriptions_.erase(topicName.toStdString());
}

bool PlotPanel::extractFieldValue(const std::shared_ptr<rclcpp::SerializedMessage>& msg,
                                   const QString& msgType, const QString& fieldPath,
                                   double& outValue) {
  // This is a simplified extraction that works with common types
  // For full introspection, we'd need rosidl_typesupport_introspection_cpp

  const uint8_t* data = msg->get_rcl_serialized_message().buffer;
  size_t size = msg->get_rcl_serialized_message().buffer_length;

  if (size < 8) return false;  // Too small

  // Skip CDR header (4 bytes)
  data += 4;

  try {
    // Handle simple numeric types
    if (msgType == "std_msgs/msg/Float32") {
      if (size >= 8) {
        float val;
        memcpy(&val, data, sizeof(float));
        outValue = val;
        return true;
      }
    }
    else if (msgType == "std_msgs/msg/Float64") {
      if (size >= 12) {
        double val;
        memcpy(&val, data, sizeof(double));
        outValue = val;
        return true;
      }
    }
    else if (msgType == "std_msgs/msg/Int32") {
      if (size >= 8) {
        int32_t val;
        memcpy(&val, data, sizeof(int32_t));
        outValue = val;
        return true;
      }
    }
    else if (msgType == "std_msgs/msg/Int64") {
      if (size >= 12) {
        int64_t val;
        memcpy(&val, data, sizeof(int64_t));
        outValue = val;
        return true;
      }
    }
    else if (msgType == "std_msgs/msg/Bool") {
      if (size >= 5) {
        outValue = data[0] ? 1.0 : 0.0;
        return true;
      }
    }
    else if (msgType == "geometry_msgs/msg/Twist") {
      // Twist: linear (Vector3) then angular (Vector3)
      // Each Vector3 is 3 doubles = 24 bytes
      if (size >= 52) {  // 4 (header) + 48 (6 doubles)
        double values[6];
        memcpy(values, data, 6 * sizeof(double));

        if (fieldPath == "linear.x") { outValue = values[0]; return true; }
        if (fieldPath == "linear.y") { outValue = values[1]; return true; }
        if (fieldPath == "linear.z") { outValue = values[2]; return true; }
        if (fieldPath == "angular.x") { outValue = values[3]; return true; }
        if (fieldPath == "angular.y") { outValue = values[4]; return true; }
        if (fieldPath == "angular.z") { outValue = values[5]; return true; }
      }
    }
    else if (msgType == "geometry_msgs/msg/Vector3") {
      if (size >= 28) {  // 4 + 24
        double values[3];
        memcpy(values, data, 3 * sizeof(double));

        if (fieldPath == "x") { outValue = values[0]; return true; }
        if (fieldPath == "y") { outValue = values[1]; return true; }
        if (fieldPath == "z") { outValue = values[2]; return true; }
      }
    }
    else if (msgType == "geometry_msgs/msg/Point") {
      if (size >= 28) {
        double values[3];
        memcpy(values, data, 3 * sizeof(double));

        if (fieldPath == "x") { outValue = values[0]; return true; }
        if (fieldPath == "y") { outValue = values[1]; return true; }
        if (fieldPath == "z") { outValue = values[2]; return true; }
      }
    }
    // Add more message types as needed...

  } catch (...) {
    return false;
  }

  return false;
}

void PlotPanel::onDataReceived(const QString& fullPath, double value, qint64 timestamp) {
  std::lock_guard<std::mutex> lock(dataMutex_);

  auto it = plotSeries_.find(fullPath.toStdString());
  if (it == plotSeries_.end()) return;

  PlotSeriesInfo& info = it->second;

  // Add to buffer
  PlotDataPoint point{timestamp, value};
  info.buffer.push_back(point);

  // Limit buffer size
  while (info.buffer.size() > MAX_BUFFER_SIZE) {
    info.buffer.pop_front();
  }

  // Update statistics
  info.currentValue = value;
  info.messageCount++;

  // Calculate rate
  if (info.lastMessageTime > 0) {
    double dt = (timestamp - info.lastMessageTime) / 1000.0;
    if (dt > 0) {
      info.publishRate = 1.0 / dt;
    }
  }
  info.lastMessageTime = timestamp;
}

void PlotPanel::onUpdateTimer() {
  if (paused_) return;

  updateChart();

  // Update details panel if a series is selected
  if (!selectedSeriesPath_.isEmpty()) {
    updateDetailsPanel();
  }
}

void PlotPanel::updateChart() {
  std::lock_guard<std::mutex> lock(dataMutex_);

  qint64 now = QDateTime::currentMSecsSinceEpoch();
  qint64 windowStart = now - (timeWindowSeconds_ * 1000);

  double globalMin = std::numeric_limits<double>::max();
  double globalMax = std::numeric_limits<double>::lowest();
  bool hasData = false;

  for (auto& [path, info] : plotSeries_) {
    QVector<QPointF> points;
    points.reserve(info.buffer.size());

    // Calculate statistics while building points
    double sum = 0.0;
    double minVal = std::numeric_limits<double>::max();
    double maxVal = std::numeric_limits<double>::lowest();
    int count = 0;

    for (const auto& dp : info.buffer) {
      if (dp.timestamp >= windowStart) {
        // Convert timestamp to seconds relative to now
        double x = (dp.timestamp - now) / 1000.0;
        points.append(QPointF(x, dp.value));

        sum += dp.value;
        minVal = std::min(minVal, dp.value);
        maxVal = std::max(maxVal, dp.value);
        count++;

        globalMin = std::min(globalMin, dp.value);
        globalMax = std::max(globalMax, dp.value);
        hasData = true;
      }
    }

    // Update series data
    info.series->replace(points);

    // Update statistics
    if (count > 0) {
      info.minValue = minVal;
      info.maxValue = maxVal;
      info.meanValue = sum / count;
    }
  }

  // Adjust Y axis
  if (hasData) {
    double range = globalMax - globalMin;
    double margin = range * 0.1;
    if (range < 0.001) margin = 0.5;  // Minimum margin for nearly constant data

    axisY_->setRange(globalMin - margin, globalMax + margin);
  }
}

void PlotPanel::updateStatistics(PlotSeriesInfo& /* info */) {
  // Statistics are updated in updateChart
}

void PlotPanel::updateDetailsPanel() {
  std::lock_guard<std::mutex> lock(dataMutex_);

  auto it = plotSeries_.find(selectedSeriesPath_.toStdString());
  if (it == plotSeries_.end()) {
    selectedSeriesLabel_->setText(tr("Select a series"));
    statsLabel_->clear();
    rateLabel_->clear();
    return;
  }

  const PlotSeriesInfo& info = it->second;

  selectedSeriesLabel_->setText(info.fullPath);
  selectedSeriesLabel_->setStyleSheet(
    QString("font-weight: bold; font-size: 11px; color: %1;").arg(info.color.name()));

  QString stats = tr("Current: %1\nMin: %2  |  Max: %3  |  Mean: %4")
    .arg(info.currentValue, 0, 'f', 4)
    .arg(info.minValue, 0, 'f', 4)
    .arg(info.maxValue, 0, 'f', 4)
    .arg(info.meanValue, 0, 'f', 4);
  statsLabel_->setText(stats);

  QString rate = tr("Messages: %1  |  Rate: %2 Hz")
    .arg(info.messageCount)
    .arg(info.publishRate, 0, 'f', 1);
  rateLabel_->setText(rate);
}

void PlotPanel::onSeriesListContextMenu(const QPoint& pos) {
  QListWidgetItem* item = seriesList_->itemAt(pos);
  if (!item) return;

  QString fullPath = item->data(Qt::UserRole).toString();

  QMenu menu(this);

  // Change color
  QAction* colorAction = menu.addAction(tr("Change Color..."));
  connect(colorAction, &QAction::triggered, [this, fullPath, item]() {
    std::lock_guard<std::mutex> lock(dataMutex_);
    auto it = plotSeries_.find(fullPath.toStdString());
    if (it != plotSeries_.end()) {
      QColor newColor = QColorDialog::getColor(it->second.color, this, tr("Select Color"));
      if (newColor.isValid()) {
        it->second.color = newColor;
        it->second.series->setColor(newColor);
        item->setForeground(newColor);
      }
    }
  });

  menu.addSeparator();

  // Remove
  QAction* removeAction = menu.addAction(tr("Remove"));
  connect(removeAction, &QAction::triggered, [this, fullPath]() {
    removePlot(fullPath);
  });

  menu.exec(seriesList_->viewport()->mapToGlobal(pos));
}

void PlotPanel::onSeriesSelectionChanged() {
  QListWidgetItem* item = seriesList_->currentItem();
  if (item) {
    selectedSeriesPath_ = item->data(Qt::UserRole).toString();
  } else {
    selectedSeriesPath_.clear();
  }
  updateDetailsPanel();
}

void PlotPanel::onTimeWindowChanged(int index) {
  int seconds = timeWindowCombo_->itemData(index).toInt();
  setTimeWindow(seconds);
}

void PlotPanel::setTimeWindow(int seconds) {
  timeWindowSeconds_ = seconds;
  axisX_->setRange(-seconds, 0);
  statusLabel_->setText(tr("Time window: %1 seconds").arg(seconds));
}

void PlotPanel::pause() {
  paused_ = true;
  statusLabel_->setText(tr("Paused"));
}

void PlotPanel::resume() {
  paused_ = false;
  statusLabel_->setText(tr("Resumed"));
}

void PlotPanel::togglePause() {
  if (paused_) {
    resume();
    pauseButton_->setChecked(false);
  } else {
    pause();
    pauseButton_->setChecked(true);
  }
}

void PlotPanel::clearData() {
  std::lock_guard<std::mutex> lock(dataMutex_);

  for (auto& [path, info] : plotSeries_) {
    info.buffer.clear();
    info.series->clear();
    info.minValue = 0.0;
    info.maxValue = 0.0;
    info.meanValue = 0.0;
    info.messageCount = 0;
  }

  statusLabel_->setText(tr("Data cleared"));
}

void PlotPanel::exportToCsv() {
  QString filename = QFileDialog::getSaveFileName(this,
    tr("Export to CSV"), QString(), tr("CSV Files (*.csv)"));

  if (filename.isEmpty()) return;

  if (!filename.endsWith(".csv", Qt::CaseInsensitive)) {
    filename += ".csv";
  }

  std::ofstream file(filename.toStdString());
  if (!file.is_open()) {
    QMessageBox::warning(this, tr("Error"), tr("Could not open file for writing"));
    return;
  }

  std::lock_guard<std::mutex> lock(dataMutex_);

  // Write header
  file << "timestamp";
  for (const auto& [path, info] : plotSeries_) {
    file << "," << path;
  }
  file << "\n";

  // Find all unique timestamps
  std::set<qint64> timestamps;
  for (const auto& [path, info] : plotSeries_) {
    for (const auto& dp : info.buffer) {
      timestamps.insert(dp.timestamp);
    }
  }

  // Write data rows
  for (qint64 ts : timestamps) {
    file << ts;
    for (const auto& [path, info] : plotSeries_) {
      file << ",";
      // Find value at this timestamp
      for (const auto& dp : info.buffer) {
        if (dp.timestamp == ts) {
          file << dp.value;
          break;
        }
      }
    }
    file << "\n";
  }

  file.close();
  statusLabel_->setText(tr("Exported to: %1").arg(filename));
}

void PlotPanel::copyToClipboard() {
  std::lock_guard<std::mutex> lock(dataMutex_);

  QString text;
  QTextStream stream(&text);

  // Header
  stream << "timestamp";
  for (const auto& [path, info] : plotSeries_) {
    stream << "\t" << QString::fromStdString(path);
  }
  stream << "\n";

  // Find all unique timestamps
  std::set<qint64> timestamps;
  for (const auto& [path, info] : plotSeries_) {
    for (const auto& dp : info.buffer) {
      timestamps.insert(dp.timestamp);
    }
  }

  // Data rows (last 100 to avoid huge clipboard)
  int count = 0;
  for (auto it = timestamps.rbegin(); it != timestamps.rend() && count < 100; ++it, ++count) {
    qint64 ts = *it;
    stream << ts;
    for (const auto& [path, info] : plotSeries_) {
      stream << "\t";
      for (const auto& dp : info.buffer) {
        if (dp.timestamp == ts) {
          stream << dp.value;
          break;
        }
      }
    }
    stream << "\n";
  }

  QApplication::clipboard()->setText(text);
  statusLabel_->setText(tr("Data copied to clipboard (%1 rows)").arg(count));
}

void PlotPanel::takeScreenshot() {
  QString filename = QFileDialog::getSaveFileName(this,
    tr("Save Screenshot"), QString(), tr("Images (*.png *.jpg)"));

  if (filename.isEmpty()) return;

  if (!filename.endsWith(".png", Qt::CaseInsensitive) &&
      !filename.endsWith(".jpg", Qt::CaseInsensitive)) {
    filename += ".png";
  }

  QPixmap pixmap = chartView_->grab();
  if (pixmap.save(filename)) {
    statusLabel_->setText(tr("Screenshot saved: %1").arg(filename));
  } else {
    QMessageBox::warning(this, tr("Error"), tr("Could not save screenshot"));
  }
}

QColor PlotPanel::generateSeriesColor() {
  QColor color = colorPalette_[colorIndex_ % colorPalette_.size()];
  colorIndex_++;
  return color;
}

}  // namespace ros_weaver
