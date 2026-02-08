#include "ros_weaver/widgets/plot_panel.hpp"
#include "ros_weaver/widgets/plot_series_config_dialog.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/core/theme_manager.hpp"
#include "ros_weaver/core/constants.hpp"

#include <QtCharts/QLegendMarker>

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
#include <QSettings>
#include <iostream>

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
#include <functional>

namespace ros_weaver {

// ---- SegmentSeriesPool implementation ----

void SegmentSeriesPool::ensureCapacity(int needed, QChart* chart,
                                        QValueAxis* axisX, QValueAxis* axisY) {
  while (pool.size() < needed && pool.size() < constants::plot::MAX_POOL_SIZE) {
    auto* s = new QLineSeries();
    s->setUseOpenGL(false);
    chart->addSeries(s);
    s->attachAxis(axisX);
    s->attachAxis(axisY);
    // Hide from legend
    auto markers = chart->legend()->markers(s);
    for (auto* m : markers) {
      m->setVisible(false);
    }
    pool.append(s);
  }
}

void SegmentSeriesPool::hideUnused() {
  for (int i = activeCount; i < pool.size(); ++i) {
    pool[i]->clear();
    pool[i]->setVisible(false);
  }
}

void SegmentSeriesPool::clearAll(QChart* chart) {
  for (auto* s : pool) {
    chart->removeSeries(s);
    delete s;
  }
  pool.clear();
  activeCount = 0;
}

// ---- PlotPanel implementation ----

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

  // Initialize color palette with defaults
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

  // Load saved settings (may override defaults)
  loadSettings();

  setupUi();
  setupConnections();

  // Initialize ROS node
  initializeRosNode();

  // Start update timer
  updateTimer_ = new QTimer(this);
  connect(updateTimer_, &QTimer::timeout, this, &PlotPanel::onUpdateTimer);
  updateTimer_->start(constants::plot::UPDATE_INTERVAL_MS);
}

PlotPanel::~PlotPanel() {
  std::cerr << "PlotPanel destructor: starting" << std::endl;
  if (updateTimer_) {
    updateTimer_->stop();
  }
  shutdownRosNode();
  std::cerr << "PlotPanel destructor: complete" << std::endl;
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

  // Right: Chart (fills available space)
  setupChart();
  applyChartTheme();  // Apply theme to chart
  mainSplitter_->addWidget(chartView_);

  // Set splitter proportions - use stretch factors only, not absolute sizes
  // to avoid forcing the dock widget to resize when this tab becomes visible
  mainSplitter_->setStretchFactor(0, 0);  // Series list - fixed size
  mainSplitter_->setStretchFactor(1, 1);  // Chart area - stretches to fill
  mainSplitter_->setCollapsible(0, false);
  mainSplitter_->setCollapsible(1, false);

  mainLayout->addWidget(mainSplitter_, 1);  // Splitter takes all available space

  // Compact details bar at bottom (horizontal layout for stats)
  setupDetailsPanel();
  mainLayout->addWidget(detailsPanel_);

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
  clearButton_ = new QPushButton(tr("Clear All"));
  clearButton_->setToolTip(tr("Remove all plots and subscriptions"));
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
  // Prevent chart from forcing dock resize - use expanding policy with small minimum
  chartView_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  chartView_->setMinimumSize(100, 100);
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
  detailsPanel_->setFixedHeight(28);  // Compact height
  QHBoxLayout* layout = new QHBoxLayout(detailsPanel_);
  layout->setContentsMargins(4, 2, 4, 2);
  layout->setSpacing(12);

  selectedSeriesLabel_ = new QLabel(tr("Select a series"));
  selectedSeriesLabel_->setStyleSheet("font-weight: bold; font-size: 10px;");
  layout->addWidget(selectedSeriesLabel_);

  statsLabel_ = new QLabel();
  statsLabel_->setStyleSheet("font-size: 10px; font-family: monospace;");
  layout->addWidget(statsLabel_);

  rateLabel_ = new QLabel();
  rateLabel_->setStyleSheet("font-size: 10px;");
  layout->addWidget(rateLabel_);

  layout->addStretch();
}

void PlotPanel::setupConnections() {
  // Toolbar buttons
  connect(addTopicButton_, &QPushButton::clicked, this, &PlotPanel::onAddTopicClicked);
  connect(clearButton_, &QPushButton::clicked, this, &PlotPanel::clearAllPlots);
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
  connect(seriesList_, &QListWidget::itemDoubleClicked, [this](QListWidgetItem* item) {
    if (item) {
      showSeriesConfigDialog(item->data(Qt::UserRole).toString());
    }
  });

  // Cross-thread data signal
  connect(this, &PlotPanel::dataReceived,
          this, &PlotPanel::onDataReceived, Qt::QueuedConnection);

  // Theme changes
  connect(&ThemeManager::instance(), &ThemeManager::themeChanged,
          this, &PlotPanel::onThemeChanged);
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
  dialog.setWindowTitle(tr("Select Topics and Fields"));
  dialog.resize(550, 450);

  QVBoxLayout* layout = new QVBoxLayout(&dialog);

  QLabel* instructionLabel = new QLabel(tr("Double-click to toggle selection. Double-click a topic to toggle all its fields:"));
  layout->addWidget(instructionLabel);

  // Tree widget for topic/field selection
  QTreeWidget* tree = new QTreeWidget();
  tree->setHeaderLabels({tr("Topic / Field"), tr("Type")});
  tree->setColumnWidth(0, 350);

  // Helper function to toggle an item's check state
  auto toggleItemCheck = [](QTreeWidgetItem* item) {
    if (item->flags() & Qt::ItemIsUserCheckable) {
      Qt::CheckState newState = (item->checkState(0) == Qt::Checked) ? Qt::Unchecked : Qt::Checked;
      item->setCheckState(0, newState);
    }
  };

  // Helper function to toggle all children of a parent
  auto toggleAllChildren = [&toggleItemCheck](QTreeWidgetItem* parent) {
    if (parent->childCount() == 0) return;

    // Determine new state: if any child is unchecked, check all; otherwise uncheck all
    bool anyUnchecked = false;
    for (int i = 0; i < parent->childCount(); ++i) {
      if (parent->child(i)->checkState(0) != Qt::Checked) {
        anyUnchecked = true;
        break;
      }
    }

    Qt::CheckState newState = anyUnchecked ? Qt::Checked : Qt::Unchecked;
    for (int i = 0; i < parent->childCount(); ++i) {
      parent->child(i)->setCheckState(0, newState);
    }
  };

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
      fieldItem->setFlags(fieldItem->flags() | Qt::ItemIsUserCheckable);
      fieldItem->setCheckState(0, Qt::Checked);
    }

    if (fields.isEmpty()) {
      // If we couldn't detect fields, allow selecting the topic directly
      // (for simple numeric types)
      topicItem->setData(1, Qt::UserRole + 1, true);
      topicItem->setFlags(topicItem->flags() | Qt::ItemIsUserCheckable);
      topicItem->setCheckState(0, Qt::Checked);
    }
  }

  tree->expandAll();
  layout->addWidget(tree);

  // Connect double-click handler
  connect(tree, &QTreeWidget::itemDoubleClicked, [&toggleItemCheck, &toggleAllChildren](QTreeWidgetItem* item, int /* column */) {
    if (!item) return;

    if (item->childCount() > 0) {
      // Parent item with children - toggle all children
      toggleAllChildren(item);
    } else {
      // Leaf item - toggle its own check state
      toggleItemCheck(item);
    }
  });

  // Status label showing selection count
  QLabel* statusLabel = new QLabel(tr("0 fields selected"));
  layout->addWidget(statusLabel);

  // Update status when checkboxes change
  connect(tree, &QTreeWidget::itemChanged, [tree, statusLabel]() {
    int count = 0;
    std::function<void(QTreeWidgetItem*)> countChecked = [&count, &countChecked](QTreeWidgetItem* item) {
      if ((item->flags() & Qt::ItemIsUserCheckable) && item->checkState(0) == Qt::Checked) {
        count++;
      }
      for (int i = 0; i < item->childCount(); ++i) {
        countChecked(item->child(i));
      }
    };

    for (int i = 0; i < tree->topLevelItemCount(); ++i) {
      countChecked(tree->topLevelItem(i));
    }

    statusLabel->setText(QObject::tr("%1 field(s) selected").arg(count));
  });

  // Buttons
  QDialogButtonBox* buttons = new QDialogButtonBox(
    QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  layout->addWidget(buttons);

  if (dialog.exec() == QDialog::Accepted) {
    // Collect all checked items and add them
    std::function<void(QTreeWidgetItem*)> processItem = [this, &processItem](QTreeWidgetItem* item) {
      // Check if this item is checked and is a selectable field
      if ((item->flags() & Qt::ItemIsUserCheckable) &&
          item->checkState(0) == Qt::Checked &&
          item->data(1, Qt::UserRole + 1).toBool()) {

        QString topicName;
        QString fieldPath;
        QString msgType;

        if (item->parent()) {
          // Field under topic
          topicName = item->parent()->data(0, Qt::UserRole).toString();
          msgType = item->parent()->data(1, Qt::UserRole).toString();
          fieldPath = item->data(0, Qt::UserRole).toString();
        } else {
          // Topic item directly selectable (simple numeric type)
          topicName = item->data(0, Qt::UserRole).toString();
          msgType = item->data(1, Qt::UserRole).toString();
          fieldPath = "";
        }

        addPlot(topicName, fieldPath);
        subscribeToTopic(topicName, msgType, fieldPath);
      }

      // Process children
      for (int i = 0; i < item->childCount(); ++i) {
        processItem(item->child(i));
      }
    };

    for (int i = 0; i < tree->topLevelItemCount(); ++i) {
      processItem(tree->topLevelItem(i));
    }
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

    // Create new series with default config
    PlotSeriesInfo info;
    info.topicName = topicName;
    info.fieldPath = fieldPath;
    info.fullPath = fullPath;
    info.config = defaultConfig_;
    info.config.color = generateSeriesColor();
    info.series = new QLineSeries();
    info.series->setName(fullPath);

    // Apply color and line thickness from config
    QPen pen(info.config.color);
    pen.setWidth(info.config.lineThickness);
    info.series->setPen(pen);

    // Pre-compute gradient palette if needed
    if (info.config.renderMode == PlotRenderMode::Gradient) {
      computeGradientPalette(info.config);
    }

    // Add to chart
    chart_->addSeries(info.series);
    info.series->attachAxis(axisX_);
    info.series->attachAxis(axisY_);

    plotSeries_[fullPath.toStdString()] = std::move(info);
  }

  // Add to list widget
  QListWidgetItem* item = new QListWidgetItem(fullPath);
  item->setData(Qt::UserRole, fullPath);
  item->setForeground(plotSeries_[fullPath.toStdString()].config.color);
  seriesList_->addItem(item);

  statusLabel_->setText(tr("Added: %1").arg(fullPath));
  emit plotAdded(fullPath);
}

void PlotPanel::removePlot(const QString& fullPath) {
  {
    std::lock_guard<std::mutex> lock(dataMutex_);

    auto it = plotSeries_.find(fullPath.toStdString());
    if (it == plotSeries_.end()) return;

    // Clean up segment pool
    it->second.segmentPool.clearAll(chart_);

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
      info.segmentPool.clearAll(chart_);
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

  // Sample rate decimation
  if (info.config.decimationEnabled && info.config.maxSampleRateHz > 0) {
    double minIntervalMs = 1000.0 / info.config.maxSampleRateHz;
    if (info.config.lastAcceptedTimestamp > 0 &&
        (timestamp - info.config.lastAcceptedTimestamp) < minIntervalMs) {
      return;  // Skip this sample
    }
    info.config.lastAcceptedTimestamp = timestamp;
  }

  // Add to buffer
  PlotDataPoint point{timestamp, value};
  info.buffer.push_back(point);

  // Limit buffer size
  while (static_cast<int>(info.buffer.size()) > constants::plot::MAX_BUFFER_SIZE) {
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
    points.reserve(static_cast<int>(info.buffer.size()));

    // Calculate statistics while building points
    double sum = 0.0;
    double minVal = std::numeric_limits<double>::max();
    double maxVal = std::numeric_limits<double>::lowest();
    int count = 0;

    for (const auto& dp : info.buffer) {
      if (dp.timestamp >= windowStart) {
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

    // Update statistics
    if (count > 0) {
      info.minValue = minVal;
      info.maxValue = maxVal;
      info.meanValue = sum / count;
    }

    // Dispatch to render mode
    switch (info.config.renderMode) {
      case PlotRenderMode::Threshold:
        renderThresholdSeries(info, points);
        break;
      case PlotRenderMode::Gradient:
        renderGradientSeries(info, points);
        break;
      default:
        renderSolidSeries(info, points);
        break;
    }
  }

  // Adjust Y axis
  if (hasData) {
    double range = globalMax - globalMin;
    double margin = range * 0.1;
    if (range < 0.001) margin = 0.5;

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
    QString("font-weight: bold; font-size: 10px; color: %1;").arg(info.config.color.name()));

  // Compact horizontal stats format
  QString stats = tr("Val: %1 | Min: %2 | Max: %3 | Avg: %4")
    .arg(info.currentValue, 0, 'f', 2)
    .arg(info.minValue, 0, 'f', 2)
    .arg(info.maxValue, 0, 'f', 2)
    .arg(info.meanValue, 0, 'f', 2);
  statsLabel_->setText(stats);

  QString rate = tr("Msgs: %1 | %2 Hz")
    .arg(info.messageCount)
    .arg(info.publishRate, 0, 'f', 1);
  rateLabel_->setText(rate);
}

// ---- Render mode implementations ----

void PlotPanel::renderSolidSeries(PlotSeriesInfo& info, const QVector<QPointF>& points) {
  // Simple: just replace points on the primary series
  info.series->replace(points);
  info.series->setVisible(true);

  // Make sure pen matches current config
  QPen pen(info.config.color);
  pen.setWidth(info.config.lineThickness);
  info.series->setPen(pen);

  // Hide any pool series from a previous mode
  info.segmentPool.hideUnused();
}

void PlotPanel::renderThresholdSeries(PlotSeriesInfo& info,
                                       const QVector<QPointF>& points) {
  if (points.isEmpty()) {
    info.series->clear();
    info.segmentPool.hideUnused();
    return;
  }

  // Hide primary series -- rendering delegated to pool
  info.series->clear();
  info.series->setVisible(false);

  const double upper = info.config.thresholdUpper;
  const double lower = info.config.thresholdLower;
  const QColor& normalColor = info.config.color;
  const QColor& alarmColor = info.config.thresholdAlarmColor;
  const int thickness = info.config.lineThickness;

  // Build segments: contiguous runs of same in/out-of-bounds state
  struct Segment {
    QVector<QPointF> pts;
    bool alarm;
  };
  QVector<Segment> segments;

  auto isAlarm = [&](double y) { return y > upper || y < lower; };

  // Linear interpolation helper for crossing point
  auto interpolateCrossing = [](const QPointF& a, const QPointF& b, double threshold) -> QPointF {
    double t = (threshold - a.y()) / (b.y() - a.y());
    return QPointF(a.x() + t * (b.x() - a.x()), threshold);
  };

  bool currentAlarm = isAlarm(points[0].y());
  Segment current;
  current.alarm = currentAlarm;
  current.pts.append(points[0]);

  for (int i = 1; i < points.size(); ++i) {
    bool ptAlarm = isAlarm(points[i].y());
    if (ptAlarm != currentAlarm) {
      // Compute interpolated crossing point
      double crossThreshold = upper;  // default
      const QPointF& prev = points[i - 1];
      const QPointF& cur = points[i];

      // Determine which threshold was crossed
      if ((prev.y() <= upper && cur.y() > upper) || (prev.y() > upper && cur.y() <= upper)) {
        crossThreshold = upper;
      } else {
        crossThreshold = lower;
      }

      QPointF crossPt = interpolateCrossing(prev, cur, crossThreshold);
      current.pts.append(crossPt);
      segments.append(current);

      // Start new segment
      current = Segment();
      current.alarm = ptAlarm;
      current.pts.append(crossPt);
    }
    current.pts.append(points[i]);
  }
  segments.append(current);

  // Assign segments to pool series
  info.segmentPool.activeCount = 0;
  info.segmentPool.ensureCapacity(segments.size(), chart_, axisX_, axisY_);

  for (int i = 0; i < segments.size() && i < info.segmentPool.pool.size(); ++i) {
    auto* s = info.segmentPool.pool[i];
    QPen pen(segments[i].alarm ? alarmColor : normalColor);
    pen.setWidth(thickness);
    s->setPen(pen);
    s->replace(segments[i].pts);
    s->setVisible(true);
    info.segmentPool.activeCount++;
  }

  info.segmentPool.hideUnused();
}

void PlotPanel::renderGradientSeries(PlotSeriesInfo& info,
                                      const QVector<QPointF>& points) {
  if (points.isEmpty()) {
    info.series->clear();
    info.segmentPool.hideUnused();
    return;
  }

  // Hide primary series -- rendering delegated to pool
  info.series->clear();
  info.series->setVisible(false);

  const auto& cfg = info.config;

  // Ensure palette is computed
  if (cfg.gradientPaletteCache.isEmpty()) {
    computeGradientPalette(info.config);
  }

  const int buckets = cfg.gradientPaletteCache.size();
  const double range = cfg.gradientMaxValue - cfg.gradientMinValue;
  const int thickness = cfg.lineThickness;

  auto bucketIndex = [&](double y) -> int {
    if (range <= 0.0) return 0;
    double t = (y - cfg.gradientMinValue) / range;
    t = std::clamp(t, 0.0, 1.0);
    int idx = static_cast<int>(t * (buckets - 1));
    return std::clamp(idx, 0, buckets - 1);
  };

  // Build segments by bucket transitions
  struct Segment {
    QVector<QPointF> pts;
    int bucket;
  };
  QVector<Segment> segments;

  int currentBucket = bucketIndex(points[0].y());
  Segment current;
  current.bucket = currentBucket;
  current.pts.append(points[0]);

  for (int i = 1; i < points.size(); ++i) {
    int ptBucket = bucketIndex(points[i].y());
    if (ptBucket != currentBucket) {
      // Add midpoint for continuity
      QPointF mid((points[i - 1].x() + points[i].x()) / 2.0,
                  (points[i - 1].y() + points[i].y()) / 2.0);
      current.pts.append(mid);
      segments.append(current);

      current = Segment();
      current.bucket = ptBucket;
      current.pts.append(mid);
      currentBucket = ptBucket;
    }
    current.pts.append(points[i]);
  }
  segments.append(current);

  // Assign segments to pool series
  info.segmentPool.activeCount = 0;
  info.segmentPool.ensureCapacity(segments.size(), chart_, axisX_, axisY_);

  for (int i = 0; i < segments.size() && i < info.segmentPool.pool.size(); ++i) {
    auto* s = info.segmentPool.pool[i];
    int bucket = std::clamp(segments[i].bucket, 0, buckets - 1);
    QPen pen(cfg.gradientPaletteCache[bucket]);
    pen.setWidth(thickness);
    s->setPen(pen);
    s->replace(segments[i].pts);
    s->setVisible(true);
    info.segmentPool.activeCount++;
  }

  info.segmentPool.hideUnused();
}

void PlotPanel::computeGradientPalette(PlotSeriesConfig& config) {
  config.gradientPaletteCache.clear();
  int n = std::max(config.gradientBuckets, 2);
  config.gradientPaletteCache.reserve(n);

  for (int i = 0; i < n; ++i) {
    double t = static_cast<double>(i) / (n - 1);
    int r = static_cast<int>(config.gradientColorLow.red() +
            t * (config.gradientColorHigh.red() - config.gradientColorLow.red()));
    int g = static_cast<int>(config.gradientColorLow.green() +
            t * (config.gradientColorHigh.green() - config.gradientColorLow.green()));
    int b = static_cast<int>(config.gradientColorLow.blue() +
            t * (config.gradientColorHigh.blue() - config.gradientColorLow.blue()));
    config.gradientPaletteCache.append(QColor(r, g, b));
  }
}

void PlotPanel::showSeriesConfigDialog(const QString& fullPath) {
  auto it = plotSeries_.find(fullPath.toStdString());
  if (it == plotSeries_.end()) return;

  PlotSeriesInfo& info = it->second;

  PlotSeriesConfigDialog dlg(info.config, this);
  if (dlg.exec() == QDialog::Accepted) {
    PlotSeriesConfig newCfg = dlg.result();
    // Preserve runtime state
    newCfg.lastAcceptedTimestamp = info.config.lastAcceptedTimestamp;

    info.config = newCfg;

    // Rebuild gradient palette if in gradient mode
    if (newCfg.renderMode == PlotRenderMode::Gradient) {
      computeGradientPalette(info.config);
    }

    // Update list widget color
    for (int i = 0; i < seriesList_->count(); ++i) {
      if (seriesList_->item(i)->data(Qt::UserRole).toString() == fullPath) {
        seriesList_->item(i)->setForeground(info.config.color);
        break;
      }
    }
  }
}

void PlotPanel::onSeriesListContextMenu(const QPoint& pos) {
  QListWidgetItem* item = seriesList_->itemAt(pos);
  if (!item) return;

  QString fullPath = item->data(Qt::UserRole).toString();

  QMenu menu(this);

  // Configure series (full dialog)
  QAction* configAction = menu.addAction(tr("Configure Series..."));
  connect(configAction, &QAction::triggered, [this, fullPath]() {
    showSeriesConfigDialog(fullPath);
  });

  // Quick change color
  QAction* colorAction = menu.addAction(tr("Change Color..."));
  connect(colorAction, &QAction::triggered, [this, fullPath, item]() {
    std::lock_guard<std::mutex> lock(dataMutex_);
    auto it = plotSeries_.find(fullPath.toStdString());
    if (it != plotSeries_.end()) {
      QColor newColor = QColorDialog::getColor(it->second.config.color, this, tr("Select Color"));
      if (newColor.isValid()) {
        it->second.config.color = newColor;
        QPen pen = it->second.series->pen();
        pen.setColor(newColor);
        it->second.series->setPen(pen);
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

void PlotPanel::onThemeChanged(Theme /* newTheme */) {
  applyChartTheme();
}

void PlotPanel::applyChartTheme() {
  if (!chart_) return;

  Theme theme = ThemeManager::instance().currentTheme();
  bool isDark = (theme == Theme::Dark);

  // Chart background and plot area
  QColor backgroundColor = isDark ? QColor(42, 42, 42) : QColor(255, 255, 255);
  QColor plotAreaColor = isDark ? QColor(35, 35, 35) : QColor(250, 250, 250);
  QColor textColor = isDark ? QColor(220, 220, 220) : QColor(30, 30, 30);
  QColor gridColor = isDark ? QColor(70, 70, 70) : QColor(200, 200, 200);
  QColor axisColor = isDark ? QColor(150, 150, 150) : QColor(80, 80, 80);

  // Set chart background
  chart_->setBackgroundBrush(QBrush(backgroundColor));
  chart_->setPlotAreaBackgroundBrush(QBrush(plotAreaColor));
  chart_->setPlotAreaBackgroundVisible(true);

  // Set title color
  chart_->setTitleBrush(QBrush(textColor));

  // Set legend colors
  chart_->legend()->setLabelColor(textColor);
  chart_->legend()->setBrush(QBrush(backgroundColor));

  // Configure X axis
  if (axisX_) {
    axisX_->setLabelsColor(textColor);
    axisX_->setTitleBrush(QBrush(textColor));
    axisX_->setGridLineColor(gridColor);
    axisX_->setLinePenColor(axisColor);
  }

  // Configure Y axis
  if (axisY_) {
    axisY_->setLabelsColor(textColor);
    axisY_->setTitleBrush(QBrush(textColor));
    axisY_->setGridLineColor(gridColor);
    axisY_->setLinePenColor(axisColor);
  }

  // Update chart view background
  if (chartView_) {
    chartView_->setBackgroundBrush(QBrush(backgroundColor));
  }
}

void PlotPanel::setLineThickness(int thickness) {
  thickness = std::clamp(thickness, constants::plot::MIN_LINE_THICKNESS,
                          constants::plot::MAX_LINE_THICKNESS);

  defaultConfig_.lineThickness = thickness;

  // Update existing series
  std::lock_guard<std::mutex> lock(dataMutex_);
  for (auto& [path, info] : plotSeries_) {
    info.config.lineThickness = thickness;
    QPen pen = info.series->pen();
    pen.setWidth(thickness);
    info.series->setPen(pen);
  }

  saveSettings();
}

void PlotPanel::setColorPalette(const QList<QColor>& colors) {
  if (!colors.isEmpty()) {
    colorPalette_ = colors;
    saveSettings();
  }
}

void PlotPanel::loadSettings() {
  QSettings settings("ROS Weaver", "ROS Weaver");
  settings.beginGroup("PlotPanel");

  defaultConfig_.lineThickness = std::clamp(
    settings.value("lineThickness", constants::plot::DEFAULT_LINE_THICKNESS).toInt(),
    constants::plot::MIN_LINE_THICKNESS, constants::plot::MAX_LINE_THICKNESS);

  // Render mode
  int mode = settings.value("defaultRenderMode", 0).toInt();
  if (mode == 1) defaultConfig_.renderMode = PlotRenderMode::Threshold;
  else if (mode == 2) defaultConfig_.renderMode = PlotRenderMode::Gradient;
  else defaultConfig_.renderMode = PlotRenderMode::Solid;

  // Threshold defaults
  defaultConfig_.thresholdUpper = settings.value("thresholdUpper",
    constants::plot::DEFAULT_THRESHOLD_UPPER).toDouble();
  defaultConfig_.thresholdLower = settings.value("thresholdLower",
    constants::plot::DEFAULT_THRESHOLD_LOWER).toDouble();
  QColor alarmColor = settings.value("thresholdAlarmColor").value<QColor>();
  if (alarmColor.isValid()) defaultConfig_.thresholdAlarmColor = alarmColor;

  // Gradient defaults
  defaultConfig_.gradientMinValue = settings.value("gradientMinValue", 0.0).toDouble();
  defaultConfig_.gradientMaxValue = settings.value("gradientMaxValue", 1.0).toDouble();
  QColor gradLow = settings.value("gradientColorLow").value<QColor>();
  if (gradLow.isValid()) defaultConfig_.gradientColorLow = gradLow;
  QColor gradHigh = settings.value("gradientColorHigh").value<QColor>();
  if (gradHigh.isValid()) defaultConfig_.gradientColorHigh = gradHigh;
  defaultConfig_.gradientBuckets = std::clamp(
    settings.value("gradientBuckets", constants::plot::DEFAULT_GRADIENT_BUCKETS).toInt(),
    constants::plot::MIN_GRADIENT_BUCKETS, constants::plot::MAX_GRADIENT_BUCKETS);

  // Decimation defaults
  defaultConfig_.decimationEnabled = settings.value("decimationEnabled", false).toBool();
  defaultConfig_.maxSampleRateHz = settings.value("maxSampleRateHz",
    constants::plot::DEFAULT_SAMPLE_RATE_HZ).toDouble();

  // Load custom color palette if saved
  int colorCount = settings.beginReadArray("colorPalette");
  if (colorCount > 0) {
    colorPalette_.clear();
    for (int i = 0; i < colorCount; ++i) {
      settings.setArrayIndex(i);
      QColor color = settings.value("color").value<QColor>();
      if (color.isValid()) {
        colorPalette_.append(color);
      }
    }
  }
  settings.endArray();

  settings.endGroup();
}

void PlotPanel::saveSettings() {
  QSettings settings("ROS Weaver", "ROS Weaver");
  settings.beginGroup("PlotPanel");

  settings.setValue("lineThickness", defaultConfig_.lineThickness);

  // Render mode
  int mode = 0;
  if (defaultConfig_.renderMode == PlotRenderMode::Threshold) mode = 1;
  else if (defaultConfig_.renderMode == PlotRenderMode::Gradient) mode = 2;
  settings.setValue("defaultRenderMode", mode);

  // Threshold defaults
  settings.setValue("thresholdUpper", defaultConfig_.thresholdUpper);
  settings.setValue("thresholdLower", defaultConfig_.thresholdLower);
  settings.setValue("thresholdAlarmColor", defaultConfig_.thresholdAlarmColor);

  // Gradient defaults
  settings.setValue("gradientMinValue", defaultConfig_.gradientMinValue);
  settings.setValue("gradientMaxValue", defaultConfig_.gradientMaxValue);
  settings.setValue("gradientColorLow", defaultConfig_.gradientColorLow);
  settings.setValue("gradientColorHigh", defaultConfig_.gradientColorHigh);
  settings.setValue("gradientBuckets", defaultConfig_.gradientBuckets);

  // Decimation defaults
  settings.setValue("decimationEnabled", defaultConfig_.decimationEnabled);
  settings.setValue("maxSampleRateHz", defaultConfig_.maxSampleRateHz);

  // Save color palette
  settings.beginWriteArray("colorPalette", colorPalette_.size());
  for (int i = 0; i < colorPalette_.size(); ++i) {
    settings.setArrayIndex(i);
    settings.setValue("color", colorPalette_[i]);
  }
  settings.endArray();

  settings.endGroup();
}

// ---- Default config setters ----

void PlotPanel::setDefaultRenderMode(PlotRenderMode mode) {
  defaultConfig_.renderMode = mode;
  saveSettings();
}

void PlotPanel::setDefaultThresholdUpper(double val) {
  defaultConfig_.thresholdUpper = val;
  saveSettings();
}

void PlotPanel::setDefaultThresholdLower(double val) {
  defaultConfig_.thresholdLower = val;
  saveSettings();
}

void PlotPanel::setDefaultThresholdAlarmColor(const QColor& color) {
  defaultConfig_.thresholdAlarmColor = color;
  saveSettings();
}

void PlotPanel::setDefaultGradientMinValue(double val) {
  defaultConfig_.gradientMinValue = val;
  saveSettings();
}

void PlotPanel::setDefaultGradientMaxValue(double val) {
  defaultConfig_.gradientMaxValue = val;
  saveSettings();
}

void PlotPanel::setDefaultGradientColorLow(const QColor& color) {
  defaultConfig_.gradientColorLow = color;
  saveSettings();
}

void PlotPanel::setDefaultGradientColorHigh(const QColor& color) {
  defaultConfig_.gradientColorHigh = color;
  saveSettings();
}

void PlotPanel::setDefaultGradientBuckets(int buckets) {
  defaultConfig_.gradientBuckets = std::clamp(buckets,
    constants::plot::MIN_GRADIENT_BUCKETS, constants::plot::MAX_GRADIENT_BUCKETS);
  saveSettings();
}

void PlotPanel::setDefaultDecimationEnabled(bool enabled) {
  defaultConfig_.decimationEnabled = enabled;
  saveSettings();
}

void PlotPanel::setDefaultMaxSampleRateHz(double hz) {
  defaultConfig_.maxSampleRateHz = hz;
  saveSettings();
}

}  // namespace ros_weaver
