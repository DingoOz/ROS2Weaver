#include "ros_weaver/widgets/latency_heatmap_panel.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/connection_line.hpp"
#include "ros_weaver/canvas/package_block.hpp"

#include <QHeaderView>
#include <QFormLayout>
#include <QScrollArea>
#include <QDateTime>
#include <QTextStream>
#include <QFile>
#include <algorithm>
#include <cmath>

namespace ros_weaver {

// Static color palette for chart series
const QList<QColor> LatencyHeatmapPanel::SERIES_COLORS = {
    QColor(31, 119, 180),   // Blue
    QColor(255, 127, 14),   // Orange
    QColor(44, 160, 44),    // Green
    QColor(214, 39, 40),    // Red
    QColor(148, 103, 189),  // Purple
    QColor(140, 86, 75),    // Brown
    QColor(227, 119, 194),  // Pink
    QColor(127, 127, 127),  // Gray
    QColor(188, 189, 34),   // Olive
    QColor(23, 190, 207)    // Cyan
};

LatencyHeatmapPanel::LatencyHeatmapPanel(QWidget* parent)
    : QWidget(parent)
    , enableCheck_(nullptr)
    , autoTrackCheck_(nullptr)
    , goodThresholdSpin_(nullptr)
    , warningThresholdSpin_(nullptr)
    , criticalThresholdSpin_(nullptr)
    , alertThresholdSpin_(nullptr)
    , connectionTable_(nullptr)
    , totalConnectionsLabel_(nullptr)
    , activeConnectionsLabel_(nullptr)
    , alertCountLabel_(nullptr)
    , avgLatencyLabel_(nullptr)
    , legendWidget_(nullptr)
    , autoTuneButton_(nullptr)
    , autoTuneMenu_(nullptr)
    , calibrationProgress_(nullptr)
    , calibrationTimer_(new QTimer(this))
    , isCalibrating_(false)
    , calibrationDurationMs_(5000)
    , calibrationStartTime_(0)
    , latencyChart_(nullptr)
    , chartView_(nullptr)
    , timeAxis_(nullptr)
    , latencyAxis_(nullptr)
    , timeWindowCombo_(nullptr)
    , timeWindowSec_(DEFAULT_TIME_WINDOW_SEC)
    , chartStartTime_(QDateTime::currentMSecsSinceEpoch())
    , exportCsvButton_(nullptr)
    , updateTimer_(new QTimer(this))
    , canvas_(nullptr)
    , tracker_(nullptr)
    , heatmapEnabled_(false)
    , alertCount_(0)
    , goodThresholdMs_(10.0)
    , warningThresholdMs_(50.0)
    , criticalThresholdMs_(100.0) {

  setupUi();

  connect(updateTimer_, &QTimer::timeout,
          this, &LatencyHeatmapPanel::updateLatencyDisplay);

  connect(calibrationTimer_, &QTimer::timeout,
          this, &LatencyHeatmapPanel::onCalibrationTimeout);
}

LatencyHeatmapPanel::~LatencyHeatmapPanel() {
  updateTimer_->stop();
  calibrationTimer_->stop();
}

void LatencyHeatmapPanel::setCanvas(WeaverCanvas* canvas) {
  canvas_ = canvas;
  refreshConnections();
}

void LatencyHeatmapPanel::setLatencyTracker(LatencyTracker* tracker) {
  if (tracker_) {
    disconnect(tracker_, nullptr, this, nullptr);
  }

  tracker_ = tracker;

  if (tracker_) {
    connect(tracker_, &LatencyTracker::latencyUpdated,
            this, &LatencyHeatmapPanel::onLatencyUpdated);
    connect(tracker_, &LatencyTracker::latencyAlert,
            this, &LatencyHeatmapPanel::onLatencyAlert);
    connect(tracker_, &LatencyTracker::trackingStarted,
            this, &LatencyHeatmapPanel::onTrackingStarted);
    connect(tracker_, &LatencyTracker::trackingStopped,
            this, &LatencyHeatmapPanel::onTrackingStopped);
  }
}

void LatencyHeatmapPanel::setRosNode(rclcpp::Node::SharedPtr node) {
  node_ = node;
  if (tracker_) {
    tracker_->setRosNode(node);
  }
}

void LatencyHeatmapPanel::setHeatmapEnabled(bool enabled) {
  if (heatmapEnabled_ != enabled) {
    heatmapEnabled_ = enabled;
    enableCheck_->setChecked(enabled);

    if (enabled) {
      updateTimer_->start(UPDATE_INTERVAL_MS);
      refreshConnections();
    } else {
      updateTimer_->stop();
      if (tracker_) {
        tracker_->stopAllTracking();
      }
    }

    // Update canvas connections
    if (canvas_) {
      for (auto* connection : canvas_->connections()) {
        connection->setLatencyHeatmapEnabled(enabled);
        if (enabled) {
          connection->setLatencyThresholds(goodThresholdMs_, warningThresholdMs_, criticalThresholdMs_);
        }
      }
    }

    emit heatmapEnabledChanged(enabled);
  }
}

void LatencyHeatmapPanel::refreshConnections() {
  connectionTable_->setRowCount(0);

  if (!canvas_) return;

  for (auto* connection : canvas_->connections()) {
    addConnectionRow(connection->id().toString(), connection);
  }

  totalConnectionsLabel_->setText(QString::number(connectionTable_->rowCount()));
}

void LatencyHeatmapPanel::updateLatencyDisplay() {
  if (!tracker_ || !canvas_) return;

  int activeCount = 0;
  double totalLatency = 0.0;
  int latencyCount = 0;

  for (int row = 0; row < connectionTable_->rowCount(); ++row) {
    auto* idItem = connectionTable_->item(row, 0);
    if (!idItem) continue;

    QString connectionId = idItem->data(Qt::UserRole).toString();
    updateConnectionRow(row, connectionId);

    if (tracker_->isTracking(connectionId)) {
      activeCount++;

      double latency = tracker_->getCurrentLatency(connectionId);
      if (latency >= 0) {
        totalLatency += latency;
        latencyCount++;
      }
    }
  }

  activeConnectionsLabel_->setText(QString::number(activeCount));

  if (latencyCount > 0) {
    avgLatencyLabel_->setText(formatLatency(totalLatency / latencyCount));
  } else {
    avgLatencyLabel_->setText("-");
  }
}

void LatencyHeatmapPanel::setupUi() {
  auto* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(4, 4, 4, 4);
  mainLayout->setSpacing(8);

  // Enable toggle
  enableCheck_ = new QCheckBox("Enable Latency Heatmap");
  connect(enableCheck_, &QCheckBox::toggled,
          this, &LatencyHeatmapPanel::onEnableToggled);
  mainLayout->addWidget(enableCheck_);

  autoTrackCheck_ = new QCheckBox("Auto-track new connections");
  autoTrackCheck_->setChecked(true);
  connect(autoTrackCheck_, &QCheckBox::toggled,
          this, &LatencyHeatmapPanel::onAutoTrackToggled);
  mainLayout->addWidget(autoTrackCheck_);

  // Thresholds group with auto-tune
  setupThresholdControls();
  setupAutoTuneControls();

  // Latency history chart
  setupLatencyChart();

  // Connection table
  setupConnectionTable();

  // Stats display
  setupStatsDisplay();

  // Button row
  auto* buttonLayout = new QHBoxLayout;
  auto* refreshBtn = new QPushButton("Refresh");
  connect(refreshBtn, &QPushButton::clicked,
          this, &LatencyHeatmapPanel::refreshConnections);
  buttonLayout->addWidget(refreshBtn);

  exportCsvButton_ = new QPushButton("Export CSV");
  connect(exportCsvButton_, &QPushButton::clicked,
          this, &LatencyHeatmapPanel::onExportCsvClicked);
  buttonLayout->addWidget(exportCsvButton_);

  buttonLayout->addStretch();
  mainLayout->addLayout(buttonLayout);
}

void LatencyHeatmapPanel::setupThresholdControls() {
  auto* thresholdGroup = new QGroupBox("Thresholds (ms)");
  auto* thresholdLayout = new QFormLayout(thresholdGroup);

  goodThresholdSpin_ = new QDoubleSpinBox;
  goodThresholdSpin_->setRange(0.1, 1000.0);
  goodThresholdSpin_->setValue(goodThresholdMs_);
  goodThresholdSpin_->setSuffix(" ms");
  goodThresholdSpin_->setDecimals(1);
  connect(goodThresholdSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LatencyHeatmapPanel::onThresholdChanged);

  warningThresholdSpin_ = new QDoubleSpinBox;
  warningThresholdSpin_->setRange(1.0, 5000.0);
  warningThresholdSpin_->setValue(warningThresholdMs_);
  warningThresholdSpin_->setSuffix(" ms");
  warningThresholdSpin_->setDecimals(1);
  connect(warningThresholdSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LatencyHeatmapPanel::onThresholdChanged);

  criticalThresholdSpin_ = new QDoubleSpinBox;
  criticalThresholdSpin_->setRange(10.0, 10000.0);
  criticalThresholdSpin_->setValue(criticalThresholdMs_);
  criticalThresholdSpin_->setSuffix(" ms");
  criticalThresholdSpin_->setDecimals(1);
  connect(criticalThresholdSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LatencyHeatmapPanel::onThresholdChanged);

  alertThresholdSpin_ = new QDoubleSpinBox;
  alertThresholdSpin_->setRange(1.0, 10000.0);
  alertThresholdSpin_->setValue(100.0);
  alertThresholdSpin_->setSuffix(" ms");
  alertThresholdSpin_->setDecimals(1);
  connect(alertThresholdSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, [this](double value) {
            if (tracker_) {
              tracker_->setAlertThreshold(value);
            }
          });

  thresholdLayout->addRow("Good (<):", goodThresholdSpin_);
  thresholdLayout->addRow("Warning (<):", warningThresholdSpin_);
  thresholdLayout->addRow("Critical (>=):", criticalThresholdSpin_);
  thresholdLayout->addRow("Alert threshold:", alertThresholdSpin_);

  // Color legend
  legendWidget_ = new QWidget;
  auto* legendLayout = new QHBoxLayout(legendWidget_);
  legendLayout->setContentsMargins(0, 4, 0, 0);

  auto addLegendItem = [legendLayout](const QString& text, const QColor& color) {
    auto* colorBox = new QLabel;
    colorBox->setFixedSize(16, 16);
    colorBox->setStyleSheet(QString("background-color: %1; border: 1px solid gray;")
                                .arg(color.name()));
    legendLayout->addWidget(colorBox);
    legendLayout->addWidget(new QLabel(text));
  };

  addLegendItem("Good", QColor(100, 200, 100));
  addLegendItem("Warning", QColor(255, 200, 50));
  addLegendItem("Critical", QColor(255, 50, 50));
  legendLayout->addStretch();

  thresholdLayout->addRow("Legend:", legendWidget_);

  static_cast<QVBoxLayout*>(layout())->addWidget(thresholdGroup);
}

void LatencyHeatmapPanel::setupConnectionTable() {
  auto* tableGroup = new QGroupBox("Connections");
  auto* tableLayout = new QVBoxLayout(tableGroup);

  connectionTable_ = new QTableWidget;
  connectionTable_->setColumnCount(5);
  connectionTable_->setHorizontalHeaderLabels(
      {"Connection", "Source", "Target", "Latency", "Status"});
  connectionTable_->horizontalHeader()->setStretchLastSection(true);
  connectionTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
  connectionTable_->setSelectionMode(QAbstractItemView::SingleSelection);
  connectionTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  connectionTable_->verticalHeader()->setVisible(false);

  connect(connectionTable_, &QTableWidget::cellClicked,
          this, &LatencyHeatmapPanel::onConnectionClicked);

  tableLayout->addWidget(connectionTable_);

  static_cast<QVBoxLayout*>(layout())->addWidget(tableGroup);
}

void LatencyHeatmapPanel::setupStatsDisplay() {
  auto* statsGroup = new QGroupBox("Statistics");
  auto* statsLayout = new QFormLayout(statsGroup);

  totalConnectionsLabel_ = new QLabel("0");
  activeConnectionsLabel_ = new QLabel("0");
  alertCountLabel_ = new QLabel("0");
  avgLatencyLabel_ = new QLabel("-");

  statsLayout->addRow("Total connections:", totalConnectionsLabel_);
  statsLayout->addRow("Active tracking:", activeConnectionsLabel_);
  statsLayout->addRow("Alerts:", alertCountLabel_);
  statsLayout->addRow("Avg latency:", avgLatencyLabel_);

  static_cast<QVBoxLayout*>(layout())->addWidget(statsGroup);
}

void LatencyHeatmapPanel::setupAutoTuneControls() {
  // Find the threshold group and add auto-tune button
  auto* thresholdGroup = findChild<QGroupBox*>();
  if (!thresholdGroup) return;

  auto* thresholdLayout = qobject_cast<QFormLayout*>(thresholdGroup->layout());
  if (!thresholdLayout) return;

  // Create auto-tune button with dropdown menu
  auto* autoTuneLayout = new QHBoxLayout;

  autoTuneButton_ = new QPushButton("Auto-tune");
  autoTuneButton_->setToolTip("Automatically calculate thresholds based on observed latency data");

  autoTuneMenu_ = new QMenu(this);
  autoTuneMenu_->addAction("Quick (5s)", [this]() { startCalibration(5000); });
  autoTuneMenu_->addAction("Normal (10s)", [this]() { startCalibration(10000); });
  autoTuneMenu_->addAction("Thorough (30s)", [this]() { startCalibration(30000); });
  autoTuneMenu_->addSeparator();
  autoTuneMenu_->addAction("Use Current Data", [this]() { calculateAutoThresholds(); });

  autoTuneButton_->setMenu(autoTuneMenu_);
  autoTuneLayout->addWidget(autoTuneButton_);

  // Calibration progress bar (hidden by default)
  calibrationProgress_ = new QProgressBar;
  calibrationProgress_->setRange(0, 100);
  calibrationProgress_->setVisible(false);
  calibrationProgress_->setTextVisible(true);
  calibrationProgress_->setFormat("Calibrating: %p%");
  autoTuneLayout->addWidget(calibrationProgress_);

  autoTuneLayout->addStretch();
  thresholdLayout->addRow("", autoTuneLayout);
}

void LatencyHeatmapPanel::setupLatencyChart() {
  auto* chartGroup = new QGroupBox("Latency History");
  auto* chartLayout = new QVBoxLayout(chartGroup);

  // Time window selector
  auto* controlLayout = new QHBoxLayout;
  controlLayout->addWidget(new QLabel("Time window:"));

  timeWindowCombo_ = new QComboBox;
  timeWindowCombo_->addItem("30 seconds", 30);
  timeWindowCombo_->addItem("1 minute", 60);
  timeWindowCombo_->addItem("5 minutes", 300);
  timeWindowCombo_->addItem("10 minutes", 600);
  connect(timeWindowCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &LatencyHeatmapPanel::onTimeWindowChanged);
  controlLayout->addWidget(timeWindowCombo_);
  controlLayout->addStretch();
  chartLayout->addLayout(controlLayout);

  // Create chart
  latencyChart_ = new QChart;
  latencyChart_->setTitle("");
  latencyChart_->legend()->setVisible(true);
  latencyChart_->legend()->setAlignment(Qt::AlignBottom);
  latencyChart_->setAnimationOptions(QChart::NoAnimation);

  // Time axis (relative seconds from now)
  timeAxis_ = new QValueAxis;
  timeAxis_->setTitleText("Time (s)");
  timeAxis_->setRange(-timeWindowSec_, 0);
  timeAxis_->setLabelFormat("%.0f");
  latencyChart_->addAxis(timeAxis_, Qt::AlignBottom);

  // Latency axis
  latencyAxis_ = new QValueAxis;
  latencyAxis_->setTitleText("Latency (ms)");
  latencyAxis_->setRange(0, 100);
  latencyAxis_->setLabelFormat("%.1f");
  latencyChart_->addAxis(latencyAxis_, Qt::AlignLeft);

  // Chart view
  chartView_ = new QChartView(latencyChart_);
  chartView_->setRenderHint(QPainter::Antialiasing);
  chartView_->setMinimumHeight(150);
  chartLayout->addWidget(chartView_);

  static_cast<QVBoxLayout*>(layout())->addWidget(chartGroup);
}

void LatencyHeatmapPanel::onEnableToggled(bool enabled) {
  setHeatmapEnabled(enabled);

  // Reset chart when enabling
  if (enabled) {
    chartStartTime_ = QDateTime::currentMSecsSinceEpoch();
    latencyHistory_.clear();
    for (auto* series : latencySeries_.values()) {
      series->clear();
    }
  }
}

void LatencyHeatmapPanel::onConnectionClicked(int row, int column) {
  Q_UNUSED(column)

  auto* idItem = connectionTable_->item(row, 0);
  if (!idItem) return;

  QString connectionId = idItem->data(Qt::UserRole).toString();
  emit connectionSelected(connectionId);

  // Toggle tracking for this connection
  if (tracker_ && heatmapEnabled_) {
    if (tracker_->isTracking(connectionId)) {
      tracker_->stopTracking(connectionId);
    } else {
      // Get connection info from canvas
      if (canvas_) {
        for (auto* connection : canvas_->connections()) {
          if (connection->id().toString() == connectionId) {
            QString inputTopic = connection->topicName();
            // For now, use the same topic for input/output (simplified)
            tracker_->trackConnection(connectionId, inputTopic, inputTopic);
            break;
          }
        }
      }
    }
  }
}

void LatencyHeatmapPanel::onThresholdChanged() {
  goodThresholdMs_ = goodThresholdSpin_->value();
  warningThresholdMs_ = warningThresholdSpin_->value();
  criticalThresholdMs_ = criticalThresholdSpin_->value();

  // Update all connections with new thresholds
  if (canvas_) {
    for (auto* connection : canvas_->connections()) {
      connection->setLatencyThresholds(goodThresholdMs_, warningThresholdMs_, criticalThresholdMs_);
    }
  }
}

void LatencyHeatmapPanel::onAutoTrackToggled(bool enabled) {
  Q_UNUSED(enabled)
  // Will be used when new connections are added
}

void LatencyHeatmapPanel::onLatencyUpdated(const QString& connectionId, double latencyMs) {
  int row = findConnectionRow(connectionId);
  if (row >= 0) {
    updateConnectionRow(row, connectionId);
  }

  // Update connection line visualization
  if (canvas_) {
    for (auto* connection : canvas_->connections()) {
      if (connection->id().toString() == connectionId) {
        connection->setCurrentLatency(latencyMs);
        break;
      }
    }
  }

  // Update latency chart
  updateLatencyChart(connectionId, latencyMs);
}

void LatencyHeatmapPanel::onLatencyAlert(const QString& connectionId, double latencyMs, double thresholdMs) {
  Q_UNUSED(connectionId)
  Q_UNUSED(latencyMs)
  Q_UNUSED(thresholdMs)

  alertCount_++;
  alertCountLabel_->setText(QString::number(alertCount_));
}

void LatencyHeatmapPanel::onTrackingStarted(const QString& connectionId) {
  int row = findConnectionRow(connectionId);
  if (row >= 0) {
    auto* statusItem = connectionTable_->item(row, 4);
    if (statusItem) {
      statusItem->setText("Tracking");
      statusItem->setForeground(QColor(100, 200, 100));
    }
  }
}

void LatencyHeatmapPanel::onTrackingStopped(const QString& connectionId) {
  int row = findConnectionRow(connectionId);
  if (row >= 0) {
    auto* statusItem = connectionTable_->item(row, 4);
    if (statusItem) {
      statusItem->setText("Stopped");
      statusItem->setForeground(QColor(150, 150, 150));
    }
  }
}

void LatencyHeatmapPanel::updateConnectionRow(int row, const QString& connectionId) {
  if (!tracker_) return;

  double latency = tracker_->getCurrentLatency(connectionId);
  bool isTracking = tracker_->isTracking(connectionId);

  // Latency column
  auto* latencyItem = connectionTable_->item(row, 3);
  if (latencyItem) {
    if (latency >= 0) {
      latencyItem->setText(formatLatency(latency));
      latencyItem->setForeground(getLatencyColor(latency));
    } else {
      latencyItem->setText("-");
      latencyItem->setForeground(QColor(150, 150, 150));
    }
  }

  // Status column
  auto* statusItem = connectionTable_->item(row, 4);
  if (statusItem) {
    if (isTracking) {
      statusItem->setText("Tracking");
      statusItem->setForeground(QColor(100, 200, 100));
    } else {
      statusItem->setText("Idle");
      statusItem->setForeground(QColor(150, 150, 150));
    }
  }
}

void LatencyHeatmapPanel::addConnectionRow(const QString& connectionId, ConnectionLine* connection) {
  int row = connectionTable_->rowCount();
  connectionTable_->insertRow(row);

  // Connection ID (hidden in UserRole)
  auto* idItem = new QTableWidgetItem(connectionId.left(8) + "...");
  idItem->setData(Qt::UserRole, connectionId);
  idItem->setToolTip(connectionId);
  connectionTable_->setItem(row, 0, idItem);

  // Source
  QString sourceName = "Unknown";
  if (connection->sourceBlock()) {
    sourceName = connection->sourceBlock()->packageName();
  }
  connectionTable_->setItem(row, 1, new QTableWidgetItem(sourceName));

  // Target
  QString targetName = "Unknown";
  if (connection->targetBlock()) {
    targetName = connection->targetBlock()->packageName();
  }
  connectionTable_->setItem(row, 2, new QTableWidgetItem(targetName));

  // Latency
  connectionTable_->setItem(row, 3, new QTableWidgetItem("-"));

  // Status
  auto* statusItem = new QTableWidgetItem("Idle");
  statusItem->setForeground(QColor(150, 150, 150));
  connectionTable_->setItem(row, 4, statusItem);
}

int LatencyHeatmapPanel::findConnectionRow(const QString& connectionId) {
  for (int row = 0; row < connectionTable_->rowCount(); ++row) {
    auto* item = connectionTable_->item(row, 0);
    if (item && item->data(Qt::UserRole).toString() == connectionId) {
      return row;
    }
  }
  return -1;
}

QString LatencyHeatmapPanel::formatLatency(double latencyMs) const {
  if (latencyMs >= 1000) {
    return QString("%1 s").arg(latencyMs / 1000.0, 0, 'f', 2);
  } else if (latencyMs >= 1) {
    return QString("%1 ms").arg(latencyMs, 0, 'f', 1);
  } else {
    return QString("%1 us").arg(latencyMs * 1000, 0, 'f', 0);
  }
}

QColor LatencyHeatmapPanel::getLatencyColor(double latencyMs) const {
  if (latencyMs < goodThresholdMs_) {
    return QColor(100, 200, 100);  // Green
  } else if (latencyMs < warningThresholdMs_) {
    return QColor(255, 200, 50);   // Yellow
  } else {
    return QColor(255, 50, 50);    // Red
  }
}

// ============================================================================
// Auto-tune methods
// ============================================================================

void LatencyHeatmapPanel::onAutoTuneClicked() {
  // This is called when the button is clicked directly (not via menu)
  // Default to quick calibration
  startCalibration(5000);
}

void LatencyHeatmapPanel::startCalibration(int durationMs) {
  if (isCalibrating_) return;

  if (!tracker_ || !heatmapEnabled_) {
    QMessageBox::warning(this, "Auto-tune",
                         "Please enable latency heatmap and start tracking some connections first.");
    return;
  }

  isCalibrating_ = true;
  calibrationDurationMs_ = durationMs;
  calibrationStartTime_ = QDateTime::currentMSecsSinceEpoch();

  // Show progress bar
  calibrationProgress_->setValue(0);
  calibrationProgress_->setVisible(true);
  autoTuneButton_->setEnabled(false);

  // Start calibration timer (update progress every 100ms)
  calibrationTimer_->start(100);
}

void LatencyHeatmapPanel::onCalibrationTimeout() {
  qint64 elapsed = QDateTime::currentMSecsSinceEpoch() - calibrationStartTime_;
  int progress = static_cast<int>((elapsed * 100) / calibrationDurationMs_);

  if (progress >= 100) {
    finishCalibration();
  } else {
    calibrationProgress_->setValue(progress);
  }
}

void LatencyHeatmapPanel::finishCalibration() {
  calibrationTimer_->stop();
  isCalibrating_ = false;

  // Hide progress bar
  calibrationProgress_->setVisible(false);
  autoTuneButton_->setEnabled(true);

  // Calculate thresholds from collected data
  calculateAutoThresholds();
}

void LatencyHeatmapPanel::calculateAutoThresholds() {
  if (!tracker_) return;

  QList<double> allLatencies;

  // Collect all latency measurements from tracker
  for (const QString& connId : tracker_->trackedConnections()) {
    auto history = tracker_->getHistory(connId);
    for (const auto& measurement : history) {
      allLatencies.append(measurement.latencyMs);
    }
  }

  if (allLatencies.isEmpty()) {
    QMessageBox::information(this, "Auto-tune",
                             "No latency data available. Start tracking some connections first.");
    return;
  }

  // Sort for percentile calculation
  std::sort(allLatencies.begin(), allLatencies.end());

  auto percentile = [&](double p) -> double {
    int idx = static_cast<int>(p * allLatencies.size() / 100.0);
    idx = qBound(0, idx, allLatencies.size() - 1);
    return allLatencies[idx];
  };

  // Set thresholds based on percentiles
  double p50 = percentile(50);
  double p75 = percentile(75);
  double p95 = percentile(95);
  double p99 = percentile(99);

  // Apply some minimum spacing between thresholds
  double goodVal = qMax(p50, 1.0);
  double warningVal = qMax(p75, goodVal * 1.5);
  double criticalVal = qMax(p95, warningVal * 1.5);
  double alertVal = qMax(p99, criticalVal * 1.1);

  goodThresholdSpin_->setValue(goodVal);
  warningThresholdSpin_->setValue(warningVal);
  criticalThresholdSpin_->setValue(criticalVal);
  alertThresholdSpin_->setValue(alertVal);

  // Show results
  QMessageBox::information(this, "Auto-tune Complete",
      QString("Thresholds calculated from %1 samples:\n\n"
              "Good (p50): %.1f ms\n"
              "Warning (p75): %.1f ms\n"
              "Critical (p95): %.1f ms\n"
              "Alert (p99): %.1f ms")
          .arg(allLatencies.size())
          .arg(goodVal)
          .arg(warningVal)
          .arg(criticalVal)
          .arg(alertVal));
}

// ============================================================================
// Chart methods
// ============================================================================

void LatencyHeatmapPanel::onTimeWindowChanged(int index) {
  timeWindowSec_ = timeWindowCombo_->itemData(index).toInt();
  timeAxis_->setRange(-timeWindowSec_, 0);
  trimChartData();
  updateChartAxes();
}

void LatencyHeatmapPanel::updateLatencyChart(const QString& connectionId, double latencyMs) {
  if (!latencyChart_ || !heatmapEnabled_) return;

  qint64 now = QDateTime::currentMSecsSinceEpoch();

  // Get or create series for this connection
  QLineSeries* series = latencySeries_.value(connectionId, nullptr);
  if (!series) {
    series = new QLineSeries;

    // Get a name for the series
    QString seriesName = connectionId.left(8);
    if (canvas_) {
      for (auto* connection : canvas_->connections()) {
        if (connection->id().toString() == connectionId) {
          if (connection->sourceBlock() && connection->targetBlock()) {
            seriesName = QString("%1 -> %2")
                             .arg(connection->sourceBlock()->packageName().split('/').last())
                             .arg(connection->targetBlock()->packageName().split('/').last());
          }
          break;
        }
      }
    }
    series->setName(seriesName);

    // Assign color from palette
    int colorIndex = latencySeries_.size() % SERIES_COLORS.size();
    series->setColor(SERIES_COLORS[colorIndex]);

    latencyChart_->addSeries(series);
    series->attachAxis(timeAxis_);
    series->attachAxis(latencyAxis_);

    latencySeries_[connectionId] = series;
    latencyHistory_[connectionId] = std::deque<QPair<qint64, double>>();
  }

  // Add data point
  latencyHistory_[connectionId].push_back({now, latencyMs});

  // Trim old data
  trimChartData();

  // Update series with relative time
  series->clear();
  for (const auto& point : latencyHistory_[connectionId]) {
    double relativeTime = (point.first - now) / 1000.0;  // Convert to seconds
    series->append(relativeTime, point.second);
  }

  // Update Y axis if needed
  updateChartAxes();
}

void LatencyHeatmapPanel::updateChartAxes() {
  if (latencyHistory_.isEmpty()) return;

  // Find max latency across all series
  double maxLatency = 0;
  for (const auto& history : latencyHistory_) {
    for (const auto& point : history) {
      maxLatency = qMax(maxLatency, point.second);
    }
  }

  // Add 10% margin and round up
  double yMax = qMax(100.0, maxLatency * 1.1);
  yMax = std::ceil(yMax / 10.0) * 10.0;  // Round to nearest 10

  latencyAxis_->setRange(0, yMax);
}

void LatencyHeatmapPanel::trimChartData() {
  qint64 now = QDateTime::currentMSecsSinceEpoch();
  qint64 cutoff = now - (timeWindowSec_ * 1000);

  for (auto it = latencyHistory_.begin(); it != latencyHistory_.end(); ++it) {
    auto& history = it.value();

    // Remove old data points
    while (!history.empty() && history.front().first < cutoff) {
      history.pop_front();
    }

    // Also limit total points
    while (history.size() > MAX_CHART_POINTS) {
      history.pop_front();
    }
  }
}

QColor LatencyHeatmapPanel::getSeriesColor(int index) const {
  return SERIES_COLORS[index % SERIES_COLORS.size()];
}

// ============================================================================
// Export methods
// ============================================================================

void LatencyHeatmapPanel::onExportCsvClicked() {
  QString defaultFileName = QString("latency_export_%1.csv")
                                .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));

  QString fileName = QFileDialog::getSaveFileName(
      this,
      "Export Latency Data",
      defaultFileName,
      "CSV Files (*.csv)");

  if (fileName.isEmpty()) return;

  // Ensure .csv extension
  if (!fileName.endsWith(".csv", Qt::CaseInsensitive)) {
    fileName += ".csv";
  }

  QString csvContent = generateCsvContent();

  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QMessageBox::warning(this, "Export Error",
                         QString("Could not open file for writing:\n%1").arg(fileName));
    return;
  }

  QTextStream stream(&file);
  stream << csvContent;
  file.close();

  QMessageBox::information(this, "Export Complete",
                           QString("Latency data exported to:\n%1").arg(fileName));
}

QString LatencyHeatmapPanel::generateCsvContent() const {
  QString content;
  QTextStream stream(&content);

  // Header with metadata
  stream << "# ROS2Weaver Latency Export\n";
  stream << "# Generated: " << QDateTime::currentDateTime().toString(Qt::ISODate) << "\n";

  int totalMeasurements = 0;
  if (tracker_) {
    for (const QString& connId : tracker_->trackedConnections()) {
      totalMeasurements += tracker_->getHistory(connId).size();
    }
  }

  stream << "# Connections: " << (tracker_ ? tracker_->trackedConnections().size() : 0) << "\n";
  stream << "# Total measurements: " << totalMeasurements << "\n";
  stream << "#\n";

  // Data header
  stream << "timestamp,connection_id,source_node,target_node,latency_ms\n";

  // Data rows
  if (tracker_ && canvas_) {
    for (const QString& connId : tracker_->trackedConnections()) {
      QString sourceName = "Unknown";
      QString targetName = "Unknown";

      // Get source/target names
      for (auto* connection : canvas_->connections()) {
        if (connection->id().toString() == connId) {
          if (connection->sourceBlock()) {
            sourceName = connection->sourceBlock()->packageName();
          }
          if (connection->targetBlock()) {
            targetName = connection->targetBlock()->packageName();
          }
          break;
        }
      }

      auto history = tracker_->getHistory(connId);
      for (const auto& measurement : history) {
        stream << measurement.timestamp << ","
               << connId << ","
               << sourceName << ","
               << targetName << ","
               << QString::number(measurement.latencyMs, 'f', 3) << "\n";
      }
    }
  }

  // Statistics section
  stream << "#\n";
  stream << "# Statistics\n";
  stream << "# connection_id,avg_ms,min_ms,max_ms,p50_ms,p95_ms,p99_ms,jitter_ms,sample_count\n";

  if (tracker_) {
    for (const QString& connId : tracker_->trackedConnections()) {
      auto stats = tracker_->getStats(connId);
      if (stats.sampleCount > 0) {
        stream << "# " << connId << ","
               << QString::number(stats.averageMs, 'f', 2) << ","
               << QString::number(stats.minMs, 'f', 2) << ","
               << QString::number(stats.maxMs, 'f', 2) << ","
               << QString::number(stats.p50Ms, 'f', 2) << ","
               << QString::number(stats.p95Ms, 'f', 2) << ","
               << QString::number(stats.p99Ms, 'f', 2) << ","
               << QString::number(stats.jitterMs, 'f', 2) << ","
               << stats.sampleCount << "\n";
      }
    }
  }

  return content;
}

}  // namespace ros_weaver
