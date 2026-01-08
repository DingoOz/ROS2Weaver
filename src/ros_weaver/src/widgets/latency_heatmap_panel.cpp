#include "ros_weaver/widgets/latency_heatmap_panel.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/connection_line.hpp"
#include "ros_weaver/canvas/package_block.hpp"

#include <QHeaderView>
#include <QFormLayout>
#include <QScrollArea>

namespace ros_weaver {

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
}

LatencyHeatmapPanel::~LatencyHeatmapPanel() {
  updateTimer_->stop();
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

  // Thresholds group
  setupThresholdControls();

  // Connection table
  setupConnectionTable();

  // Stats display
  setupStatsDisplay();

  // Refresh button
  auto* refreshBtn = new QPushButton("Refresh Connections");
  connect(refreshBtn, &QPushButton::clicked,
          this, &LatencyHeatmapPanel::refreshConnections);
  mainLayout->addWidget(refreshBtn);
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

void LatencyHeatmapPanel::onEnableToggled(bool enabled) {
  setHeatmapEnabled(enabled);
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

}  // namespace ros_weaver
