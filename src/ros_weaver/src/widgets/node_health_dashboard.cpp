#include "ros_weaver/widgets/node_health_dashboard.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/package_block.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QMessageBox>

namespace ros_weaver {

NodeHealthDashboard::NodeHealthDashboard(QWidget* parent)
    : QWidget(parent) {

  healthMonitor_ = new NodeHealthMonitor(this);
  setupUi();
  setupConnections();
}

void NodeHealthDashboard::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(8, 8, 8, 8);
  mainLayout->setSpacing(8);

  // Controls section
  QHBoxLayout* controlsLayout = new QHBoxLayout();

  startStopButton_ = new QPushButton(tr("Start Monitoring"), this);
  startStopButton_->setCheckable(true);
  controlsLayout->addWidget(startStopButton_);

  refreshButton_ = new QPushButton(tr("Sync Nodes"), this);
  refreshButton_->setToolTip(tr("Sync node list from canvas"));
  controlsLayout->addWidget(refreshButton_);

  controlsLayout->addSpacing(20);

  controlsLayout->addWidget(new QLabel(tr("Update:"), this));
  intervalSpin_ = new QSpinBox(this);
  intervalSpin_->setRange(100, 10000);
  intervalSpin_->setValue(1000);
  intervalSpin_->setSuffix(" ms");
  controlsLayout->addWidget(intervalSpin_);

  controlsLayout->addStretch();

  heatmapCheck_ = new QCheckBox(tr("Heatmap"), this);
  heatmapCheck_->setToolTip(tr("Color canvas nodes by health metrics"));
  controlsLayout->addWidget(heatmapCheck_);

  heatmapModeCombo_ = new QComboBox(this);
  heatmapModeCombo_->addItem(tr("CPU Usage"));
  heatmapModeCombo_->addItem(tr("Memory Usage"));
  heatmapModeCombo_->addItem(tr("Health Status"));
  heatmapModeCombo_->setEnabled(false);
  controlsLayout->addWidget(heatmapModeCombo_);

  mainLayout->addLayout(controlsLayout);

  // Summary section
  QGroupBox* summaryGroup = new QGroupBox(tr("System Health"), this);
  QVBoxLayout* summaryLayout = new QVBoxLayout(summaryGroup);

  QHBoxLayout* countsLayout = new QHBoxLayout();

  healthyCountLabel_ = new QLabel("0", this);
  healthyCountLabel_->setStyleSheet("font-size: 18px; font-weight: bold; color: #4caf50;");
  QLabel* healthyLabel = new QLabel(tr("Healthy"), this);
  healthyLabel->setStyleSheet("color: #4caf50;");
  countsLayout->addWidget(healthyCountLabel_);
  countsLayout->addWidget(healthyLabel);

  countsLayout->addSpacing(20);

  warningCountLabel_ = new QLabel("0", this);
  warningCountLabel_->setStyleSheet("font-size: 18px; font-weight: bold; color: #ff9800;");
  QLabel* warningLabel = new QLabel(tr("Warning"), this);
  warningLabel->setStyleSheet("color: #ff9800;");
  countsLayout->addWidget(warningCountLabel_);
  countsLayout->addWidget(warningLabel);

  countsLayout->addSpacing(20);

  criticalCountLabel_ = new QLabel("0", this);
  criticalCountLabel_->setStyleSheet("font-size: 18px; font-weight: bold; color: #f44336;");
  QLabel* criticalLabel = new QLabel(tr("Critical"), this);
  criticalLabel->setStyleSheet("color: #f44336;");
  countsLayout->addWidget(criticalCountLabel_);
  countsLayout->addWidget(criticalLabel);

  countsLayout->addStretch();
  summaryLayout->addLayout(countsLayout);

  overallHealthBar_ = new QProgressBar(this);
  overallHealthBar_->setRange(0, 100);
  overallHealthBar_->setValue(100);
  overallHealthBar_->setFormat(tr("Overall Health: %p%"));
  overallHealthBar_->setStyleSheet(R"(
    QProgressBar {
      border: 1px solid #ccc;
      border-radius: 4px;
      text-align: center;
      height: 20px;
    }
    QProgressBar::chunk {
      background-color: #4caf50;
      border-radius: 3px;
    }
  )");
  summaryLayout->addWidget(overallHealthBar_);

  mainLayout->addWidget(summaryGroup);

  // Health table
  healthTable_ = new QTableWidget(this);
  healthTable_->setColumnCount(7);
  healthTable_->setHorizontalHeaderLabels({
    tr("Status"), tr("Node Name"), tr("CPU %"), tr("Memory"),
    tr("Latency"), tr("Dropped"), tr("Rate")
  });
  healthTable_->horizontalHeader()->setStretchLastSection(true);
  healthTable_->horizontalHeader()->setSectionResizeMode(COL_NAME, QHeaderView::Stretch);
  healthTable_->setColumnWidth(COL_STATUS, 60);
  healthTable_->setColumnWidth(COL_CPU, 70);
  healthTable_->setColumnWidth(COL_MEMORY, 80);
  healthTable_->setColumnWidth(COL_LATENCY, 80);
  healthTable_->setColumnWidth(COL_DROPPED, 70);
  healthTable_->setColumnWidth(COL_RATE, 70);
  healthTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
  healthTable_->setSelectionMode(QAbstractItemView::SingleSelection);
  healthTable_->setAlternatingRowColors(true);
  healthTable_->setSortingEnabled(true);
  healthTable_->setStyleSheet(R"(
    QTableWidget {
      border: 1px solid #ccc;
      border-radius: 4px;
    }
    QTableWidget::item {
      padding: 4px;
    }
  )");

  mainLayout->addWidget(healthTable_, 1);

  summaryLabel_ = new QLabel(tr("Not monitoring"), this);
  summaryLabel_->setStyleSheet("color: #666; font-size: 11px;");
  mainLayout->addWidget(summaryLabel_);
}

void NodeHealthDashboard::setupConnections() {
  connect(startStopButton_, &QPushButton::clicked, this, &NodeHealthDashboard::onStartStopClicked);
  connect(refreshButton_, &QPushButton::clicked, this, &NodeHealthDashboard::onRefreshClicked);
  connect(heatmapCheck_, &QCheckBox::toggled, this, &NodeHealthDashboard::onHeatmapToggled);
  connect(heatmapModeCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &NodeHealthDashboard::onHeatmapModeChanged);
  connect(intervalSpin_, QOverload<int>::of(&QSpinBox::valueChanged),
          this, &NodeHealthDashboard::onUpdateIntervalChanged);
  connect(healthTable_, &QTableWidget::cellDoubleClicked,
          this, &NodeHealthDashboard::onTableDoubleClicked);

  connect(healthMonitor_, &NodeHealthMonitor::healthUpdated,
          this, &NodeHealthDashboard::onHealthUpdated);
  connect(healthMonitor_, &NodeHealthMonitor::warningTriggered,
          this, &NodeHealthDashboard::onWarningTriggered);
  connect(healthMonitor_, &NodeHealthMonitor::criticalTriggered,
          this, &NodeHealthDashboard::onCriticalTriggered);
  connect(healthMonitor_, &NodeHealthMonitor::healthRecovered,
          this, &NodeHealthDashboard::onHealthRecovered);
  connect(healthMonitor_, &NodeHealthMonitor::monitoringStateChanged,
          this, &NodeHealthDashboard::onMonitoringStateChanged);
}

void NodeHealthDashboard::setCanvas(WeaverCanvas* canvas) {
  canvas_ = canvas;
}

void NodeHealthDashboard::syncNodesFromCanvas() {
  if (!canvas_) return;

  healthMonitor_->clearNodes();
  healthTable_->setRowCount(0);

  // Get all blocks from canvas and add them as nodes
  for (auto* block : canvas_->allBlocks()) {
    QString nodeName = "/" + block->packageName();
    healthMonitor_->addNode(nodeName);
  }

  summaryLabel_->setText(tr("Synced %1 nodes from canvas").arg(healthMonitor_->monitoredNodes().size()));
}

void NodeHealthDashboard::startMonitoring() {
  if (healthMonitor_->monitoredNodes().isEmpty()) {
    syncNodesFromCanvas();
  }
  healthMonitor_->start();
}

void NodeHealthDashboard::stopMonitoring() {
  healthMonitor_->stop();
}

void NodeHealthDashboard::setHeatmapEnabled(bool enabled) {
  heatmapEnabled_ = enabled;
  heatmapCheck_->setChecked(enabled);
  heatmapModeCombo_->setEnabled(enabled);
  updateHeatmapVisualization();
}

void NodeHealthDashboard::onHealthUpdated(const QString& nodeName, const NodeHealthData& health) {
  updateNodeRow(nodeName, health);
  updateSummary();

  if (heatmapEnabled_) {
    updateHeatmapVisualization();
  }
}

void NodeHealthDashboard::onWarningTriggered(const QString& nodeName, const QString& reason) {
  Q_UNUSED(reason)
  summaryLabel_->setText(tr("Warning: %1").arg(nodeName));
  summaryLabel_->setStyleSheet("color: #ff9800; font-size: 11px;");
}

void NodeHealthDashboard::onCriticalTriggered(const QString& nodeName, const QString& reason) {
  Q_UNUSED(reason)
  summaryLabel_->setText(tr("Critical: %1").arg(nodeName));
  summaryLabel_->setStyleSheet("color: #f44336; font-size: 11px; font-weight: bold;");
}

void NodeHealthDashboard::onHealthRecovered(const QString& nodeName) {
  summaryLabel_->setText(tr("Recovered: %1").arg(nodeName));
  summaryLabel_->setStyleSheet("color: #4caf50; font-size: 11px;");
}

void NodeHealthDashboard::onMonitoringStateChanged(bool isMonitoring) {
  startStopButton_->setText(isMonitoring ? tr("Stop Monitoring") : tr("Start Monitoring"));
  startStopButton_->setChecked(isMonitoring);

  if (!isMonitoring) {
    summaryLabel_->setText(tr("Monitoring stopped"));
    summaryLabel_->setStyleSheet("color: #666; font-size: 11px;");
  }
}

void NodeHealthDashboard::onStartStopClicked() {
  if (healthMonitor_->isMonitoring()) {
    stopMonitoring();
  } else {
    startMonitoring();
  }
}

void NodeHealthDashboard::onHeatmapToggled(bool checked) {
  heatmapEnabled_ = checked;
  heatmapModeCombo_->setEnabled(checked);
  updateHeatmapVisualization();
}

void NodeHealthDashboard::onHeatmapModeChanged(int index) {
  heatmapMode_ = index;
  updateHeatmapVisualization();
}

void NodeHealthDashboard::onTableDoubleClicked(int row, int column) {
  Q_UNUSED(column)
  QTableWidgetItem* item = healthTable_->item(row, COL_NAME);
  if (item) {
    QString nodeName = item->text();
    emit nodeSelected(nodeName);
    emit showHistoryRequested(nodeName);
  }
}

void NodeHealthDashboard::onRefreshClicked() {
  syncNodesFromCanvas();
}

void NodeHealthDashboard::onUpdateIntervalChanged(int value) {
  healthMonitor_->setUpdateInterval(value);
}

void NodeHealthDashboard::updateSummary() {
  int healthy = 0, warning = 0, critical = 0;

  for (const auto& health : healthMonitor_->getAllNodeHealth()) {
    switch (health.status) {
      case HealthStatus::Healthy:  healthy++;  break;
      case HealthStatus::Warning:  warning++;  break;
      case HealthStatus::Critical: critical++; break;
      default: break;
    }
  }

  healthyCountLabel_->setText(QString::number(healthy));
  warningCountLabel_->setText(QString::number(warning));
  criticalCountLabel_->setText(QString::number(critical));

  int total = healthy + warning + critical;
  if (total > 0) {
    int healthPercent = (healthy * 100) / total;
    overallHealthBar_->setValue(healthPercent);

    // Color the progress bar based on health
    if (critical > 0) {
      overallHealthBar_->setStyleSheet(R"(
        QProgressBar { border: 1px solid #ccc; border-radius: 4px; text-align: center; height: 20px; }
        QProgressBar::chunk { background-color: #f44336; border-radius: 3px; }
      )");
    } else if (warning > 0) {
      overallHealthBar_->setStyleSheet(R"(
        QProgressBar { border: 1px solid #ccc; border-radius: 4px; text-align: center; height: 20px; }
        QProgressBar::chunk { background-color: #ff9800; border-radius: 3px; }
      )");
    } else {
      overallHealthBar_->setStyleSheet(R"(
        QProgressBar { border: 1px solid #ccc; border-radius: 4px; text-align: center; height: 20px; }
        QProgressBar::chunk { background-color: #4caf50; border-radius: 3px; }
      )");
    }
  }
}

void NodeHealthDashboard::updateNodeRow(const QString& nodeName, const NodeHealthData& health) {
  int row = findNodeRow(nodeName);
  if (row < 0) {
    addNodeRow(nodeName, health);
    return;
  }

  // Update existing row
  healthTable_->item(row, COL_STATUS)->setText(health.getStatusString());
  healthTable_->item(row, COL_STATUS)->setBackground(health.getStatusColor());

  healthTable_->item(row, COL_CPU)->setText(QString::number(health.cpuPercent, 'f', 1));
  healthTable_->item(row, COL_MEMORY)->setText(QString("%1 MB").arg(health.memoryMB, 0, 'f', 1));
  healthTable_->item(row, COL_LATENCY)->setText(QString("%1 ms").arg(health.callbackLatencyMs, 0, 'f', 1));
  healthTable_->item(row, COL_DROPPED)->setText(QString::number(health.droppedMessages));
  healthTable_->item(row, COL_RATE)->setText(QString("%1 Hz").arg(health.messageRate, 0, 'f', 1));
}

int NodeHealthDashboard::findNodeRow(const QString& nodeName) {
  for (int row = 0; row < healthTable_->rowCount(); row++) {
    QTableWidgetItem* item = healthTable_->item(row, COL_NAME);
    if (item && item->text() == nodeName) {
      return row;
    }
  }
  return -1;
}

void NodeHealthDashboard::addNodeRow(const QString& nodeName, const NodeHealthData& health) {
  int row = healthTable_->rowCount();
  healthTable_->insertRow(row);

  QTableWidgetItem* statusItem = new QTableWidgetItem(health.getStatusString());
  statusItem->setBackground(health.getStatusColor());
  statusItem->setTextAlignment(Qt::AlignCenter);
  healthTable_->setItem(row, COL_STATUS, statusItem);

  QTableWidgetItem* nameItem = new QTableWidgetItem(nodeName);
  healthTable_->setItem(row, COL_NAME, nameItem);

  QTableWidgetItem* cpuItem = new QTableWidgetItem(QString::number(health.cpuPercent, 'f', 1));
  cpuItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
  healthTable_->setItem(row, COL_CPU, cpuItem);

  QTableWidgetItem* memItem = new QTableWidgetItem(QString("%1 MB").arg(health.memoryMB, 0, 'f', 1));
  memItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
  healthTable_->setItem(row, COL_MEMORY, memItem);

  QTableWidgetItem* latItem = new QTableWidgetItem(QString("%1 ms").arg(health.callbackLatencyMs, 0, 'f', 1));
  latItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
  healthTable_->setItem(row, COL_LATENCY, latItem);

  QTableWidgetItem* dropItem = new QTableWidgetItem(QString::number(health.droppedMessages));
  dropItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
  healthTable_->setItem(row, COL_DROPPED, dropItem);

  QTableWidgetItem* rateItem = new QTableWidgetItem(QString("%1 Hz").arg(health.messageRate, 0, 'f', 1));
  rateItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
  healthTable_->setItem(row, COL_RATE, rateItem);
}

void NodeHealthDashboard::updateHeatmapVisualization() {
  if (!canvas_ || !heatmapEnabled_) {
    // Reset all blocks to default color
    if (canvas_) {
      for (auto* block : canvas_->allBlocks()) {
        block->setHealthOverlayColor(QColor());  // Clear overlay
      }
    }
    return;
  }

  // Apply heatmap colors to canvas blocks
  for (auto* block : canvas_->allBlocks()) {
    QString nodeName = "/" + block->packageName();
    NodeHealthData health = healthMonitor_->getNodeHealth(nodeName);

    QColor heatColor = getHeatmapColor(health);
    block->setHealthOverlayColor(heatColor);
  }

  canvas_->update();
}

QColor NodeHealthDashboard::getHeatmapColor(const NodeHealthData& health) {
  switch (heatmapMode_) {
    case 0:  // CPU
      return health.getCpuHeatmapColor();
    case 1:  // Memory
      return health.getMemoryHeatmapColor();
    case 2:  // Status
    default:
      return health.getStatusColor();
  }
}

}  // namespace ros_weaver
