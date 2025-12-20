#include "ros_weaver/widgets/system_mapping_panel.hpp"
#include "ros_weaver/core/system_discovery.hpp"
#include "ros_weaver/core/theme_manager.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QDateTime>
#include <QMenu>
#include <QAction>

namespace ros_weaver {

SystemMappingPanel::SystemMappingPanel(QWidget* parent)
  : QWidget(parent)
  , titleLabel_(nullptr)
  , summaryLabel_(nullptr)
  , lastScanLabel_(nullptr)
  , scanButton_(nullptr)
  , progressBar_(nullptr)
  , autoScanCheck_(nullptr)
  , autoScanIntervalSpin_(nullptr)
  , resultsTree_(nullptr)
  , matchedNodesRoot_(nullptr)
  , unmatchedCanvasRoot_(nullptr)
  , unmatchedSystemRoot_(nullptr)
  , topicsRoot_(nullptr)
  , discovery_(nullptr)
  , mapper_(nullptr)
  , isScanning_(false)
{
  setupUi();
}

SystemMappingPanel::~SystemMappingPanel() = default;

void SystemMappingPanel::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(8, 8, 8, 8);
  mainLayout->setSpacing(8);

  // Title
  titleLabel_ = new QLabel(tr("System Mapping"), this);
  QFont titleFont = titleLabel_->font();
  titleFont.setBold(true);
  titleFont.setPointSize(titleFont.pointSize() + 1);
  titleLabel_->setFont(titleFont);
  mainLayout->addWidget(titleLabel_);

  // Summary row
  QHBoxLayout* summaryLayout = new QHBoxLayout();
  summaryLabel_ = new QLabel(tr("No scan performed"), this);
  summaryLayout->addWidget(summaryLabel_);
  summaryLayout->addStretch();
  lastScanLabel_ = new QLabel(this);
  lastScanLabel_->setStyleSheet("color: gray;");
  summaryLayout->addWidget(lastScanLabel_);
  mainLayout->addLayout(summaryLayout);

  // Scan controls row
  QHBoxLayout* controlsLayout = new QHBoxLayout();

  scanButton_ = new QPushButton(tr("Scan System"), this);
  scanButton_->setIcon(QIcon::fromTheme("view-refresh"));
  scanButton_->setToolTip(tr("Scan running ROS2 system (Ctrl+Shift+R)"));
  connect(scanButton_, &QPushButton::clicked, this, &SystemMappingPanel::onScanClicked);
  controlsLayout->addWidget(scanButton_);

  controlsLayout->addSpacing(16);

  autoScanCheck_ = new QCheckBox(tr("Auto-scan"), this);
  autoScanCheck_->setToolTip(tr("Automatically scan at regular intervals"));
  connect(autoScanCheck_, &QCheckBox::toggled, this, &SystemMappingPanel::onAutoScanToggled);
  controlsLayout->addWidget(autoScanCheck_);

  autoScanIntervalSpin_ = new QSpinBox(this);
  autoScanIntervalSpin_->setRange(5, 60);
  autoScanIntervalSpin_->setValue(10);
  autoScanIntervalSpin_->setSuffix(tr(" sec"));
  autoScanIntervalSpin_->setToolTip(tr("Auto-scan interval"));
  autoScanIntervalSpin_->setEnabled(false);
  connect(autoScanIntervalSpin_, QOverload<int>::of(&QSpinBox::valueChanged),
          this, &SystemMappingPanel::onAutoScanIntervalChanged);
  controlsLayout->addWidget(autoScanIntervalSpin_);

  controlsLayout->addStretch();
  mainLayout->addLayout(controlsLayout);

  // Progress bar (hidden by default)
  progressBar_ = new QProgressBar(this);
  progressBar_->setVisible(false);
  progressBar_->setTextVisible(true);
  mainLayout->addWidget(progressBar_);

  // Results tree
  resultsTree_ = new QTreeWidget(this);
  resultsTree_->setHeaderLabels({tr("Name"), tr("ROS2 Node"), tr("Status")});
  resultsTree_->setAlternatingRowColors(true);
  resultsTree_->setRootIsDecorated(true);
  resultsTree_->setSelectionMode(QAbstractItemView::SingleSelection);
  resultsTree_->header()->setStretchLastSection(true);
  resultsTree_->header()->setSectionResizeMode(0, QHeaderView::Interactive);
  resultsTree_->header()->setSectionResizeMode(1, QHeaderView::Interactive);
  resultsTree_->setColumnWidth(0, 180);
  resultsTree_->setColumnWidth(1, 180);

  connect(resultsTree_, &QTreeWidget::itemClicked,
          this, &SystemMappingPanel::onTreeItemClicked);
  connect(resultsTree_, &QTreeWidget::itemDoubleClicked,
          this, &SystemMappingPanel::onTreeItemDoubleClicked);

  // Create root items
  matchedNodesRoot_ = new QTreeWidgetItem(resultsTree_);
  matchedNodesRoot_->setText(0, tr("Matched Nodes"));
  matchedNodesRoot_->setExpanded(true);

  unmatchedCanvasRoot_ = new QTreeWidgetItem(resultsTree_);
  unmatchedCanvasRoot_->setText(0, tr("Unmatched Canvas Blocks"));
  unmatchedCanvasRoot_->setExpanded(true);

  unmatchedSystemRoot_ = new QTreeWidgetItem(resultsTree_);
  unmatchedSystemRoot_->setText(0, tr("Extra Nodes in System"));
  unmatchedSystemRoot_->setExpanded(false);

  topicsRoot_ = new QTreeWidgetItem(resultsTree_);
  topicsRoot_->setText(0, tr("Topic Summary"));
  topicsRoot_->setExpanded(false);

  mainLayout->addWidget(resultsTree_, 1);

  // Context menu for tree
  resultsTree_->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(resultsTree_, &QTreeWidget::customContextMenuRequested,
          this, [this](const QPoint& pos) {
    QTreeWidgetItem* item = resultsTree_->itemAt(pos);
    if (!item) return;

    // Check if this is an unmatched system node
    if (item->parent() == unmatchedSystemRoot_) {
      QMenu menu(this);
      QAction* addAction = menu.addAction(tr("Add to Canvas"));
      connect(addAction, &QAction::triggered, this, [this, item]() {
        emit addNodeToCanvasRequested(item->text(0));
      });
      menu.exec(resultsTree_->mapToGlobal(pos));
    }
  });
}

void SystemMappingPanel::setSystemDiscovery(SystemDiscovery* discovery) {
  if (discovery_ == discovery) return;

  // Disconnect old
  if (discovery_) {
    disconnect(discovery_, nullptr, this, nullptr);
  }

  discovery_ = discovery;

  // Connect new
  if (discovery_) {
    connect(discovery_, &SystemDiscovery::scanStarted,
            this, &SystemMappingPanel::onScanStarted);
    connect(discovery_, &SystemDiscovery::scanProgress,
            this, &SystemMappingPanel::onScanProgress);
    connect(discovery_, &SystemDiscovery::scanCompleted,
            this, &SystemMappingPanel::onScanCompleted);
    connect(discovery_, &SystemDiscovery::scanFailed,
            this, &SystemMappingPanel::onScanFailed);

    // Update auto-scan state
    autoScanCheck_->setChecked(discovery_->isAutoScanEnabled());
    autoScanIntervalSpin_->setValue(discovery_->autoScanInterval());
  }
}

void SystemMappingPanel::setCanvasMapper(CanvasMapper* mapper) {
  if (mapper_ == mapper) return;

  // Disconnect old
  if (mapper_) {
    disconnect(mapper_, nullptr, this, nullptr);
  }

  mapper_ = mapper;

  // Connect new
  if (mapper_) {
    connect(mapper_, &CanvasMapper::mappingCompleted,
            this, &SystemMappingPanel::onMappingCompleted);
  }
}

void SystemMappingPanel::updateResults(const MappingResults& results) {
  lastResults_ = results;

  // Clear existing items (keep root items)
  while (matchedNodesRoot_->childCount() > 0) {
    delete matchedNodesRoot_->takeChild(0);
  }
  while (unmatchedCanvasRoot_->childCount() > 0) {
    delete unmatchedCanvasRoot_->takeChild(0);
  }
  while (unmatchedSystemRoot_->childCount() > 0) {
    delete unmatchedSystemRoot_->takeChild(0);
  }
  while (topicsRoot_->childCount() > 0) {
    delete topicsRoot_->takeChild(0);
  }

  // Populate matched and unmatched canvas blocks
  for (const BlockMappingResult& result : results.blockMappings) {
    QTreeWidgetItem* item = new QTreeWidgetItem();
    item->setText(0, result.canvasName);
    item->setData(0, Qt::UserRole, result.canvasBlockId);

    if (result.confidence != MatchConfidence::None) {
      item->setText(1, result.ros2NodeName);
      item->setText(2, confidenceToString(result.confidence));
      item->setForeground(2, confidenceToColor(result.confidence));
      item->setToolTip(0, result.matchReason);

      // Add topic details as children
      auto& theme = ThemeManager::instance();
      for (const TopicMapping& tm : result.topicMappings) {
        QTreeWidgetItem* topicItem = new QTreeWidgetItem(item);
        QString direction = tm.isPublisher ? QString::fromUtf8("\u2192") : QString::fromUtf8("\u2190");
        topicItem->setText(0, QString("%1 %2").arg(direction, tm.canvasPinName));

        if (!tm.ros2TopicName.isEmpty()) {
          topicItem->setText(1, tm.ros2TopicName);
          topicItem->setText(2, tm.isActive ? tr("Active") : tr("Inactive"));
          topicItem->setForeground(2, tm.isActive ? theme.successColor() : theme.textSecondaryColor());
        } else {
          topicItem->setText(1, tr("(not found)"));
          topicItem->setText(2, tr("Missing"));
          topicItem->setForeground(2, theme.errorColor());
        }
      }

      matchedNodesRoot_->addChild(item);
    } else {
      item->setText(1, tr("(not found)"));
      item->setText(2, tr("Not Running"));
      item->setForeground(2, ThemeManager::instance().textSecondaryColor());
      unmatchedCanvasRoot_->addChild(item);
    }
  }

  // Populate unmatched system nodes
  for (const QString& nodeName : results.summary.unmatchedRos2Nodes) {
    QTreeWidgetItem* item = new QTreeWidgetItem(unmatchedSystemRoot_);
    item->setText(0, nodeName);
    item->setText(2, tr("Not on Canvas"));
    item->setForeground(2, ThemeManager::instance().primaryColor());
  }

  // Update root item labels with counts
  matchedNodesRoot_->setText(0, tr("Matched Nodes (%1)").arg(results.summary.matchedBlocks));
  unmatchedCanvasRoot_->setText(0, tr("Unmatched Canvas Blocks (%1)")
    .arg(results.summary.totalCanvasBlocks - results.summary.matchedBlocks));
  unmatchedSystemRoot_->setText(0, tr("Extra Nodes in System (%1)")
    .arg(results.summary.unmatchedRos2Nodes.size()));

  // Add topic summary
  QTreeWidgetItem* topicMatchItem = new QTreeWidgetItem(topicsRoot_);
  topicMatchItem->setText(0, tr("Matched Topics"));
  topicMatchItem->setText(1, tr("%1 / %2")
    .arg(results.summary.matchedTopics)
    .arg(results.summary.totalCanvasTopics));

  QTreeWidgetItem* topicActiveItem = new QTreeWidgetItem(topicsRoot_);
  topicActiveItem->setText(0, tr("Active Topics"));
  topicActiveItem->setText(1, QString::number(results.summary.activeTopics));

  QTreeWidgetItem* topicExtraItem = new QTreeWidgetItem(topicsRoot_);
  topicExtraItem->setText(0, tr("Extra System Topics"));
  topicExtraItem->setText(1, QString::number(results.summary.unmatchedRos2Topics.size()));

  topicsRoot_->setText(0, tr("Topic Summary (%1/%2 matched)")
    .arg(results.summary.matchedTopics)
    .arg(results.summary.totalCanvasTopics));

  updateSummaryLabel();

  // Update last scan time
  QDateTime scanTime = QDateTime::fromMSecsSinceEpoch(results.summary.timestamp);
  lastScanLabel_->setText(tr("Last: %1").arg(scanTime.toString("hh:mm:ss")));
}

void SystemMappingPanel::clearResults() {
  while (matchedNodesRoot_->childCount() > 0) {
    delete matchedNodesRoot_->takeChild(0);
  }
  while (unmatchedCanvasRoot_->childCount() > 0) {
    delete unmatchedCanvasRoot_->takeChild(0);
  }
  while (unmatchedSystemRoot_->childCount() > 0) {
    delete unmatchedSystemRoot_->takeChild(0);
  }
  while (topicsRoot_->childCount() > 0) {
    delete topicsRoot_->takeChild(0);
  }

  lastResults_ = MappingResults();
  summaryLabel_->setText(tr("No scan performed"));
  lastScanLabel_->clear();
}

void SystemMappingPanel::updateSummaryLabel() {
  const auto& s = lastResults_.summary;
  summaryLabel_->setText(tr("Nodes: %1/%2 matched | Topics: %3/%4 active")
    .arg(s.matchedBlocks)
    .arg(s.totalCanvasBlocks)
    .arg(s.activeTopics)
    .arg(s.totalCanvasTopics));
}

QString SystemMappingPanel::confidenceToString(MatchConfidence confidence) const {
  switch (confidence) {
    case MatchConfidence::High: return tr("High Match");
    case MatchConfidence::Medium: return tr("Medium Match");
    case MatchConfidence::Low: return tr("Low Match");
    case MatchConfidence::None: return tr("No Match");
  }
  return tr("Unknown");
}

QString SystemMappingPanel::confidenceToIcon(MatchConfidence confidence) const {
  switch (confidence) {
    case MatchConfidence::High: return QString::fromUtf8("\u2714");   // Check mark
    case MatchConfidence::Medium: return QString::fromUtf8("\u223C"); // Tilde
    case MatchConfidence::Low: return QString::fromUtf8("\u2248");    // Almost equal
    case MatchConfidence::None: return QString::fromUtf8("\u2718");   // X mark
  }
  return "?";
}

QColor SystemMappingPanel::confidenceToColor(MatchConfidence confidence) const {
  auto& theme = ThemeManager::instance();
  switch (confidence) {
    case MatchConfidence::High: return theme.successColor();
    case MatchConfidence::Medium: return theme.warningColor();
    case MatchConfidence::Low: return theme.warningColor().darker(120);
    case MatchConfidence::None: return theme.textSecondaryColor();
  }
  return QColor();
}

void SystemMappingPanel::onScanClicked() {
  emit scanRequested();
  if (discovery_ && !isScanning_) {
    discovery_->scan();
  }
}

void SystemMappingPanel::onAutoScanToggled(bool enabled) {
  autoScanIntervalSpin_->setEnabled(enabled);
  if (discovery_) {
    discovery_->setAutoScanEnabled(enabled);
  }
}

void SystemMappingPanel::onAutoScanIntervalChanged(int seconds) {
  if (discovery_) {
    discovery_->setAutoScanInterval(seconds);
  }
}

void SystemMappingPanel::onScanStarted() {
  isScanning_ = true;
  scanButton_->setEnabled(false);
  progressBar_->setVisible(true);
  progressBar_->setValue(0);
}

void SystemMappingPanel::onScanProgress(int percent, const QString& message) {
  progressBar_->setValue(percent);
  progressBar_->setFormat(message);
}

void SystemMappingPanel::onScanCompleted(const SystemGraph& /* graph */) {
  isScanning_ = false;
  scanButton_->setEnabled(true);
  progressBar_->setVisible(false);
}

void SystemMappingPanel::onScanFailed(const QString& error) {
  isScanning_ = false;
  scanButton_->setEnabled(true);
  progressBar_->setVisible(false);
  summaryLabel_->setText(tr("Scan failed: %1").arg(error));
}

void SystemMappingPanel::onMappingCompleted(const MappingResults& results) {
  updateResults(results);
}

void SystemMappingPanel::onTreeItemClicked(QTreeWidgetItem* item, int /* column */) {
  if (!item || !item->parent()) return;

  // Get block ID if available
  QVariant blockIdVar = item->data(0, Qt::UserRole);
  if (blockIdVar.isValid()) {
    QUuid blockId = blockIdVar.toUuid();
    if (!blockId.isNull()) {
      emit blockSelected(blockId);
    }
  }
}

void SystemMappingPanel::onTreeItemDoubleClicked(QTreeWidgetItem* item, int /* column */) {
  // Expand/collapse on double-click for items with children
  if (item && item->childCount() > 0) {
    item->setExpanded(!item->isExpanded());
  }
}

}  // namespace ros_weaver
