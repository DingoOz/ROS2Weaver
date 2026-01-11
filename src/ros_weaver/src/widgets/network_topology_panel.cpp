#include "ros_weaver/widgets/network_topology_panel.hpp"

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QSpinBox>
#include <QSplitter>
#include <QStackedWidget>
#include <QStyle>
#include <QTableWidget>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <cmath>

namespace ros_weaver {

const QList<QColor> NetworkTopologyPanel::kHostColors = {
    QColor(66, 133, 244),   // Blue
    QColor(52, 168, 83),    // Green
    QColor(251, 188, 4),    // Yellow
    QColor(234, 67, 53),    // Red
    QColor(154, 160, 166),  // Gray
    QColor(255, 112, 67),   // Orange
    QColor(156, 39, 176),   // Purple
    QColor(0, 172, 193),    // Cyan
    QColor(121, 85, 72),    // Brown
    QColor(233, 30, 99)     // Pink
};

// HostClusterItem implementation
HostClusterItem::HostClusterItem(const HostInfo& host, const QColor& color,
                                 QGraphicsItem* parent)
    : QGraphicsItem(parent)
    , hostInfo_(host)
    , baseColor_(color) {
  setFlag(QGraphicsItem::ItemIsSelectable);
  setAcceptHoverEvents(true);

  // Calculate bounds based on node count
  int nodeCount = qMax(1, host.nodeNames.size());
  int rows = static_cast<int>(std::ceil(std::sqrt(nodeCount)));
  int cols = static_cast<int>(std::ceil(static_cast<double>(nodeCount) / rows));

  double width = qMax(150.0, cols * 80.0 + 40.0);
  double height = qMax(100.0, rows * 30.0 + 60.0);
  bounds_ = QRectF(0, 0, width, height);
}

QRectF HostClusterItem::boundingRect() const {
  return bounds_.adjusted(-5, -5, 5, 5);
}

void HostClusterItem::paint(QPainter* painter,
                            const QStyleOptionGraphicsItem* option,
                            QWidget* widget) {
  Q_UNUSED(option)
  Q_UNUSED(widget)

  painter->setRenderHint(QPainter::Antialiasing);

  // Background
  QColor bgColor = baseColor_.lighter(180);
  if (highlighted_) {
    bgColor = baseColor_.lighter(140);
  }

  painter->setBrush(bgColor);
  painter->setPen(QPen(baseColor_, hostInfo_.isLocal ? 3 : 2));
  painter->drawRoundedRect(bounds_, 10, 10);

  // Local host indicator
  if (hostInfo_.isLocal) {
    painter->setPen(QPen(baseColor_.darker(120), 2, Qt::DashLine));
    painter->setBrush(Qt::NoBrush);
    painter->drawRoundedRect(bounds_.adjusted(4, 4, -4, -4), 8, 8);
  }

  // Host name header
  QRectF headerRect(0, 0, bounds_.width(), 30);
  painter->setBrush(baseColor_);
  painter->setPen(Qt::NoPen);
  painter->drawRoundedRect(headerRect, 10, 10);
  // Square off bottom corners
  painter->drawRect(QRectF(0, 20, bounds_.width(), 10));

  painter->setPen(Qt::white);
  painter->setFont(QFont("Sans", 10, QFont::Bold));
  QString displayName = hostInfo_.displayName();
  if (hostInfo_.isLocal) {
    displayName += " (local)";
  }
  painter->drawText(headerRect, Qt::AlignCenter, displayName);

  // Draw nodes inside
  painter->setFont(QFont("Sans", 8));
  int x = 20;
  int y = 45;
  int nodeWidth = 70;
  int nodeHeight = 24;
  int padding = 10;

  for (const QString& nodeName : hostInfo_.nodeNames) {
    if (x + nodeWidth > bounds_.width() - 10) {
      x = 20;
      y += nodeHeight + padding;
    }

    QRectF nodeRect(x, y, nodeWidth, nodeHeight);

    // Node background
    painter->setBrush(QColor(255, 255, 255, 200));
    painter->setPen(QPen(baseColor_.darker(110), 1));
    painter->drawRoundedRect(nodeRect, 4, 4);

    // Node name (shortened)
    painter->setPen(Qt::black);
    QString shortName = nodeName.split('/').last();
    if (shortName.length() > 10) {
      shortName = shortName.left(8) + "..";
    }
    painter->drawText(nodeRect, Qt::AlignCenter, shortName);

    x += nodeWidth + padding;
  }
}

void HostClusterItem::setHighlighted(bool highlighted) {
  highlighted_ = highlighted;
  update();
}

// NetworkTopologyPanel implementation
NetworkTopologyPanel::NetworkTopologyPanel(QWidget* parent)
    : QWidget(parent) {
  setupUi();
  setupConnections();
}

NetworkTopologyPanel::~NetworkTopologyPanel() = default;

void NetworkTopologyPanel::setCanvas(WeaverCanvas* canvas) {
  canvas_ = canvas;
}

void NetworkTopologyPanel::setNetworkTopologyManager(NetworkTopologyManager* manager) {
  if (manager_) {
    disconnect(manager_, nullptr, this, nullptr);
  }

  manager_ = manager;

  if (manager_) {
    connect(manager_, &NetworkTopologyManager::scanStarted,
            this, &NetworkTopologyPanel::onScanStarted);
    connect(manager_, &NetworkTopologyManager::scanProgress,
            this, &NetworkTopologyPanel::onScanProgress);
    connect(manager_, &NetworkTopologyManager::scanCompleted,
            this, &NetworkTopologyPanel::onTopologyUpdated);
    connect(manager_, &NetworkTopologyManager::scanFailed,
            this, &NetworkTopologyPanel::onScanFailed);

    // Sync UI with manager settings
    autoRefreshCheck_->setChecked(manager_->isAutoRefreshEnabled());
    refreshIntervalSpin_->setValue(manager_->autoRefreshInterval());

    // Set view mode from settings
    QString viewMode = manager_->viewMode();
    viewModeCombo_->setCurrentIndex(viewMode == "table" ? 1 : 0);
  }
}

void NetworkTopologyPanel::setupUi() {
  auto* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(8, 8, 8, 8);
  mainLayout->setSpacing(8);

  setupToolbar();
  setupStatusBar();

  viewStack_ = new QStackedWidget(this);
  setupGraphView();
  setupTableView();

  mainLayout->addWidget(viewStack_, 1);
}

void NetworkTopologyPanel::setupToolbar() {
  auto* toolbarLayout = new QHBoxLayout();
  toolbarLayout->setSpacing(8);

  refreshButton_ = new QPushButton(tr("Refresh"), this);
  refreshButton_->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));
  refreshButton_->setToolTip(tr("Scan network topology (R)"));
  refreshButton_->setShortcut(QKeySequence(Qt::Key_R));
  toolbarLayout->addWidget(refreshButton_);

  autoRefreshCheck_ = new QCheckBox(tr("Auto"), this);
  autoRefreshCheck_->setToolTip(tr("Enable automatic refresh"));
  toolbarLayout->addWidget(autoRefreshCheck_);

  refreshIntervalSpin_ = new QSpinBox(this);
  refreshIntervalSpin_->setRange(10, 300);
  refreshIntervalSpin_->setValue(30);
  refreshIntervalSpin_->setSuffix(tr("s"));
  refreshIntervalSpin_->setToolTip(tr("Auto-refresh interval in seconds"));
  refreshIntervalSpin_->setEnabled(false);
  toolbarLayout->addWidget(refreshIntervalSpin_);

  toolbarLayout->addSpacing(16);

  auto* viewLabel = new QLabel(tr("View:"), this);
  toolbarLayout->addWidget(viewLabel);

  viewModeCombo_ = new QComboBox(this);
  viewModeCombo_->addItem(tr("Graph"), QStringLiteral("graph"));
  viewModeCombo_->addItem(tr("Table"), QStringLiteral("table"));
  viewModeCombo_->setToolTip(tr("Switch between graph and table view"));
  toolbarLayout->addWidget(viewModeCombo_);

  toolbarLayout->addSpacing(16);

  zoomInButton_ = new QToolButton(this);
  zoomInButton_->setText(QStringLiteral("+"));
  zoomInButton_->setToolTip(tr("Zoom in"));
  toolbarLayout->addWidget(zoomInButton_);

  zoomOutButton_ = new QToolButton(this);
  zoomOutButton_->setText(QStringLiteral("-"));
  zoomOutButton_->setToolTip(tr("Zoom out"));
  toolbarLayout->addWidget(zoomOutButton_);

  fitViewButton_ = new QToolButton(this);
  fitViewButton_->setText(tr("Fit"));
  fitViewButton_->setToolTip(tr("Fit all hosts in view (F)"));
  toolbarLayout->addWidget(fitViewButton_);

  toolbarLayout->addStretch();

  qobject_cast<QVBoxLayout*>(layout())->addLayout(toolbarLayout);
}

void NetworkTopologyPanel::setupStatusBar() {
  auto* statusLayout = new QHBoxLayout();
  statusLayout->setSpacing(16);

  const QString boldStyle = QStringLiteral("font-weight: bold;");

  auto addLabelPair = [&](const QString& title, QLabel*& valueLabel,
                          const QString& defaultValue) {
    auto* titleLabel = new QLabel(title, this);
    titleLabel->setStyleSheet(boldStyle);
    statusLayout->addWidget(titleLabel);
    valueLabel = new QLabel(defaultValue, this);
    statusLayout->addWidget(valueLabel);
  };

  addLabelPair(tr("Domain:"), domainIdLabel_, QStringLiteral("0"));
  addLabelPair(tr("RMW:"), rmwLabel_, QStringLiteral("-"));
  rmwLabel_->setMaximumWidth(150);
  addLabelPair(tr("Discovery:"), discoveryTypeLabel_, QStringLiteral("-"));
  addLabelPair(tr("Hosts:"), hostCountLabel_, QStringLiteral("0"));
  addLabelPair(tr("Nodes:"), nodeCountLabel_, QStringLiteral("0"));

  statusLayout->addStretch();

  statusLabel_ = new QLabel(tr("Ready"), this);
  statusLabel_->setStyleSheet(QStringLiteral("color: #888;"));
  statusLayout->addWidget(statusLabel_);

  qobject_cast<QVBoxLayout*>(layout())->addLayout(statusLayout);
}

void NetworkTopologyPanel::setupGraphView() {
  graphScene_ = new QGraphicsScene(this);
  graphScene_->setBackgroundBrush(QColor(245, 245, 245));

  graphView_ = new QGraphicsView(graphScene_, this);
  graphView_->setRenderHint(QPainter::Antialiasing);
  graphView_->setDragMode(QGraphicsView::ScrollHandDrag);
  graphView_->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
  graphView_->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  graphView_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  viewStack_->addWidget(graphView_);
}

void NetworkTopologyPanel::setupTableView() {
  tableSplitter_ = new QSplitter(Qt::Vertical, this);

  // Hosts table
  hostsTable_ = new QTableWidget(this);
  hostsTable_->setColumnCount(4);
  hostsTable_->setHorizontalHeaderLabels({tr("Host"), tr("IP Address"),
                                          tr("Nodes"), tr("Status")});
  hostsTable_->horizontalHeader()->setStretchLastSection(true);
  hostsTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
  hostsTable_->setAlternatingRowColors(true);
  hostsTable_->setSortingEnabled(true);
  tableSplitter_->addWidget(hostsTable_);

  // Nodes tree
  nodesTree_ = new QTreeWidget(this);
  nodesTree_->setHeaderLabels({tr("Node"), tr("Publishers"), tr("Subscribers")});
  nodesTree_->setAlternatingRowColors(true);
  nodesTree_->header()->setStretchLastSection(true);
  tableSplitter_->addWidget(nodesTree_);

  tableSplitter_->setSizes({200, 300});

  viewStack_->addWidget(tableSplitter_);
}

void NetworkTopologyPanel::setupConnections() {
  connect(refreshButton_, &QPushButton::clicked,
          this, &NetworkTopologyPanel::onRefreshClicked);
  connect(autoRefreshCheck_, &QCheckBox::toggled,
          this, &NetworkTopologyPanel::onAutoRefreshToggled);
  connect(refreshIntervalSpin_, QOverload<int>::of(&QSpinBox::valueChanged),
          this, &NetworkTopologyPanel::onRefreshIntervalChanged);
  connect(viewModeCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &NetworkTopologyPanel::onViewModeChanged);

  connect(zoomInButton_, &QToolButton::clicked, this, &NetworkTopologyPanel::onZoomIn);
  connect(zoomOutButton_, &QToolButton::clicked, this, &NetworkTopologyPanel::onZoomOut);
  connect(fitViewButton_, &QToolButton::clicked, this, &NetworkTopologyPanel::onFitView);

  connect(graphScene_, &QGraphicsScene::selectionChanged, [this]() {
    QList<QGraphicsItem*> selected = graphScene_->selectedItems();
    if (!selected.isEmpty()) {
      onHostClicked(selected.first());
    }
  });

  connect(hostsTable_, &QTableWidget::itemClicked,
          this, &NetworkTopologyPanel::onTableItemClicked);
  connect(nodesTree_, &QTreeWidget::itemClicked,
          this, &NetworkTopologyPanel::onHostTreeItemClicked);
}

void NetworkTopologyPanel::refresh() {
  if (manager_) {
    manager_->scan();
  }
}

void NetworkTopologyPanel::setAutoRefreshEnabled(bool enabled) {
  autoRefreshCheck_->setChecked(enabled);
}

void NetworkTopologyPanel::onRefreshClicked() {
  refresh();
}

void NetworkTopologyPanel::onAutoRefreshToggled(bool enabled) {
  refreshIntervalSpin_->setEnabled(enabled);
  if (manager_) {
    manager_->setAutoRefreshEnabled(enabled);
  }
}

void NetworkTopologyPanel::onRefreshIntervalChanged(int value) {
  if (manager_) {
    manager_->setAutoRefreshInterval(value);
  }
}

void NetworkTopologyPanel::onViewModeChanged(int index) {
  viewStack_->setCurrentIndex(index);

  bool isGraphView = (index == 0);
  zoomInButton_->setEnabled(isGraphView);
  zoomOutButton_->setEnabled(isGraphView);
  fitViewButton_->setEnabled(isGraphView);

  if (manager_) {
    manager_->setViewMode(index == 0 ? "graph" : "table");
  }
}

void NetworkTopologyPanel::onScanStarted() {
  refreshButton_->setEnabled(false);
  statusLabel_->setText(tr("Scanning..."));
}

void NetworkTopologyPanel::onScanProgress(int percent, const QString& message) {
  statusLabel_->setText(QString("%1% - %2").arg(percent).arg(message));
}

void NetworkTopologyPanel::onScanFailed(const QString& error) {
  refreshButton_->setEnabled(true);
  statusLabel_->setText(tr("Error: %1").arg(error));
  statusLabel_->setStyleSheet("color: #d32f2f;");
}

void NetworkTopologyPanel::onTopologyUpdated(const NetworkTopology& topology) {
  currentTopology_ = topology;
  refreshButton_->setEnabled(true);
  statusLabel_->setText(tr("Ready"));
  statusLabel_->setStyleSheet("color: #888;");

  // Update status bar
  domainIdLabel_->setText(QString::number(topology.domainId));
  rmwLabel_->setText(topology.rmwImplementation);
  rmwLabel_->setToolTip(topology.rmwImplementation);
  discoveryTypeLabel_->setText(topology.discoveryTypeString());
  hostCountLabel_->setText(QString::number(topology.hosts.size()));
  nodeCountLabel_->setText(QString::number(topology.participants.size()));

  // Update views
  updateGraphView(topology);
  updateTableView(topology);
}

void NetworkTopologyPanel::updateGraphView(const NetworkTopology& topology) {
  clearGraphView();

  if (topology.hosts.isEmpty()) {
    return;
  }

  // Arrange hosts in a grid
  int cols = static_cast<int>(std::ceil(std::sqrt(topology.hosts.size())));
  double xSpacing = 250;
  double ySpacing = 200;

  int index = 0;
  for (const HostInfo& host : topology.hosts) {
    int row = index / cols;
    int col = index % cols;

    QPointF position(col * xSpacing, row * ySpacing);
    drawHostCluster(host, position);
    index++;
  }

  // Fit view to content
  QRectF sceneRect = graphScene_->itemsBoundingRect();
  graphScene_->setSceneRect(sceneRect.adjusted(-50, -50, 50, 50));
}

void NetworkTopologyPanel::drawHostCluster(const HostInfo& host,
                                            const QPointF& position) {
  int colorIndex = hostItems_.size() % kHostColors.size();
  QColor color = getHostColor(colorIndex);

  auto* item = new HostClusterItem(host, color);
  item->setPos(position);
  graphScene_->addItem(item);

  hostItems_[host.displayName()] = item;
}

void NetworkTopologyPanel::clearGraphView() {
  graphScene_->clear();
  hostItems_.clear();
}

void NetworkTopologyPanel::updateTableView(const NetworkTopology& topology) {
  updateHostsTable(topology.hosts);
  updateNodesTree(topology.participants);
}

void NetworkTopologyPanel::updateHostsTable(const QList<HostInfo>& hosts) {
  hostsTable_->setRowCount(0);
  hostsTable_->setRowCount(hosts.size());

  int row = 0;
  for (const HostInfo& host : hosts) {
    auto* nameItem = new QTableWidgetItem(host.displayName());
    auto* ipItem = new QTableWidgetItem(host.ipAddress);
    auto* nodesItem = new QTableWidgetItem(QString::number(host.nodeCount));
    auto* statusItem = new QTableWidgetItem(host.isLocal ? tr("Local") : tr("Remote"));

    // Color coding for local host
    if (host.isLocal) {
      QColor localColor(66, 133, 244, 50);
      nameItem->setBackground(localColor);
      ipItem->setBackground(localColor);
      nodesItem->setBackground(localColor);
      statusItem->setBackground(localColor);
    }

    hostsTable_->setItem(row, 0, nameItem);
    hostsTable_->setItem(row, 1, ipItem);
    hostsTable_->setItem(row, 2, nodesItem);
    hostsTable_->setItem(row, 3, statusItem);

    row++;
  }

  hostsTable_->resizeColumnsToContents();
}

void NetworkTopologyPanel::updateNodesTree(const QList<ParticipantInfo>& participants) {
  nodesTree_->clear();

  // Group by host
  QMap<QString, QList<ParticipantInfo>> byHost;
  for (const ParticipantInfo& p : participants) {
    QString hostKey = p.hostname.isEmpty() ? p.hostAddress : p.hostname;
    byHost[hostKey].append(p);
  }

  for (auto it = byHost.begin(); it != byHost.end(); ++it) {
    auto* hostItem = new QTreeWidgetItem(nodesTree_);
    hostItem->setText(0, it.key());
    hostItem->setExpanded(true);
    hostItem->setData(0, Qt::UserRole, "host");

    for (const ParticipantInfo& p : it.value()) {
      auto* nodeItem = new QTreeWidgetItem(hostItem);
      nodeItem->setText(0, p.nodeName);
      nodeItem->setText(1, QString::number(p.publishers.size()));
      nodeItem->setText(2, QString::number(p.subscribers.size()));
      nodeItem->setData(0, Qt::UserRole, "node");
      nodeItem->setData(0, Qt::UserRole + 1, p.nodeName);

      // Tooltip with details
      QString tooltip = QString("Publishers:\n%1\n\nSubscribers:\n%2")
                            .arg(p.publishers.isEmpty() ? "  (none)" : "  " + p.publishers.join("\n  "))
                            .arg(p.subscribers.isEmpty() ? "  (none)" : "  " + p.subscribers.join("\n  "));
      nodeItem->setToolTip(0, tooltip);
    }
  }

  nodesTree_->expandAll();
}

void NetworkTopologyPanel::onHostClicked(QGraphicsItem* item) {
  auto* hostItem = dynamic_cast<HostClusterItem*>(item);
  if (hostItem) {
    QString hostname = hostItem->hostName();
    QStringList nodes = hostItem->nodeNames();

    emit hostHighlighted(hostname);
    highlightCanvasNodes(nodes);
  }
}

void NetworkTopologyPanel::onTableItemClicked(QTableWidgetItem* item) {
  int row = item->row();
  QString hostname = hostsTable_->item(row, 0)->text();

  // Find host and highlight its nodes
  for (const HostInfo& host : currentTopology_.hosts) {
    if (host.displayName() == hostname) {
      highlightCanvasNodes(host.nodeNames);
      highlightHostInGraph(hostname);
      break;
    }
  }
}

void NetworkTopologyPanel::onHostTreeItemClicked(QTreeWidgetItem* item, int column) {
  Q_UNUSED(column)

  QString type = item->data(0, Qt::UserRole).toString();

  if (type == "node") {
    QString nodeName = item->data(0, Qt::UserRole + 1).toString();
    emit nodeSelectedOnCanvas(nodeName);
    highlightNodeInGraph(nodeName);
  } else if (type == "host") {
    QString hostname = item->text(0);
    // Get all nodes for this host
    QStringList nodes;
    for (int i = 0; i < item->childCount(); ++i) {
      nodes.append(item->child(i)->data(0, Qt::UserRole + 1).toString());
    }
    highlightCanvasNodes(nodes);
    highlightHostInGraph(hostname);
  }
}

void NetworkTopologyPanel::highlightCanvasNodes(const QStringList& nodeNames) {
  for (const QString& name : nodeNames) {
    emit nodeSelectedOnCanvas(name);
  }
}

void NetworkTopologyPanel::highlightHostInGraph(const QString& hostname) {
  // Clear all highlights
  for (auto* item : hostItems_) {
    item->setHighlighted(false);
  }

  // Highlight the selected host
  if (hostItems_.contains(hostname)) {
    hostItems_[hostname]->setHighlighted(true);
    graphView_->centerOn(hostItems_[hostname]);
  }
}

void NetworkTopologyPanel::highlightNodeInGraph(const QString& nodeName) {
  // Find which host contains this node and highlight it
  for (const HostInfo& host : currentTopology_.hosts) {
    if (host.nodeNames.contains(nodeName)) {
      highlightHostInGraph(host.displayName());
      break;
    }
  }
}

void NetworkTopologyPanel::onCanvasSelectionChanged() {
  // This would be called when canvas selection changes
  // to sync the topology view - implementation depends on canvas API
}

void NetworkTopologyPanel::onZoomIn() {
  if (currentZoom_ < kMaxZoom) {
    currentZoom_ += kZoomStep;
    graphView_->setTransform(QTransform::fromScale(currentZoom_, currentZoom_));
  }
}

void NetworkTopologyPanel::onZoomOut() {
  if (currentZoom_ > kMinZoom) {
    currentZoom_ -= kZoomStep;
    graphView_->setTransform(QTransform::fromScale(currentZoom_, currentZoom_));
  }
}

void NetworkTopologyPanel::onFitView() {
  graphView_->fitInView(graphScene_->itemsBoundingRect(),
                        Qt::KeepAspectRatio);
  currentZoom_ = graphView_->transform().m11();
}

QColor NetworkTopologyPanel::getHostColor(int index) const {
  return kHostColors.at(index % kHostColors.size());
}

}  // namespace ros_weaver
