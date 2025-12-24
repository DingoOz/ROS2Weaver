#include "ros_weaver/widgets/topic_viewer_panel.hpp"
#include "ros_weaver/widgets/topic_list_model.hpp"
#include "ros_weaver/widgets/lineage_dialog.hpp"
#include "ros_weaver/core/lineage_provider.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/connection_line.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QMenu>
#include <QAction>
#include <QMessageBox>
#include <QDateTime>
#include <QApplication>
#include <QClipboard>
#include <iostream>

namespace ros_weaver {

TopicViewerPanel::TopicViewerPanel(QWidget* parent)
  : QWidget(parent)
  , splitter_(nullptr)
  , searchEdit_(nullptr)
  , quickFilterCombo_(nullptr)
  , refreshButton_(nullptr)
  , autoRefreshButton_(nullptr)
  , monitorButton_(nullptr)
  , autoMonitorButton_(nullptr)
  , statusLabel_(nullptr)
  , topicTreeView_(nullptr)
  , topicModel_(nullptr)
  , proxyModel_(nullptr)
  , detailsPanel_(nullptr)
  , topicNameLabel_(nullptr)
  , topicTypeLabel_(nullptr)
  , topicStatsLabel_(nullptr)
  , messagePreviewEdit_(nullptr)
  , canvas_(nullptr)
  , rosNode_(nullptr)
  , spinning_(false)
  , monitoring_(false)
  , autoRefreshTimer_(nullptr)
  , autoRefreshEnabled_(true)
  , autoRefreshInterval_(2)
{
  // Register meta types for cross-thread signals
  static bool typesRegistered = false;
  if (!typesRegistered) {
    qRegisterMetaType<TopicDisplayInfo>("TopicDisplayInfo");
    qRegisterMetaType<QList<TopicDisplayInfo>>("QList<TopicDisplayInfo>");
    typesRegistered = true;
  }

  setupUi();
  setupConnections();

  // Initialize ROS node
  initializeRosNode();

  // Start auto-refresh timer
  autoRefreshTimer_ = new QTimer(this);
  connect(autoRefreshTimer_, &QTimer::timeout, this, &TopicViewerPanel::onAutoRefreshTimer);
  if (autoRefreshEnabled_) {
    autoRefreshTimer_->start(autoRefreshInterval_ * 1000);
  }

  // Inactivity detection timer - checks for topics that stopped sending
  inactivityTimer_ = new QTimer(this);
  connect(inactivityTimer_, &QTimer::timeout, this, [this]() {
    checkTopicInactivity();
  });
  inactivityTimer_->start(INACTIVITY_CHECK_INTERVAL_MS);

  // Initial topic discovery
  QTimer::singleShot(500, this, &TopicViewerPanel::refreshTopics);
}

TopicViewerPanel::~TopicViewerPanel() {
  std::cerr << "TopicViewerPanel destructor: starting" << std::endl;
  stopMonitoring();
  std::cerr << "TopicViewerPanel: shutdownRosNode..." << std::endl;
  shutdownRosNode();
  std::cerr << "TopicViewerPanel destructor: complete" << std::endl;
}

void TopicViewerPanel::setupUi() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(4, 4, 4, 4);
  mainLayout->setSpacing(4);

  setupToolbar();
  mainLayout->addWidget(createToolbarWidget());

  // Splitter for topic list and details
  splitter_ = new QSplitter(Qt::Vertical, this);

  setupTopicList();
  splitter_->addWidget(topicTreeView_);

  setupDetailsPanel();
  splitter_->addWidget(detailsPanel_);

  splitter_->setStretchFactor(0, 3);
  splitter_->setStretchFactor(1, 1);

  mainLayout->addWidget(splitter_);

  // Status bar
  statusLabel_ = new QLabel(tr("Ready"), this);
  statusLabel_->setStyleSheet("color: gray; font-size: 10px;");
  mainLayout->addWidget(statusLabel_);
}

QWidget* TopicViewerPanel::createToolbarWidget() {
  QWidget* toolbar = new QWidget();
  QHBoxLayout* layout = new QHBoxLayout(toolbar);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(4);

  // Search box
  searchEdit_ = new QLineEdit();
  searchEdit_->setPlaceholderText(tr("Search topics..."));
  searchEdit_->setClearButtonEnabled(true);
  searchEdit_->setMaximumWidth(200);
  layout->addWidget(searchEdit_);

  // Quick filter combo
  quickFilterCombo_ = new QComboBox();
  quickFilterCombo_->addItem(tr("All Topics"), "all");
  quickFilterCombo_->addItem(tr("Active Only"), "active");
  quickFilterCombo_->addItem(tr("With Publishers"), "pubs");
  quickFilterCombo_->addItem(tr("With Subscribers"), "subs");
  quickFilterCombo_->addItem(tr("sensor_msgs/*"), "sensor");
  quickFilterCombo_->addItem(tr("geometry_msgs/*"), "geometry");
  quickFilterCombo_->addItem(tr("nav_msgs/*"), "nav");
  quickFilterCombo_->setMaximumWidth(140);
  layout->addWidget(quickFilterCombo_);

  layout->addStretch();

  // Refresh button
  refreshButton_ = new QPushButton(tr("Refresh"));
  refreshButton_->setToolTip(tr("Refresh topic list (F5)"));
  refreshButton_->setShortcut(QKeySequence(Qt::Key_F5));
  layout->addWidget(refreshButton_);

  // Auto-refresh toggle
  autoRefreshButton_ = new QToolButton();
  autoRefreshButton_->setText(tr("Auto"));
  autoRefreshButton_->setCheckable(true);
  autoRefreshButton_->setChecked(autoRefreshEnabled_);
  autoRefreshButton_->setToolTip(tr("Toggle auto-refresh (every %1 seconds)").arg(autoRefreshInterval_));
  layout->addWidget(autoRefreshButton_);

  // Monitor toggle
  monitorButton_ = new QToolButton();
  monitorButton_->setText(tr("Live"));
  monitorButton_->setCheckable(true);
  monitorButton_->setChecked(false);
  monitorButton_->setToolTip(tr("Toggle live message monitoring"));
  layout->addWidget(monitorButton_);

  // Auto-monitor canvas topics button
  autoMonitorButton_ = new QPushButton(tr("Canvas"));
  autoMonitorButton_->setToolTip(tr("Auto-monitor topics matching canvas connections"));
  layout->addWidget(autoMonitorButton_);

  return toolbar;
}

void TopicViewerPanel::setupToolbar() {
  // Toolbar setup is done in createToolbarWidget
}

void TopicViewerPanel::setupTopicList() {
  topicTreeView_ = new QTreeView();
  topicModel_ = new TopicListModel(this);

  proxyModel_ = new QSortFilterProxyModel(this);
  proxyModel_->setSourceModel(topicModel_);
  proxyModel_->setFilterCaseSensitivity(Qt::CaseInsensitive);
  proxyModel_->setFilterKeyColumn(0);  // Filter on topic name

  topicTreeView_->setModel(proxyModel_);
  topicTreeView_->setRootIsDecorated(false);
  topicTreeView_->setAlternatingRowColors(true);
  topicTreeView_->setSortingEnabled(true);
  topicTreeView_->setSelectionMode(QAbstractItemView::SingleSelection);
  topicTreeView_->setContextMenuPolicy(Qt::CustomContextMenu);

  // Configure header
  QHeaderView* header = topicTreeView_->header();
  header->setStretchLastSection(false);
  header->setSectionResizeMode(0, QHeaderView::Stretch);  // Name
  header->setSectionResizeMode(1, QHeaderView::ResizeToContents);  // Type
  header->setSectionResizeMode(2, QHeaderView::ResizeToContents);  // Rate
  header->setSectionResizeMode(3, QHeaderView::ResizeToContents);  // Pubs
  header->setSectionResizeMode(4, QHeaderView::ResizeToContents);  // Subs

  // Sort by name initially
  topicTreeView_->sortByColumn(0, Qt::AscendingOrder);
}

void TopicViewerPanel::setupDetailsPanel() {
  detailsPanel_ = new QWidget();
  QVBoxLayout* layout = new QVBoxLayout(detailsPanel_);
  layout->setContentsMargins(4, 4, 4, 4);
  layout->setSpacing(2);

  // Topic info labels
  topicNameLabel_ = new QLabel(tr("Select a topic"));
  topicNameLabel_->setStyleSheet("font-weight: bold; font-size: 12px;");
  layout->addWidget(topicNameLabel_);

  topicTypeLabel_ = new QLabel();
  topicTypeLabel_->setStyleSheet("color: #888; font-size: 10px;");
  layout->addWidget(topicTypeLabel_);

  topicStatsLabel_ = new QLabel();
  topicStatsLabel_->setStyleSheet("font-size: 10px;");
  layout->addWidget(topicStatsLabel_);

  // Message preview
  QLabel* previewLabel = new QLabel(tr("Message Preview:"));
  previewLabel->setStyleSheet("font-size: 10px; margin-top: 4px;");
  layout->addWidget(previewLabel);

  messagePreviewEdit_ = new QTextEdit();
  messagePreviewEdit_->setReadOnly(true);
  messagePreviewEdit_->setFont(QFont("Monospace", 9));
  messagePreviewEdit_->setPlaceholderText(tr("No message data. Enable 'Live' monitoring to see messages."));
  messagePreviewEdit_->setMaximumHeight(150);
  layout->addWidget(messagePreviewEdit_);

  layout->addStretch();
}

void TopicViewerPanel::setupConnections() {
  // Search filter
  connect(searchEdit_, &QLineEdit::textChanged, this, &TopicViewerPanel::setSearchFilter);

  // Quick filter
  connect(quickFilterCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &TopicViewerPanel::setQuickFilter);

  // Refresh button
  connect(refreshButton_, &QPushButton::clicked, this, &TopicViewerPanel::refreshTopics);

  // Auto-refresh toggle
  connect(autoRefreshButton_, &QToolButton::toggled, [this](bool checked) {
    autoRefreshEnabled_ = checked;
    if (checked) {
      autoRefreshTimer_->start(autoRefreshInterval_ * 1000);
    } else {
      autoRefreshTimer_->stop();
    }
  });

  // Monitor toggle
  connect(monitorButton_, &QToolButton::toggled, [this](bool checked) {
    if (checked) {
      startMonitoring();
    } else {
      stopMonitoring();
    }
  });

  // Auto-monitor canvas topics
  connect(autoMonitorButton_, &QPushButton::clicked, this, &TopicViewerPanel::autoMonitorCanvasTopics);

  // Topic selection
  connect(topicTreeView_->selectionModel(), &QItemSelectionModel::currentChanged,
          [this](const QModelIndex& current, const QModelIndex&) {
            onTopicSelected(current);
          });

  // Double-click
  connect(topicTreeView_, &QTreeView::doubleClicked, this, &TopicViewerPanel::onTopicDoubleClicked);

  // Context menu
  connect(topicTreeView_, &QTreeView::customContextMenuRequested, this, &TopicViewerPanel::onContextMenu);

  // Cross-thread signals
  connect(this, &TopicViewerPanel::topicsDiscovered,
          this, &TopicViewerPanel::onTopicsDiscovered, Qt::QueuedConnection);
  connect(this, &TopicViewerPanel::messageReceived,
          this, &TopicViewerPanel::onMessageReceived, Qt::QueuedConnection);

  // Model updates
  connect(topicModel_, &TopicListModel::topicUpdated, [this](const QString& topicName) {
    if (topicName == selectedTopic_) {
      updateTopicDetails(topicName);
    }
  });
}

void TopicViewerPanel::initializeRosNode() {
  if (rosNode_) return;

  try {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    std::string nodeName = "ros_weaver_topic_viewer_" +
                           std::to_string(QDateTime::currentMSecsSinceEpoch());
    rosNode_ = std::make_shared<rclcpp::Node>(nodeName);

    spinning_ = true;
    spinThread_ = std::make_unique<std::thread>([this]() {
      while (spinning_ && rclcpp::ok()) {
        rclcpp::spin_some(rosNode_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });

  } catch (const std::exception& e) {
    statusLabel_->setText(tr("ROS2 Error: %1").arg(e.what()));
  }
}

void TopicViewerPanel::shutdownRosNode() {
  std::cerr << "TopicViewerPanel::shutdownRosNode() starting" << std::endl;
  spinning_ = false;

  // Clear subscriptions first
  {
    std::lock_guard<std::mutex> lock(topicsMutex_);
    activeSubscriptions_.clear();
  }

  // Wait for discovery thread to complete to avoid use-after-free
  if (discoveryThread_ && discoveryThread_->joinable()) {
    std::cerr << "TopicViewerPanel: joining discoveryThread..." << std::endl;
    discoveryThread_->join();
    std::cerr << "TopicViewerPanel: discoveryThread joined" << std::endl;
  }
  discoveryThread_.reset();

  if (spinThread_ && spinThread_->joinable()) {
    std::cerr << "TopicViewerPanel: joining spinThread..." << std::endl;
    spinThread_->join();
    std::cerr << "TopicViewerPanel: spinThread joined" << std::endl;
  }
  spinThread_.reset();
  rosNode_.reset();
  std::cerr << "TopicViewerPanel::shutdownRosNode() complete" << std::endl;
}

void TopicViewerPanel::refreshTopics() {
  if (!rosNode_) {
    initializeRosNode();
    if (!rosNode_) return;
  }

  statusLabel_->setText(tr("Discovering topics..."));

  // Wait for any previous discovery thread to complete
  if (discoveryThread_ && discoveryThread_->joinable()) {
    discoveryThread_->join();
  }

  // Perform discovery in background thread (tracked to avoid use-after-free)
  discoveryThread_ = std::make_unique<std::thread>([this]() {
    if (!spinning_.load()) return;  // Early exit if shutting down
    performTopicDiscovery();
  });
}

void TopicViewerPanel::performTopicDiscovery() {
  if (!rosNode_) return;

  QList<TopicDisplayInfo> topics;

  try {
    auto topicNamesAndTypes = rosNode_->get_topic_names_and_types();

    for (const auto& [topicName, types] : topicNamesAndTypes) {
      TopicDisplayInfo info;
      info.name = QString::fromStdString(topicName);

      if (!types.empty()) {
        info.type = QString::fromStdString(types[0]);
        // Extract short type name (e.g., "LaserScan" from "sensor_msgs/msg/LaserScan")
        QStringList parts = info.type.split('/');
        if (parts.size() >= 3) {
          info.shortType = parts.last();
        } else {
          info.shortType = info.type;
        }
      }

      // Get publisher/subscriber counts
      auto pubInfo = rosNode_->get_publishers_info_by_topic(topicName);
      auto subInfo = rosNode_->get_subscriptions_info_by_topic(topicName);
      info.publisherCount = static_cast<int>(pubInfo.size());
      info.subscriberCount = static_cast<int>(subInfo.size());

      topics.append(info);
    }

  } catch (const std::exception& e) {
    // Error handled via signal
  }

  emit topicsDiscovered(topics);
}

void TopicViewerPanel::onTopicsDiscovered(const QList<TopicDisplayInfo>& topics) {
  {
    std::lock_guard<std::mutex> lock(topicsMutex_);
    allTopics_ = topics;
  }

  topicModel_->setTopics(topics);
  statusLabel_->setText(tr("%1 topics discovered").arg(topics.size()));
  onFilterChanged();
}

void TopicViewerPanel::onAutoRefreshTimer() {
  refreshTopics();
}

void TopicViewerPanel::setSearchFilter(const QString& text) {
  filterSettings_.searchText = text;
  onFilterChanged();
}

void TopicViewerPanel::setQuickFilter(int filterIndex) {
  QString filterType = quickFilterCombo_->itemData(filterIndex).toString();

  // Reset filters
  filterSettings_.showActive = true;
  filterSettings_.showInactive = true;
  filterSettings_.showWithPubs = true;
  filterSettings_.showWithSubs = true;
  filterSettings_.typeFilter.clear();

  if (filterType == "active") {
    filterSettings_.showInactive = false;
  } else if (filterType == "pubs") {
    filterSettings_.showWithSubs = false;
  } else if (filterType == "subs") {
    filterSettings_.showWithPubs = false;
  } else if (filterType == "sensor") {
    filterSettings_.typeFilter = "sensor_msgs";
  } else if (filterType == "geometry") {
    filterSettings_.typeFilter = "geometry_msgs";
  } else if (filterType == "nav") {
    filterSettings_.typeFilter = "nav_msgs";
  }

  onFilterChanged();
}

void TopicViewerPanel::onFilterChanged() {
  // Apply search filter to proxy model
  QString pattern = filterSettings_.searchText;
  if (!filterSettings_.typeFilter.isEmpty()) {
    // Combine search with type filter
    // For now, just use search text
  }
  proxyModel_->setFilterRegularExpression(
    QRegularExpression(pattern, QRegularExpression::CaseInsensitiveOption));
}

void TopicViewerPanel::clearFilters() {
  searchEdit_->clear();
  quickFilterCombo_->setCurrentIndex(0);
  filterSettings_ = TopicFilterSettings();
  onFilterChanged();
}

void TopicViewerPanel::onTopicSelected(const QModelIndex& index) {
  if (!index.isValid()) {
    selectedTopic_.clear();
    topicNameLabel_->setText(tr("Select a topic"));
    topicTypeLabel_->clear();
    topicStatsLabel_->clear();
    return;
  }

  QModelIndex sourceIndex = proxyModel_->mapToSource(index);
  TopicDisplayInfo topic = topicModel_->getTopicAt(sourceIndex.row());
  selectedTopic_ = topic.name;

  updateTopicDetails(topic.name);
  emit topicSelected(topic.name);
}

void TopicViewerPanel::updateTopicDetails(const QString& topicName) {
  TopicDisplayInfo topic = topicModel_->getTopicByName(topicName);
  if (topic.name.isEmpty()) return;

  topicNameLabel_->setText(topic.name);
  topicTypeLabel_->setText(topic.type);

  QString stats = tr("Publishers: %1 | Subscribers: %2").arg(topic.publisherCount).arg(topic.subscriberCount);
  if (topic.publishRate > 0) {
    stats += tr(" | Rate: %1 Hz").arg(topic.publishRate, 0, 'f', 1);
  }
  topicStatsLabel_->setText(stats);

  if (!topic.lastMessagePreview.isEmpty()) {
    messagePreviewEdit_->setPlainText(topic.lastMessagePreview);
  }
}

void TopicViewerPanel::onTopicDoubleClicked(const QModelIndex& index) {
  if (!index.isValid()) return;

  QModelIndex sourceIndex = proxyModel_->mapToSource(index);
  TopicDisplayInfo topic = topicModel_->getTopicAt(sourceIndex.row());

  // Toggle monitoring for this topic
  if (topicModel_->isTopicMonitored(topic.name)) {
    stopMonitoringTopic(topic.name);
  } else {
    monitorTopic(topic.name);
  }
}

void TopicViewerPanel::onContextMenu(const QPoint& pos) {
  QModelIndex index = topicTreeView_->indexAt(pos);
  if (!index.isValid()) return;

  QModelIndex sourceIndex = proxyModel_->mapToSource(index);
  TopicDisplayInfo topic = topicModel_->getTopicAt(sourceIndex.row());

  QMenu menu(this);

  // Monitor action
  bool isMonitored = topicModel_->isTopicMonitored(topic.name);
  QAction* monitorAction = menu.addAction(isMonitored ? tr("Stop Monitoring") : tr("Monitor Topic"));
  connect(monitorAction, &QAction::triggered, [this, topic, isMonitored]() {
    if (isMonitored) {
      stopMonitoringTopic(topic.name);
    } else {
      monitorTopic(topic.name);
    }
  });

  menu.addSeparator();

  // Echo to output
  QAction* echoAction = menu.addAction(tr("Echo to Output Panel"));
  connect(echoAction, &QAction::triggered, [this, topic]() {
    emit echoTopicRequested(topic.name);
  });

  // Show on canvas
  QAction* canvasAction = menu.addAction(tr("Show on Canvas"));
  connect(canvasAction, &QAction::triggered, [this, topic]() {
    emit showTopicOnCanvas(topic.name);
  });

  menu.addSeparator();

  // Copy actions
  QAction* copyNameAction = menu.addAction(tr("Copy Topic Name"));
  connect(copyNameAction, &QAction::triggered, [topic]() {
    QApplication::clipboard()->setText(topic.name);
  });

  QAction* copyTypeAction = menu.addAction(tr("Copy Message Type"));
  connect(copyTypeAction, &QAction::triggered, [topic]() {
    QApplication::clipboard()->setText(topic.type);
  });

  menu.addSeparator();

  // Show Data Origin action
  QAction* lineageAction = menu.addAction(tr("Show Data Origin..."));
  connect(lineageAction, &QAction::triggered, [this, topic]() {
    DataLineage lineage = globalLineageProvider().getRosTopicLineage(topic.name, topic.type);
    LineageContextMenu::showLineageDialog(lineage, this);
  });

  menu.exec(topicTreeView_->viewport()->mapToGlobal(pos));
}

void TopicViewerPanel::startMonitoring() {
  if (monitoring_.load()) return;
  monitoring_ = true;

  statusLabel_->setText(tr("Live monitoring enabled"));
}

void TopicViewerPanel::stopMonitoring() {
  monitoring_ = false;

  // Clear all subscriptions
  {
    std::lock_guard<std::mutex> lock(topicsMutex_);
    activeSubscriptions_.clear();
    messageCounters_.clear();
    lastMessageTimes_.clear();
  }

  // Update model
  for (const auto& topic : allTopics_) {
    topicModel_->setTopicMonitored(topic.name, false);
  }

  statusLabel_->setText(tr("Monitoring stopped"));
}

void TopicViewerPanel::toggleMonitoring() {
  if (monitoring_.load()) {
    stopMonitoring();
    monitorButton_->setChecked(false);
  } else {
    startMonitoring();
    monitorButton_->setChecked(true);
  }
}

void TopicViewerPanel::monitorTopic(const QString& topicName) {
  if (!rosNode_ || !monitoring_.load()) {
    // Enable monitoring if not already
    if (!monitoring_.load()) {
      startMonitoring();
      monitorButton_->setChecked(true);
    }
  }

  std::string topic = topicName.toStdString();

  // Check if already monitoring
  {
    std::lock_guard<std::mutex> lock(topicsMutex_);
    if (activeSubscriptions_.find(topic) != activeSubscriptions_.end()) {
      return;  // Already subscribed
    }
  }

  // Get topic type
  TopicDisplayInfo info = topicModel_->getTopicByName(topicName);
  if (info.type.isEmpty()) {
    return;
  }

  try {
    std::string msgType = info.type.toStdString();

    // Create generic subscription
    auto callback = [this, topicName](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      std::string topic = topicName.toStdString();

      qint64 now = QDateTime::currentMSecsSinceEpoch();
      double rate = 0.0;

      {
        std::lock_guard<std::mutex> lock(topicsMutex_);
        messageCounters_[topic]++;

        // Calculate rate
        auto it = lastMessageTimes_.find(topic);
        if (it != lastMessageTimes_.end() && it->second > 0) {
          double dt = (now - it->second) / 1000.0;
          if (dt > 0) {
            rate = 1.0 / dt;
          }
        }
        lastMessageTimes_[topic] = now;
      }

      // Create simple preview (size info for now)
      QString preview = tr("Message received (%1 bytes)\nTimestamp: %2")
        .arg(msg->size())
        .arg(QDateTime::fromMSecsSinceEpoch(now).toString("hh:mm:ss.zzz"));

      emit messageReceived(topicName, preview, rate);
    };

    auto sub = rosNode_->create_generic_subscription(
      topic, msgType, rclcpp::QoS(10), callback);

    {
      std::lock_guard<std::mutex> lock(topicsMutex_);
      activeSubscriptions_[topic] = sub;
      messageCounters_[topic] = 0;
      lastMessageTimes_[topic] = 0;
    }

    topicModel_->setTopicMonitored(topicName, true);
    statusLabel_->setText(tr("Monitoring: %1").arg(topicName));

  } catch (const std::exception& e) {
    statusLabel_->setText(tr("Failed to subscribe: %1").arg(e.what()));
  }
}

void TopicViewerPanel::stopMonitoringTopic(const QString& topicName) {
  std::string topic = topicName.toStdString();

  {
    std::lock_guard<std::mutex> lock(topicsMutex_);
    activeSubscriptions_.erase(topic);
    messageCounters_.erase(topic);
    lastMessageTimes_.erase(topic);
  }

  topicModel_->setTopicMonitored(topicName, false);
  topicModel_->updateTopicRate(topicName, 0.0);
}

void TopicViewerPanel::onMessageReceived(const QString& topicName, const QString& preview, double rate) {
  topicModel_->updateTopicRate(topicName, rate);
  topicModel_->updateTopicMessage(topicName, preview, QDateTime::currentMSecsSinceEpoch());

  // Update details panel if this topic is selected
  if (topicName == selectedTopic_) {
    messagePreviewEdit_->setPlainText(preview);
    updateTopicDetails(topicName);
  }
}

QList<TopicDisplayInfo> TopicViewerPanel::getFilteredTopics() const {
  std::lock_guard<std::mutex> lock(topicsMutex_);

  QList<TopicDisplayInfo> filtered;
  for (const auto& topic : allTopics_) {
    // Apply filters
    if (!filterSettings_.showActive && topic.publishRate > 0) continue;
    if (!filterSettings_.showInactive && topic.publishRate == 0) continue;
    if (!filterSettings_.showWithPubs && topic.publisherCount == 0) continue;
    if (!filterSettings_.showWithSubs && topic.subscriberCount == 0) continue;

    if (!filterSettings_.typeFilter.isEmpty() &&
        !topic.type.contains(filterSettings_.typeFilter, Qt::CaseInsensitive)) {
      continue;
    }

    if (!filterSettings_.searchText.isEmpty() &&
        !topic.name.contains(filterSettings_.searchText, Qt::CaseInsensitive)) {
      continue;
    }

    filtered.append(topic);
  }

  return filtered;
}

void TopicViewerPanel::checkTopicInactivity() {
  if (!monitoring_.load()) return;

  qint64 now = QDateTime::currentMSecsSinceEpoch();
  QStringList inactiveTopics;

  {
    std::lock_guard<std::mutex> lock(topicsMutex_);
    for (const auto& [topic, lastTime] : lastMessageTimes_) {
      if (lastTime > 0 && (now - lastTime) > INACTIVITY_TIMEOUT_MS) {
        inactiveTopics.append(QString::fromStdString(topic));
      }
    }
  }

  // Emit rate=0 for inactive topics
  for (const QString& topicName : inactiveTopics) {
    emit messageReceived(topicName, QString(), 0.0);
    topicModel_->updateTopicRate(topicName, 0.0);
  }

  // Reset the last message times to prevent repeated emissions (single lock for efficiency)
  if (!inactiveTopics.isEmpty()) {
    std::lock_guard<std::mutex> lock(topicsMutex_);
    for (const QString& topicName : inactiveTopics) {
      lastMessageTimes_[topicName.toStdString()] = 0;
    }
  }
}

void TopicViewerPanel::autoMonitorCanvasTopics() {
  if (!canvas_) {
    statusLabel_->setText(tr("No canvas connected"));
    return;
  }

  // Get all topic names from canvas connections
  QSet<QString> canvasTopics;
  for (ConnectionLine* conn : canvas_->connections()) {
    QString topicName = conn->topicName();
    if (!topicName.isEmpty()) {
      canvasTopics.insert(topicName);
    }
  }

  if (canvasTopics.isEmpty()) {
    statusLabel_->setText(tr("No topics found on canvas"));
    return;
  }

  // Ensure monitoring is started
  if (!monitoring_.load()) {
    startMonitoring();
    monitorButton_->setChecked(true);
  }

  // Refresh topics if needed
  if (allTopics_.isEmpty()) {
    refreshTopics();
    // Give discovery time to complete
    QTimer::singleShot(1000, this, [this, canvasTopics]() {
      doAutoMonitorMatching(canvasTopics);
    });
  } else {
    doAutoMonitorMatching(canvasTopics);
  }
}

void TopicViewerPanel::doAutoMonitorMatching(const QSet<QString>& canvasTopics) {
  int matchCount = 0;

  // For each canvas topic, find matching ROS topics and subscribe
  for (const QString& canvasTopic : canvasTopics) {
    // Try exact match first
    bool found = false;
    for (const TopicDisplayInfo& topic : allTopics_) {
      if (topic.name == canvasTopic) {
        monitorTopic(topic.name);
        matchCount++;
        found = true;
        break;
      }
    }

    // If no exact match, try suffix match (e.g., "/scan" matches "/tb3/scan")
    if (!found) {
      QString suffix = canvasTopic;
      if (!suffix.startsWith('/')) suffix = "/" + suffix;

      for (const TopicDisplayInfo& topic : allTopics_) {
        if (topic.name.endsWith(suffix)) {
          monitorTopic(topic.name);
          matchCount++;
          break;
        }
      }
    }
  }

  statusLabel_->setText(tr("Monitoring %1 canvas topics").arg(matchCount));
}

}  // namespace ros_weaver
