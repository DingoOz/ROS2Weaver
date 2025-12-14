#ifndef ROS_WEAVER_TOPIC_VIEWER_PANEL_HPP
#define ROS_WEAVER_TOPIC_VIEWER_PANEL_HPP

#include <QWidget>
#include <QTreeView>
#include <QLineEdit>
#include <QComboBox>
#include <QPushButton>
#include <QToolButton>
#include <QLabel>
#include <QSplitter>
#include <QTextEdit>
#include <QTimer>
#include <QSortFilterProxyModel>
#include <QSet>

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

namespace ros_weaver {

class TopicListModel;
class WeaverCanvas;

// Information about a topic for display
struct TopicDisplayInfo {
  QString name;
  QString type;
  QString shortType;  // Just the message name without package
  int publisherCount = 0;
  int subscriberCount = 0;
  double publishRate = 0.0;  // Hz, 0 if unknown
  bool isMonitored = false;
  qint64 lastMessageTime = 0;
  QString lastMessagePreview;
};

// Filter settings
struct TopicFilterSettings {
  QString searchText;
  bool showActive = true;      // Topics with Hz > 0
  bool showInactive = true;    // Topics with Hz == 0
  bool showWithPubs = true;    // Topics with publishers
  bool showWithSubs = true;    // Topics with subscribers
  QString typeFilter;          // Message type filter (empty = all)
  QString namespaceFilter;     // Namespace filter (empty = all)
};

class TopicViewerPanel : public QWidget {
  Q_OBJECT

public:
  explicit TopicViewerPanel(QWidget* parent = nullptr);
  ~TopicViewerPanel() override;

  // Set canvas for integration
  void setCanvas(WeaverCanvas* canvas) { canvas_ = canvas; }

  // Check if panel is actively monitoring
  bool isMonitoring() const { return monitoring_.load(); }

public slots:
  // Refresh topic list
  void refreshTopics();

  // Start/stop monitoring
  void startMonitoring();
  void stopMonitoring();
  void toggleMonitoring();

  // Filter control
  void setSearchFilter(const QString& text);
  void setQuickFilter(int filterIndex);
  void clearFilters();

  // Topic selection
  void onTopicSelected(const QModelIndex& index);
  void onTopicDoubleClicked(const QModelIndex& index);

  // Monitor specific topic
  void monitorTopic(const QString& topicName);
  void stopMonitoringTopic(const QString& topicName);

  // Auto-monitor topics that match canvas connections
  void autoMonitorCanvasTopics();

signals:
  // Emitted when a topic is selected (for canvas highlighting)
  void topicSelected(const QString& topicName);

  // Emitted when user wants to show topic on canvas
  void showTopicOnCanvas(const QString& topicName);

  // Emitted when user wants to echo topic
  void echoTopicRequested(const QString& topicName);

  // Internal signal for thread-safe UI updates
  void topicsDiscovered(const QList<TopicDisplayInfo>& topics);
  void messageReceived(const QString& topicName, const QString& preview, double rate);

private slots:
  void onTopicsDiscovered(const QList<TopicDisplayInfo>& topics);
  void onMessageReceived(const QString& topicName, const QString& preview, double rate);
  void onAutoRefreshTimer();
  void onContextMenu(const QPoint& pos);
  void onFilterChanged();

private:
  void setupUi();
  QWidget* createToolbarWidget();
  void setupToolbar();
  void setupTopicList();
  void setupDetailsPanel();
  void setupConnections();

  void initializeRosNode();
  void shutdownRosNode();

  void performTopicDiscovery();
  void updateTopicDetails(const QString& topicName);
  void checkTopicInactivity();

  // Get topics matching current filter
  QList<TopicDisplayInfo> getFilteredTopics() const;

  // Helper for auto-monitor
  void doAutoMonitorMatching(const QSet<QString>& canvasTopics);

  // UI Components
  QSplitter* splitter_;

  // Toolbar
  QLineEdit* searchEdit_;
  QComboBox* quickFilterCombo_;
  QPushButton* refreshButton_;
  QToolButton* autoRefreshButton_;
  QToolButton* monitorButton_;
  QPushButton* autoMonitorButton_;
  QLabel* statusLabel_;

  // Topic list
  QTreeView* topicTreeView_;
  TopicListModel* topicModel_;
  QSortFilterProxyModel* proxyModel_;

  // Details panel
  QWidget* detailsPanel_;
  QLabel* topicNameLabel_;
  QLabel* topicTypeLabel_;
  QLabel* topicStatsLabel_;
  QTextEdit* messagePreviewEdit_;

  // Canvas integration
  WeaverCanvas* canvas_;

  // ROS2 components
  std::shared_ptr<rclcpp::Node> rosNode_;
  std::unique_ptr<std::thread> spinThread_;
  std::unique_ptr<std::thread> discoveryThread_;  // Track discovery thread to avoid use-after-free
  std::atomic<bool> spinning_;
  std::atomic<bool> monitoring_;

  // Topic data
  QList<TopicDisplayInfo> allTopics_;
  mutable std::mutex topicsMutex_;
  QString selectedTopic_;

  // Active subscriptions for monitoring
  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> activeSubscriptions_;
  std::map<std::string, int> messageCounters_;
  std::map<std::string, qint64> lastMessageTimes_;

  // Filter settings
  TopicFilterSettings filterSettings_;

  // Timers
  QTimer* autoRefreshTimer_;
  bool autoRefreshEnabled_;
  int autoRefreshInterval_;  // seconds

  // Inactivity detection
  QTimer* inactivityTimer_;
  static constexpr int INACTIVITY_CHECK_INTERVAL_MS = 250;
  static constexpr qint64 INACTIVITY_TIMEOUT_MS = 500;
};

}  // namespace ros_weaver

Q_DECLARE_METATYPE(ros_weaver::TopicDisplayInfo)
Q_DECLARE_METATYPE(QList<ros_weaver::TopicDisplayInfo>)

#endif  // ROS_WEAVER_TOPIC_VIEWER_PANEL_HPP
