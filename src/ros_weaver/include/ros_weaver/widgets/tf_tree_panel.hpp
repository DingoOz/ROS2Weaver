#ifndef ROS_WEAVER_TF_TREE_PANEL_HPP
#define ROS_WEAVER_TF_TREE_PANEL_HPP

#include <QWidget>
#include <QTreeWidget>
#include <QLineEdit>
#include <QComboBox>
#include <QPushButton>
#include <QToolButton>
#include <QLabel>
#include <QSplitter>
#include <QTextEdit>
#include <QTimer>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QUuid>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <map>
#include <set>

namespace ros_weaver {

class WeaverCanvas;
class PackageBlock;

// Frame status for visualization
enum class FrameStatus {
  Healthy,    // Recent updates, connected to tree
  Stale,      // No update in 1-5 seconds
  Critical,   // Very stale (>5s) or critical issue
  Static,     // Published once on /tf_static
  Orphan      // No parent frame connection
};

// Information about a single TF frame
struct TFFrameInfo {
  QString name;
  QString parentName;
  QStringList childNames;

  // Transform data
  double translationX = 0.0;
  double translationY = 0.0;
  double translationZ = 0.0;
  double rotationX = 0.0;
  double rotationY = 0.0;
  double rotationZ = 0.0;
  double rotationW = 1.0;

  // Status
  FrameStatus status = FrameStatus::Healthy;
  bool isStatic = false;
  qint64 lastUpdateTime = 0;
  double updateRateHz = 0.0;
  QString publisherNode;

  // Depth in tree (0 = root)
  int depth = 0;
};

// Link to a canvas block
struct CanvasFrameLink {
  QString blockName;
  QString parameterName;
  QString parameterValue;
  PackageBlock* block = nullptr;
  QUuid blockId;  // Stored for safe lookup (avoids dangling pointer)
};

// Link to a YAML parameter
struct YamlFrameLink {
  QString filePath;
  QString parameterPath;
  int lineNumber = 0;
};

// Link to a topic
struct TopicFrameLink {
  QString topicName;
  QString fieldName;  // e.g., "header.frame_id"
  double rateHz = 0.0;
};

// All links for a frame
struct FrameLinks {
  QList<CanvasFrameLink> canvasLinks;
  QList<YamlFrameLink> yamlLinks;
  QList<TopicFrameLink> topicLinks;
};

class TFTreePanel : public QWidget {
  Q_OBJECT

public:
  explicit TFTreePanel(QWidget* parent = nullptr);
  ~TFTreePanel() override;

  // Set canvas for link discovery
  void setCanvas(WeaverCanvas* canvas) { canvas_ = canvas; }

  // Check if panel is actively listening
  bool isListening() const { return listening_.load(); }

public slots:
  // Start/stop TF listening
  void startListening();
  void stopListening();
  void toggleListening();

  // Refresh the tree
  void refreshTree();

  // Search/filter
  void setSearchFilter(const QString& text);

  // Frame selection
  void onFrameSelected(QTreeWidgetItem* item, int column);
  void onFrameDoubleClicked(QTreeWidgetItem* item, int column);

signals:
  // Emitted when a frame is selected
  void frameSelected(const QString& frameName);

  // Emitted when user wants to show frame on canvas
  void showFrameOnCanvas(const QString& frameName);

  // Emitted when user wants to open YAML at specific line
  void openYamlAtLine(const QString& filePath, int lineNumber);

  // Internal signals for thread-safe UI updates
  void tfDataReceived();

private slots:
  void onTfDataReceived();
  void onUpdateTimer();
  void onContextMenu(const QPoint& pos);
  void onLinkClicked();

private:
  void setupUi();
  QWidget* createToolbar();
  void setupTreeWidget();
  void setupDetailsPanel();
  void setupConnections();

  void initializeRosNode();
  void shutdownRosNode();

  // TF processing
  void processTfMessage(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool isStatic);
  void rebuildTree();
  void updateFrameItem(QTreeWidgetItem* item, const TFFrameInfo& frame);
  QTreeWidgetItem* findOrCreateFrameItem(const QString& frameName);

  // Link discovery
  FrameLinks discoverFrameLinks(const QString& frameName);
  void updateDetailsPanel(const QString& frameName);

  // Helpers
  QColor getStatusColor(FrameStatus status) const;
  QString getStatusText(FrameStatus status) const;
  QString formatTransform(const TFFrameInfo& frame) const;
  QString formatRPY(double x, double y, double z, double w) const;
  void checkFrameHealth();

  // UI Components
  QSplitter* splitter_;

  // Toolbar
  QLineEdit* searchEdit_;
  QComboBox* viewModeCombo_;
  QPushButton* refreshButton_;
  QToolButton* liveButton_;
  QLabel* statusLabel_;

  // Tree view
  QTreeWidget* treeWidget_;
  QMap<QString, QTreeWidgetItem*> frameItems_;

  // Details panel
  QWidget* detailsPanel_;
  QLabel* frameNameLabel_;
  QLabel* parentLabel_;
  QLabel* childrenLabel_;
  QLabel* transformLabel_;
  QLabel* statusInfoLabel_;
  QLabel* rateLabel_;
  QGroupBox* linksGroupBox_;
  QVBoxLayout* linksLayout_;

  // Canvas integration
  WeaverCanvas* canvas_;

  // ROS2 components
  std::shared_ptr<rclcpp::Node> rosNode_;
  std::unique_ptr<std::thread> spinThread_;
  std::atomic<bool> spinning_;
  std::atomic<bool> listening_;

  // TF2 components
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tfSubscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tfStaticSubscription_;

  // Frame data
  QMap<QString, TFFrameInfo> frames_;
  QSet<QString> rootFrames_;  // Frames with no parent
  mutable std::mutex framesMutex_;
  QString selectedFrame_;

  // Timers
  QTimer* updateTimer_;
  bool liveUpdateEnabled_;

  // Update tracking
  QMap<QString, int> frameUpdateCounts_;
  qint64 lastRateCalcTime_;

  // Configuration
  static constexpr int UPDATE_INTERVAL_MS = 100;
  static constexpr double STALE_THRESHOLD_SEC = 1.0;
  static constexpr double CRITICAL_STALE_SEC = 5.0;
};

}  // namespace ros_weaver

Q_DECLARE_METATYPE(ros_weaver::TFFrameInfo)

#endif  // ROS_WEAVER_TF_TREE_PANEL_HPP
