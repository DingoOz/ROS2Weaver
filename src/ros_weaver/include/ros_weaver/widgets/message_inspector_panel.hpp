#ifndef ROS_WEAVER_WIDGETS_MESSAGE_INSPECTOR_PANEL_HPP
#define ROS_WEAVER_WIDGETS_MESSAGE_INSPECTOR_PANEL_HPP

#include <QWidget>
#include <QComboBox>
#include <QPushButton>
#include <QToolButton>
#include <QTreeWidget>
#include <QListWidget>
#include <QTextEdit>
#include <QSpinBox>
#include <QLabel>
#include <QGroupBox>
#include <QSplitter>
#include <QTimer>
#include <QJsonObject>
#include <QList>
#include <QPair>

#include <rclcpp/rclcpp.hpp>

namespace ros_weaver {

class GenericMessageHandler;
class WeaverCanvas;

/**
 * @brief Panel for inspecting live ROS2 messages and publishing test messages
 *
 * Features:
 * - Subscribe to any topic and view live messages in a tree view
 * - Message history buffer with configurable size
 * - Pause/resume message reception
 * - Publish custom messages via JSON editor
 * - Load message schema templates
 * - Message rate display
 */
class MessageInspectorPanel : public QWidget {
  Q_OBJECT

public:
  explicit MessageInspectorPanel(QWidget* parent = nullptr);
  ~MessageInspectorPanel() override;

  /**
   * @brief Set the canvas reference for topic selection
   */
  void setCanvas(WeaverCanvas* canvas);

  /**
   * @brief Get the message handler
   */
  GenericMessageHandler* messageHandler() const { return messageHandler_; }

public slots:
  /**
   * @brief Start inspecting a specific topic
   * @param topicName The topic to inspect
   * @param messageType The message type
   */
  void inspectTopic(const QString& topicName, const QString& messageType);

  /**
   * @brief Stop inspecting current topic
   */
  void stopInspecting();

  /**
   * @brief Refresh the topic list
   */
  void refreshTopics();

signals:
  /**
   * @brief Emitted when user wants to highlight a topic on canvas
   */
  void highlightTopicRequested(const QString& topicName);

private slots:
  void onTopicSelected(int index);
  void onRefreshClicked();
  void onPauseToggled(bool paused);
  void onClearClicked();
  void onMessageReceived(const QString& topicName, const QJsonObject& message,
                         qint64 timestamp);
  void onHistoryItemClicked(QListWidgetItem* item);
  void onLoadSchemaClicked();
  void onPublishClicked();
  void onCopyMessageClicked();
  void onBufferSizeChanged(int size);
  void updateRateDisplay();

private:
  void setupUi();
  void setupConnections();
  void initializeRosNode();
  void shutdownRosNode();

  /**
   * @brief Populate message tree from JSON object
   */
  void populateTreeFromJson(QTreeWidgetItem* parent, const QJsonObject& obj);

  /**
   * @brief Populate message tree from JSON array
   */
  void populateTreeFromJsonArray(QTreeWidgetItem* parent, const QJsonArray& arr);

  /**
   * @brief Get display string for JSON type
   */
  QString getJsonTypeName(const QJsonValue& value);

  /**
   * @brief Format timestamp for display
   */
  QString formatTimestamp(qint64 timestamp);

  /**
   * @brief Update UI based on inspection state
   */
  void updateUiState();

  /**
   * @brief Add message to history buffer
   */
  void addToHistory(const QJsonObject& message, qint64 timestamp);

  // UI Components - Topic Selection
  QComboBox* topicSelector_;
  QPushButton* refreshTopicsButton_;
  QToolButton* inspectButton_;

  // UI Components - Message View
  QTreeWidget* messageTreeView_;
  QLabel* currentTopicLabel_;
  QLabel* messageTypeLabel_;
  QLabel* rateLabel_;
  QLabel* lastReceivedLabel_;

  // UI Components - Controls
  QToolButton* pauseButton_;
  QPushButton* clearButton_;
  QPushButton* copyButton_;
  QSpinBox* bufferSizeSpinBox_;

  // UI Components - History
  QListWidget* messageHistoryList_;

  // UI Components - Publisher
  QGroupBox* publisherGroup_;
  QTextEdit* jsonEditor_;
  QPushButton* loadSchemaButton_;
  QPushButton* publishButton_;

  // Splitter for layout
  QSplitter* mainSplitter_;
  QSplitter* viewSplitter_;

  // Core components
  GenericMessageHandler* messageHandler_;
  WeaverCanvas* canvas_;
  rclcpp::Node::SharedPtr rosNode_;
  std::thread spinThread_;
  std::atomic<bool> spinning_;

  // State
  QString currentTopic_;
  QString currentMessageType_;
  bool isPaused_;
  int maxBufferSize_;

  // Message buffer: (timestamp, message)
  QList<QPair<qint64, QJsonObject>> messageBuffer_;

  // Rate calculation
  QTimer* rateTimer_;
  int messageCountSinceLastUpdate_;
  double currentRate_;

  // Topic types cache: topicName -> messageType
  QMap<QString, QString> topicTypes_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_MESSAGE_INSPECTOR_PANEL_HPP
