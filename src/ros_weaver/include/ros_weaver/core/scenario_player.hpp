#ifndef ROS_WEAVER_CORE_SCENARIO_PLAYER_HPP
#define ROS_WEAVER_CORE_SCENARIO_PLAYER_HPP

#include <QObject>
#include <QString>
#include <QJsonObject>
#include <QJsonArray>
#include <QList>
#include <QTimer>
#include <QMutex>
#include <QElapsedTimer>

#include <rclcpp/rclcpp.hpp>

namespace ros_weaver {

class GenericMessageHandler;

/**
 * @brief A single step in a test scenario
 */
struct ScenarioStep {
  enum Type {
    PublishMessage,    // Publish a message to a topic
    WaitForMessage,    // Wait for a message on a topic matching a condition
    WaitTime,          // Wait for a specified duration
    Conditional        // Conditional branch based on expression
  };

  Type type = PublishMessage;
  QString topicName;
  QString messageType;
  QJsonObject messageData;
  int waitTimeMs = 1000;
  QString conditionExpression;  // e.g., "msg.status == 1"
  int timeoutMs = 5000;
  QString description;          // User-friendly description
  bool enabled = true;          // Can be disabled without removing

  // Serialize to/from JSON
  QJsonObject toJson() const;
  static ScenarioStep fromJson(const QJsonObject& json);
};

/**
 * @brief A complete test scenario containing multiple steps
 */
struct TestScenario {
  QString name;
  QString description;
  QList<ScenarioStep> steps;
  bool loopEnabled = false;
  int loopCount = 1;
  QString author;
  QString createdDate;
  QString modifiedDate;

  // Serialize to/from JSON
  QJsonObject toJson() const;
  static TestScenario fromJson(const QJsonObject& json);
};

/**
 * @brief Result of executing a scenario step
 */
struct StepResult {
  int stepIndex = 0;
  bool success = false;
  QString errorMessage;
  qint64 executionTimeMs = 0;
  QJsonObject receivedMessage;  // For WaitForMessage steps
};

/**
 * @brief Scenario execution state
 */
enum class ScenarioState {
  Idle,
  Running,
  Paused,
  Completed,
  Failed,
  Cancelled
};

/**
 * @brief Executes test message scenarios
 *
 * Features:
 * - Play/pause/stop scenario execution
 * - Publish custom messages
 * - Wait for messages with conditions
 * - Time-based delays
 * - Loop support
 * - Record scenarios from live system
 */
class ScenarioPlayer : public QObject {
  Q_OBJECT

public:
  explicit ScenarioPlayer(QObject* parent = nullptr);
  ~ScenarioPlayer() override;

  /**
   * @brief Set the message handler for publishing/subscribing
   */
  void setMessageHandler(GenericMessageHandler* handler);

  /**
   * @brief Set the ROS node
   */
  void setRosNode(rclcpp::Node::SharedPtr node);

  // Scenario management
  void loadScenario(const TestScenario& scenario);
  TestScenario currentScenario() const { return scenario_; }
  void clearScenario();

  // Playback control
  void play();
  void pause();
  void resume();
  void stop();
  void stepForward();  // Execute single step

  // State
  ScenarioState state() const { return state_; }
  int currentStepIndex() const { return currentStepIndex_; }
  int totalSteps() const { return scenario_.steps.size(); }
  double progress() const;  // 0.0 - 1.0

  // Step execution
  bool executeStep(int index);
  bool executeStep(const ScenarioStep& step);

  // Recording
  void startRecording(const QStringList& topicsToRecord);
  void stopRecording();
  bool isRecording() const { return isRecording_; }
  TestScenario getRecordedScenario() const;

  // File I/O
  bool saveScenario(const QString& filePath);
  bool loadScenarioFromFile(const QString& filePath);

  // Export to Python pytest script
  QString exportToPytest() const;

signals:
  void stateChanged(ScenarioState newState);
  void stepStarted(int stepIndex, const ScenarioStep& step);
  void stepCompleted(int stepIndex, const StepResult& result);
  void stepFailed(int stepIndex, const QString& error);
  void scenarioCompleted(bool success);
  void progressUpdated(double progress);
  void loopCompleted(int loopNumber);

  // Recording signals
  void recordingStarted();
  void recordingStopped();
  void messageRecorded(const QString& topicName, const QJsonObject& message);

private slots:
  void onExecutionTimer();
  void onWaitTimeout();
  void onMessageReceived(const QString& topicName, const QJsonObject& message, qint64 timestamp);

private:
  void setState(ScenarioState newState);
  void advanceToNextStep();
  void completeScenario(bool success);

  // Step execution helpers
  bool executePublishStep(const ScenarioStep& step);
  bool executeWaitForMessageStep(const ScenarioStep& step);
  bool executeWaitTimeStep(const ScenarioStep& step);
  bool executeConditionalStep(const ScenarioStep& step);

  // Condition evaluation
  bool evaluateCondition(const QString& expression, const QJsonObject& message);

  GenericMessageHandler* messageHandler_;
  rclcpp::Node::SharedPtr node_;

  TestScenario scenario_;
  ScenarioState state_;
  int currentStepIndex_;
  int currentLoopNumber_;

  QTimer* executionTimer_;
  QTimer* waitTimer_;
  QElapsedTimer stepElapsedTimer_;

  // Wait for message state
  QString waitingForTopic_;
  QString waitingCondition_;
  bool messageReceived_;
  QJsonObject lastReceivedMessage_;

  // Recording state
  bool isRecording_;
  TestScenario recordedScenario_;
  QStringList recordingTopics_;
  QElapsedTimer recordingTimer_;

  mutable QMutex mutex_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_SCENARIO_PLAYER_HPP
