#include "ros_weaver/core/scenario_player.hpp"
#include "ros_weaver/core/generic_message_handler.hpp"

#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QDateTime>
#include <QRegularExpression>

namespace ros_weaver {

// ScenarioStep JSON serialization

QJsonObject ScenarioStep::toJson() const {
  QJsonObject json;

  QString typeStr;
  switch (type) {
    case PublishMessage: typeStr = "publish"; break;
    case WaitForMessage: typeStr = "wait_for_message"; break;
    case WaitTime: typeStr = "wait_time"; break;
    case Conditional: typeStr = "conditional"; break;
  }
  json["type"] = typeStr;
  json["topic"] = topicName;
  json["message_type"] = messageType;
  json["message_data"] = messageData;
  json["wait_time_ms"] = waitTimeMs;
  json["condition"] = conditionExpression;
  json["timeout_ms"] = timeoutMs;
  json["description"] = description;
  json["enabled"] = enabled;

  return json;
}

ScenarioStep ScenarioStep::fromJson(const QJsonObject& json) {
  ScenarioStep step;

  QString typeStr = json["type"].toString();
  if (typeStr == "publish") step.type = PublishMessage;
  else if (typeStr == "wait_for_message") step.type = WaitForMessage;
  else if (typeStr == "wait_time") step.type = WaitTime;
  else if (typeStr == "conditional") step.type = Conditional;

  step.topicName = json["topic"].toString();
  step.messageType = json["message_type"].toString();
  step.messageData = json["message_data"].toObject();
  step.waitTimeMs = json["wait_time_ms"].toInt(1000);
  step.conditionExpression = json["condition"].toString();
  step.timeoutMs = json["timeout_ms"].toInt(5000);
  step.description = json["description"].toString();
  step.enabled = json["enabled"].toBool(true);

  return step;
}

// TestScenario JSON serialization

QJsonObject TestScenario::toJson() const {
  QJsonObject json;

  json["name"] = name;
  json["description"] = description;
  json["loop_enabled"] = loopEnabled;
  json["loop_count"] = loopCount;
  json["author"] = author;
  json["created_date"] = createdDate;
  json["modified_date"] = modifiedDate;

  QJsonArray stepsArray;
  for (const auto& step : steps) {
    stepsArray.append(step.toJson());
  }
  json["steps"] = stepsArray;

  return json;
}

TestScenario TestScenario::fromJson(const QJsonObject& json) {
  TestScenario scenario;

  scenario.name = json["name"].toString();
  scenario.description = json["description"].toString();
  scenario.loopEnabled = json["loop_enabled"].toBool(false);
  scenario.loopCount = json["loop_count"].toInt(1);
  scenario.author = json["author"].toString();
  scenario.createdDate = json["created_date"].toString();
  scenario.modifiedDate = json["modified_date"].toString();

  QJsonArray stepsArray = json["steps"].toArray();
  for (const auto& stepVal : stepsArray) {
    scenario.steps.append(ScenarioStep::fromJson(stepVal.toObject()));
  }

  return scenario;
}

// ScenarioPlayer implementation

ScenarioPlayer::ScenarioPlayer(QObject* parent)
    : QObject(parent)
    , messageHandler_(nullptr)
    , state_(ScenarioState::Idle)
    , currentStepIndex_(0)
    , currentLoopNumber_(0)
    , executionTimer_(new QTimer(this))
    , waitTimer_(new QTimer(this))
    , messageReceived_(false)
    , isRecording_(false) {

  connect(executionTimer_, &QTimer::timeout,
          this, &ScenarioPlayer::onExecutionTimer);
  connect(waitTimer_, &QTimer::timeout,
          this, &ScenarioPlayer::onWaitTimeout);

  executionTimer_->setSingleShot(true);
  waitTimer_->setSingleShot(true);
}

ScenarioPlayer::~ScenarioPlayer() {
  stop();
}

void ScenarioPlayer::setMessageHandler(GenericMessageHandler* handler) {
  if (messageHandler_) {
    disconnect(messageHandler_, &GenericMessageHandler::messageReceived,
               this, &ScenarioPlayer::onMessageReceived);
  }

  messageHandler_ = handler;

  if (messageHandler_) {
    connect(messageHandler_, &GenericMessageHandler::messageReceived,
            this, &ScenarioPlayer::onMessageReceived);
  }
}

void ScenarioPlayer::setRosNode(rclcpp::Node::SharedPtr node) {
  node_ = node;
}

void ScenarioPlayer::loadScenario(const TestScenario& scenario) {
  QMutexLocker locker(&mutex_);

  if (state_ == ScenarioState::Running) {
    stop();
  }

  scenario_ = scenario;
  currentStepIndex_ = 0;
  currentLoopNumber_ = 0;
  setState(ScenarioState::Idle);
}

void ScenarioPlayer::clearScenario() {
  QMutexLocker locker(&mutex_);

  stop();
  scenario_ = TestScenario();
  currentStepIndex_ = 0;
  currentLoopNumber_ = 0;
}

void ScenarioPlayer::play() {
  QMutexLocker locker(&mutex_);

  if (scenario_.steps.isEmpty()) {
    return;
  }

  if (state_ == ScenarioState::Paused) {
    resume();
    return;
  }

  currentStepIndex_ = 0;
  currentLoopNumber_ = 0;
  setState(ScenarioState::Running);

  locker.unlock();

  // Start executing first step
  executionTimer_->start(0);
}

void ScenarioPlayer::pause() {
  QMutexLocker locker(&mutex_);

  if (state_ == ScenarioState::Running) {
    executionTimer_->stop();
    waitTimer_->stop();
    setState(ScenarioState::Paused);
  }
}

void ScenarioPlayer::resume() {
  QMutexLocker locker(&mutex_);

  if (state_ == ScenarioState::Paused) {
    setState(ScenarioState::Running);
    locker.unlock();
    executionTimer_->start(0);
  }
}

void ScenarioPlayer::stop() {
  QMutexLocker locker(&mutex_);

  executionTimer_->stop();
  waitTimer_->stop();
  waitingForTopic_.clear();
  messageReceived_ = false;
  currentStepIndex_ = 0;
  currentLoopNumber_ = 0;
  setState(ScenarioState::Cancelled);
}

void ScenarioPlayer::stepForward() {
  QMutexLocker locker(&mutex_);

  if (scenario_.steps.isEmpty()) {
    return;
  }

  if (state_ != ScenarioState::Idle && state_ != ScenarioState::Paused) {
    return;
  }

  if (currentStepIndex_ >= scenario_.steps.size()) {
    return;
  }

  locker.unlock();

  executeStep(currentStepIndex_);

  locker.relock();
  currentStepIndex_++;

  if (currentStepIndex_ >= scenario_.steps.size()) {
    setState(ScenarioState::Completed);
    locker.unlock();
    emit scenarioCompleted(true);
  }
}

double ScenarioPlayer::progress() const {
  QMutexLocker locker(&mutex_);

  if (scenario_.steps.isEmpty()) {
    return 0.0;
  }

  double loopProgress = static_cast<double>(currentLoopNumber_) / scenario_.loopCount;
  double stepProgress = static_cast<double>(currentStepIndex_) / scenario_.steps.size();

  if (scenario_.loopEnabled && scenario_.loopCount > 1) {
    return (loopProgress + stepProgress / scenario_.loopCount);
  }

  return stepProgress;
}

bool ScenarioPlayer::executeStep(int index) {
  QMutexLocker locker(&mutex_);

  if (index < 0 || index >= scenario_.steps.size()) {
    return false;
  }

  const ScenarioStep& step = scenario_.steps[index];
  locker.unlock();

  return executeStep(step);
}

bool ScenarioPlayer::executeStep(const ScenarioStep& step) {
  if (!step.enabled) {
    return true;  // Skip disabled steps
  }

  stepElapsedTimer_.start();

  emit stepStarted(currentStepIndex_, step);

  bool success = false;

  switch (step.type) {
    case ScenarioStep::PublishMessage:
      success = executePublishStep(step);
      break;
    case ScenarioStep::WaitForMessage:
      success = executeWaitForMessageStep(step);
      break;
    case ScenarioStep::WaitTime:
      success = executeWaitTimeStep(step);
      break;
    case ScenarioStep::Conditional:
      success = executeConditionalStep(step);
      break;
  }

  if (success && step.type != ScenarioStep::WaitForMessage &&
      step.type != ScenarioStep::WaitTime) {
    // Immediate completion for publish and conditional
    StepResult result;
    result.stepIndex = currentStepIndex_;
    result.success = true;
    result.executionTimeMs = stepElapsedTimer_.elapsed();
    emit stepCompleted(currentStepIndex_, result);
  }

  return success;
}

bool ScenarioPlayer::executePublishStep(const ScenarioStep& step) {
  if (!messageHandler_) {
    emit stepFailed(currentStepIndex_, "Message handler not available");
    return false;
  }

  bool result = messageHandler_->publishMessage(step.topicName, step.messageType, step.messageData);

  if (!result) {
    emit stepFailed(currentStepIndex_, QString("Failed to publish to %1").arg(step.topicName));
  }

  return result;
}

bool ScenarioPlayer::executeWaitForMessageStep(const ScenarioStep& step) {
  if (!messageHandler_) {
    emit stepFailed(currentStepIndex_, "Message handler not available");
    return false;
  }

  // Subscribe to the topic
  messageHandler_->subscribeToTopic(step.topicName, step.messageType);

  waitingForTopic_ = step.topicName;
  waitingCondition_ = step.conditionExpression;
  messageReceived_ = false;

  // Start timeout timer
  waitTimer_->start(step.timeoutMs);

  return true;
}

bool ScenarioPlayer::executeWaitTimeStep(const ScenarioStep& step) {
  // Start execution timer with the wait time
  executionTimer_->start(step.waitTimeMs);
  return true;
}

bool ScenarioPlayer::executeConditionalStep(const ScenarioStep& step) {
  // Evaluate condition against last received message
  bool result = evaluateCondition(step.conditionExpression, lastReceivedMessage_);
  return result;
}

bool ScenarioPlayer::evaluateCondition(const QString& expression, const QJsonObject& message) {
  if (expression.isEmpty()) {
    return true;  // Empty condition always passes
  }

  // Simple condition parsing: "msg.field == value" or "msg.field != value"
  // For more complex expressions, a proper scripting engine would be needed

  QRegularExpression re(R"(msg\.(\w+(?:\.\w+)*)\s*(==|!=|>|<|>=|<=)\s*(.+))");
  auto match = re.match(expression);

  if (!match.hasMatch()) {
    return false;  // Invalid expression
  }

  QString fieldPath = match.captured(1);
  QString op = match.captured(2);
  QString expectedValue = match.captured(3).trimmed();

  // Remove quotes from string values
  if (expectedValue.startsWith('"') && expectedValue.endsWith('"')) {
    expectedValue = expectedValue.mid(1, expectedValue.length() - 2);
  }

  // Navigate to the field
  QJsonValue currentValue = message;
  QStringList pathParts = fieldPath.split('.');
  for (const QString& part : pathParts) {
    if (currentValue.isObject()) {
      currentValue = currentValue.toObject()[part];
    } else {
      return false;  // Field not found
    }
  }

  // Compare values
  QString actualValue = currentValue.toVariant().toString();

  if (op == "==") {
    return actualValue == expectedValue;
  } else if (op == "!=") {
    return actualValue != expectedValue;
  } else if (op == ">") {
    return actualValue.toDouble() > expectedValue.toDouble();
  } else if (op == "<") {
    return actualValue.toDouble() < expectedValue.toDouble();
  } else if (op == ">=") {
    return actualValue.toDouble() >= expectedValue.toDouble();
  } else if (op == "<=") {
    return actualValue.toDouble() <= expectedValue.toDouble();
  }

  return false;
}

void ScenarioPlayer::setState(ScenarioState newState) {
  if (state_ != newState) {
    state_ = newState;
    emit stateChanged(newState);
  }
}

void ScenarioPlayer::advanceToNextStep() {
  QMutexLocker locker(&mutex_);

  currentStepIndex_++;
  emit progressUpdated(progress());

  if (currentStepIndex_ >= scenario_.steps.size()) {
    // Loop handling
    if (scenario_.loopEnabled && (currentLoopNumber_ + 1) < scenario_.loopCount) {
      currentLoopNumber_++;
      currentStepIndex_ = 0;
      locker.unlock();
      emit loopCompleted(currentLoopNumber_);
      executionTimer_->start(0);
      return;
    }

    locker.unlock();
    completeScenario(true);
    return;
  }

  locker.unlock();

  // Execute next step
  executionTimer_->start(0);
}

void ScenarioPlayer::completeScenario(bool success) {
  if (success) {
    setState(ScenarioState::Completed);
  } else {
    setState(ScenarioState::Failed);
  }
  emit scenarioCompleted(success);
}

void ScenarioPlayer::onExecutionTimer() {
  QMutexLocker locker(&mutex_);

  if (state_ != ScenarioState::Running) {
    return;
  }

  if (currentStepIndex_ >= scenario_.steps.size()) {
    locker.unlock();
    completeScenario(true);
    return;
  }

  const ScenarioStep& step = scenario_.steps[currentStepIndex_];

  // Check if this is a completion of a WaitTime step
  if (step.type == ScenarioStep::WaitTime) {
    StepResult result;
    result.stepIndex = currentStepIndex_;
    result.success = true;
    result.executionTimeMs = stepElapsedTimer_.elapsed();

    locker.unlock();
    emit stepCompleted(currentStepIndex_, result);
    advanceToNextStep();
    return;
  }

  locker.unlock();

  // Execute the current step
  bool success = executeStep(step);

  if (!success) {
    completeScenario(false);
    return;
  }

  // For immediate steps, advance to next
  if (step.type == ScenarioStep::PublishMessage ||
      step.type == ScenarioStep::Conditional) {
    advanceToNextStep();
  }
}

void ScenarioPlayer::onWaitTimeout() {
  QMutexLocker locker(&mutex_);

  if (!waitingForTopic_.isEmpty() && !messageReceived_) {
    QString topic = waitingForTopic_;
    waitingForTopic_.clear();

    locker.unlock();

    emit stepFailed(currentStepIndex_, QString("Timeout waiting for message on %1").arg(topic));
    completeScenario(false);
  }
}

void ScenarioPlayer::onMessageReceived(const QString& topicName, const QJsonObject& message,
                                        qint64 timestamp) {
  Q_UNUSED(timestamp)

  // Recording mode
  if (isRecording_ && recordingTopics_.contains(topicName)) {
    ScenarioStep step;
    step.type = ScenarioStep::PublishMessage;
    step.topicName = topicName;
    step.messageData = message;
    step.description = QString("Recorded from %1").arg(topicName);

    recordedScenario_.steps.append(step);
    emit messageRecorded(topicName, message);
  }

  // Wait for message mode
  QMutexLocker locker(&mutex_);

  if (waitingForTopic_ == topicName) {
    // Check condition
    bool conditionMet = evaluateCondition(waitingCondition_, message);

    if (conditionMet) {
      waitTimer_->stop();
      messageReceived_ = true;
      lastReceivedMessage_ = message;
      waitingForTopic_.clear();

      StepResult result;
      result.stepIndex = currentStepIndex_;
      result.success = true;
      result.executionTimeMs = stepElapsedTimer_.elapsed();
      result.receivedMessage = message;

      locker.unlock();
      emit stepCompleted(currentStepIndex_, result);
      advanceToNextStep();
    }
  }
}

// Recording

void ScenarioPlayer::startRecording(const QStringList& topicsToRecord) {
  QMutexLocker locker(&mutex_);

  if (isRecording_) {
    return;
  }

  recordedScenario_ = TestScenario();
  recordedScenario_.name = "Recorded Scenario";
  recordedScenario_.createdDate = QDateTime::currentDateTime().toString(Qt::ISODate);
  recordingTopics_ = topicsToRecord;

  // Subscribe to all topics
  if (messageHandler_) {
    for (const QString& topic : topicsToRecord) {
      auto availableTopics = messageHandler_->getAvailableTopics();
      if (availableTopics.contains(topic)) {
        messageHandler_->subscribeToTopic(topic, availableTopics[topic]);
      }
    }
  }

  recordingTimer_.start();
  isRecording_ = true;

  locker.unlock();
  emit recordingStarted();
}

void ScenarioPlayer::stopRecording() {
  QMutexLocker locker(&mutex_);

  if (!isRecording_) {
    return;
  }

  isRecording_ = false;

  // Unsubscribe from recording topics
  if (messageHandler_) {
    for (const QString& topic : recordingTopics_) {
      messageHandler_->unsubscribeFromTopic(topic);
    }
  }

  recordingTopics_.clear();
  recordedScenario_.modifiedDate = QDateTime::currentDateTime().toString(Qt::ISODate);

  locker.unlock();
  emit recordingStopped();
}

TestScenario ScenarioPlayer::getRecordedScenario() const {
  QMutexLocker locker(&mutex_);
  return recordedScenario_;
}

// File I/O

bool ScenarioPlayer::saveScenario(const QString& filePath) {
  QMutexLocker locker(&mutex_);

  QFile file(filePath);
  if (!file.open(QIODevice::WriteOnly)) {
    return false;
  }

  scenario_.modifiedDate = QDateTime::currentDateTime().toString(Qt::ISODate);

  QJsonDocument doc(scenario_.toJson());
  file.write(doc.toJson(QJsonDocument::Indented));
  file.close();

  return true;
}

bool ScenarioPlayer::loadScenarioFromFile(const QString& filePath) {
  QFile file(filePath);
  if (!file.open(QIODevice::ReadOnly)) {
    return false;
  }

  QByteArray data = file.readAll();
  file.close();

  QJsonParseError error;
  QJsonDocument doc = QJsonDocument::fromJson(data, &error);
  if (error.error != QJsonParseError::NoError) {
    return false;
  }

  loadScenario(TestScenario::fromJson(doc.object()));
  return true;
}

QString ScenarioPlayer::exportToPytest() const {
  QMutexLocker locker(&mutex_);

  QString code;
  code += "#!/usr/bin/env python3\n";
  code += "\"\"\"Auto-generated pytest script from ROS2Weaver scenario\"\"\"\n\n";
  code += "import pytest\n";
  code += "import rclpy\n";
  code += "from rclpy.node import Node\n";
  code += "import time\n";
  code += "import json\n\n";

  // Add imports for message types
  QSet<QString> messageTypes;
  for (const auto& step : scenario_.steps) {
    if (!step.messageType.isEmpty()) {
      messageTypes.insert(step.messageType);
    }
  }

  for (const QString& msgType : messageTypes) {
    // Convert msg/Type format to python import
    QStringList parts = msgType.split("/");
    if (parts.size() >= 3) {
      QString pkg = parts[0];
      QString msgName = parts[2];
      code += QString("from %1.msg import %2\n").arg(pkg, msgName);
    }
  }

  code += "\n\n";
  QString className = scenario_.name;
  className.replace(" ", "").replace("-", "_");
  code += QString("class Test%1:\n").arg(className);
  code += QString("    \"\"\"%1\"\"\"\n\n").arg(scenario_.description);

  code += "    @pytest.fixture(autouse=True)\n";
  code += "    def setup_ros(self):\n";
  code += "        rclpy.init()\n";
  code += "        self.node = rclpy.create_node('test_scenario_node')\n";
  code += "        yield\n";
  code += "        self.node.destroy_node()\n";
  code += "        rclpy.shutdown()\n\n";

  QString testName = scenario_.name.toLower();
  testName.replace(" ", "_").replace("-", "_");
  code += QString("    def test_%1(self):\n").arg(testName);

  for (int i = 0; i < scenario_.steps.size(); ++i) {
    const auto& step = scenario_.steps[i];

    if (!step.enabled) {
      code += QString("        # Step %1 (DISABLED): %2\n").arg(i + 1).arg(step.description);
      continue;
    }

    code += QString("        # Step %1: %2\n").arg(i + 1).arg(step.description);

    switch (step.type) {
      case ScenarioStep::PublishMessage:
        code += QString("        pub = self.node.create_publisher(%1, '%2', 10)\n")
                    .arg(step.messageType.split("/").last(), step.topicName);
        code += QString("        msg = %1()\n").arg(step.messageType.split("/").last());
        code += "        # TODO: Set message fields from: " + QJsonDocument(step.messageData).toJson(QJsonDocument::Compact) + "\n";
        code += "        pub.publish(msg)\n";
        code += "        time.sleep(0.1)  # Allow message to be sent\n\n";
        break;

      case ScenarioStep::WaitTime:
        code += QString("        time.sleep(%1)\n\n").arg(step.waitTimeMs / 1000.0, 0, 'f', 3);
        break;

      case ScenarioStep::WaitForMessage:
        code += QString("        # Wait for message on %1 with condition: %2\n")
                    .arg(step.topicName, step.conditionExpression);
        code += "        received = False\n";
        code += QString("        timeout = %1\n").arg(step.timeoutMs / 1000.0, 0, 'f', 3);
        code += "        start_time = time.time()\n";
        code += "        while not received and (time.time() - start_time) < timeout:\n";
        code += "            rclpy.spin_once(self.node, timeout_sec=0.1)\n";
        code += "        assert received, f'Timeout waiting for message on " + step.topicName + "'\n\n";
        break;

      case ScenarioStep::Conditional:
        code += QString("        # Conditional: %1\n").arg(step.conditionExpression);
        code += "        # TODO: Implement condition check\n\n";
        break;
    }
  }

  code += "        pass  # Test completed\n";

  return code;
}

}  // namespace ros_weaver
