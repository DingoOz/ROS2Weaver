#include "ros_weaver/core/ollama_manager.hpp"
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QNetworkRequest>
#include <QSettings>
#include <QDebug>
#include <QUuid>

namespace ros_weaver {

// Helper function to format file size
static QString formatSize(qint64 bytes) {
  const char* units[] = {"B", "KB", "MB", "GB", "TB"};
  int unitIndex = 0;
  double size = bytes;

  while (size >= 1024 && unitIndex < 4) {
    size /= 1024;
    unitIndex++;
  }

  return QString::number(size, 'f', 1) + " " + units[unitIndex];
}

OllamaManager& OllamaManager::instance() {
  static OllamaManager instance;
  return instance;
}

OllamaManager::OllamaManager()
    : networkManager_(new QNetworkAccessManager(this))
    , statusCheckTimer_(new QTimer(this)) {
  loadSettings();

  // Periodic status check
  connect(statusCheckTimer_, &QTimer::timeout, this, &OllamaManager::checkOllamaStatus);
  statusCheckTimer_->start(10000);  // Check every 10 seconds

  // Initial status check
  checkOllamaStatus();
}

OllamaManager::~OllamaManager() {
  saveSettings();
  if (currentPullReply_) {
    currentPullReply_->abort();
  }
}

QStringList OllamaManager::recommendedModels() {
  return QStringList{
    // Good for tool calling / structured output
    "llama3.1:8b",
    "qwen2.5:7b",
    "mistral:7b",
    "mistral-small:latest",
    // Coding focused
    "qwen2.5-coder:7b",
    "qwen2.5-coder:3b",
    "deepseek-coder-v2:16b",
    "codellama:7b",
    // Smaller/faster
    "llama3.2:3b",
    "llama3.2:1b",
    "qwen2.5:3b",
    "phi3:mini",
    "gemma2:2b"
  };
}

void OllamaManager::checkOllamaStatus() {
  QNetworkRequest request(QUrl(endpoint_ + "/api/tags"));
  request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

  QNetworkReply* reply = networkManager_->get(request);
  connect(reply, &QNetworkReply::finished, this, [this, reply]() {
    onStatusCheckFinished(reply);
  });
}

void OllamaManager::onStatusCheckFinished(QNetworkReply* reply) {
  bool wasRunning = ollamaRunning_;

  if (reply->error() == QNetworkReply::NoError) {
    ollamaRunning_ = true;

    // Parse the models from the response
    QJsonDocument doc = QJsonDocument::fromJson(reply->readAll());
    QJsonObject obj = doc.object();
    QJsonArray modelsArray = obj["models"].toArray();

    localModels_.clear();
    for (const QJsonValue& val : modelsArray) {
      QJsonObject modelObj = val.toObject();
      OllamaModel model;
      model.name = modelObj["name"].toString();
      model.size = formatSize(modelObj["size"].toVariant().toLongLong());
      model.modifiedAt = modelObj["modified_at"].toString();
      model.digest = modelObj["digest"].toString();
      model.isDownloaded = true;
      localModels_.append(model);
    }

    emit localModelsUpdated(localModels_);
  } else {
    ollamaRunning_ = false;
    localModels_.clear();
    emit localModelsUpdated(localModels_);
  }

  if (wasRunning != ollamaRunning_) {
    emit ollamaStatusChanged(ollamaRunning_);
  }

  reply->deleteLater();
}

void OllamaManager::refreshLocalModels() {
  checkOllamaStatus();
}

void OllamaManager::pullModel(const QString& modelName) {
  if (currentPullReply_) {
    return;  // Already pulling
  }

  currentPullingModel_ = modelName;
  pullError_.clear();  // Reset any previous error

  QNetworkRequest request(QUrl(endpoint_ + "/api/pull"));
  request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

  QJsonObject json;
  json["name"] = modelName;
  json["stream"] = true;

  currentPullReply_ = networkManager_->post(request, QJsonDocument(json).toJson());

  connect(currentPullReply_, &QNetworkReply::readyRead, this, &OllamaManager::onPullReadyRead);
  connect(currentPullReply_, &QNetworkReply::finished, this, &OllamaManager::onPullFinished);
}

void OllamaManager::onPullReadyRead() {
  if (!currentPullReply_) return;

  while (currentPullReply_->canReadLine()) {
    QByteArray line = currentPullReply_->readLine();
    QJsonDocument doc = QJsonDocument::fromJson(line);
    QJsonObject obj = doc.object();

    // Check for error in response (Ollama returns {"error": "message"} for invalid models)
    if (obj.contains("error")) {
      pullError_ = obj["error"].toString();
      emit pullProgress(currentPullingModel_, -1, tr("Error: %1").arg(pullError_));
      return;
    }

    QString status = obj["status"].toString();

    if (obj.contains("total") && obj.contains("completed")) {
      qint64 total = obj["total"].toVariant().toLongLong();
      qint64 completed = obj["completed"].toVariant().toLongLong();
      double progress = (total > 0) ? (static_cast<double>(completed) / total) * 100.0 : 0.0;
      emit pullProgress(currentPullingModel_, progress, status);
    } else {
      emit pullProgress(currentPullingModel_, -1, status);  // -1 indicates indeterminate
    }
  }
}

void OllamaManager::onPullFinished() {
  if (!currentPullReply_) return;

  // Check both HTTP error AND error from response body
  bool httpSuccess = (currentPullReply_->error() == QNetworkReply::NoError);
  bool ollamaSuccess = pullError_.isEmpty();
  bool success = httpSuccess && ollamaSuccess;

  QString error;
  if (!httpSuccess) {
    error = currentPullReply_->errorString();
  } else if (!ollamaSuccess) {
    error = pullError_;
  }

  QString modelName = currentPullingModel_;

  currentPullReply_->deleteLater();
  currentPullReply_ = nullptr;
  currentPullingModel_.clear();
  pullError_.clear();

  // Refresh the models list BEFORE emitting pullCompleted
  // This ensures the model list is updated before any UI response
  if (success) {
    refreshLocalModels();
  }

  emit pullCompleted(modelName, success, error);
}

void OllamaManager::cancelPull() {
  if (currentPullReply_) {
    currentPullReply_->abort();
  }
}

void OllamaManager::deleteModel(const QString& modelName) {
  QNetworkRequest request(QUrl(endpoint_ + "/api/delete"));
  request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

  QJsonObject json;
  json["name"] = modelName;

  QNetworkReply* reply = networkManager_->sendCustomRequest(request, "DELETE", QJsonDocument(json).toJson());
  connect(reply, &QNetworkReply::finished, this, [this, reply, modelName]() {
    onDeleteFinished(reply);
  });
}

void OllamaManager::onDeleteFinished(QNetworkReply* reply) {
  bool success = (reply->error() == QNetworkReply::NoError);

  // Extract model name from the original request
  QJsonDocument doc = QJsonDocument::fromJson(reply->request().attribute(QNetworkRequest::User).toByteArray());
  QString modelName = doc.object()["name"].toString();

  emit modelDeleted(modelName, success);

  if (success) {
    refreshLocalModels();
  }

  reply->deleteLater();
}

void OllamaManager::generateCompletion(const QString& prompt, const QString& systemPrompt,
                                        const QStringList& images) {
  if (!ollamaRunning_ || selectedModel_.isEmpty()) {
    emit completionError(tr("Ollama is not running or no model selected"));
    return;
  }

  // Cancel any existing completion
  if (currentCompletionReply_) {
    cancelCompletion();
  }

  accumulatedResponse_.clear();

  QNetworkRequest request(QUrl(endpoint_ + "/api/generate"));
  request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

  QJsonObject json;
  json["model"] = selectedModel_;
  json["prompt"] = prompt;
  json["stream"] = true;  // Enable streaming

  if (!systemPrompt.isEmpty()) {
    json["system"] = systemPrompt;
  }

  // Add images for multimodal models (base64 encoded)
  if (!images.isEmpty()) {
    QJsonArray imagesArray;
    for (const QString& img : images) {
      imagesArray.append(img);
    }
    json["images"] = imagesArray;
  }

  // Add options (like num_thread) if configured
  if (numThreads_ > 0) {
    QJsonObject options;
    options["num_thread"] = numThreads_;
    json["options"] = options;
  }

  currentCompletionReply_ = networkManager_->post(request, QJsonDocument(json).toJson());

  connect(currentCompletionReply_, &QNetworkReply::readyRead,
          this, &OllamaManager::onCompletionReadyRead);
  connect(currentCompletionReply_, &QNetworkReply::finished,
          this, &OllamaManager::onCompletionFinished);

  emit completionStarted();
}

bool OllamaManager::isVisionModel(const QString& modelName) {
  // Known vision/multimodal models
  QString lower = modelName.toLower();
  return lower.contains("llava") ||
         lower.contains("bakllava") ||
         lower.contains("llama3.2-vision") ||
         lower.contains("minicpm-v") ||
         lower.contains("moondream") ||
         lower.contains("cogvlm") ||
         lower.contains("yi-vl");
}

void OllamaManager::cancelCompletion() {
  if (currentCompletionReply_) {
    currentCompletionReply_->abort();
    currentCompletionReply_->deleteLater();
    currentCompletionReply_ = nullptr;
    accumulatedResponse_.clear();
  }
}

void OllamaManager::onCompletionReadyRead() {
  if (!currentCompletionReply_) return;

  // Read all available data
  while (currentCompletionReply_->canReadLine()) {
    QByteArray line = currentCompletionReply_->readLine();
    if (line.isEmpty()) continue;

    QJsonDocument doc = QJsonDocument::fromJson(line);
    if (doc.isNull()) continue;

    QJsonObject obj = doc.object();
    QString token = obj["response"].toString();

    if (!token.isEmpty()) {
      accumulatedResponse_ += token;
      emit completionToken(token);
    }

    // Check if this is the final message
    if (obj["done"].toBool()) {
      // Final message received
    }
  }
}

void OllamaManager::onCompletionFinished() {
  if (!currentCompletionReply_) return;

  if (currentCompletionReply_->error() == QNetworkReply::NoError) {
    // Process any remaining data
    QByteArray remaining = currentCompletionReply_->readAll();
    for (const QByteArray& line : remaining.split('\n')) {
      if (line.isEmpty()) continue;
      QJsonDocument doc = QJsonDocument::fromJson(line);
      if (doc.isNull()) continue;
      QString token = doc.object()["response"].toString();
      if (!token.isEmpty()) {
        accumulatedResponse_ += token;
        emit completionToken(token);
      }
    }
    emit completionFinished(accumulatedResponse_);
  } else if (currentCompletionReply_->error() != QNetworkReply::OperationCanceledError) {
    emit completionError(currentCompletionReply_->errorString());
  }

  currentCompletionReply_->deleteLater();
  currentCompletionReply_ = nullptr;
}

void OllamaManager::setEndpoint(const QString& endpoint) {
  if (endpoint_ != endpoint) {
    endpoint_ = endpoint;
    saveSettings();
    checkOllamaStatus();
    emit settingsChanged();
  }
}

void OllamaManager::setSelectedModel(const QString& model) {
  if (selectedModel_ != model) {
    selectedModel_ = model;
    saveSettings();
    emit settingsChanged();
  }
}

void OllamaManager::setEnabled(bool enabled) {
  if (enabled_ != enabled) {
    enabled_ = enabled;
    saveSettings();
    emit settingsChanged();
  }
}

void OllamaManager::setAutoLoadModel(bool autoLoad) {
  if (autoLoadModel_ != autoLoad) {
    autoLoadModel_ = autoLoad;
    saveSettings();
    emit settingsChanged();
  }
}

void OllamaManager::setSystemPrompt(const QString& prompt) {
  if (systemPrompt_ != prompt) {
    systemPrompt_ = prompt;
    saveSettings();
    emit settingsChanged();
  }
}

void OllamaManager::setNumThreads(int threads) {
  if (numThreads_ != threads) {
    numThreads_ = threads;
    saveSettings();
    emit settingsChanged();
  }
}

// Chat appearance setters
void OllamaManager::setChatFont(const QFont& font) {
  if (chatFont_ != font) {
    chatFont_ = font;
    saveSettings();
    emit settingsChanged();
  }
}

void OllamaManager::setChatFontSize(int size) {
  if (chatFontSize_ != size) {
    chatFontSize_ = size;
    saveSettings();
    emit settingsChanged();
  }
}

void OllamaManager::setUserMessageColor(const QColor& color) {
  if (userMessageColor_ != color) {
    userMessageColor_ = color;
    saveSettings();
    emit settingsChanged();
  }
}

void OllamaManager::setUserBackgroundColor(const QColor& color) {
  if (userBackgroundColor_ != color) {
    userBackgroundColor_ = color;
    saveSettings();
    emit settingsChanged();
  }
}

void OllamaManager::setAssistantMessageColor(const QColor& color) {
  if (assistantMessageColor_ != color) {
    assistantMessageColor_ = color;
    saveSettings();
    emit settingsChanged();
  }
}

void OllamaManager::setAssistantBackgroundColor(const QColor& color) {
  if (assistantBackgroundColor_ != color) {
    assistantBackgroundColor_ = color;
    saveSettings();
    emit settingsChanged();
  }
}

void OllamaManager::setCodeBackgroundColor(const QColor& color) {
  if (codeBackgroundColor_ != color) {
    codeBackgroundColor_ = color;
    saveSettings();
    emit settingsChanged();
  }
}

// Default appearance values
QFont OllamaManager::defaultChatFont() {
  return QFont("Sans Serif", 13);
}

int OllamaManager::defaultChatFontSize() {
  return 13;
}

QColor OllamaManager::defaultUserMessageColor() {
  return QColor("#e0e0e0");
}

QColor OllamaManager::defaultUserBackgroundColor() {
  return QColor("#1e3a5f");
}

QColor OllamaManager::defaultAssistantMessageColor() {
  return QColor("#e0e0e0");
}

QColor OllamaManager::defaultAssistantBackgroundColor() {
  return QColor("#2d2d2d");
}

QColor OllamaManager::defaultCodeBackgroundColor() {
  return QColor("#1a1a1a");
}

QString OllamaManager::defaultSystemPrompt() {
  return QStringLiteral(
    "You are a ROS2 assistant integrated with ROS Weaver, a visual canvas editor for ROS2 projects.\n\n"
    "You have access to tools that can control the canvas:\n"
    "- load_example: Load example projects (turtlesim_teleop, turtlebot3_navigation)\n"
    "- add_block: Add ROS2 nodes to the canvas\n"
    "- remove_block: Remove nodes from the canvas\n"
    "- set_parameter: Modify node parameters\n"
    "- create_connection: Connect nodes via topics\n"
    "- remove_connection: Disconnect nodes\n"
    "- create_group: Group nodes together\n"
    "- get_project_state: Get current canvas state\n"
    "- get_block_info: Get details about a specific node\n"
    "- list_available_packages: List available ROS2 packages\n\n"
    "When the user asks you to modify the canvas, use the appropriate tool.\n"
    "For general ROS2 questions, answer directly without using tools.\n"
    "Keep responses concise and helpful."
  );
}

void OllamaManager::loadSettings() {
  QSettings settings("ROS Weaver", "ROS Weaver");
  settings.beginGroup(SETTINGS_GROUP);
  endpoint_ = settings.value(KEY_ENDPOINT, "http://localhost:11434").toString();
  selectedModel_ = settings.value(KEY_SELECTED_MODEL, "").toString();
  enabled_ = settings.value(KEY_ENABLED, false).toBool();
  autoLoadModel_ = settings.value(KEY_AUTO_LOAD, true).toBool();
  systemPrompt_ = settings.value(KEY_SYSTEM_PROMPT, defaultSystemPrompt()).toString();
  numThreads_ = settings.value(KEY_NUM_THREADS, 0).toInt();

  // Load chat appearance settings
  QString fontFamily = settings.value(KEY_CHAT_FONT_FAMILY, defaultChatFont().family()).toString();
  chatFontSize_ = settings.value(KEY_CHAT_FONT_SIZE, defaultChatFontSize()).toInt();
  bool fontBold = settings.value(KEY_CHAT_FONT_BOLD, false).toBool();
  bool fontItalic = settings.value(KEY_CHAT_FONT_ITALIC, false).toBool();
  chatFont_ = QFont(fontFamily, chatFontSize_);
  chatFont_.setBold(fontBold);
  chatFont_.setItalic(fontItalic);

  userMessageColor_ = QColor(settings.value(KEY_USER_MESSAGE_COLOR, defaultUserMessageColor().name()).toString());
  userBackgroundColor_ = QColor(settings.value(KEY_USER_BACKGROUND_COLOR, defaultUserBackgroundColor().name()).toString());
  assistantMessageColor_ = QColor(settings.value(KEY_ASSISTANT_MESSAGE_COLOR, defaultAssistantMessageColor().name()).toString());
  assistantBackgroundColor_ = QColor(settings.value(KEY_ASSISTANT_BACKGROUND_COLOR, defaultAssistantBackgroundColor().name()).toString());
  codeBackgroundColor_ = QColor(settings.value(KEY_CODE_BACKGROUND_COLOR, defaultCodeBackgroundColor().name()).toString());

  settings.endGroup();
}

void OllamaManager::saveSettings() {
  QSettings settings("ROS Weaver", "ROS Weaver");
  settings.beginGroup(SETTINGS_GROUP);
  settings.setValue(KEY_ENDPOINT, endpoint_);
  settings.setValue(KEY_SELECTED_MODEL, selectedModel_);
  settings.setValue(KEY_ENABLED, enabled_);
  settings.setValue(KEY_AUTO_LOAD, autoLoadModel_);
  settings.setValue(KEY_SYSTEM_PROMPT, systemPrompt_);
  settings.setValue(KEY_NUM_THREADS, numThreads_);

  // Save chat appearance settings
  settings.setValue(KEY_CHAT_FONT_FAMILY, chatFont_.family());
  settings.setValue(KEY_CHAT_FONT_SIZE, chatFontSize_);
  settings.setValue(KEY_CHAT_FONT_BOLD, chatFont_.bold());
  settings.setValue(KEY_CHAT_FONT_ITALIC, chatFont_.italic());
  settings.setValue(KEY_USER_MESSAGE_COLOR, userMessageColor_.name());
  settings.setValue(KEY_USER_BACKGROUND_COLOR, userBackgroundColor_.name());
  settings.setValue(KEY_ASSISTANT_MESSAGE_COLOR, assistantMessageColor_.name());
  settings.setValue(KEY_ASSISTANT_BACKGROUND_COLOR, assistantBackgroundColor_.name());
  settings.setValue(KEY_CODE_BACKGROUND_COLOR, codeBackgroundColor_.name());

  settings.endGroup();
}

// ============================================================================
// Native Tool Calling API Implementation (/api/chat with tools)
// ============================================================================

void OllamaManager::clearConversation() {
  conversationHistory_.clear();
}

OllamaTool OllamaManager::convertFromAITool(const QString& name, const QString& description,
                                             const QJsonObject& parametersSchema) {
  OllamaTool tool;
  tool.name = name;
  tool.description = description;
  tool.parameters = parametersSchema;
  return tool;
}

void OllamaManager::chatWithTools(const QList<ChatMessage>& messages,
                                   const QList<OllamaTool>& tools,
                                   const QStringList& images) {
  if (!ollamaRunning_ || selectedModel_.isEmpty()) {
    emit chatError(tr("Ollama is not running or no model selected"));
    return;
  }

  // Cancel any existing chat request
  if (currentChatReply_) {
    currentChatReply_->abort();
    currentChatReply_->deleteLater();
    currentChatReply_ = nullptr;
  }

  accumulatedChatResponse_.clear();
  pendingToolCalls_.clear();
  currentAssistantMessage_ = ChatMessage();
  currentAssistantMessage_.role = "assistant";
  currentRequestTools_ = tools;

  QNetworkRequest request(QUrl(endpoint_ + "/api/chat"));
  request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

  QJsonObject json;
  json["model"] = selectedModel_;
  json["stream"] = true;

  // Build messages array
  QJsonArray messagesArray;
  for (const ChatMessage& msg : messages) {
    QJsonObject msgObj;
    msgObj["role"] = msg.role;
    msgObj["content"] = msg.content;

    // Add tool_calls for assistant messages if present
    if (msg.role == "assistant" && !msg.toolCalls.isEmpty()) {
      msgObj["tool_calls"] = msg.toolCalls;
    }

    // Add tool_call_id for tool response messages
    if (msg.role == "tool" && !msg.toolCallId.isEmpty()) {
      // For Ollama, tool results are just sent as content
      // The context is maintained by the conversation order
    }

    messagesArray.append(msgObj);
  }
  json["messages"] = messagesArray;

  // Add tools if provided and tool calling is enabled
  if (toolCallingEnabled_ && !tools.isEmpty()) {
    QJsonArray toolsArray;
    for (const OllamaTool& tool : tools) {
      QJsonObject toolObj;
      toolObj["type"] = "function";

      QJsonObject functionObj;
      functionObj["name"] = tool.name;
      functionObj["description"] = tool.description;
      functionObj["parameters"] = tool.parameters;
      toolObj["function"] = functionObj;

      toolsArray.append(toolObj);
    }
    json["tools"] = toolsArray;
  }

  // Add images for multimodal models
  if (!images.isEmpty()) {
    // For chat API, images go in the last user message
    // This is handled by the caller who should include images in the message content
    // or we can add them to the request options
  }

  // Add options if configured
  if (numThreads_ > 0) {
    QJsonObject options;
    options["num_thread"] = numThreads_;
    json["options"] = options;
  }

  qDebug() << "Sending chat request with" << tools.size() << "tools";

  currentChatReply_ = networkManager_->post(request, QJsonDocument(json).toJson());

  connect(currentChatReply_, &QNetworkReply::readyRead,
          this, &OllamaManager::onChatReadyRead);
  connect(currentChatReply_, &QNetworkReply::finished,
          this, &OllamaManager::onChatFinished);

  emit chatStarted();
}

void OllamaManager::sendToolResults(const QList<ChatMessage>& messages,
                                     const QList<OllamaTool>& tools) {
  // This is essentially another chat request with the tool results included
  chatWithTools(messages, tools, QStringList());
}

void OllamaManager::onChatReadyRead() {
  if (!currentChatReply_) return;

  while (currentChatReply_->canReadLine()) {
    QByteArray line = currentChatReply_->readLine();
    if (line.isEmpty()) continue;

    QJsonDocument doc = QJsonDocument::fromJson(line);
    if (doc.isNull()) continue;

    QJsonObject obj = doc.object();

    // Check for error
    if (obj.contains("error")) {
      emit chatError(obj["error"].toString());
      return;
    }

    // Get the message object
    QJsonObject messageObj = obj["message"].toObject();
    QString role = messageObj["role"].toString();

    // Handle content tokens (streaming)
    QString content = messageObj["content"].toString();
    if (!content.isEmpty()) {
      accumulatedChatResponse_ += content;
      currentAssistantMessage_.content += content;
      emit chatToken(content);
    }

    // Handle tool_calls
    if (messageObj.contains("tool_calls")) {
      QJsonArray toolCallsArray = messageObj["tool_calls"].toArray();
      currentAssistantMessage_.toolCalls = toolCallsArray;

      for (const QJsonValue& tcVal : toolCallsArray) {
        QJsonObject tcObj = tcVal.toObject();
        OllamaToolCall toolCall;

        // Generate an ID if not provided
        toolCall.id = tcObj.contains("id") ?
            tcObj["id"].toString() :
            QUuid::createUuid().toString(QUuid::WithoutBraces);

        QJsonObject functionObj = tcObj["function"].toObject();
        toolCall.functionName = functionObj["name"].toString();

        // Arguments can be a string (JSON) or an object
        QJsonValue argsVal = functionObj["arguments"];
        if (argsVal.isString()) {
          QJsonDocument argsDoc = QJsonDocument::fromJson(argsVal.toString().toUtf8());
          toolCall.arguments = argsDoc.object();
        } else if (argsVal.isObject()) {
          toolCall.arguments = argsVal.toObject();
        }

        pendingToolCalls_.append(toolCall);

        qDebug() << "Tool call received:" << toolCall.functionName
                 << "with args:" << QJsonDocument(toolCall.arguments).toJson(QJsonDocument::Compact);
      }
    }

    // Check if this is the final message
    if (obj["done"].toBool()) {
      // Message complete
    }
  }
}

void OllamaManager::onChatFinished() {
  if (!currentChatReply_) return;

  if (currentChatReply_->error() == QNetworkReply::NoError) {
    // Process any remaining data
    QByteArray remaining = currentChatReply_->readAll();
    for (const QByteArray& line : remaining.split('\n')) {
      if (line.isEmpty()) continue;

      QJsonDocument doc = QJsonDocument::fromJson(line);
      if (doc.isNull()) continue;

      QJsonObject obj = doc.object();
      QJsonObject messageObj = obj["message"].toObject();

      QString content = messageObj["content"].toString();
      if (!content.isEmpty()) {
        accumulatedChatResponse_ += content;
        currentAssistantMessage_.content += content;
        emit chatToken(content);
      }

      // Handle tool_calls in final message
      if (messageObj.contains("tool_calls") && currentAssistantMessage_.toolCalls.isEmpty()) {
        QJsonArray toolCallsArray = messageObj["tool_calls"].toArray();
        currentAssistantMessage_.toolCalls = toolCallsArray;

        for (const QJsonValue& tcVal : toolCallsArray) {
          QJsonObject tcObj = tcVal.toObject();
          OllamaToolCall toolCall;

          toolCall.id = tcObj.contains("id") ?
              tcObj["id"].toString() :
              QUuid::createUuid().toString(QUuid::WithoutBraces);

          QJsonObject functionObj = tcObj["function"].toObject();
          toolCall.functionName = functionObj["name"].toString();

          QJsonValue argsVal = functionObj["arguments"];
          if (argsVal.isString()) {
            QJsonDocument argsDoc = QJsonDocument::fromJson(argsVal.toString().toUtf8());
            toolCall.arguments = argsDoc.object();
          } else if (argsVal.isObject()) {
            toolCall.arguments = argsVal.toObject();
          }

          pendingToolCalls_.append(toolCall);
        }
      }
    }

    // Emit appropriate signal based on whether tool calls are pending
    if (!pendingToolCalls_.isEmpty()) {
      qDebug() << "Emitting toolCallsReceived with" << pendingToolCalls_.size() << "calls";
      emit toolCallsReceived(pendingToolCalls_);
    } else {
      emit chatFinished(accumulatedChatResponse_, currentAssistantMessage_);
    }

  } else if (currentChatReply_->error() != QNetworkReply::OperationCanceledError) {
    emit chatError(currentChatReply_->errorString());
  }

  currentChatReply_->deleteLater();
  currentChatReply_ = nullptr;
}

}  // namespace ros_weaver
