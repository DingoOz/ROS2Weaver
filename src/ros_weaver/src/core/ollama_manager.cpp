#include "ros_weaver/core/ollama_manager.hpp"
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QNetworkRequest>
#include <QSettings>
#include <QDebug>

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
    "qwen2.5-coder:7b",
    "qwen2.5-coder:3b",
    "qwen2.5-coder:1.5b",
    "deepseek-coder-v2:16b",
    "codellama:7b",
    "llama3.2:3b",
    "llama3.2:1b",
    "phi3:mini",
    "mistral:7b",
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

  bool success = (currentPullReply_->error() == QNetworkReply::NoError);
  QString error = success ? QString() : currentPullReply_->errorString();

  QString modelName = currentPullingModel_;

  currentPullReply_->deleteLater();
  currentPullReply_ = nullptr;
  currentPullingModel_.clear();

  emit pullCompleted(modelName, success, error);

  if (success) {
    refreshLocalModels();
  }
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

QString OllamaManager::defaultSystemPrompt() {
  return QStringLiteral(
    "You are a ROS2 robotics assistant running locally with limited resources.\n\n"
    "CRITICAL RULES:\n"
    "- Keep responses SHORT and DIRECT. No lengthy explanations.\n"
    "- Answer in 1-3 sentences when possible.\n"
    "- Use bullet points for lists, not paragraphs.\n"
    "- Show code only when asked or essential.\n"
    "- Don't repeat the question or add filler phrases.\n\n"
    "ROS2 EXPERTISE:\n"
    "- Focus on ROS2 Humble/Iron/Jazzy patterns.\n"
    "- Topics: nodes, topics, services, actions, launch files, parameters, tf2, nav2, MoveIt2.\n"
    "- Prefer Python for quick examples, C++ when performance matters.\n"
    "- Use standard ROS2 conventions and best practices.\n\n"
    "FORMAT:\n"
    "- Use markdown for code blocks.\n"
    "- One code block per response unless comparing approaches.\n"
    "- If unsure, ask ONE clarifying question first."
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
  settings.endGroup();
}

}  // namespace ros_weaver
