#ifndef ROS_WEAVER_OLLAMA_MANAGER_HPP
#define ROS_WEAVER_OLLAMA_MANAGER_HPP

#include <QObject>
#include <QString>
#include <QStringList>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QTimer>

namespace ros_weaver {

struct OllamaModel {
  QString name;
  QString size;
  QString modifiedAt;
  QString digest;
  bool isDownloaded = false;
};

class OllamaManager : public QObject {
  Q_OBJECT

public:
  static OllamaManager& instance();

  // Connection status
  bool isOllamaRunning() const { return ollamaRunning_; }
  void checkOllamaStatus();

  // Model management
  QList<OllamaModel> localModels() const { return localModels_; }
  void refreshLocalModels();
  void pullModel(const QString& modelName);
  void deleteModel(const QString& modelName);
  void cancelPull();

  // Settings
  QString endpoint() const { return endpoint_; }
  void setEndpoint(const QString& endpoint);

  QString selectedModel() const { return selectedModel_; }
  void setSelectedModel(const QString& model);

  bool isEnabled() const { return enabled_; }
  void setEnabled(bool enabled);

  bool autoLoadModel() const { return autoLoadModel_; }
  void setAutoLoadModel(bool autoLoad);

  // API operations
  void generateCompletion(const QString& prompt, const QString& systemPrompt = QString());

  // Settings persistence
  void loadSettings();
  void saveSettings();

  // Recommended models list
  static QStringList recommendedModels();

signals:
  void ollamaStatusChanged(bool running);
  void localModelsUpdated(const QList<OllamaModel>& models);
  void pullProgress(const QString& modelName, double progress, const QString& status);
  void pullCompleted(const QString& modelName, bool success, const QString& error);
  void modelDeleted(const QString& modelName, bool success);
  void completionReceived(const QString& response);
  void completionError(const QString& error);
  void settingsChanged();

private slots:
  void onStatusCheckFinished(QNetworkReply* reply);
  void onPullReadyRead();
  void onPullFinished();
  void onDeleteFinished(QNetworkReply* reply);
  void onCompletionFinished(QNetworkReply* reply);

private:
  OllamaManager();
  ~OllamaManager();
  OllamaManager(const OllamaManager&) = delete;
  OllamaManager& operator=(const OllamaManager&) = delete;

  QNetworkAccessManager* networkManager_;
  QNetworkReply* currentPullReply_ = nullptr;
  QString currentPullingModel_;
  QTimer* statusCheckTimer_;

  // State
  bool ollamaRunning_ = false;
  QList<OllamaModel> localModels_;

  // Settings
  QString endpoint_ = "http://localhost:11434";
  QString selectedModel_;
  bool enabled_ = false;
  bool autoLoadModel_ = true;

  // Settings keys
  static constexpr const char* SETTINGS_GROUP = "Ollama";
  static constexpr const char* KEY_ENDPOINT = "endpoint";
  static constexpr const char* KEY_SELECTED_MODEL = "selectedModel";
  static constexpr const char* KEY_ENABLED = "enabled";
  static constexpr const char* KEY_AUTO_LOAD = "autoLoadModel";
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_OLLAMA_MANAGER_HPP
