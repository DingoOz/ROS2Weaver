#ifndef ROS_WEAVER_OLLAMA_MANAGER_HPP
#define ROS_WEAVER_OLLAMA_MANAGER_HPP

#include <QObject>
#include <QString>
#include <QStringList>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QTimer>
#include <QFont>
#include <QColor>

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

  QString systemPrompt() const { return systemPrompt_; }
  void setSystemPrompt(const QString& prompt);
  static QString defaultSystemPrompt();

  // CPU threads for inference (0 = auto/default)
  int numThreads() const { return numThreads_; }
  void setNumThreads(int threads);

  // Chat appearance settings
  QFont chatFont() const { return chatFont_; }
  void setChatFont(const QFont& font);

  int chatFontSize() const { return chatFontSize_; }
  void setChatFontSize(int size);

  QColor userMessageColor() const { return userMessageColor_; }
  void setUserMessageColor(const QColor& color);

  QColor userBackgroundColor() const { return userBackgroundColor_; }
  void setUserBackgroundColor(const QColor& color);

  QColor assistantMessageColor() const { return assistantMessageColor_; }
  void setAssistantMessageColor(const QColor& color);

  QColor assistantBackgroundColor() const { return assistantBackgroundColor_; }
  void setAssistantBackgroundColor(const QColor& color);

  QColor codeBackgroundColor() const { return codeBackgroundColor_; }
  void setCodeBackgroundColor(const QColor& color);

  // Default appearance values
  static QFont defaultChatFont();
  static int defaultChatFontSize();
  static QColor defaultUserMessageColor();
  static QColor defaultUserBackgroundColor();
  static QColor defaultAssistantMessageColor();
  static QColor defaultAssistantBackgroundColor();
  static QColor defaultCodeBackgroundColor();

  // API operations - streaming completion
  // images parameter accepts base64-encoded image data for multimodal models (llava, etc.)
  void generateCompletion(const QString& prompt, const QString& systemPrompt = QString(),
                          const QStringList& images = QStringList());
  void cancelCompletion();
  bool isGenerating() const { return currentCompletionReply_ != nullptr; }

  // Check if a model likely supports vision (multimodal)
  static bool isVisionModel(const QString& modelName);

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
  // Streaming completion signals
  void completionStarted();
  void completionToken(const QString& token);
  void completionFinished(const QString& fullResponse);
  void completionError(const QString& error);
  void settingsChanged();

private slots:
  void onStatusCheckFinished(QNetworkReply* reply);
  void onPullReadyRead();
  void onPullFinished();
  void onDeleteFinished(QNetworkReply* reply);
  void onCompletionReadyRead();
  void onCompletionFinished();

private:
  OllamaManager();
  ~OllamaManager();
  OllamaManager(const OllamaManager&) = delete;
  OllamaManager& operator=(const OllamaManager&) = delete;

  QNetworkAccessManager* networkManager_;
  QNetworkReply* currentPullReply_ = nullptr;
  QNetworkReply* currentCompletionReply_ = nullptr;
  QString currentPullingModel_;
  QString pullError_;  // Stores error from Ollama's response body
  QString accumulatedResponse_;
  QTimer* statusCheckTimer_;

  // State
  bool ollamaRunning_ = false;
  QList<OllamaModel> localModels_;

  // Settings
  QString endpoint_ = "http://localhost:11434";
  QString selectedModel_;
  QString systemPrompt_;
  bool enabled_ = false;
  bool autoLoadModel_ = true;
  int numThreads_ = 0;  // 0 = auto/default

  // Chat appearance settings
  QFont chatFont_;
  int chatFontSize_ = 13;
  QColor userMessageColor_;
  QColor userBackgroundColor_;
  QColor assistantMessageColor_;
  QColor assistantBackgroundColor_;
  QColor codeBackgroundColor_;

  // Settings keys
  static constexpr const char* SETTINGS_GROUP = "Ollama";
  static constexpr const char* KEY_ENDPOINT = "endpoint";
  static constexpr const char* KEY_SELECTED_MODEL = "selectedModel";
  static constexpr const char* KEY_ENABLED = "enabled";
  static constexpr const char* KEY_AUTO_LOAD = "autoLoadModel";
  static constexpr const char* KEY_SYSTEM_PROMPT = "systemPrompt";
  static constexpr const char* KEY_NUM_THREADS = "numThreads";
  static constexpr const char* KEY_CHAT_FONT_FAMILY = "chatFontFamily";
  static constexpr const char* KEY_CHAT_FONT_SIZE = "chatFontSize";
  static constexpr const char* KEY_CHAT_FONT_BOLD = "chatFontBold";
  static constexpr const char* KEY_CHAT_FONT_ITALIC = "chatFontItalic";
  static constexpr const char* KEY_USER_MESSAGE_COLOR = "userMessageColor";
  static constexpr const char* KEY_USER_BACKGROUND_COLOR = "userBackgroundColor";
  static constexpr const char* KEY_ASSISTANT_MESSAGE_COLOR = "assistantMessageColor";
  static constexpr const char* KEY_ASSISTANT_BACKGROUND_COLOR = "assistantBackgroundColor";
  static constexpr const char* KEY_CODE_BACKGROUND_COLOR = "codeBackgroundColor";
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_OLLAMA_MANAGER_HPP
