#ifndef ROS_WEAVER_CORE_AI_ERROR_ADVISOR_HPP
#define ROS_WEAVER_CORE_AI_ERROR_ADVISOR_HPP

#include <QObject>
#include <QString>
#include <QList>
#include <QMap>
#include <QVariant>

#include "ros_weaver/core/static_analyzer.hpp"

namespace ros_weaver {

// A suggested fix from the AI
struct AISuggestion {
  QString title;           // Short title for the suggestion
  QString description;     // Detailed explanation
  QString codeExample;     // Example code if applicable
  QString rationale;       // Why this fix is recommended
  double confidence = 0.0; // 0-1 confidence score
  bool canAutoApply = false;
  QMap<QString, QVariant> fixData;  // Data needed to apply the fix
};

// Result of asking AI about an issue
struct AIAdvisorResult {
  AnalysisIssue originalIssue;
  QList<AISuggestion> suggestions;
  QString context;          // Additional context provided
  QString modelUsed;        // Which AI model was used
  bool success = false;
  QString errorMessage;
};

// AI-powered error advisor that provides fix suggestions
class AIErrorAdvisor : public QObject {
  Q_OBJECT

public:
  static AIErrorAdvisor& instance();

  // Prevent copying
  AIErrorAdvisor(const AIErrorAdvisor&) = delete;
  AIErrorAdvisor& operator=(const AIErrorAdvisor&) = delete;

  // Get AI suggestions for a specific issue
  AIAdvisorResult getAdvice(const AnalysisIssue& issue);

  // Get AI suggestions for multiple issues
  QList<AIAdvisorResult> getAdviceForAll(const QList<AnalysisIssue>& issues);

  // Ask a custom question about an error
  QString askAboutError(const QString& errorMessage, const QString& context = "");

  // Generate fix code for an issue
  QString generateFixCode(const AnalysisIssue& issue, const QString& language = "cpp");

  // Check if AI backend is available
  bool isAvailable() const;

  // Get/set the AI model to use
  QString model() const { return model_; }
  void setModel(const QString& model) { model_ = model; }

signals:
  void adviceRequested(const AnalysisIssue& issue);
  void adviceReceived(const AIAdvisorResult& result);
  void errorOccurred(const QString& message);

private:
  AIErrorAdvisor();
  ~AIErrorAdvisor() override = default;

  // Build prompt for the AI
  QString buildPrompt(const AnalysisIssue& issue) const;

  // Parse AI response into suggestions
  QList<AISuggestion> parseResponse(const QString& response) const;

  // Get context about the issue type
  QString getIssueContext(const AnalysisIssue& issue) const;

  // Send request to AI backend
  QString sendToAI(const QString& prompt);

  QString model_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_AI_ERROR_ADVISOR_HPP
