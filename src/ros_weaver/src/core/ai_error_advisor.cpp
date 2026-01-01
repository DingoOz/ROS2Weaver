#include "ros_weaver/core/ai_error_advisor.hpp"
#include "ros_weaver/core/ollama_manager.hpp"

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QRegularExpression>
#include <rclcpp/rclcpp.hpp>

namespace ros_weaver {

AIErrorAdvisor& AIErrorAdvisor::instance() {
  static AIErrorAdvisor instance;
  return instance;
}

AIErrorAdvisor::AIErrorAdvisor()
    : QObject(nullptr)
    , model_("llama3.2")
{
}

bool AIErrorAdvisor::isAvailable() const {
  return OllamaManager::instance().isOllamaRunning();
}

AIAdvisorResult AIErrorAdvisor::getAdvice(const AnalysisIssue& issue) {
  AIAdvisorResult result;
  result.originalIssue = issue;

  emit adviceRequested(issue);

  if (!isAvailable()) {
    result.success = false;
    result.errorMessage = tr("AI backend (Ollama) is not available");
    emit errorOccurred(result.errorMessage);
    return result;
  }

  QString prompt = buildPrompt(issue);
  QString response = sendToAI(prompt);

  if (response.isEmpty()) {
    result.success = false;
    result.errorMessage = tr("No response from AI");
    emit errorOccurred(result.errorMessage);
    return result;
  }

  result.suggestions = parseResponse(response);
  result.context = getIssueContext(issue);
  result.modelUsed = model_;
  result.success = !result.suggestions.isEmpty();

  emit adviceReceived(result);
  return result;
}

QList<AIAdvisorResult> AIErrorAdvisor::getAdviceForAll(const QList<AnalysisIssue>& issues) {
  QList<AIAdvisorResult> results;

  for (const AnalysisIssue& issue : issues) {
    results.append(getAdvice(issue));
  }

  return results;
}

QString AIErrorAdvisor::askAboutError(const QString& errorMessage, const QString& context) {
  if (!isAvailable()) {
    return tr("AI backend is not available");
  }

  QString prompt = QString(
      "You are a ROS2 expert. A user encountered this error:\n\n"
      "Error: %1\n\n"
      "%2"
      "Please explain what this error means and provide specific steps to fix it. "
      "Include any relevant ROS2 commands or code examples."
  ).arg(errorMessage, context.isEmpty() ? "" : QString("Context: %1\n\n").arg(context));

  return sendToAI(prompt);
}

QString AIErrorAdvisor::generateFixCode(const AnalysisIssue& issue, const QString& language) {
  if (!isAvailable()) {
    return "";
  }

  QString prompt = QString(
      "You are a ROS2 expert. Generate %1 code to fix this issue:\n\n"
      "Issue: %2\n"
      "Description: %3\n\n"
      "Provide only the code needed to fix this issue, with brief comments. "
      "Do not include explanations outside the code."
  ).arg(language, issue.title, issue.description);

  return sendToAI(prompt);
}

QString AIErrorAdvisor::buildPrompt(const AnalysisIssue& issue) const {
  QString severityStr = StaticAnalyzer::severityToString(issue.severity);
  QString categoryStr = StaticAnalyzer::categoryToString(issue.category);

  QString prompt = QString(
      "You are a ROS2 robotics expert. Analyze this issue and provide fix suggestions.\n\n"
      "## Issue Details\n"
      "- **Severity**: %1\n"
      "- **Category**: %2\n"
      "- **Title**: %3\n"
      "- **Description**: %4\n"
      "- **Affected Element**: %5\n"
      "- **Current Suggestion**: %6\n\n"
      "## Your Task\n"
      "Provide 1-3 specific suggestions to fix this issue. For each suggestion include:\n"
      "1. A short title\n"
      "2. Detailed explanation of the fix\n"
      "3. Code example if applicable (use ROS2 Humble/Jazzy syntax)\n"
      "4. Potential side effects or considerations\n\n"
      "Format your response as JSON with this structure:\n"
      "```json\n"
      "{\n"
      "  \"suggestions\": [\n"
      "    {\n"
      "      \"title\": \"Fix title\",\n"
      "      \"description\": \"Detailed explanation\",\n"
      "      \"code\": \"example code if any\",\n"
      "      \"rationale\": \"Why this fix works\",\n"
      "      \"confidence\": 0.9\n"
      "    }\n"
      "  ]\n"
      "}\n"
      "```"
  ).arg(severityStr, categoryStr, issue.title, issue.description,
        issue.affectedElement, issue.suggestion);

  return prompt;
}

QList<AISuggestion> AIErrorAdvisor::parseResponse(const QString& response) const {
  QList<AISuggestion> suggestions;

  // Try to find JSON in the response
  static QRegularExpression jsonPattern(R"(\{[\s\S]*\"suggestions\"[\s\S]*\})");
  QRegularExpressionMatch match = jsonPattern.match(response);

  if (match.hasMatch()) {
    QString jsonStr = match.captured(0);
    QJsonDocument doc = QJsonDocument::fromJson(jsonStr.toUtf8());

    if (!doc.isNull() && doc.isObject()) {
      QJsonObject obj = doc.object();
      QJsonArray suggestionsArray = obj["suggestions"].toArray();

      for (const QJsonValue& val : suggestionsArray) {
        QJsonObject sugObj = val.toObject();
        AISuggestion sug;
        sug.title = sugObj["title"].toString();
        sug.description = sugObj["description"].toString();
        sug.codeExample = sugObj["code"].toString();
        sug.rationale = sugObj["rationale"].toString();
        sug.confidence = sugObj["confidence"].toDouble(0.5);
        suggestions.append(sug);
      }
    }
  }

  // If JSON parsing failed, try to extract suggestions from plain text
  if (suggestions.isEmpty() && !response.isEmpty()) {
    // Create a single suggestion from the raw response
    AISuggestion sug;
    sug.title = tr("AI Suggestion");
    sug.description = response;
    sug.confidence = 0.5;
    suggestions.append(sug);
  }

  return suggestions;
}

QString AIErrorAdvisor::getIssueContext(const AnalysisIssue& issue) const {
  QString context;

  switch (issue.category) {
    case IssueCategory::TypeMismatch:
      context = tr("Type mismatches occur when a publisher and subscriber use different "
                   "message types. ROS2 requires exact type matching for topic communication.");
      break;

    case IssueCategory::UnusedPublisher:
      context = tr("Unused publishers may indicate dead code or missing subscribers. "
                   "They consume resources without providing value.");
      break;

    case IssueCategory::UnusedSubscriber:
      context = tr("Unused subscribers wait for data that never arrives. This may "
                   "indicate missing publishers or configuration issues.");
      break;

    case IssueCategory::QoSIncompatible:
      context = tr("QoS (Quality of Service) settings must be compatible between "
                   "publishers and subscribers. Common issues include reliability "
                   "and durability mismatches.");
      break;

    case IssueCategory::CyclicDependency:
      context = tr("Cyclic dependencies can cause deadlocks or infinite loops. "
                   "Review the data flow to break the cycle.");
      break;

    case IssueCategory::MissingDependency:
      context = tr("Missing dependencies prevent the system from functioning. "
                   "Ensure all required nodes and packages are installed.");
      break;

    case IssueCategory::InvalidParameter:
      context = tr("Invalid parameters may cause nodes to fail at startup or "
                   "behave unexpectedly. Check parameter types and ranges.");
      break;

    case IssueCategory::NamingConvention:
      context = tr("ROS2 naming conventions help maintain consistency. Node names "
                   "should use snake_case and topics should use meaningful names.");
      break;

    case IssueCategory::Performance:
      context = tr("Performance issues can affect real-time behavior. Consider "
                   "message frequency, size, and processing time.");
      break;

    case IssueCategory::Security:
      context = tr("Security issues should be addressed before deployment. "
                   "Consider using SROS2 for production systems.");
      break;

    case IssueCategory::Deprecated:
      context = tr("Deprecated features may be removed in future ROS2 versions. "
                   "Consider migrating to the recommended alternatives.");
      break;
  }

  return context;
}

QString AIErrorAdvisor::sendToAI(const QString& prompt) {
  OllamaManager& ollama = OllamaManager::instance();

  if (!ollama.isOllamaRunning()) {
    RCLCPP_WARN(rclcpp::get_logger("ros_weaver"),
                "Ollama not connected, cannot get AI advice");
    return "";
  }

  // Use a synchronous request through the manager
  // Note: In production, this should be async to avoid blocking
  QString response;

  // Create a simple request - the OllamaManager handles the actual HTTP request
  // For now, we'll construct a basic prompt and use the manager's facilities

  // Build messages for the chat API
  QJsonArray messages;
  QJsonObject systemMsg;
  systemMsg["role"] = "system";
  systemMsg["content"] = "You are a helpful ROS2 robotics expert assistant. "
                         "Provide clear, actionable advice for fixing issues.";
  messages.append(systemMsg);

  QJsonObject userMsg;
  userMsg["role"] = "user";
  userMsg["content"] = prompt;
  messages.append(userMsg);

  // The actual sending would go through OllamaManager's sendMessage
  // For integration, we rely on the existing chat infrastructure
  // This is a simplified synchronous version

  RCLCPP_INFO(rclcpp::get_logger("ros_weaver"),
              "Sending error advice request to AI...");

  // Return empty for now - actual implementation would use async signals
  // from OllamaManager
  return "";
}

}  // namespace ros_weaver
