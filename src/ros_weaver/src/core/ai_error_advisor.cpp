#include "ros_weaver/core/ai_error_advisor.hpp"
#include "ros_weaver/core/ollama_manager.hpp"

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QRegularExpression>
#include <QEventLoop>
#include <QTimer>
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

  if (ollama.selectedModel().isEmpty()) {
    RCLCPP_WARN(rclcpp::get_logger("ros_weaver"),
                "No model selected, cannot get AI advice");
    return "";
  }

  RCLCPP_INFO(rclcpp::get_logger("ros_weaver"),
              "Sending error advice request to AI...");

  // Use an event loop to wait for the async response synchronously
  QString response;
  bool completed = false;
  bool hasError = false;

  QEventLoop loop;

  // Set up timeout (30 seconds for AI response)
  QTimer timeout;
  timeout.setSingleShot(true);
  QObject::connect(&timeout, &QTimer::timeout, &loop, &QEventLoop::quit);

  // Connect to completion signals
  auto tokenConn = QObject::connect(&ollama, &OllamaManager::completionToken,
      [&response](const QString& token) {
        response += token;
      });

  auto finishConn = QObject::connect(&ollama, &OllamaManager::completionFinished,
      [&completed, &loop](const QString& fullResponse) {
        Q_UNUSED(fullResponse);
        completed = true;
        loop.quit();
      });

  auto errorConn = QObject::connect(&ollama, &OllamaManager::completionError,
      [&hasError, &loop](const QString& error) {
        Q_UNUSED(error);
        hasError = true;
        loop.quit();
      });

  // Build system prompt for ROS2 error fixing
  QString systemPrompt = "You are a helpful ROS2 robotics expert assistant. "
                         "Provide clear, actionable advice for fixing issues. "
                         "When providing suggestions, format your response as JSON "
                         "with a 'suggestions' array containing objects with "
                         "'title', 'description', 'code', 'rationale', and 'confidence' fields.";

  // Send the request
  ollama.generateCompletion(prompt, systemPrompt, QStringList());

  // Start timeout
  timeout.start(30000);

  // Wait for response
  loop.exec();

  // Disconnect signals
  QObject::disconnect(tokenConn);
  QObject::disconnect(finishConn);
  QObject::disconnect(errorConn);

  if (!completed || hasError) {
    if (!completed && !hasError) {
      RCLCPP_WARN(rclcpp::get_logger("ros_weaver"),
                  "AI request timed out after 30 seconds");
    }
    return "";
  }

  RCLCPP_INFO(rclcpp::get_logger("ros_weaver"),
              "Received AI advice response (%d characters)", response.length());

  return response;
}

}  // namespace ros_weaver
