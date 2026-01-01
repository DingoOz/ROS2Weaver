#ifndef ROS_WEAVER_CORE_ARCHITECTURE_GENERATOR_HPP
#define ROS_WEAVER_CORE_ARCHITECTURE_GENERATOR_HPP

#include <QObject>
#include <QString>
#include <QList>
#include <QMap>
#include <QJsonObject>
#include <QPointF>

namespace ros_weaver {

class Project;

// A node in a generated architecture
struct GeneratedNode {
  QString name;
  QString packageName;
  QString description;
  QPointF position;
  QList<QPair<QString, QString>> inputPins;   // (dataType, name)
  QList<QPair<QString, QString>> outputPins;  // (dataType, name)
  QMap<QString, QString> parameters;
};

// A connection in a generated architecture
struct GeneratedConnection {
  QString sourceNode;
  int sourcePin;
  QString targetNode;
  int targetPin;
  QString topicName;
  QString messageType;
};

// A group in a generated architecture
struct GeneratedGroup {
  QString title;
  QList<QString> nodeNames;
  QString color;
};

// Result of architecture generation
struct ArchitectureGenerationResult {
  bool success = false;
  QString errorMessage;
  QString architectureName;
  QString description;
  QList<GeneratedNode> nodes;
  QList<GeneratedConnection> connections;
  QList<GeneratedGroup> groups;
  QString aiRationale;  // Explanation from AI
};

// Template types for pre-built architectures
enum class ArchitectureTemplate {
  SlamStack,
  Navigation2,
  Perception,
  Manipulation,
  Teleop,
  Custom
};

// Generates ROS2 architectures from templates or AI prompts
class ArchitectureGenerator : public QObject {
  Q_OBJECT

public:
  static ArchitectureGenerator& instance();

  // Prevent copying
  ArchitectureGenerator(const ArchitectureGenerator&) = delete;
  ArchitectureGenerator& operator=(const ArchitectureGenerator&) = delete;

  // Generate from a pre-built template
  ArchitectureGenerationResult generateFromTemplate(
      ArchitectureTemplate templateType,
      const QMap<QString, QString>& options = {});

  // Generate from a natural language description using AI
  ArchitectureGenerationResult generateFromDescription(
      const QString& description,
      const QString& context = "");

  // Suggest an architecture based on robot type and use case
  ArchitectureGenerationResult suggestArchitecture(
      const QString& robotType,
      const QString& useCase,
      const QStringList& sensors = {});

  // Apply a generated architecture to a project
  bool applyToProject(const ArchitectureGenerationResult& result, Project& project);

  // Get available template names
  QStringList availableTemplates() const;

  // Get template description
  QString templateDescription(ArchitectureTemplate templateType) const;

  // Check if AI backend is available for custom generation
  bool isAIAvailable() const;

signals:
  void generationStarted(const QString& description);
  void generationProgress(int percent, const QString& status);
  void generationCompleted(const ArchitectureGenerationResult& result);
  void generationFailed(const QString& error);

private:
  ArchitectureGenerator();
  ~ArchitectureGenerator() override = default;

  // Template generators
  ArchitectureGenerationResult generateSlamStack(const QMap<QString, QString>& options);
  ArchitectureGenerationResult generateNav2Stack(const QMap<QString, QString>& options);
  ArchitectureGenerationResult generatePerceptionStack(const QMap<QString, QString>& options);
  ArchitectureGenerationResult generateManipulationStack(const QMap<QString, QString>& options);
  ArchitectureGenerationResult generateTeleopStack(const QMap<QString, QString>& options);

  // Parse AI response into architecture
  ArchitectureGenerationResult parseAIResponse(const QString& response);

  // Build prompt for architecture generation
  QString buildPrompt(const QString& description, const QString& context) const;

  // Send to AI backend
  QString sendToAI(const QString& prompt);

  // Calculate node positions for layout
  void calculateLayout(ArchitectureGenerationResult& result);
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_ARCHITECTURE_GENERATOR_HPP
