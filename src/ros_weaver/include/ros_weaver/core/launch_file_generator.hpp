#ifndef ROS_WEAVER_CORE_LAUNCH_FILE_GENERATOR_HPP
#define ROS_WEAVER_CORE_LAUNCH_FILE_GENERATOR_HPP

#include <QObject>
#include <QString>
#include <QList>
#include <QMap>
#include <QUuid>

namespace ros_weaver {

class Project;
struct BlockData;
struct ConnectionData;

/**
 * @brief Launch file format options
 */
enum class LaunchFormat {
  Python,     // Python launch files (.launch.py)
  XML         // XML launch files (.launch.xml)
};

/**
 * @brief Node configuration for launch file generation
 */
struct LaunchNodeConfig {
  QString name;
  QString packageName;
  QString executableName;
  QString namespace_;
  QMap<QString, QVariant> parameters;
  QList<QPair<QString, QString>> remappings;  // (from, to)
  bool useComposition = false;
  QString composableContainer;
  bool respawn = false;
  double respawnDelay = 1.0;
  QString output = "screen";
  QStringList arguments;
};

/**
 * @brief Options for launch file generation
 */
struct LaunchGeneratorOptions {
  LaunchFormat format = LaunchFormat::Python;
  QString packageName;
  QString namespace_;  // Global namespace
  bool useSimTime = false;
  bool generateComposableContainer = false;
  QString containerName = "component_container";
  bool includeParamsFile = true;
  QString paramsFilePath = "config/params.yaml";
  bool generateArguments = true;
  QString description;
};

/**
 * @brief Generator for ROS2 launch files from canvas configurations
 *
 * Supports both Python and XML launch file formats with features:
 * - Node declarations with parameters
 * - Topic remappings from canvas connections
 * - Namespace configurations
 * - Composable node containers
 */
class LaunchFileGenerator : public QObject {
  Q_OBJECT

public:
  explicit LaunchFileGenerator(QObject* parent = nullptr);
  ~LaunchFileGenerator() override;

  /**
   * @brief Generate launch file from project
   * @param project The project containing blocks and connections
   * @param options Generation options
   * @return Generated launch file content
   */
  QString generate(const Project& project, const LaunchGeneratorOptions& options);

  /**
   * @brief Generate launch file for specific nodes
   * @param nodes List of node configurations
   * @param options Generation options
   * @return Generated launch file content
   */
  QString generate(const QList<LaunchNodeConfig>& nodes, const LaunchGeneratorOptions& options);

  /**
   * @brief Extract node configurations from project
   * @param project The project
   * @param options Generation options
   * @return List of node configurations
   */
  QList<LaunchNodeConfig> extractNodeConfigs(const Project& project,
                                              const LaunchGeneratorOptions& options);

  /**
   * @brief Get file extension for format
   */
  static QString fileExtension(LaunchFormat format);

  /**
   * @brief Get last error message
   */
  QString lastError() const { return lastError_; }

signals:
  void generationProgress(int percent, const QString& message);

private:
  // Python format generation
  QString generatePython(const QList<LaunchNodeConfig>& nodes,
                          const LaunchGeneratorOptions& options);
  QString generatePythonNode(const LaunchNodeConfig& node, int indent);
  QString generatePythonComposableNode(const LaunchNodeConfig& node, int indent);
  QString generatePythonContainer(const QString& containerName, int indent);

  // XML format generation
  QString generateXML(const QList<LaunchNodeConfig>& nodes,
                       const LaunchGeneratorOptions& options);
  QString generateXMLNode(const LaunchNodeConfig& node, int indent);
  QString generateXMLComposableNode(const LaunchNodeConfig& node, int indent);
  QString generateXMLContainer(const QString& containerName, int indent);

  // Helpers
  QString indent(int spaces) const;
  QString escapeXml(const QString& str) const;
  QString formatPythonValue(const QVariant& value) const;
  QString formatXmlValue(const QVariant& value) const;
  QString toSnakeCase(const QString& name) const;

  // Build remappings from connections
  void buildRemappings(const Project& project,
                       QMap<QUuid, QList<QPair<QString, QString>>>& remappings);

  QString lastError_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_LAUNCH_FILE_GENERATOR_HPP
