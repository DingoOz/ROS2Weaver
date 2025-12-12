#ifndef ROS_WEAVER_CORE_CODE_GENERATOR_HPP
#define ROS_WEAVER_CORE_CODE_GENERATOR_HPP

#include <QObject>
#include <QString>
#include <QDir>
#include <QMap>

class QTextStream;

namespace ros_weaver {

class Project;
struct BlockData;
struct BlockParamData;
struct ConnectionData;
struct ParamDefinition;

// Code generation options
struct GeneratorOptions {
  QString outputPath;
  QString packageName;
  QString nodeName;
  bool generateLaunchFile = true;
  bool generateTests = false;
  bool useCppStyle = true;  // C++ vs Python
  QString rosDistro = "humble";
  QString maintainer = "user";
  QString maintainerEmail = "user@example.com";
  QString license = "Apache-2.0";
};

class CodeGenerator : public QObject {
  Q_OBJECT

public:
  explicit CodeGenerator(QObject* parent = nullptr);
  ~CodeGenerator() override;

  // Generate complete ROS2 package from project
  bool generatePackage(const Project& project, const GeneratorOptions& options);

  // Generate individual files
  QString generateCMakeLists(const Project& project, const GeneratorOptions& options);
  QString generatePackageXml(const Project& project, const GeneratorOptions& options);
  QString generateNodeCpp(const BlockData& block, const QList<ConnectionData>& connections,
                          const QList<ParamDefinition>& params);
  QString generateNodePython(const BlockData& block, const QList<ConnectionData>& connections,
                             const QList<ParamDefinition>& params);
  QString generateLaunchFile(const Project& project, const GeneratorOptions& options);
  QString generateParamsYaml(const Project& project);

  // Get last error
  QString lastError() const { return lastError_; }

signals:
  void generationStarted();
  void generationProgress(int percent, const QString& message);
  void generationFinished(bool success);
  void fileGenerated(const QString& filePath);

private:
  bool createDirectory(const QString& path);
  bool writeFile(const QString& path, const QString& content);

  // Template helpers
  QString indentCode(const QString& code, int spaces);
  QString toCamelCase(const QString& name);
  QString toSnakeCase(const QString& name);
  QString getMessageInclude(const QString& messageType);
  QString getMessageType(const QString& fullType);

  // YAML helpers
  void writeParamYaml(QTextStream& stream, const BlockParamData& param, int indentSpaces);
  QString formatYamlValue(const QVariant& value, const QString& type);

  QString lastError_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_CODE_GENERATOR_HPP
