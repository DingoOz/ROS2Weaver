#ifndef ROS_WEAVER_CORE_CARET_IMPORTER_HPP
#define ROS_WEAVER_CORE_CARET_IMPORTER_HPP

#include <QObject>
#include <QString>
#include <QStringList>
#include <QPointF>
#include <QMap>
#include <QList>
#include <QVariant>

namespace ros_weaver {

// CARET node definition
struct CaretNode {
  QString name;              // Node name
  QString executorType;      // Executor type (single_threaded, multi_threaded)
  QStringList callbackGroups;
  QMap<QString, QVariant> parameters;
};

// CARET callback definition
struct CaretCallback {
  QString name;
  QString type;              // timer, subscription, service
  QString topicName;         // For subscription callbacks
  QString serviceName;       // For service callbacks
  double period = 0.0;       // For timer callbacks (in ms)
  QString callbackGroup;
};

// CARET path (communication path)
struct CaretPath {
  QString name;
  QStringList nodeNames;     // Ordered list of nodes in the path
  QStringList topicNames;    // Topics connecting the nodes
};

// CARET executor definition
struct CaretExecutor {
  QString name;
  QString type;              // single_threaded_executor, etc.
  QStringList nodeNames;     // Nodes managed by this executor
};

// Complete CARET architecture
struct CaretArchitecture {
  QString name;
  QList<CaretNode> nodes;
  QList<CaretCallback> callbacks;
  QList<CaretPath> paths;
  QList<CaretExecutor> executors;
  QMap<QString, QVariant> metadata;
  QStringList errors;
  bool isValid = false;
};

// Importer for CARET architecture YAML files
class CaretImporter : public QObject {
  Q_OBJECT

public:
  static CaretImporter& instance();

  // Prevent copying
  CaretImporter(const CaretImporter&) = delete;
  CaretImporter& operator=(const CaretImporter&) = delete;

  // Parse a CARET architecture YAML file
  CaretArchitecture parseFile(const QString& filePath);

  // Parse CARET YAML content from string
  CaretArchitecture parseString(const QString& content);

  // Import CARET architecture into canvas
  int importToCanvas(const CaretArchitecture& arch, class WeaverCanvas* canvas);

  // Get last error
  QString lastError() const { return lastError_; }

signals:
  void importStarted();
  void importProgress(int percent, const QString& message);
  void importCompleted(int nodesCreated, int connectionsCreated);
  void importFailed(const QString& error);

private:
  CaretImporter();
  ~CaretImporter() override = default;

  // Internal parsing helpers
  CaretNode parseNode(const QVariant& nodeData);
  CaretCallback parseCallback(const QVariant& callbackData);
  CaretPath parsePath(const QVariant& pathData);
  CaretExecutor parseExecutor(const QVariant& executorData);

  QString lastError_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_CARET_IMPORTER_HPP
