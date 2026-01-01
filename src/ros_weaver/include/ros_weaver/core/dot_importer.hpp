#ifndef ROS_WEAVER_CORE_DOT_IMPORTER_HPP
#define ROS_WEAVER_CORE_DOT_IMPORTER_HPP

#include <QObject>
#include <QString>
#include <QStringList>
#include <QPointF>
#include <QMap>
#include <QList>
#include <QUuid>

namespace ros_weaver {

// Represents a node parsed from DOT
struct DotNode {
  QString id;
  QString label;
  QString shape;      // "ellipse" for nodes, "box" for topics
  QString url;        // URL attribute if present
  QPointF position;   // Position from pos attribute if present
  QMap<QString, QString> attributes;
};

// Represents an edge parsed from DOT
struct DotEdge {
  QString sourceId;
  QString targetId;
  QString label;      // Edge label (topic name)
  QString url;
  QMap<QString, QString> attributes;
};

// Result of parsing a DOT file
struct DotGraph {
  QString graphName;
  bool isDigraph = true;
  QList<DotNode> nodes;
  QList<DotEdge> edges;
  QMap<QString, QString> graphAttributes;
  QStringList errors;
  bool isValid = false;
};

// Importer for DOT files (rqt_graph output)
class DotImporter : public QObject {
  Q_OBJECT

public:
  static DotImporter& instance();

  // Prevent copying
  DotImporter(const DotImporter&) = delete;
  DotImporter& operator=(const DotImporter&) = delete;

  // Parse a DOT file
  DotGraph parseFile(const QString& filePath);

  // Parse DOT content from string
  DotGraph parseString(const QString& content);

  // Import DOT graph into current project
  // Returns the number of nodes created
  int importToCanvas(const DotGraph& graph, class WeaverCanvas* canvas);

  // Get last error message
  QString lastError() const { return lastError_; }

signals:
  void importStarted();
  void importProgress(int percent, const QString& message);
  void importCompleted(int nodesCreated, int connectionsCreated);
  void importFailed(const QString& error);

private:
  DotImporter();
  ~DotImporter() override = default;

  // Internal parsing helpers
  bool parseGraphHeader(const QString& line, DotGraph& graph);
  bool parseNode(const QString& line, DotGraph& graph);
  bool parseEdge(const QString& line, DotGraph& graph);
  bool parseAttribute(const QString& line, DotGraph& graph);
  QMap<QString, QString> parseAttributeList(const QString& attrStr);
  QString unquoteString(const QString& str);

  // Tokenizer for DOT parsing
  QStringList tokenize(const QString& content);

  // Convert rqt_graph node types to canvas representations
  bool isTopicNode(const DotNode& node) const;
  bool isActionNode(const DotNode& node) const;
  bool isServiceNode(const DotNode& node) const;

  QString lastError_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_DOT_IMPORTER_HPP
