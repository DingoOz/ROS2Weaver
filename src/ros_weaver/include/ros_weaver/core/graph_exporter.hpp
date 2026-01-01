#ifndef ROS_WEAVER_CORE_GRAPH_EXPORTER_HPP
#define ROS_WEAVER_CORE_GRAPH_EXPORTER_HPP

#include <QObject>
#include <QString>
#include <QStringList>

namespace ros_weaver {

class Project;

enum class ExportFormat {
  PlantUML,
  Mermaid,
  Graphviz
};

struct ExportOptions {
  bool includeMessageTypes = true;
  bool includeParameters = false;
  bool includeGroups = true;
  bool useNamespacePrefix = true;
  QString layoutDirection = "TB";  // TB (top-bottom), LR (left-right), BT, RL
  QString title;
};

class GraphExporter : public QObject {
  Q_OBJECT

public:
  static GraphExporter& instance();

  // Export to string
  QString exportToPlantUML(const Project& project, const ExportOptions& options = ExportOptions());
  QString exportToMermaid(const Project& project, const ExportOptions& options = ExportOptions());
  QString exportToGraphviz(const Project& project, const ExportOptions& options = ExportOptions());

  // Export to file
  bool exportToFile(const Project& project, const QString& filePath,
                    ExportFormat format, const ExportOptions& options = ExportOptions());

  // Get file extension for format
  static QString fileExtension(ExportFormat format);
  static QString formatName(ExportFormat format);
  static QString fileFilter(ExportFormat format);

signals:
  void exportComplete(const QString& filePath);
  void exportError(const QString& error);

private:
  GraphExporter(QObject* parent = nullptr);
  ~GraphExporter() = default;
  GraphExporter(const GraphExporter&) = delete;
  GraphExporter& operator=(const GraphExporter&) = delete;

  // PlantUML helpers
  QString generatePlantUMLHeader(const ExportOptions& options) const;
  QString generatePlantUMLNode(const struct BlockData& block, const ExportOptions& options) const;
  QString generatePlantUMLConnection(const struct ConnectionData& conn,
                                      const Project& project, const ExportOptions& options) const;
  QString generatePlantUMLGroup(const struct NodeGroupData& group,
                                 const Project& project, const ExportOptions& options) const;
  QString generatePlantUMLFooter() const;

  // Mermaid helpers
  QString generateMermaidHeader(const ExportOptions& options) const;
  QString generateMermaidNode(const struct BlockData& block, const ExportOptions& options) const;
  QString generateMermaidConnection(const struct ConnectionData& conn,
                                     const Project& project, const ExportOptions& options) const;
  QString generateMermaidGroup(const struct NodeGroupData& group,
                                const Project& project, const ExportOptions& options) const;

  // Graphviz helpers
  QString generateGraphvizHeader(const ExportOptions& options) const;
  QString generateGraphvizNode(const struct BlockData& block, const ExportOptions& options) const;
  QString generateGraphvizConnection(const struct ConnectionData& conn,
                                      const Project& project, const ExportOptions& options) const;
  QString generateGraphvizGroup(const struct NodeGroupData& group,
                                 const Project& project, const ExportOptions& options) const;
  QString generateGraphvizFooter() const;

  // Utility
  QString sanitizeId(const QString& name) const;
  QString escapeLabel(const QString& label) const;
  QString getBlockName(const QUuid& blockId, const Project& project) const;
  QString getPinLabel(const struct BlockData& block, int pinIndex, bool isOutput) const;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_GRAPH_EXPORTER_HPP
