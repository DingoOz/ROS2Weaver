#ifndef ROS_WEAVER_CORE_ARCHITECTURE_DOC_GENERATOR_HPP
#define ROS_WEAVER_CORE_ARCHITECTURE_DOC_GENERATOR_HPP

#include <QObject>
#include <QString>
#include <QStringList>

namespace ros_weaver {

class Project;
struct BlockData;
struct ConnectionData;
struct NodeGroupData;
struct PinData;
struct BlockParamData;

/**
 * @brief Output format for architecture documentation
 */
enum class DocFormat {
  Markdown,
  HTML,
  PDF
};

/**
 * @brief Options for architecture documentation generation
 */
struct DocGeneratorOptions {
  bool includeOverview = true;
  bool includeDiagram = true;
  bool includeNodes = true;
  bool includeTopics = true;
  bool includeParameters = true;
  bool includeMessageSchemas = false;
  bool includeLaunchConfig = true;
  bool includeGroups = true;
  QString diagramFormat = "mermaid";  // "mermaid", "plantuml", "graphviz"
  QString layoutDirection = "TB";
  QString customCss;  // For HTML output
};

/**
 * @brief Generates comprehensive architecture documentation from canvas data
 *
 * Supports output formats:
 * - Markdown: README-style documentation
 * - HTML: Standalone documentation site
 * - PDF: Print-ready documentation (via Qt PDF)
 */
class ArchitectureDocGenerator : public QObject {
  Q_OBJECT

public:
  static ArchitectureDocGenerator& instance();

  // Generate documentation as string
  QString generateMarkdown(const Project& project, const DocGeneratorOptions& options = DocGeneratorOptions());
  QString generateHTML(const Project& project, const DocGeneratorOptions& options = DocGeneratorOptions());

  // Generate and save to file
  bool exportToFile(const Project& project, const QString& filePath,
                    DocFormat format, const DocGeneratorOptions& options = DocGeneratorOptions());

  // Export to PDF (uses Qt's PDF writer)
  bool exportToPdf(const Project& project, const QString& filePath,
                   const DocGeneratorOptions& options = DocGeneratorOptions());

  // Get file extension for format
  static QString fileExtension(DocFormat format);
  static QString formatName(DocFormat format);
  static QString fileFilter(DocFormat format);

  // Get last error
  QString lastError() const { return lastError_; }

signals:
  void exportComplete(const QString& filePath);
  void exportError(const QString& error);
  void generationProgress(int percent, const QString& message);

private:
  ArchitectureDocGenerator(QObject* parent = nullptr);
  ~ArchitectureDocGenerator() = default;
  ArchitectureDocGenerator(const ArchitectureDocGenerator&) = delete;
  ArchitectureDocGenerator& operator=(const ArchitectureDocGenerator&) = delete;

  // Markdown section generators
  QString generateMarkdownHeader(const Project& project) const;
  QString generateMarkdownOverview(const Project& project) const;
  QString generateMarkdownDiagram(const Project& project, const DocGeneratorOptions& options) const;
  QString generateMarkdownNodes(const Project& project, const DocGeneratorOptions& options) const;
  QString generateMarkdownNodeSection(const BlockData& block, const Project& project,
                                       const DocGeneratorOptions& options) const;
  QString generateMarkdownTopics(const Project& project) const;
  QString generateMarkdownGroups(const Project& project) const;
  QString generateMarkdownLaunchConfig(const Project& project) const;
  QString generateMarkdownFooter() const;

  // HTML generation
  QString generateHTMLHeader(const Project& project, const DocGeneratorOptions& options) const;
  QString generateHTMLStyles(const DocGeneratorOptions& options) const;
  QString generateHTMLFooter() const;
  QString markdownToBasicHTML(const QString& markdown) const;

  // Table generators
  QString generateSubscriptionsTable(const BlockData& block) const;
  QString generatePublicationsTable(const BlockData& block) const;
  QString generateParametersTable(const BlockData& block) const;
  QString generateTopicsTable(const Project& project) const;

  // Diagram generation (reuses GraphExporter)
  QString generateMermaidDiagram(const Project& project, const DocGeneratorOptions& options) const;
  QString generatePlantUMLDiagram(const Project& project, const DocGeneratorOptions& options) const;

  // Utility
  QString sanitizeForMarkdown(const QString& text) const;
  QString extractMessageTypeName(const QString& fullType) const;
  QString formatPinDataType(const QString& dataType) const;
  QStringList getUniqueTopics(const Project& project) const;
  QMap<QString, QStringList> getTopicPublishers(const Project& project) const;
  QMap<QString, QStringList> getTopicSubscribers(const Project& project) const;
  QString getBlockName(const QUuid& blockId, const Project& project) const;

  QString lastError_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_ARCHITECTURE_DOC_GENERATOR_HPP
