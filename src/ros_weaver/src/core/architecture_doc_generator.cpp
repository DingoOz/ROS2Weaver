#include "ros_weaver/core/architecture_doc_generator.hpp"
#include "ros_weaver/core/project.hpp"
#include "ros_weaver/core/graph_exporter.hpp"

#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QRegularExpression>
#include <QTextDocument>
#include <QPrinter>
#include <QMap>
#include <QSet>

namespace ros_weaver {

ArchitectureDocGenerator& ArchitectureDocGenerator::instance() {
  static ArchitectureDocGenerator instance;
  return instance;
}

ArchitectureDocGenerator::ArchitectureDocGenerator(QObject* parent)
    : QObject(parent) {
}

QString ArchitectureDocGenerator::generateMarkdown(const Project& project,
                                                    const DocGeneratorOptions& options) {
  QString output;
  QTextStream stream(&output);

  emit generationProgress(0, tr("Generating header..."));
  stream << generateMarkdownHeader(project);

  if (options.includeOverview) {
    emit generationProgress(10, tr("Generating overview..."));
    stream << generateMarkdownOverview(project);
  }

  if (options.includeDiagram) {
    emit generationProgress(20, tr("Generating diagram..."));
    stream << generateMarkdownDiagram(project, options);
  }

  if (options.includeGroups && !project.nodeGroups().isEmpty()) {
    emit generationProgress(30, tr("Generating groups..."));
    stream << generateMarkdownGroups(project);
  }

  if (options.includeNodes) {
    emit generationProgress(40, tr("Generating nodes..."));
    stream << generateMarkdownNodes(project, options);
  }

  if (options.includeTopics) {
    emit generationProgress(70, tr("Generating topics..."));
    stream << generateMarkdownTopics(project);
  }

  if (options.includeLaunchConfig) {
    emit generationProgress(90, tr("Generating launch configuration..."));
    stream << generateMarkdownLaunchConfig(project);
  }

  stream << generateMarkdownFooter();
  emit generationProgress(100, tr("Complete"));

  return output;
}

QString ArchitectureDocGenerator::generateHTML(const Project& project,
                                                const DocGeneratorOptions& options) {
  QString output;
  QTextStream stream(&output);

  stream << generateHTMLHeader(project, options);

  // Convert markdown content to HTML
  QString markdownContent = generateMarkdown(project, options);
  stream << markdownToBasicHTML(markdownContent);

  stream << generateHTMLFooter();

  return output;
}

bool ArchitectureDocGenerator::exportToFile(const Project& project, const QString& filePath,
                                             DocFormat format, const DocGeneratorOptions& options) {
  QString content;

  switch (format) {
    case DocFormat::Markdown:
      content = generateMarkdown(project, options);
      break;
    case DocFormat::HTML:
      content = generateHTML(project, options);
      break;
    case DocFormat::PDF:
      return exportToPdf(project, filePath, options);
  }

  QFile file(filePath);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    lastError_ = QString("Failed to open file for writing: %1").arg(filePath);
    emit exportError(lastError_);
    return false;
  }

  QTextStream stream(&file);
  stream << content;
  file.close();

  emit exportComplete(filePath);
  return true;
}

bool ArchitectureDocGenerator::exportToPdf(const Project& project, const QString& filePath,
                                            const DocGeneratorOptions& options) {
  QString htmlContent = generateHTML(project, options);

  QTextDocument doc;
  doc.setHtml(htmlContent);

  QPrinter printer(QPrinter::HighResolution);
  printer.setOutputFormat(QPrinter::PdfFormat);
  printer.setOutputFileName(filePath);
  printer.setPageSize(QPageSize(QPageSize::A4));
  printer.setPageMargins(QMarginsF(15, 15, 15, 15), QPageLayout::Millimeter);

  doc.print(&printer);

  emit exportComplete(filePath);
  return true;
}

QString ArchitectureDocGenerator::fileExtension(DocFormat format) {
  switch (format) {
    case DocFormat::Markdown:
      return "md";
    case DocFormat::HTML:
      return "html";
    case DocFormat::PDF:
      return "pdf";
  }
  return "txt";
}

QString ArchitectureDocGenerator::formatName(DocFormat format) {
  switch (format) {
    case DocFormat::Markdown:
      return "Markdown";
    case DocFormat::HTML:
      return "HTML";
    case DocFormat::PDF:
      return "PDF";
  }
  return "Unknown";
}

QString ArchitectureDocGenerator::fileFilter(DocFormat format) {
  switch (format) {
    case DocFormat::Markdown:
      return "Markdown Files (*.md *.markdown)";
    case DocFormat::HTML:
      return "HTML Files (*.html *.htm)";
    case DocFormat::PDF:
      return "PDF Files (*.pdf)";
  }
  return "All Files (*)";
}

// Markdown section generators

QString ArchitectureDocGenerator::generateMarkdownHeader(const Project& project) const {
  QString output;
  QTextStream stream(&output);

  const auto& meta = project.metadata();
  QString title = meta.name.isEmpty() ? "ROS2 Architecture" : meta.name;

  stream << "# " << title << " Architecture\n\n";

  // Metadata table
  stream << "| Property | Value |\n";
  stream << "|----------|-------|\n";

  if (!meta.version.isEmpty()) {
    stream << "| Version | " << meta.version << " |\n";
  }
  if (!meta.author.isEmpty()) {
    stream << "| Author | " << meta.author << " |\n";
  }
  if (!meta.rosDistro.isEmpty()) {
    stream << "| ROS Distribution | " << meta.rosDistro << " |\n";
  }
  stream << "| Generated | " << QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm") << " |\n";
  stream << "| Nodes | " << project.blocks().size() << " |\n";
  stream << "| Connections | " << project.connections().size() << " |\n";

  stream << "\n";

  return output;
}

QString ArchitectureDocGenerator::generateMarkdownOverview(const Project& project) const {
  QString output;
  QTextStream stream(&output);

  stream << "## Overview\n\n";

  const auto& meta = project.metadata();
  if (!meta.description.isEmpty()) {
    stream << meta.description << "\n\n";
  } else {
    // Generate a basic description
    stream << "This document describes the ROS2 architecture consisting of "
           << project.blocks().size() << " nodes and "
           << project.connections().size() << " topic connections.\n\n";
  }

  // Quick stats
  int inputCount = 0;
  int outputCount = 0;
  int paramCount = 0;

  for (const auto& block : project.blocks()) {
    inputCount += block.inputPins.size();
    outputCount += block.outputPins.size();
    paramCount += block.parameters.size();
  }

  stream << "### Quick Statistics\n\n";
  stream << "- **Total Subscriptions:** " << inputCount << "\n";
  stream << "- **Total Publications:** " << outputCount << "\n";
  stream << "- **Total Parameters:** " << paramCount << "\n";
  stream << "- **Node Groups:** " << project.nodeGroups().size() << "\n";
  stream << "\n";

  return output;
}

QString ArchitectureDocGenerator::generateMarkdownDiagram(const Project& project,
                                                           const DocGeneratorOptions& options) const {
  QString output;
  QTextStream stream(&output);

  stream << "## System Diagram\n\n";

  if (options.diagramFormat == "mermaid") {
    stream << "```mermaid\n";
    stream << generateMermaidDiagram(project, options);
    stream << "```\n\n";
  } else if (options.diagramFormat == "plantuml") {
    stream << "```plantuml\n";
    stream << generatePlantUMLDiagram(project, options);
    stream << "```\n\n";
  } else {
    // Default to mermaid
    stream << "```mermaid\n";
    stream << generateMermaidDiagram(project, options);
    stream << "```\n\n";
  }

  return output;
}

QString ArchitectureDocGenerator::generateMarkdownGroups(const Project& project) const {
  QString output;
  QTextStream stream(&output);

  stream << "## Node Groups\n\n";

  for (const auto& group : project.nodeGroups()) {
    stream << "### " << group.title << "\n\n";

    if (!group.containedNodeIds.isEmpty()) {
      stream << "**Contained Nodes:**\n\n";
      for (const auto& nodeId : group.containedNodeIds) {
        QString nodeName = getBlockName(nodeId, project);
        if (!nodeName.isEmpty()) {
          stream << "- " << nodeName << "\n";
        }
      }
      stream << "\n";
    }
  }

  return output;
}

QString ArchitectureDocGenerator::generateMarkdownNodes(const Project& project,
                                                         const DocGeneratorOptions& options) const {
  QString output;
  QTextStream stream(&output);

  stream << "## Nodes\n\n";

  for (const auto& block : project.blocks()) {
    stream << generateMarkdownNodeSection(block, project, options);
  }

  return output;
}

QString ArchitectureDocGenerator::generateMarkdownNodeSection(const BlockData& block,
                                                               const Project& project,
                                                               const DocGeneratorOptions& options) const {
  Q_UNUSED(project)

  QString output;
  QTextStream stream(&output);

  stream << "### " << block.name << "\n\n";

  // Node info
  stream << "- **Package:** " << block.name << "\n";
  stream << "- **Subscriptions:** " << block.inputPins.size() << "\n";
  stream << "- **Publications:** " << block.outputPins.size() << "\n";
  if (!block.parameters.isEmpty()) {
    stream << "- **Parameters:** " << block.parameters.size() << "\n";
  }
  stream << "\n";

  // Subscriptions table
  if (!block.inputPins.isEmpty()) {
    stream << "#### Subscriptions\n\n";
    stream << generateSubscriptionsTable(block);
    stream << "\n";
  }

  // Publications table
  if (!block.outputPins.isEmpty()) {
    stream << "#### Publications\n\n";
    stream << generatePublicationsTable(block);
    stream << "\n";
  }

  // Parameters table
  if (options.includeParameters && !block.parameters.isEmpty()) {
    stream << "#### Parameters\n\n";
    stream << generateParametersTable(block);
    stream << "\n";
  }

  stream << "---\n\n";

  return output;
}

QString ArchitectureDocGenerator::generateMarkdownTopics(const Project& project) const {
  QString output;
  QTextStream stream(&output);

  stream << "## Topics\n\n";

  QStringList topics = getUniqueTopics(project);
  if (topics.isEmpty()) {
    stream << "No topics defined.\n\n";
    return output;
  }

  QMap<QString, QStringList> publishers = getTopicPublishers(project);
  QMap<QString, QStringList> subscribers = getTopicSubscribers(project);

  stream << "| Topic | Message Type | Publishers | Subscribers |\n";
  stream << "|-------|--------------|------------|-------------|\n";

  // Collect topics from all pins
  QMap<QString, QString> topicTypes;
  for (const auto& block : project.blocks()) {
    for (const auto& pin : block.outputPins) {
      if (!pin.name.isEmpty() && pin.dataType == "topic") {
        topicTypes[pin.name] = pin.messageType;
      }
    }
    for (const auto& pin : block.inputPins) {
      if (!pin.name.isEmpty() && pin.dataType == "topic") {
        if (!topicTypes.contains(pin.name)) {
          topicTypes[pin.name] = pin.messageType;
        }
      }
    }
  }

  for (auto it = topicTypes.begin(); it != topicTypes.end(); ++it) {
    QString topic = it.key();
    QString msgType = extractMessageTypeName(it.value());
    QString pubs = publishers.value(topic).join(", ");
    QString subs = subscribers.value(topic).join(", ");

    stream << "| `" << topic << "` | " << msgType << " | " << pubs << " | " << subs << " |\n";
  }

  stream << "\n";

  return output;
}

QString ArchitectureDocGenerator::generateMarkdownLaunchConfig(const Project& project) const {
  QString output;
  QTextStream stream(&output);

  stream << "## Launch Configuration\n\n";
  stream << "Example Python launch file for this architecture:\n\n";

  stream << "```python\n";
  stream << "from launch import LaunchDescription\n";
  stream << "from launch_ros.actions import Node\n\n";
  stream << "def generate_launch_description():\n";
  stream << "    return LaunchDescription([\n";

  for (int i = 0; i < project.blocks().size(); ++i) {
    const auto& block = project.blocks()[i];
    stream << "        Node(\n";
    stream << "            package='" << block.name << "',\n";
    stream << "            executable='" << block.name << "_node',\n";
    stream << "            name='" << block.name << "',\n";

    // Parameters
    if (!block.parameters.isEmpty()) {
      stream << "            parameters=[{\n";
      for (int j = 0; j < block.parameters.size(); ++j) {
        const auto& param = block.parameters[j];
        QString value;
        if (param.type == "string") {
          value = QString("'%1'").arg(param.currentValue.toString());
        } else {
          value = param.currentValue.toString();
        }
        stream << "                '" << param.name << "': " << value;
        if (j < block.parameters.size() - 1) {
          stream << ",";
        }
        stream << "\n";
      }
      stream << "            }],\n";
    }

    stream << "            output='screen'\n";
    stream << "        )";
    if (i < project.blocks().size() - 1) {
      stream << ",";
    }
    stream << "\n";
  }

  stream << "    ])\n";
  stream << "```\n\n";

  return output;
}

QString ArchitectureDocGenerator::generateMarkdownFooter() const {
  QString output;
  QTextStream stream(&output);

  stream << "---\n\n";
  stream << "*Generated by ROS2Weaver - Architecture Documentation Generator*\n";

  return output;
}

// Table generators

QString ArchitectureDocGenerator::generateSubscriptionsTable(const BlockData& block) const {
  QString output;
  QTextStream stream(&output);

  stream << "| Topic | Message Type | Data Type |\n";
  stream << "|-------|--------------|----------|\n";

  for (const auto& pin : block.inputPins) {
    stream << "| `" << pin.name << "` | "
           << extractMessageTypeName(pin.messageType) << " | "
           << formatPinDataType(pin.dataType) << " |\n";
  }

  return output;
}

QString ArchitectureDocGenerator::generatePublicationsTable(const BlockData& block) const {
  QString output;
  QTextStream stream(&output);

  stream << "| Topic | Message Type | Data Type |\n";
  stream << "|-------|--------------|----------|\n";

  for (const auto& pin : block.outputPins) {
    stream << "| `" << pin.name << "` | "
           << extractMessageTypeName(pin.messageType) << " | "
           << formatPinDataType(pin.dataType) << " |\n";
  }

  return output;
}

QString ArchitectureDocGenerator::generateParametersTable(const BlockData& block) const {
  QString output;
  QTextStream stream(&output);

  stream << "| Name | Type | Default | Description |\n";
  stream << "|------|------|---------|-------------|\n";

  for (const auto& param : block.parameters) {
    QString defaultVal = param.defaultValue.toString();
    if (defaultVal.isEmpty()) {
      defaultVal = "-";
    }
    QString desc = param.description.isEmpty() ? "-" : param.description;

    stream << "| `" << param.name << "` | "
           << param.type << " | "
           << defaultVal << " | "
           << desc << " |\n";
  }

  return output;
}

// HTML generation

QString ArchitectureDocGenerator::generateHTMLHeader(const Project& project,
                                                      const DocGeneratorOptions& options) const {
  QString output;
  QTextStream stream(&output);

  const auto& meta = project.metadata();
  QString title = meta.name.isEmpty() ? "ROS2 Architecture" : meta.name;

  stream << "<!DOCTYPE html>\n";
  stream << "<html lang=\"en\">\n";
  stream << "<head>\n";
  stream << "  <meta charset=\"UTF-8\">\n";
  stream << "  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n";
  stream << "  <title>" << title << " - Architecture Documentation</title>\n";
  stream << generateHTMLStyles(options);
  stream << "</head>\n";
  stream << "<body>\n";
  stream << "<div class=\"container\">\n";

  return output;
}

QString ArchitectureDocGenerator::generateHTMLStyles(const DocGeneratorOptions& options) const {
  QString output;
  QTextStream stream(&output);

  stream << "<style>\n";
  stream << "  :root {\n";
  stream << "    --primary-color: #1976D2;\n";
  stream << "    --secondary-color: #424242;\n";
  stream << "    --background-color: #FAFAFA;\n";
  stream << "    --card-background: #FFFFFF;\n";
  stream << "    --border-color: #E0E0E0;\n";
  stream << "    --code-background: #F5F5F5;\n";
  stream << "  }\n";
  stream << "  body {\n";
  stream << "    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, sans-serif;\n";
  stream << "    line-height: 1.6;\n";
  stream << "    color: var(--secondary-color);\n";
  stream << "    background-color: var(--background-color);\n";
  stream << "    margin: 0;\n";
  stream << "    padding: 0;\n";
  stream << "  }\n";
  stream << "  .container {\n";
  stream << "    max-width: 1200px;\n";
  stream << "    margin: 0 auto;\n";
  stream << "    padding: 20px 40px;\n";
  stream << "  }\n";
  stream << "  h1 {\n";
  stream << "    color: var(--primary-color);\n";
  stream << "    border-bottom: 3px solid var(--primary-color);\n";
  stream << "    padding-bottom: 10px;\n";
  stream << "  }\n";
  stream << "  h2 {\n";
  stream << "    color: var(--primary-color);\n";
  stream << "    margin-top: 40px;\n";
  stream << "    border-bottom: 1px solid var(--border-color);\n";
  stream << "    padding-bottom: 5px;\n";
  stream << "  }\n";
  stream << "  h3 {\n";
  stream << "    color: var(--secondary-color);\n";
  stream << "    margin-top: 30px;\n";
  stream << "  }\n";
  stream << "  table {\n";
  stream << "    border-collapse: collapse;\n";
  stream << "    width: 100%;\n";
  stream << "    margin: 15px 0;\n";
  stream << "    background: var(--card-background);\n";
  stream << "    box-shadow: 0 1px 3px rgba(0,0,0,0.1);\n";
  stream << "  }\n";
  stream << "  th, td {\n";
  stream << "    border: 1px solid var(--border-color);\n";
  stream << "    padding: 12px;\n";
  stream << "    text-align: left;\n";
  stream << "  }\n";
  stream << "  th {\n";
  stream << "    background-color: var(--primary-color);\n";
  stream << "    color: white;\n";
  stream << "  }\n";
  stream << "  tr:nth-child(even) {\n";
  stream << "    background-color: #F5F5F5;\n";
  stream << "  }\n";
  stream << "  code {\n";
  stream << "    background-color: var(--code-background);\n";
  stream << "    padding: 2px 6px;\n";
  stream << "    border-radius: 3px;\n";
  stream << "    font-family: 'Consolas', 'Monaco', monospace;\n";
  stream << "    font-size: 0.9em;\n";
  stream << "  }\n";
  stream << "  pre {\n";
  stream << "    background-color: #263238;\n";
  stream << "    color: #ECEFF1;\n";
  stream << "    padding: 20px;\n";
  stream << "    border-radius: 5px;\n";
  stream << "    overflow-x: auto;\n";
  stream << "  }\n";
  stream << "  pre code {\n";
  stream << "    background: none;\n";
  stream << "    padding: 0;\n";
  stream << "    color: inherit;\n";
  stream << "  }\n";
  stream << "  hr {\n";
  stream << "    border: none;\n";
  stream << "    border-top: 1px solid var(--border-color);\n";
  stream << "    margin: 30px 0;\n";
  stream << "  }\n";
  stream << "  ul {\n";
  stream << "    padding-left: 25px;\n";
  stream << "  }\n";
  stream << "  li {\n";
  stream << "    margin: 5px 0;\n";
  stream << "  }\n";
  stream << "  .mermaid {\n";
  stream << "    background: white;\n";
  stream << "    padding: 20px;\n";
  stream << "    border-radius: 5px;\n";
  stream << "    margin: 20px 0;\n";
  stream << "  }\n";
  stream << "  .plantuml-diagram {\n";
  stream << "    background: white;\n";
  stream << "    padding: 20px;\n";
  stream << "    border-radius: 5px;\n";
  stream << "    margin: 20px 0;\n";
  stream << "    text-align: center;\n";
  stream << "  }\n";
  stream << "  .plantuml-diagram img {\n";
  stream << "    max-width: 100%;\n";
  stream << "    height: auto;\n";
  stream << "  }\n";
  stream << "  .plantuml-diagram pre {\n";
  stream << "    display: none;\n";
  stream << "  }\n";

  // Add custom CSS if provided
  if (!options.customCss.isEmpty()) {
    stream << options.customCss << "\n";
  }

  stream << "</style>\n";

  // Include Mermaid for diagram rendering
  stream << "<script src=\"https://cdn.jsdelivr.net/npm/mermaid/dist/mermaid.min.js\"></script>\n";
  stream << "<script>mermaid.initialize({startOnLoad:true});</script>\n";

  // Include pako for PlantUML deflate compression
  stream << "<script src=\"https://cdn.jsdelivr.net/npm/pako@2.1.0/dist/pako.min.js\"></script>\n";

  // PlantUML encoding and rendering script
  stream << "<script>\n";
  stream << "function encodePlantUML(text) {\n";
  stream << "  // PlantUML uses deflate compression then custom base64 encoding\n";
  stream << "  const deflated = pako.deflateRaw(text, { level: 9, to: 'string' });\n";
  stream << "  const uint8 = new Uint8Array(deflated.length);\n";
  stream << "  for (let i = 0; i < deflated.length; i++) {\n";
  stream << "    uint8[i] = deflated.charCodeAt(i);\n";
  stream << "  }\n";
  stream << "  return encode64(uint8);\n";
  stream << "}\n";
  stream << "function encode64(data) {\n";
  stream << "  // PlantUML uses a custom base64 alphabet\n";
  stream << "  const chars = '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz-_';\n";
  stream << "  let result = '';\n";
  stream << "  for (let i = 0; i < data.length; i += 3) {\n";
  stream << "    const b1 = data[i] || 0;\n";
  stream << "    const b2 = data[i + 1] || 0;\n";
  stream << "    const b3 = data[i + 2] || 0;\n";
  stream << "    result += chars[b1 >> 2];\n";
  stream << "    result += chars[((b1 & 0x3) << 4) | (b2 >> 4)];\n";
  stream << "    result += chars[((b2 & 0xF) << 2) | (b3 >> 6)];\n";
  stream << "    result += chars[b3 & 0x3F];\n";
  stream << "  }\n";
  stream << "  return result;\n";
  stream << "}\n";
  stream << "function renderPlantUML() {\n";
  stream << "  document.querySelectorAll('.plantuml-diagram').forEach(function(div) {\n";
  stream << "    const pre = div.querySelector('pre');\n";
  stream << "    if (!pre) return;\n";
  stream << "    const code = pre.textContent.trim();\n";
  stream << "    if (!code) return;\n";
  stream << "    try {\n";
  stream << "      const encoded = encodePlantUML(code);\n";
  stream << "      const img = document.createElement('img');\n";
  stream << "      img.src = 'https://www.plantuml.com/plantuml/svg/' + encoded;\n";
  stream << "      img.alt = 'PlantUML Diagram';\n";
  stream << "      img.onerror = function() {\n";
  stream << "        pre.style.display = 'block';\n";
  stream << "        this.remove();\n";
  stream << "      };\n";
  stream << "      div.insertBefore(img, pre);\n";
  stream << "    } catch (e) {\n";
  stream << "      console.error('PlantUML encoding error:', e);\n";
  stream << "      pre.style.display = 'block';\n";
  stream << "    }\n";
  stream << "  });\n";
  stream << "}\n";
  stream << "if (document.readyState === 'loading') {\n";
  stream << "  document.addEventListener('DOMContentLoaded', renderPlantUML);\n";
  stream << "} else {\n";
  stream << "  renderPlantUML();\n";
  stream << "}\n";
  stream << "</script>\n";

  return output;
}

QString ArchitectureDocGenerator::generateHTMLFooter() const {
  return "</div>\n</body>\n</html>\n";
}

QString ArchitectureDocGenerator::markdownToBasicHTML(const QString& markdown) const {
  QString html = markdown;

  // Headers
  html.replace(QRegularExpression("^### (.+)$", QRegularExpression::MultilineOption), "<h3>\\1</h3>");
  html.replace(QRegularExpression("^## (.+)$", QRegularExpression::MultilineOption), "<h2>\\1</h2>");
  html.replace(QRegularExpression("^# (.+)$", QRegularExpression::MultilineOption), "<h1>\\1</h1>");

  // Bold and italic
  html.replace(QRegularExpression("\\*\\*(.+?)\\*\\*"), "<strong>\\1</strong>");
  html.replace(QRegularExpression("\\*(.+?)\\*"), "<em>\\1</em>");

  // Code blocks
  html.replace(QRegularExpression("```mermaid\\n([\\s\\S]*?)```"),
               "<div class=\"mermaid\">\\1</div>");
  html.replace(QRegularExpression("```python\\n([\\s\\S]*?)```"),
               "<pre><code class=\"language-python\">\\1</code></pre>");
  html.replace(QRegularExpression("```plantuml\\n([\\s\\S]*?)```"),
               "<div class=\"plantuml-diagram\"><pre>\\1</pre></div>");
  html.replace(QRegularExpression("```\\n([\\s\\S]*?)```"),
               "<pre><code>\\1</code></pre>");

  // Inline code
  html.replace(QRegularExpression("`([^`]+)`"), "<code>\\1</code>");

  // Tables (basic support)
  QRegularExpression tableRowRegex("^\\|(.+)\\|$", QRegularExpression::MultilineOption);
  QRegularExpressionMatchIterator i = tableRowRegex.globalMatch(html);

  // Lists
  html.replace(QRegularExpression("^- (.+)$", QRegularExpression::MultilineOption), "<li>\\1</li>");
  html.replace(QRegularExpression("(<li>.*</li>\\n)+"), "<ul>\\0</ul>");

  // Horizontal rules
  html.replace(QRegularExpression("^---$", QRegularExpression::MultilineOption), "<hr>");

  // Line breaks to paragraphs (simplified)
  html.replace(QRegularExpression("\\n\\n"), "</p><p>");
  html = "<p>" + html + "</p>";

  // Clean up empty paragraphs
  html.replace(QRegularExpression("<p>\\s*</p>"), "");
  html.replace(QRegularExpression("<p>(<h[123]>)"), "\\1");
  html.replace(QRegularExpression("(</h[123]>)</p>"), "\\1");
  html.replace(QRegularExpression("<p>(<ul>)"), "\\1");
  html.replace(QRegularExpression("(</ul>)</p>"), "\\1");
  html.replace(QRegularExpression("<p>(<pre>)"), "\\1");
  html.replace(QRegularExpression("(</pre>)</p>"), "\\1");
  html.replace(QRegularExpression("<p>(<div)"), "\\1");
  html.replace(QRegularExpression("(</div>)</p>"), "\\1");
  html.replace(QRegularExpression("<p>(<table)"), "\\1");
  html.replace(QRegularExpression("(</table>)</p>"), "\\1");
  html.replace(QRegularExpression("<p>(<hr>)"), "\\1");

  return html;
}

// Diagram generation

QString ArchitectureDocGenerator::generateMermaidDiagram(const Project& project,
                                                          const DocGeneratorOptions& options) const {
  ExportOptions expOptions;
  expOptions.includeMessageTypes = true;
  expOptions.includeGroups = options.includeGroups;
  expOptions.layoutDirection = options.layoutDirection;

  return GraphExporter::instance().exportToMermaid(project, expOptions);
}

QString ArchitectureDocGenerator::generatePlantUMLDiagram(const Project& project,
                                                           const DocGeneratorOptions& options) const {
  ExportOptions expOptions;
  expOptions.includeMessageTypes = true;
  expOptions.includeGroups = options.includeGroups;
  expOptions.layoutDirection = options.layoutDirection;

  return GraphExporter::instance().exportToPlantUML(project, expOptions);
}

// Utility functions

QString ArchitectureDocGenerator::sanitizeForMarkdown(const QString& text) const {
  QString sanitized = text;
  sanitized.replace("|", "\\|");
  sanitized.replace("*", "\\*");
  sanitized.replace("_", "\\_");
  sanitized.replace("`", "\\`");
  return sanitized;
}

QString ArchitectureDocGenerator::extractMessageTypeName(const QString& fullType) const {
  if (fullType.isEmpty()) {
    return "-";
  }
  int lastSlash = fullType.lastIndexOf('/');
  if (lastSlash >= 0) {
    return fullType.mid(lastSlash + 1);
  }
  return fullType;
}

QString ArchitectureDocGenerator::formatPinDataType(const QString& dataType) const {
  if (dataType == "topic") return "Topic";
  if (dataType == "service") return "Service";
  if (dataType == "action") return "Action";
  if (dataType == "parameter") return "Parameter";
  return dataType;
}

QStringList ArchitectureDocGenerator::getUniqueTopics(const Project& project) const {
  QSet<QString> topics;

  for (const auto& block : project.blocks()) {
    for (const auto& pin : block.inputPins) {
      if (!pin.name.isEmpty() && pin.dataType == "topic") {
        topics.insert(pin.name);
      }
    }
    for (const auto& pin : block.outputPins) {
      if (!pin.name.isEmpty() && pin.dataType == "topic") {
        topics.insert(pin.name);
      }
    }
  }

  return topics.values();
}

QMap<QString, QStringList> ArchitectureDocGenerator::getTopicPublishers(const Project& project) const {
  QMap<QString, QStringList> publishers;

  for (const auto& block : project.blocks()) {
    for (const auto& pin : block.outputPins) {
      if (!pin.name.isEmpty() && pin.dataType == "topic") {
        publishers[pin.name].append(block.name);
      }
    }
  }

  return publishers;
}

QMap<QString, QStringList> ArchitectureDocGenerator::getTopicSubscribers(const Project& project) const {
  QMap<QString, QStringList> subscribers;

  for (const auto& block : project.blocks()) {
    for (const auto& pin : block.inputPins) {
      if (!pin.name.isEmpty() && pin.dataType == "topic") {
        subscribers[pin.name].append(block.name);
      }
    }
  }

  return subscribers;
}

QString ArchitectureDocGenerator::getBlockName(const QUuid& blockId, const Project& project) const {
  for (const auto& block : project.blocks()) {
    if (block.id == blockId) {
      return block.name;
    }
  }
  return QString();
}

}  // namespace ros_weaver
