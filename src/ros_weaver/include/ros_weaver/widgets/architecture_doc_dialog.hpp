#ifndef ROS_WEAVER_WIDGETS_ARCHITECTURE_DOC_DIALOG_HPP
#define ROS_WEAVER_WIDGETS_ARCHITECTURE_DOC_DIALOG_HPP

#include <QDialog>
#include <QPlainTextEdit>
#include <QComboBox>
#include <QCheckBox>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QGroupBox>
#include <QProgressBar>
#include <QSyntaxHighlighter>
#include <QTextCharFormat>
#include <QRegularExpression>

namespace ros_weaver {

class Project;
class ArchitectureDocGenerator;
enum class DocFormat;

/**
 * @brief Syntax highlighter for Markdown documentation
 */
class MarkdownHighlighter : public QSyntaxHighlighter {
  Q_OBJECT

public:
  explicit MarkdownHighlighter(QTextDocument* parent = nullptr);

protected:
  void highlightBlock(const QString& text) override;

private:
  struct HighlightingRule {
    QRegularExpression pattern;
    QTextCharFormat format;
  };
  QList<HighlightingRule> highlightingRules_;

  QTextCharFormat headerFormat_;
  QTextCharFormat boldFormat_;
  QTextCharFormat italicFormat_;
  QTextCharFormat codeFormat_;
  QTextCharFormat linkFormat_;
  QTextCharFormat listFormat_;
  QTextCharFormat tableFormat_;
};

/**
 * @brief Dialog for generating and previewing architecture documentation
 *
 * Features:
 * - Multiple output formats (Markdown, HTML, PDF)
 * - Live preview with syntax highlighting
 * - Configurable content sections
 * - Diagram format selection
 * - Export to file
 */
class ArchitectureDocDialog : public QDialog {
  Q_OBJECT

public:
  explicit ArchitectureDocDialog(const Project* project, QWidget* parent = nullptr);
  ~ArchitectureDocDialog() override;

  QString generatedContent() const;

public slots:
  void regenerate();

private slots:
  void onFormatChanged(int index);
  void onOptionsChanged();
  void onDiagramFormatChanged(int index);
  void exportToFile();
  void copyToClipboard();
  void onGenerationProgress(int percent, const QString& message);

private:
  void setupUI();
  void createOptionsGroup();
  void createPreviewGroup();
  void createButtonsLayout();
  void updatePreview();

  const Project* project_;

  // Format selection
  QComboBox* formatCombo_;
  QComboBox* diagramFormatCombo_;
  QComboBox* layoutDirectionCombo_;

  // Content options
  QGroupBox* optionsGroup_;
  QCheckBox* includeOverviewCheck_;
  QCheckBox* includeDiagramCheck_;
  QCheckBox* includeNodesCheck_;
  QCheckBox* includeTopicsCheck_;
  QCheckBox* includeParametersCheck_;
  QCheckBox* includeGroupsCheck_;
  QCheckBox* includeLaunchConfigCheck_;

  // Preview
  QPlainTextEdit* previewEdit_;
  QProgressBar* progressBar_;
  QLabel* statusLabel_;

  // Actions
  QPushButton* exportButton_;
  QPushButton* copyButton_;
  QPushButton* closeButton_;

  // Highlighter
  MarkdownHighlighter* markdownHighlighter_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_ARCHITECTURE_DOC_DIALOG_HPP
