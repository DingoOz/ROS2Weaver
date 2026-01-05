#ifndef ROS_WEAVER_WIDGETS_LAUNCH_PREVIEW_DIALOG_HPP
#define ROS_WEAVER_WIDGETS_LAUNCH_PREVIEW_DIALOG_HPP

#include <QDialog>
#include <QPlainTextEdit>
#include <QComboBox>
#include <QCheckBox>
#include <QLineEdit>
#include <QSyntaxHighlighter>
#include <QTextCharFormat>
#include <QRegularExpression>

namespace ros_weaver {

class Project;
class LaunchFileGenerator;
enum class LaunchFormat;

/**
 * @brief Syntax highlighter for Python launch files
 */
class PythonHighlighter : public QSyntaxHighlighter {
  Q_OBJECT

public:
  explicit PythonHighlighter(QTextDocument* parent = nullptr);

protected:
  void highlightBlock(const QString& text) override;

private:
  struct HighlightingRule {
    QRegularExpression pattern;
    QTextCharFormat format;
  };
  QList<HighlightingRule> highlightingRules_;

  QTextCharFormat keywordFormat_;
  QTextCharFormat stringFormat_;
  QTextCharFormat commentFormat_;
  QTextCharFormat functionFormat_;
  QTextCharFormat numberFormat_;
};

/**
 * @brief Syntax highlighter for XML launch files
 */
class XmlHighlighter : public QSyntaxHighlighter {
  Q_OBJECT

public:
  explicit XmlHighlighter(QTextDocument* parent = nullptr);

protected:
  void highlightBlock(const QString& text) override;

private:
  struct HighlightingRule {
    QRegularExpression pattern;
    QTextCharFormat format;
  };
  QList<HighlightingRule> highlightingRules_;

  QTextCharFormat tagFormat_;
  QTextCharFormat attributeFormat_;
  QTextCharFormat valueFormat_;
  QTextCharFormat commentFormat_;
};

/**
 * @brief Dialog for previewing and exporting launch files
 *
 * Features:
 * - Python and XML format selection
 * - Live preview with syntax highlighting
 * - Configuration options (namespace, composable nodes, etc.)
 * - Export to file
 */
class LaunchPreviewDialog : public QDialog {
  Q_OBJECT

public:
  explicit LaunchPreviewDialog(const Project* project, QWidget* parent = nullptr);
  ~LaunchPreviewDialog() override;

  QString generatedContent() const;
  QString selectedFileName() const;

public slots:
  void regenerate();

private slots:
  void onFormatChanged(int index);
  void onOptionsChanged();
  void exportToFile();
  void copyToClipboard();

private:
  void setupUI();
  void updateHighlighter();

  const Project* project_;
  LaunchFileGenerator* generator_;

  // UI elements
  QPlainTextEdit* previewEdit_;
  QComboBox* formatCombo_;
  QLineEdit* packageNameEdit_;
  QLineEdit* namespaceEdit_;
  QCheckBox* useSimTimeCheck_;
  QCheckBox* composableCheck_;
  QLineEdit* containerNameEdit_;
  QCheckBox* includeParamsCheck_;

  // Highlighters
  PythonHighlighter* pythonHighlighter_;
  XmlHighlighter* xmlHighlighter_;
  QSyntaxHighlighter* currentHighlighter_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_LAUNCH_PREVIEW_DIALOG_HPP
