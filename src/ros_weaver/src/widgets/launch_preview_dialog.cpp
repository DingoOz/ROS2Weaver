#include "ros_weaver/widgets/launch_preview_dialog.hpp"
#include "ros_weaver/core/launch_file_generator.hpp"
#include "ros_weaver/core/project.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QLabel>
#include <QFileDialog>
#include <QMessageBox>
#include <QClipboard>
#include <QApplication>
#include <QFont>

namespace ros_weaver {

// Python Highlighter

PythonHighlighter::PythonHighlighter(QTextDocument* parent)
  : QSyntaxHighlighter(parent) {

  // Keywords
  keywordFormat_.setForeground(QColor(198, 120, 221));  // Purple
  keywordFormat_.setFontWeight(QFont::Bold);
  QStringList keywords = {
    "\\bdef\\b", "\\bclass\\b", "\\breturn\\b", "\\bimport\\b", "\\bfrom\\b",
    "\\bif\\b", "\\belse\\b", "\\belif\\b", "\\bfor\\b", "\\bwhile\\b",
    "\\btry\\b", "\\bexcept\\b", "\\bfinally\\b", "\\bwith\\b", "\\bas\\b",
    "\\bTrue\\b", "\\bFalse\\b", "\\bNone\\b", "\\band\\b", "\\bor\\b", "\\bnot\\b"
  };
  for (const QString& pattern : keywords) {
    HighlightingRule rule;
    rule.pattern = QRegularExpression(pattern);
    rule.format = keywordFormat_;
    highlightingRules_.append(rule);
  }

  // Strings (single and double quoted)
  stringFormat_.setForeground(QColor(152, 195, 121));  // Green
  HighlightingRule stringRule;
  stringRule.pattern = QRegularExpression("\"[^\"]*\"|'[^']*'");
  stringRule.format = stringFormat_;
  highlightingRules_.append(stringRule);

  // Comments
  commentFormat_.setForeground(QColor(128, 128, 128));  // Gray
  commentFormat_.setFontItalic(true);
  HighlightingRule commentRule;
  commentRule.pattern = QRegularExpression("#[^\n]*");
  commentRule.format = commentFormat_;
  highlightingRules_.append(commentRule);

  // Functions
  functionFormat_.setForeground(QColor(97, 175, 239));  // Blue
  HighlightingRule funcRule;
  funcRule.pattern = QRegularExpression("\\b[A-Za-z_][A-Za-z0-9_]*(?=\\()");
  funcRule.format = functionFormat_;
  highlightingRules_.append(funcRule);

  // Numbers
  numberFormat_.setForeground(QColor(209, 154, 102));  // Orange
  HighlightingRule numRule;
  numRule.pattern = QRegularExpression("\\b[0-9]+(\\.[0-9]+)?\\b");
  numRule.format = numberFormat_;
  highlightingRules_.append(numRule);
}

void PythonHighlighter::highlightBlock(const QString& text) {
  for (const HighlightingRule& rule : highlightingRules_) {
    QRegularExpressionMatchIterator matchIterator = rule.pattern.globalMatch(text);
    while (matchIterator.hasNext()) {
      QRegularExpressionMatch match = matchIterator.next();
      setFormat(match.capturedStart(), match.capturedLength(), rule.format);
    }
  }
}

// XML Highlighter

XmlHighlighter::XmlHighlighter(QTextDocument* parent)
  : QSyntaxHighlighter(parent) {

  // XML tags
  tagFormat_.setForeground(QColor(224, 108, 117));  // Red
  HighlightingRule tagRule;
  tagRule.pattern = QRegularExpression("</?[a-zA-Z_][a-zA-Z0-9_-]*|/?>");
  tagRule.format = tagFormat_;
  highlightingRules_.append(tagRule);

  // Attributes
  attributeFormat_.setForeground(QColor(209, 154, 102));  // Orange
  HighlightingRule attrRule;
  attrRule.pattern = QRegularExpression("\\b[a-zA-Z_][a-zA-Z0-9_-]*(?=\\s*=)");
  attrRule.format = attributeFormat_;
  highlightingRules_.append(attrRule);

  // Attribute values
  valueFormat_.setForeground(QColor(152, 195, 121));  // Green
  HighlightingRule valueRule;
  valueRule.pattern = QRegularExpression("\"[^\"]*\"");
  valueRule.format = valueFormat_;
  highlightingRules_.append(valueRule);

  // Comments
  commentFormat_.setForeground(QColor(128, 128, 128));  // Gray
  commentFormat_.setFontItalic(true);
  HighlightingRule commentRule;
  commentRule.pattern = QRegularExpression("<!--[^>]*-->");
  commentRule.format = commentFormat_;
  highlightingRules_.append(commentRule);
}

void XmlHighlighter::highlightBlock(const QString& text) {
  for (const HighlightingRule& rule : highlightingRules_) {
    QRegularExpressionMatchIterator matchIterator = rule.pattern.globalMatch(text);
    while (matchIterator.hasNext()) {
      QRegularExpressionMatch match = matchIterator.next();
      setFormat(match.capturedStart(), match.capturedLength(), rule.format);
    }
  }
}

// Launch Preview Dialog

LaunchPreviewDialog::LaunchPreviewDialog(const Project* project, QWidget* parent)
  : QDialog(parent)
  , project_(project)
  , generator_(new LaunchFileGenerator(this))
  , pythonHighlighter_(nullptr)
  , xmlHighlighter_(nullptr)
  , currentHighlighter_(nullptr)
{
  setWindowTitle(tr("Launch File Preview"));
  setMinimumSize(800, 600);

  setupUI();
  regenerate();
}

LaunchPreviewDialog::~LaunchPreviewDialog() = default;

void LaunchPreviewDialog::setupUI() {
  auto* mainLayout = new QVBoxLayout(this);

  // Options group
  auto* optionsGroup = new QGroupBox(tr("Generation Options"));
  auto* optionsLayout = new QGridLayout(optionsGroup);

  int row = 0;

  // Format
  optionsLayout->addWidget(new QLabel(tr("Format:")), row, 0);
  formatCombo_ = new QComboBox();
  formatCombo_->addItem(tr("Python (.launch.py)"), static_cast<int>(LaunchFormat::Python));
  formatCombo_->addItem(tr("XML (.launch.xml)"), static_cast<int>(LaunchFormat::XML));
  connect(formatCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &LaunchPreviewDialog::onFormatChanged);
  optionsLayout->addWidget(formatCombo_, row++, 1);

  // Package name
  optionsLayout->addWidget(new QLabel(tr("Package Name:")), row, 0);
  packageNameEdit_ = new QLineEdit();
  packageNameEdit_->setText("my_robot");
  packageNameEdit_->setPlaceholderText("my_package");
  connect(packageNameEdit_, &QLineEdit::textChanged,
          this, &LaunchPreviewDialog::onOptionsChanged);
  optionsLayout->addWidget(packageNameEdit_, row++, 1);

  // Namespace
  optionsLayout->addWidget(new QLabel(tr("Namespace:")), row, 0);
  namespaceEdit_ = new QLineEdit();
  namespaceEdit_->setPlaceholderText("(optional)");
  connect(namespaceEdit_, &QLineEdit::textChanged,
          this, &LaunchPreviewDialog::onOptionsChanged);
  optionsLayout->addWidget(namespaceEdit_, row++, 1);

  // Use sim time
  useSimTimeCheck_ = new QCheckBox(tr("Use Simulation Time"));
  connect(useSimTimeCheck_, &QCheckBox::toggled,
          this, &LaunchPreviewDialog::onOptionsChanged);
  optionsLayout->addWidget(useSimTimeCheck_, row++, 0, 1, 2);

  // Composable nodes
  composableCheck_ = new QCheckBox(tr("Use Composable Node Container"));
  connect(composableCheck_, &QCheckBox::toggled, this, [this](bool checked) {
    containerNameEdit_->setEnabled(checked);
    onOptionsChanged();
  });
  optionsLayout->addWidget(composableCheck_, row++, 0, 1, 2);

  // Container name
  optionsLayout->addWidget(new QLabel(tr("Container Name:")), row, 0);
  containerNameEdit_ = new QLineEdit();
  containerNameEdit_->setText("component_container");
  containerNameEdit_->setEnabled(false);
  connect(containerNameEdit_, &QLineEdit::textChanged,
          this, &LaunchPreviewDialog::onOptionsChanged);
  optionsLayout->addWidget(containerNameEdit_, row++, 1);

  // Include params file
  includeParamsCheck_ = new QCheckBox(tr("Include Parameters File Reference"));
  includeParamsCheck_->setChecked(true);
  connect(includeParamsCheck_, &QCheckBox::toggled,
          this, &LaunchPreviewDialog::onOptionsChanged);
  optionsLayout->addWidget(includeParamsCheck_, row++, 0, 1, 2);

  mainLayout->addWidget(optionsGroup);

  // Preview group
  auto* previewGroup = new QGroupBox(tr("Preview"));
  auto* previewLayout = new QVBoxLayout(previewGroup);

  previewEdit_ = new QPlainTextEdit();
  previewEdit_->setReadOnly(true);
  previewEdit_->setFont(QFont("Monospace", 10));
  previewEdit_->setLineWrapMode(QPlainTextEdit::NoWrap);

  // Set dark background for code preview
  previewEdit_->setStyleSheet(
    "QPlainTextEdit {"
    "  background-color: #282c34;"
    "  color: #abb2bf;"
    "  selection-background-color: #3e4451;"
    "  border: 1px solid #3e4451;"
    "}"
  );

  // Create highlighters
  pythonHighlighter_ = new PythonHighlighter(nullptr);
  xmlHighlighter_ = new XmlHighlighter(nullptr);

  previewLayout->addWidget(previewEdit_);
  mainLayout->addWidget(previewGroup, 1);

  // Buttons
  auto* buttonLayout = new QHBoxLayout();

  auto* copyButton = new QPushButton(tr("Copy to Clipboard"));
  connect(copyButton, &QPushButton::clicked, this, &LaunchPreviewDialog::copyToClipboard);
  buttonLayout->addWidget(copyButton);

  auto* exportButton = new QPushButton(tr("Export to File..."));
  connect(exportButton, &QPushButton::clicked, this, &LaunchPreviewDialog::exportToFile);
  buttonLayout->addWidget(exportButton);

  buttonLayout->addStretch();

  auto* closeButton = new QPushButton(tr("Close"));
  connect(closeButton, &QPushButton::clicked, this, &QDialog::accept);
  buttonLayout->addWidget(closeButton);

  mainLayout->addLayout(buttonLayout);

  // Set initial highlighter
  updateHighlighter();
}

void LaunchPreviewDialog::regenerate() {
  if (!project_) return;

  LaunchGeneratorOptions options;
  options.format = static_cast<LaunchFormat>(formatCombo_->currentData().toInt());
  options.packageName = packageNameEdit_->text().isEmpty() ?
                        "my_package" : packageNameEdit_->text();
  options.namespace_ = namespaceEdit_->text();
  options.useSimTime = useSimTimeCheck_->isChecked();
  options.generateComposableContainer = composableCheck_->isChecked();
  options.containerName = containerNameEdit_->text();
  options.includeParamsFile = includeParamsCheck_->isChecked();

  QString content = generator_->generate(*project_, options);
  previewEdit_->setPlainText(content);
}

void LaunchPreviewDialog::onFormatChanged(int index) {
  Q_UNUSED(index)
  updateHighlighter();
  regenerate();
}

void LaunchPreviewDialog::onOptionsChanged() {
  regenerate();
}

void LaunchPreviewDialog::updateHighlighter() {
  LaunchFormat format = static_cast<LaunchFormat>(formatCombo_->currentData().toInt());

  // Disconnect current highlighter
  if (currentHighlighter_) {
    currentHighlighter_->setDocument(nullptr);
  }

  // Connect appropriate highlighter
  switch (format) {
    case LaunchFormat::Python:
      pythonHighlighter_->setDocument(previewEdit_->document());
      currentHighlighter_ = pythonHighlighter_;
      break;
    case LaunchFormat::XML:
      xmlHighlighter_->setDocument(previewEdit_->document());
      currentHighlighter_ = xmlHighlighter_;
      break;
  }
}

void LaunchPreviewDialog::exportToFile() {
  LaunchFormat format = static_cast<LaunchFormat>(formatCombo_->currentData().toInt());
  QString ext = LaunchFileGenerator::fileExtension(format);
  QString defaultName = packageNameEdit_->text() + "_launch" + ext;

  QString filter = format == LaunchFormat::Python ?
                   tr("Python Launch Files (*.launch.py)") :
                   tr("XML Launch Files (*.launch.xml)");

  QString fileName = QFileDialog::getSaveFileName(this,
    tr("Export Launch File"), defaultName, filter);

  if (fileName.isEmpty()) return;

  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QMessageBox::critical(this, tr("Error"),
      tr("Failed to open file for writing: %1").arg(file.errorString()));
    return;
  }

  QTextStream out(&file);
  out << previewEdit_->toPlainText();
  file.close();

  QMessageBox::information(this, tr("Export Complete"),
    tr("Launch file exported to:\n%1").arg(fileName));
}

void LaunchPreviewDialog::copyToClipboard() {
  QApplication::clipboard()->setText(previewEdit_->toPlainText());

  // Show brief feedback
  QMessageBox::information(this, tr("Copied"),
    tr("Launch file content copied to clipboard."));
}

QString LaunchPreviewDialog::generatedContent() const {
  return previewEdit_->toPlainText();
}

QString LaunchPreviewDialog::selectedFileName() const {
  LaunchFormat format = static_cast<LaunchFormat>(formatCombo_->currentData().toInt());
  return packageNameEdit_->text() + "_launch" + LaunchFileGenerator::fileExtension(format);
}

}  // namespace ros_weaver
