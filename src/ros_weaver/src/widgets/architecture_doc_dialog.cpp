#include "ros_weaver/widgets/architecture_doc_dialog.hpp"
#include "ros_weaver/core/architecture_doc_generator.hpp"
#include "ros_weaver/core/project.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFormLayout>
#include <QFileDialog>
#include <QClipboard>
#include <QApplication>
#include <QMessageBox>
#include <QScrollBar>

namespace ros_weaver {

// MarkdownHighlighter implementation

MarkdownHighlighter::MarkdownHighlighter(QTextDocument* parent)
    : QSyntaxHighlighter(parent) {
  // Header format (# Header)
  headerFormat_.setForeground(QColor("#1976D2"));
  headerFormat_.setFontWeight(QFont::Bold);

  // Bold format (**bold**)
  boldFormat_.setFontWeight(QFont::Bold);

  // Italic format (*italic*)
  italicFormat_.setFontItalic(true);

  // Code format (`code`)
  codeFormat_.setForeground(QColor("#D32F2F"));
  codeFormat_.setBackground(QColor("#F5F5F5"));
  codeFormat_.setFontFamily("Consolas");

  // Link format
  linkFormat_.setForeground(QColor("#1565C0"));
  linkFormat_.setFontUnderline(true);

  // List format (- item)
  listFormat_.setForeground(QColor("#388E3C"));

  // Table format
  tableFormat_.setForeground(QColor("#7B1FA2"));

  // Build highlighting rules
  HighlightingRule rule;

  // Headers (# ## ###)
  rule.pattern = QRegularExpression("^#{1,6}\\s+.*$");
  rule.format = headerFormat_;
  highlightingRules_.append(rule);

  // Bold
  rule.pattern = QRegularExpression("\\*\\*[^*]+\\*\\*");
  rule.format = boldFormat_;
  highlightingRules_.append(rule);

  // Italic
  rule.pattern = QRegularExpression("\\*[^*]+\\*");
  rule.format = italicFormat_;
  highlightingRules_.append(rule);

  // Inline code
  rule.pattern = QRegularExpression("`[^`]+`");
  rule.format = codeFormat_;
  highlightingRules_.append(rule);

  // Links
  rule.pattern = QRegularExpression("\\[([^\\]]+)\\]\\(([^\\)]+)\\)");
  rule.format = linkFormat_;
  highlightingRules_.append(rule);

  // List items
  rule.pattern = QRegularExpression("^\\s*[-*+]\\s+");
  rule.format = listFormat_;
  highlightingRules_.append(rule);

  // Table separators
  rule.pattern = QRegularExpression("^\\|.*\\|$");
  rule.format = tableFormat_;
  highlightingRules_.append(rule);
}

void MarkdownHighlighter::highlightBlock(const QString& text) {
  for (const HighlightingRule& rule : highlightingRules_) {
    QRegularExpressionMatchIterator matchIterator = rule.pattern.globalMatch(text);
    while (matchIterator.hasNext()) {
      QRegularExpressionMatch match = matchIterator.next();
      setFormat(match.capturedStart(), match.capturedLength(), rule.format);
    }
  }
}

// ArchitectureDocDialog implementation

ArchitectureDocDialog::ArchitectureDocDialog(const Project* project, QWidget* parent)
    : QDialog(parent)
    , project_(project)
    , formatCombo_(nullptr)
    , diagramFormatCombo_(nullptr)
    , layoutDirectionCombo_(nullptr)
    , optionsGroup_(nullptr)
    , includeOverviewCheck_(nullptr)
    , includeDiagramCheck_(nullptr)
    , includeNodesCheck_(nullptr)
    , includeTopicsCheck_(nullptr)
    , includeParametersCheck_(nullptr)
    , includeGroupsCheck_(nullptr)
    , includeLaunchConfigCheck_(nullptr)
    , previewEdit_(nullptr)
    , progressBar_(nullptr)
    , statusLabel_(nullptr)
    , exportButton_(nullptr)
    , copyButton_(nullptr)
    , closeButton_(nullptr)
    , markdownHighlighter_(nullptr) {
  setWindowTitle(tr("Architecture Documentation Generator"));
  setMinimumSize(900, 700);
  resize(1000, 800);

  setupUI();

  // Connect to generator signals
  connect(&ArchitectureDocGenerator::instance(), &ArchitectureDocGenerator::generationProgress,
          this, &ArchitectureDocDialog::onGenerationProgress);

  // Initial generation
  regenerate();
}

ArchitectureDocDialog::~ArchitectureDocDialog() {
}

QString ArchitectureDocDialog::generatedContent() const {
  return previewEdit_->toPlainText();
}

void ArchitectureDocDialog::setupUI() {
  QVBoxLayout* mainLayout = new QVBoxLayout(this);

  // Top section: format selection and options
  QHBoxLayout* topLayout = new QHBoxLayout();

  // Format selection
  QGroupBox* formatGroup = new QGroupBox(tr("Output Format"));
  QFormLayout* formatLayout = new QFormLayout(formatGroup);

  formatCombo_ = new QComboBox();
  formatCombo_->addItem(tr("Markdown"), static_cast<int>(DocFormat::Markdown));
  formatCombo_->addItem(tr("HTML"), static_cast<int>(DocFormat::HTML));
  formatCombo_->addItem(tr("PDF"), static_cast<int>(DocFormat::PDF));
  formatLayout->addRow(tr("Format:"), formatCombo_);

  diagramFormatCombo_ = new QComboBox();
  diagramFormatCombo_->addItem(tr("Mermaid"), "mermaid");
  diagramFormatCombo_->addItem(tr("PlantUML"), "plantuml");
  formatLayout->addRow(tr("Diagram:"), diagramFormatCombo_);

  layoutDirectionCombo_ = new QComboBox();
  layoutDirectionCombo_->addItem(tr("Top to Bottom"), "TB");
  layoutDirectionCombo_->addItem(tr("Left to Right"), "LR");
  layoutDirectionCombo_->addItem(tr("Bottom to Top"), "BT");
  layoutDirectionCombo_->addItem(tr("Right to Left"), "RL");
  formatLayout->addRow(tr("Layout:"), layoutDirectionCombo_);

  topLayout->addWidget(formatGroup);

  // Content options
  createOptionsGroup();
  topLayout->addWidget(optionsGroup_);

  topLayout->setStretch(0, 1);
  topLayout->setStretch(1, 2);

  mainLayout->addLayout(topLayout);

  // Preview section
  createPreviewGroup();
  mainLayout->addWidget(previewEdit_, 1);

  // Progress bar
  progressBar_ = new QProgressBar();
  progressBar_->setRange(0, 100);
  progressBar_->setValue(0);
  progressBar_->setTextVisible(true);
  mainLayout->addWidget(progressBar_);

  // Status label
  statusLabel_ = new QLabel(tr("Ready"));
  mainLayout->addWidget(statusLabel_);

  // Buttons
  createButtonsLayout();
  QHBoxLayout* buttonLayout = new QHBoxLayout();
  buttonLayout->addStretch();
  buttonLayout->addWidget(copyButton_);
  buttonLayout->addWidget(exportButton_);
  buttonLayout->addWidget(closeButton_);
  mainLayout->addLayout(buttonLayout);

  // Connect signals
  connect(formatCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &ArchitectureDocDialog::onFormatChanged);
  connect(diagramFormatCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &ArchitectureDocDialog::onDiagramFormatChanged);
  connect(layoutDirectionCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &ArchitectureDocDialog::onOptionsChanged);

  connect(includeOverviewCheck_, &QCheckBox::toggled, this, &ArchitectureDocDialog::onOptionsChanged);
  connect(includeDiagramCheck_, &QCheckBox::toggled, this, &ArchitectureDocDialog::onOptionsChanged);
  connect(includeNodesCheck_, &QCheckBox::toggled, this, &ArchitectureDocDialog::onOptionsChanged);
  connect(includeTopicsCheck_, &QCheckBox::toggled, this, &ArchitectureDocDialog::onOptionsChanged);
  connect(includeParametersCheck_, &QCheckBox::toggled, this, &ArchitectureDocDialog::onOptionsChanged);
  connect(includeGroupsCheck_, &QCheckBox::toggled, this, &ArchitectureDocDialog::onOptionsChanged);
  connect(includeLaunchConfigCheck_, &QCheckBox::toggled, this, &ArchitectureDocDialog::onOptionsChanged);

  connect(exportButton_, &QPushButton::clicked, this, &ArchitectureDocDialog::exportToFile);
  connect(copyButton_, &QPushButton::clicked, this, &ArchitectureDocDialog::copyToClipboard);
  connect(closeButton_, &QPushButton::clicked, this, &QDialog::accept);
}

void ArchitectureDocDialog::createOptionsGroup() {
  optionsGroup_ = new QGroupBox(tr("Content Options"));
  QGridLayout* layout = new QGridLayout(optionsGroup_);

  includeOverviewCheck_ = new QCheckBox(tr("Overview"));
  includeOverviewCheck_->setChecked(true);
  layout->addWidget(includeOverviewCheck_, 0, 0);

  includeDiagramCheck_ = new QCheckBox(tr("System Diagram"));
  includeDiagramCheck_->setChecked(true);
  layout->addWidget(includeDiagramCheck_, 0, 1);

  includeNodesCheck_ = new QCheckBox(tr("Node Details"));
  includeNodesCheck_->setChecked(true);
  layout->addWidget(includeNodesCheck_, 1, 0);

  includeTopicsCheck_ = new QCheckBox(tr("Topics Table"));
  includeTopicsCheck_->setChecked(true);
  layout->addWidget(includeTopicsCheck_, 1, 1);

  includeParametersCheck_ = new QCheckBox(tr("Parameters"));
  includeParametersCheck_->setChecked(true);
  layout->addWidget(includeParametersCheck_, 2, 0);

  includeGroupsCheck_ = new QCheckBox(tr("Node Groups"));
  includeGroupsCheck_->setChecked(true);
  layout->addWidget(includeGroupsCheck_, 2, 1);

  includeLaunchConfigCheck_ = new QCheckBox(tr("Launch Config"));
  includeLaunchConfigCheck_->setChecked(true);
  layout->addWidget(includeLaunchConfigCheck_, 3, 0, 1, 2);
}

void ArchitectureDocDialog::createPreviewGroup() {
  previewEdit_ = new QPlainTextEdit();
  previewEdit_->setReadOnly(true);
  previewEdit_->setLineWrapMode(QPlainTextEdit::NoWrap);

  // Set monospace font
  QFont font("Consolas", 10);
  font.setStyleHint(QFont::Monospace);
  previewEdit_->setFont(font);

  // Apply highlighter
  markdownHighlighter_ = new MarkdownHighlighter(previewEdit_->document());
}

void ArchitectureDocDialog::createButtonsLayout() {
  copyButton_ = new QPushButton(tr("Copy to Clipboard"));
  copyButton_->setIcon(QIcon::fromTheme("edit-copy"));

  exportButton_ = new QPushButton(tr("Export to File..."));
  exportButton_->setIcon(QIcon::fromTheme("document-save-as"));
  exportButton_->setDefault(true);

  closeButton_ = new QPushButton(tr("Close"));
  closeButton_->setIcon(QIcon::fromTheme("window-close"));
}

void ArchitectureDocDialog::regenerate() {
  updatePreview();
}

void ArchitectureDocDialog::onFormatChanged(int index) {
  Q_UNUSED(index)
  updatePreview();
}

void ArchitectureDocDialog::onOptionsChanged() {
  updatePreview();
}

void ArchitectureDocDialog::onDiagramFormatChanged(int index) {
  Q_UNUSED(index)
  updatePreview();
}

void ArchitectureDocDialog::updatePreview() {
  if (!project_) {
    previewEdit_->setPlainText(tr("No project loaded."));
    return;
  }

  DocGeneratorOptions options;
  options.includeOverview = includeOverviewCheck_->isChecked();
  options.includeDiagram = includeDiagramCheck_->isChecked();
  options.includeNodes = includeNodesCheck_->isChecked();
  options.includeTopics = includeTopicsCheck_->isChecked();
  options.includeParameters = includeParametersCheck_->isChecked();
  options.includeGroups = includeGroupsCheck_->isChecked();
  options.includeLaunchConfig = includeLaunchConfigCheck_->isChecked();
  options.diagramFormat = diagramFormatCombo_->currentData().toString();
  options.layoutDirection = layoutDirectionCombo_->currentData().toString();

  DocFormat format = static_cast<DocFormat>(formatCombo_->currentData().toInt());

  QString content;
  switch (format) {
    case DocFormat::Markdown:
      content = ArchitectureDocGenerator::instance().generateMarkdown(*project_, options);
      break;
    case DocFormat::HTML:
      content = ArchitectureDocGenerator::instance().generateHTML(*project_, options);
      break;
    case DocFormat::PDF:
      // Show markdown preview for PDF (actual PDF generated on export)
      content = ArchitectureDocGenerator::instance().generateMarkdown(*project_, options);
      break;
  }

  // Save scroll position
  int scrollPos = previewEdit_->verticalScrollBar()->value();

  previewEdit_->setPlainText(content);

  // Restore scroll position
  previewEdit_->verticalScrollBar()->setValue(scrollPos);

  statusLabel_->setText(tr("Generated %1 characters").arg(content.length()));
}

void ArchitectureDocDialog::exportToFile() {
  if (!project_) {
    QMessageBox::warning(this, tr("Export Error"), tr("No project loaded."));
    return;
  }

  DocFormat format = static_cast<DocFormat>(formatCombo_->currentData().toInt());
  QString filter = ArchitectureDocGenerator::fileFilter(format);
  QString defaultExt = ArchitectureDocGenerator::fileExtension(format);

  QString defaultName = project_->metadata().name;
  if (defaultName.isEmpty()) {
    defaultName = "architecture";
  }
  defaultName += "_documentation." + defaultExt;

  QString filePath = QFileDialog::getSaveFileName(
      this,
      tr("Export Architecture Documentation"),
      defaultName,
      filter
  );

  if (filePath.isEmpty()) {
    return;
  }

  // Ensure correct extension
  if (!filePath.endsWith("." + defaultExt, Qt::CaseInsensitive)) {
    filePath += "." + defaultExt;
  }

  DocGeneratorOptions options;
  options.includeOverview = includeOverviewCheck_->isChecked();
  options.includeDiagram = includeDiagramCheck_->isChecked();
  options.includeNodes = includeNodesCheck_->isChecked();
  options.includeTopics = includeTopicsCheck_->isChecked();
  options.includeParameters = includeParametersCheck_->isChecked();
  options.includeGroups = includeGroupsCheck_->isChecked();
  options.includeLaunchConfig = includeLaunchConfigCheck_->isChecked();
  options.diagramFormat = diagramFormatCombo_->currentData().toString();
  options.layoutDirection = layoutDirectionCombo_->currentData().toString();

  bool success = ArchitectureDocGenerator::instance().exportToFile(*project_, filePath, format, options);

  if (success) {
    QMessageBox::information(this, tr("Export Successful"),
                              tr("Documentation exported to:\n%1").arg(filePath));
  } else {
    QMessageBox::warning(this, tr("Export Failed"),
                          tr("Failed to export documentation:\n%1")
                              .arg(ArchitectureDocGenerator::instance().lastError()));
  }
}

void ArchitectureDocDialog::copyToClipboard() {
  QClipboard* clipboard = QApplication::clipboard();
  clipboard->setText(previewEdit_->toPlainText());
  statusLabel_->setText(tr("Copied to clipboard"));
}

void ArchitectureDocDialog::onGenerationProgress(int percent, const QString& message) {
  progressBar_->setValue(percent);
  statusLabel_->setText(message);
}

}  // namespace ros_weaver
