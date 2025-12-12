#include "ros_weaver/wizards/package_wizard.hpp"
#include "ros_weaver/core/project.hpp"
#include "ros_weaver/core/code_generator.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFormLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QStandardPaths>
#include <QRegularExpression>
#include <QDir>
#include <QFileInfo>
#include <QHeaderView>
#include <QFont>
#include <QAbstractButton>
#include <QTimer>

namespace ros_weaver {

// =============================================================================
// WizardProgressWidget
// =============================================================================

WizardProgressWidget::WizardProgressWidget(QWidget* parent)
  : QFrame(parent)
  , currentStep_(0)
{
  setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
  setAutoFillBackground(true);

  // Set a subtle background
  QPalette pal = palette();
  pal.setColor(QPalette::Window, pal.color(QPalette::Window).darker(105));
  setPalette(pal);

  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(12, 8, 12, 8);
  mainLayout->setSpacing(6);

  // Top row: step indicators (circles)
  indicatorContainer_ = new QWidget();
  QHBoxLayout* indicatorLayout = new QHBoxLayout(indicatorContainer_);
  indicatorLayout->setContentsMargins(0, 0, 0, 0);
  indicatorLayout->setSpacing(0);
  mainLayout->addWidget(indicatorContainer_);

  // Progress bar
  progressBar_ = new QProgressBar();
  progressBar_->setTextVisible(false);
  progressBar_->setFixedHeight(6);
  progressBar_->setStyleSheet(
    "QProgressBar { border: none; background-color: #404040; border-radius: 3px; }"
    "QProgressBar::chunk { background-color: #3daee9; border-radius: 3px; }"
  );
  mainLayout->addWidget(progressBar_);

  // Bottom row: step number and title
  QHBoxLayout* labelLayout = new QHBoxLayout();
  labelLayout->setContentsMargins(0, 0, 0, 0);

  stepLabel_ = new QLabel();
  stepLabel_->setStyleSheet("font-weight: bold; color: #3daee9;");
  labelLayout->addWidget(stepLabel_);

  titleLabel_ = new QLabel();
  titleLabel_->setStyleSheet("color: #888888;");
  labelLayout->addWidget(titleLabel_);

  labelLayout->addStretch();
  mainLayout->addLayout(labelLayout);
}

void WizardProgressWidget::setSteps(const QStringList& stepNames) {
  stepNames_ = stepNames;

  // Clear old indicators
  qDeleteAll(stepIndicators_);
  stepIndicators_.clear();

  QHBoxLayout* layout = qobject_cast<QHBoxLayout*>(indicatorContainer_->layout());
  if (!layout) return;

  // Clear layout
  QLayoutItem* item;
  while ((item = layout->takeAt(0)) != nullptr) {
    delete item;
  }

  // Add stretch at start
  layout->addStretch();

  // Create step indicators
  for (int i = 0; i < stepNames_.size(); ++i) {
    if (i > 0) {
      // Add connector line between circles
      QFrame* line = new QFrame();
      line->setFrameShape(QFrame::HLine);
      line->setFixedWidth(30);
      line->setFixedHeight(2);
      line->setStyleSheet("background-color: #404040;");
      layout->addWidget(line);
    }

    QLabel* indicator = new QLabel(QString::number(i + 1));
    indicator->setAlignment(Qt::AlignCenter);
    indicator->setFixedSize(28, 28);
    indicator->setToolTip(stepNames_[i]);
    stepIndicators_.append(indicator);
    layout->addWidget(indicator);
  }

  // Add stretch at end
  layout->addStretch();

  progressBar_->setMaximum(stepNames_.size() > 0 ? stepNames_.size() - 1 : 1);
  updateDisplay();
}

void WizardProgressWidget::setCurrentStep(int step) {
  currentStep_ = step;
  updateDisplay();
}

void WizardProgressWidget::updateDisplay() {
  if (stepNames_.isEmpty()) return;

  // Update progress bar
  progressBar_->setValue(currentStep_);

  // Update step label
  stepLabel_->setText(tr("Step %1 of %2:").arg(currentStep_ + 1).arg(stepNames_.size()));

  // Update title
  if (currentStep_ >= 0 && currentStep_ < stepNames_.size()) {
    titleLabel_->setText(stepNames_[currentStep_]);
  }

  // Update step indicators
  for (int i = 0; i < stepIndicators_.size(); ++i) {
    QLabel* indicator = stepIndicators_[i];
    if (i < currentStep_) {
      // Completed step - green checkmark
      indicator->setText(QString::fromUtf8("\u2713")); // ✓
      indicator->setStyleSheet(
        "QLabel { background-color: #27ae60; color: white; font-weight: bold; "
        "border-radius: 14px; font-size: 14px; }"
      );
    } else if (i == currentStep_) {
      // Current step - blue highlight
      indicator->setText(QString::number(i + 1));
      indicator->setStyleSheet(
        "QLabel { background-color: #3daee9; color: white; font-weight: bold; "
        "border-radius: 14px; font-size: 12px; }"
      );
    } else {
      // Future step - gray
      indicator->setText(QString::number(i + 1));
      indicator->setStyleSheet(
        "QLabel { background-color: #404040; color: #888888; font-weight: bold; "
        "border-radius: 14px; font-size: 12px; }"
      );
    }
  }
}

// =============================================================================
// PackageWizard
// =============================================================================

PackageWizard::PackageWizard(const Project& project, QWidget* parent)
  : QWizard(parent)
  , project_(project)
  , progressWidget_(nullptr)
{
  setWindowTitle(tr("ROS2 Package Generation Wizard"));
  setWizardStyle(QWizard::ModernStyle);
  setMinimumSize(700, 600);

  // Create pages
  packageInfoPage_ = new PackageInfoPage(this);
  outputConfigPage_ = new OutputConfigPage(this);
  nodeSelectionPage_ = new NodeSelectionPage(project_, this);
  languageStylePage_ = new LanguageStylePage(this);
  generatedFilesPage_ = new GeneratedFilesPage(this);
  parametersConfigPage_ = new ParametersConfigPage(project_, this);
  reviewGeneratePage_ = new ReviewGeneratePage(project_, this);

  // Add pages
  setPage(Page_PackageInfo, packageInfoPage_);
  setPage(Page_OutputConfig, outputConfigPage_);
  setPage(Page_NodeSelection, nodeSelectionPage_);
  setPage(Page_LanguageStyle, languageStylePage_);
  setPage(Page_GeneratedFiles, generatedFilesPage_);
  setPage(Page_ParametersConfig, parametersConfigPage_);
  setPage(Page_ReviewGenerate, reviewGeneratePage_);

  // Connect generation signals
  connect(reviewGeneratePage_, &ReviewGeneratePage::generationFinished,
          this, [this](bool success) {
    emit generationComplete(success, generatedPackagePath_);
  });

  // Set up progress widget
  setupProgressWidget();

  // Connect page change signal
  connect(this, &QWizard::currentIdChanged, this, &PackageWizard::onCurrentPageChanged);

  setOption(QWizard::NoBackButtonOnStartPage, true);
}

void PackageWizard::setupProgressWidget() {
  // Create progress widget
  progressWidget_ = new WizardProgressWidget(this);

  // Set step names
  QStringList stepNames = {
    tr("Package Info"),
    tr("Output"),
    tr("Nodes"),
    tr("Language"),
    tr("Files"),
    tr("Parameters"),
    tr("Generate")
  };
  progressWidget_->setSteps(stepNames);
  progressWidget_->setCurrentStep(0);

  // QWizard has a complex internal layout. The most reliable way to add
  // a progress widget at the top is to find the main layout and insert there.
  // QWizard in ModernStyle uses a QVBoxLayout internally.

  // We need to delay insertion until after the wizard's layout is fully set up
  QTimer::singleShot(0, this, [this]() {
    // Find the wizard's main layout
    if (QVBoxLayout* vboxLayout = qobject_cast<QVBoxLayout*>(layout())) {
      // Insert at the top (position 0)
      vboxLayout->insertWidget(0, progressWidget_);
    } else if (QGridLayout* gridLayout = qobject_cast<QGridLayout*>(layout())) {
      // Some wizard styles use grid layout - insert at top spanning all columns
      gridLayout->addWidget(progressWidget_, 0, 0, 1, gridLayout->columnCount());
    } else {
      // Fallback: just parent it and position manually
      progressWidget_->setParent(this);
      progressWidget_->move(0, 0);
      progressWidget_->resize(width(), progressWidget_->sizeHint().height());
      progressWidget_->show();
      progressWidget_->raise();
    }
  });
}

void PackageWizard::onCurrentPageChanged(int id) {
  // Map page ID to step index
  int stepIndex = 0;
  switch (id) {
    case Page_PackageInfo:     stepIndex = 0; break;
    case Page_OutputConfig:    stepIndex = 1; break;
    case Page_NodeSelection:   stepIndex = 2; break;
    case Page_LanguageStyle:   stepIndex = 3; break;
    case Page_GeneratedFiles:  stepIndex = 4; break;
    case Page_ParametersConfig: stepIndex = 5; break;
    case Page_ReviewGenerate:  stepIndex = 6; break;
  }
  progressWidget_->setCurrentStep(stepIndex);
}

PackageWizard::~PackageWizard() = default;

void PackageWizard::accept() {
  // Collect all options from pages
  options_.packageName = packageInfoPage_->packageName();
  options_.packageVersion = packageInfoPage_->packageVersion();
  options_.description = packageInfoPage_->description();
  options_.maintainer = packageInfoPage_->maintainer();
  options_.maintainerEmail = packageInfoPage_->maintainerEmail();
  options_.license = packageInfoPage_->license();
  options_.rosDistro = packageInfoPage_->rosDistro();

  options_.outputPath = outputConfigPage_->outputPath();
  options_.createSubdirectory = outputConfigPage_->createSubdirectory();
  options_.overwriteBehavior = outputConfigPage_->overwriteBehavior();

  options_.selectedNodeIds = nodeSelectionPage_->selectedNodeIds();

  options_.useCpp = languageStylePage_->useCpp();
  options_.namespaceConvention = languageStylePage_->namespaceConvention();
  options_.commentStyle = languageStylePage_->commentStyle();
  options_.ros2StyleCompliance = languageStylePage_->ros2StyleCompliance();

  options_.generateCMakeLists = generatedFilesPage_->generateCMakeLists();
  options_.generatePackageXml = generatedFilesPage_->generatePackageXml();
  options_.generateLaunchFile = generatedFilesPage_->generateLaunchFile();
  options_.generateParamsYaml = generatedFilesPage_->generateParamsYaml();
  options_.generateReadme = generatedFilesPage_->generateReadme();
  options_.generateTestStubs = generatedFilesPage_->generateTestStubs();
  options_.launchFileXml = generatedFilesPage_->launchFileXml();

  options_.exportOnlyModifiedParams = parametersConfigPage_->exportOnlyModifiedParams();

  // Store generated path
  generatedPackagePath_ = options_.outputPath;
  if (options_.createSubdirectory) {
    generatedPackagePath_ += "/" + options_.packageName;
  }

  QWizard::accept();
}

// =============================================================================
// PackageInfoPage (Step 1)
// =============================================================================

PackageInfoPage::PackageInfoPage(QWidget* parent)
  : QWizardPage(parent)
{
  setTitle(tr("Package Information"));
  setSubTitle(tr("Enter the basic information for your ROS2 package."));
  setupUi();
}

void PackageInfoPage::setupUi() {
  QFormLayout* layout = new QFormLayout(this);
  layout->setSpacing(10);

  // Package name
  QVBoxLayout* nameLayout = new QVBoxLayout();
  nameEdit_ = new QLineEdit();
  nameEdit_->setPlaceholderText(tr("my_ros_package"));
  nameValidationLabel_ = new QLabel();
  nameValidationLabel_->setStyleSheet("color: red; font-size: 11px;");
  nameLayout->addWidget(nameEdit_);
  nameLayout->addWidget(nameValidationLabel_);
  layout->addRow(tr("Package Name*:"), nameLayout);

  connect(nameEdit_, &QLineEdit::textChanged, this, &PackageInfoPage::onPackageNameChanged);
  connect(this, &QWizardPage::completeChanged, this, &PackageInfoPage::updateNextButtonTooltip);

  // Version
  versionEdit_ = new QLineEdit("0.1.0");
  layout->addRow(tr("Version:"), versionEdit_);

  // Description
  descriptionEdit_ = new QTextEdit();
  descriptionEdit_->setMaximumHeight(80);
  descriptionEdit_->setPlaceholderText(tr("A brief description of your package..."));
  layout->addRow(tr("Description:"), descriptionEdit_);

  // Maintainer
  maintainerEdit_ = new QLineEdit();
  maintainerEdit_->setPlaceholderText(tr("Your Name"));
  QString defaultName = qgetenv("USER");
  if (defaultName.isEmpty()) {
    defaultName = qgetenv("USERNAME");
  }
  maintainerEdit_->setText(defaultName);
  layout->addRow(tr("Maintainer*:"), maintainerEdit_);

  // Email
  emailEdit_ = new QLineEdit();
  emailEdit_->setPlaceholderText(tr("your.email@example.com"));
  layout->addRow(tr("Email*:"), emailEdit_);

  // License
  licenseCombo_ = new QComboBox();
  licenseCombo_->addItems({
    "Apache-2.0",
    "MIT",
    "BSD-3-Clause",
    "GPL-3.0",
    "LGPL-3.0",
    "Proprietary"
  });
  layout->addRow(tr("License:"), licenseCombo_);

  // ROS Distribution
  distroCombo_ = new QComboBox();
  distroCombo_->addItems({
    "humble",
    "iron",
    "jazzy",
    "rolling"
  });
  layout->addRow(tr("ROS Distribution:"), distroCombo_);

  // Register fields
  registerField("packageName*", nameEdit_);
  registerField("maintainer*", maintainerEdit_);
  registerField("email*", emailEdit_);
}

void PackageInfoPage::initializePage() {
  // Set focus to name field
  nameEdit_->setFocus();
  updateNextButtonTooltip();
}

void PackageInfoPage::updateNextButtonTooltip() {
  if (wizard()) {
    QAbstractButton* nextButton = wizard()->button(QWizard::NextButton);
    if (nextButton) {
      if (isComplete()) {
        nextButton->setToolTip(QString());
      } else {
        nextButton->setToolTip(getIncompleteReason());
      }
    }
  }
}

QString PackageInfoPage::getIncompleteReason() const {
  if (nameEdit_->text().isEmpty()) {
    return tr("Enter a package name to continue");
  }
  QRegularExpression validName("^[a-z][a-z0-9_]*$");
  if (!validName.match(nameEdit_->text()).hasMatch()) {
    return tr("Package name must start with lowercase letter and contain only lowercase letters, numbers, and underscores");
  }
  if (maintainerEdit_->text().isEmpty()) {
    return tr("Enter a maintainer name to continue");
  }
  if (emailEdit_->text().isEmpty()) {
    return tr("Enter a maintainer email to continue");
  }
  if (!emailEdit_->text().contains('@')) {
    return tr("Enter a valid email address to continue");
  }
  return QString();
}

bool PackageInfoPage::isComplete() const {
  return getIncompleteReason().isEmpty();
}

void PackageInfoPage::onPackageNameChanged(const QString& text) {
  QRegularExpression validName("^[a-z][a-z0-9_]*$");
  if (text.isEmpty()) {
    nameValidationLabel_->setText("");
  } else if (!validName.match(text).hasMatch()) {
    nameValidationLabel_->setText(tr("Must start with lowercase letter, contain only lowercase letters, numbers, and underscores"));
  } else {
    nameValidationLabel_->setText("");
  }
  emit completeChanged();
}

bool PackageInfoPage::validatePage() {
  QRegularExpression validName("^[a-z][a-z0-9_]*$");
  if (!validName.match(nameEdit_->text()).hasMatch()) {
    QMessageBox::warning(this, tr("Invalid Package Name"),
      tr("Package name must start with a lowercase letter and contain only "
         "lowercase letters, numbers, and underscores."));
    return false;
  }

  // Simple email validation
  if (!emailEdit_->text().contains('@')) {
    QMessageBox::warning(this, tr("Invalid Email"),
      tr("Please enter a valid email address."));
    return false;
  }

  return true;
}

QString PackageInfoPage::packageName() const { return nameEdit_->text(); }
QString PackageInfoPage::packageVersion() const { return versionEdit_->text(); }
QString PackageInfoPage::description() const { return descriptionEdit_->toPlainText(); }
QString PackageInfoPage::maintainer() const { return maintainerEdit_->text(); }
QString PackageInfoPage::maintainerEmail() const { return emailEdit_->text(); }
QString PackageInfoPage::license() const { return licenseCombo_->currentText(); }
QString PackageInfoPage::rosDistro() const { return distroCombo_->currentText(); }

// =============================================================================
// OutputConfigPage (Step 2)
// =============================================================================

OutputConfigPage::OutputConfigPage(QWidget* parent)
  : QWizardPage(parent)
{
  setTitle(tr("Output Configuration"));
  setSubTitle(tr("Choose where to generate the ROS2 package."));
  setupUi();
}

void OutputConfigPage::setupUi() {
  QVBoxLayout* layout = new QVBoxLayout(this);

  // Output path
  QHBoxLayout* pathLayout = new QHBoxLayout();
  pathEdit_ = new QLineEdit();
  pathEdit_->setPlaceholderText(tr("Select output directory..."));
  browseButton_ = new QPushButton(tr("Browse..."));
  pathLayout->addWidget(pathEdit_);
  pathLayout->addWidget(browseButton_);
  layout->addLayout(pathLayout);

  connect(browseButton_, &QPushButton::clicked, this, &OutputConfigPage::onBrowse);
  connect(pathEdit_, &QLineEdit::textChanged, this, &OutputConfigPage::updatePreview);
  connect(pathEdit_, &QLineEdit::textChanged, this, &OutputConfigPage::updateNextButtonTooltip);
  connect(this, &QWizardPage::completeChanged, this, &OutputConfigPage::updateNextButtonTooltip);

  // Create subdirectory option
  subdirCheck_ = new QCheckBox(tr("Create subdirectory with package name"));
  subdirCheck_->setChecked(true);
  layout->addWidget(subdirCheck_);
  connect(subdirCheck_, &QCheckBox::toggled, this, &OutputConfigPage::updatePreview);

  // Overwrite behavior
  QGroupBox* overwriteGroup = new QGroupBox(tr("If files exist:"));
  QVBoxLayout* overwriteLayout = new QVBoxLayout(overwriteGroup);
  overwriteGroup_ = new QButtonGroup(this);

  askRadio_ = new QRadioButton(tr("Ask before overwriting"));
  overwriteRadio_ = new QRadioButton(tr("Overwrite existing files"));
  skipRadio_ = new QRadioButton(tr("Skip existing files"));

  askRadio_->setChecked(true);
  overwriteGroup_->addButton(askRadio_, 0);
  overwriteGroup_->addButton(overwriteRadio_, 1);
  overwriteGroup_->addButton(skipRadio_, 2);

  overwriteLayout->addWidget(askRadio_);
  overwriteLayout->addWidget(overwriteRadio_);
  overwriteLayout->addWidget(skipRadio_);
  layout->addWidget(overwriteGroup);

  // Preview
  QLabel* previewLabel = new QLabel(tr("Output directory structure:"));
  layout->addWidget(previewLabel);

  previewTree_ = new QTreeWidget();
  previewTree_->setHeaderHidden(true);
  previewTree_->setMaximumHeight(150);
  layout->addWidget(previewTree_);

  registerField("outputPath*", pathEdit_);
}

void OutputConfigPage::initializePage() {
  if (pathEdit_->text().isEmpty()) {
    QString defaultPath = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
    pathEdit_->setText(defaultPath);
  }
  updatePreview();
  updateNextButtonTooltip();
}

void OutputConfigPage::updateNextButtonTooltip() {
  if (wizard()) {
    QAbstractButton* nextButton = wizard()->button(QWizard::NextButton);
    if (nextButton) {
      if (isComplete()) {
        nextButton->setToolTip(QString());
      } else {
        nextButton->setToolTip(getIncompleteReason());
      }
    }
  }
}

QString OutputConfigPage::getIncompleteReason() const {
  if (pathEdit_->text().isEmpty()) {
    return tr("Select an output directory to continue");
  }
  return QString();
}

bool OutputConfigPage::isComplete() const {
  return !pathEdit_->text().isEmpty();
}

void OutputConfigPage::onBrowse() {
  QString dir = QFileDialog::getExistingDirectory(
    this,
    tr("Select Output Directory"),
    pathEdit_->text().isEmpty() ?
      QStandardPaths::writableLocation(QStandardPaths::HomeLocation) :
      pathEdit_->text()
  );

  if (!dir.isEmpty()) {
    pathEdit_->setText(dir);
    updatePreview();
  }
}

void OutputConfigPage::updatePreview() {
  previewTree_->clear();

  QString basePath = pathEdit_->text();
  if (basePath.isEmpty()) {
    return;
  }

  QString packageName = field("packageName").toString();
  if (packageName.isEmpty()) {
    packageName = "my_ros_package";
  }

  QString fullPath = basePath;
  if (subdirCheck_->isChecked()) {
    fullPath += "/" + packageName;
  }

  QTreeWidgetItem* rootItem = new QTreeWidgetItem(previewTree_);
  rootItem->setText(0, QFileInfo(fullPath).fileName() + "/");
  rootItem->setExpanded(true);

  // Add expected files
  QStringList files = {
    "CMakeLists.txt",
    "package.xml",
    "src/",
    "include/" + packageName + "/",
    "launch/",
    "config/"
  };

  for (const QString& file : files) {
    QTreeWidgetItem* item = new QTreeWidgetItem(rootItem);
    item->setText(0, file);
  }
}

bool OutputConfigPage::validatePage() {
  QString path = pathEdit_->text();
  if (path.isEmpty()) {
    QMessageBox::warning(this, tr("Invalid Path"),
      tr("Please select an output directory."));
    return false;
  }

  QDir dir(path);
  if (!dir.exists()) {
    QMessageBox::StandardButton reply = QMessageBox::question(this,
      tr("Create Directory?"),
      tr("The directory does not exist. Create it?"),
      QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::Yes) {
      if (!dir.mkpath(".")) {
        QMessageBox::critical(this, tr("Error"),
          tr("Failed to create directory."));
        return false;
      }
    } else {
      return false;
    }
  }

  return true;
}

QString OutputConfigPage::outputPath() const { return pathEdit_->text(); }
bool OutputConfigPage::createSubdirectory() const { return subdirCheck_->isChecked(); }

WizardGeneratorOptions::OverwriteBehavior OutputConfigPage::overwriteBehavior() const {
  if (overwriteRadio_->isChecked()) return WizardGeneratorOptions::Overwrite;
  if (skipRadio_->isChecked()) return WizardGeneratorOptions::Skip;
  return WizardGeneratorOptions::Ask;
}

// =============================================================================
// NodeSelectionPage (Step 3)
// =============================================================================

NodeSelectionPage::NodeSelectionPage(const Project& project, QWidget* parent)
  : QWizardPage(parent)
  , project_(project)
{
  setTitle(tr("Node Selection"));
  setSubTitle(tr("Select the nodes to include in the generated package."));
  setupUi();
}

void NodeSelectionPage::setupUi() {
  QVBoxLayout* layout = new QVBoxLayout(this);

  // Selection buttons
  QHBoxLayout* buttonLayout = new QHBoxLayout();
  selectAllButton_ = new QPushButton(tr("Select All"));
  deselectAllButton_ = new QPushButton(tr("Deselect All"));
  selectionCountLabel_ = new QLabel();
  buttonLayout->addWidget(selectAllButton_);
  buttonLayout->addWidget(deselectAllButton_);
  buttonLayout->addStretch();
  buttonLayout->addWidget(selectionCountLabel_);
  layout->addLayout(buttonLayout);

  connect(selectAllButton_, &QPushButton::clicked, this, &NodeSelectionPage::onSelectAll);
  connect(deselectAllButton_, &QPushButton::clicked, this, &NodeSelectionPage::onDeselectAll);

  // Node list
  nodeList_ = new QListWidget();
  nodeList_->setSelectionMode(QAbstractItemView::NoSelection);
  layout->addWidget(nodeList_);

  connect(nodeList_, &QListWidget::itemChanged, this, &NodeSelectionPage::updateSelectionCount);
  connect(this, &QWizardPage::completeChanged, this, &NodeSelectionPage::updateNextButtonTooltip);
}

void NodeSelectionPage::initializePage() {
  nodeList_->clear();

  for (const BlockData& block : project_.blocks()) {
    QListWidgetItem* item = new QListWidgetItem(block.name);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(Qt::Checked);
    item->setData(Qt::UserRole, block.id);

    // Show pin info
    QString tooltip = QString("Inputs: %1, Outputs: %2")
      .arg(block.inputPins.size())
      .arg(block.outputPins.size());
    if (!block.parameters.isEmpty()) {
      tooltip += QString(", Parameters: %1").arg(block.parameters.size());
    }
    item->setToolTip(tooltip);

    nodeList_->addItem(item);
  }

  updateSelectionCount();
  updateNextButtonTooltip();
}

void NodeSelectionPage::updateNextButtonTooltip() {
  if (wizard()) {
    QAbstractButton* nextButton = wizard()->button(QWizard::NextButton);
    if (nextButton) {
      if (isComplete()) {
        nextButton->setToolTip(QString());
      } else {
        nextButton->setToolTip(getIncompleteReason());
      }
    }
  }
}

QString NodeSelectionPage::getIncompleteReason() const {
  if (selectedNodeIds().isEmpty()) {
    return tr("Select at least one node to continue");
  }
  return QString();
}

bool NodeSelectionPage::isComplete() const {
  return !selectedNodeIds().isEmpty();
}

void NodeSelectionPage::onSelectAll() {
  for (int i = 0; i < nodeList_->count(); ++i) {
    nodeList_->item(i)->setCheckState(Qt::Checked);
  }
}

void NodeSelectionPage::onDeselectAll() {
  for (int i = 0; i < nodeList_->count(); ++i) {
    nodeList_->item(i)->setCheckState(Qt::Unchecked);
  }
}

void NodeSelectionPage::updateSelectionCount() {
  int selected = 0;
  for (int i = 0; i < nodeList_->count(); ++i) {
    if (nodeList_->item(i)->checkState() == Qt::Checked) {
      selected++;
    }
  }
  selectionCountLabel_->setText(tr("%1 of %2 nodes selected")
    .arg(selected).arg(nodeList_->count()));
  emit completeChanged();
}

bool NodeSelectionPage::validatePage() {
  if (selectedNodeIds().isEmpty()) {
    QMessageBox::warning(this, tr("No Nodes Selected"),
      tr("Please select at least one node to include in the package."));
    return false;
  }
  return true;
}

QList<QUuid> NodeSelectionPage::selectedNodeIds() const {
  QList<QUuid> ids;
  for (int i = 0; i < nodeList_->count(); ++i) {
    if (nodeList_->item(i)->checkState() == Qt::Checked) {
      ids.append(nodeList_->item(i)->data(Qt::UserRole).toUuid());
    }
  }
  return ids;
}

// =============================================================================
// LanguageStylePage (Step 4)
// =============================================================================

LanguageStylePage::LanguageStylePage(QWidget* parent)
  : QWizardPage(parent)
{
  setTitle(tr("Language & Style"));
  setSubTitle(tr("Choose the programming language and code style options."));
  setupUi();
}

void LanguageStylePage::setupUi() {
  QVBoxLayout* layout = new QVBoxLayout(this);

  // Language selection
  QGroupBox* languageGroup = new QGroupBox(tr("Programming Language"));
  QHBoxLayout* langLayout = new QHBoxLayout(languageGroup);
  languageGroup_ = new QButtonGroup(this);

  cppRadio_ = new QRadioButton(tr("C++ (Recommended)"));
  pythonRadio_ = new QRadioButton(tr("Python"));
  cppRadio_->setChecked(true);

  languageGroup_->addButton(cppRadio_, 0);
  languageGroup_->addButton(pythonRadio_, 1);

  langLayout->addWidget(cppRadio_);
  langLayout->addWidget(pythonRadio_);
  langLayout->addStretch();
  layout->addWidget(languageGroup);

  // Code style options
  QGroupBox* styleGroup = new QGroupBox(tr("Code Style Options"));
  QFormLayout* styleLayout = new QFormLayout(styleGroup);

  namespaceCombo_ = new QComboBox();
  namespaceCombo_->addItems({
    "snake_case (recommended)",
    "CamelCase",
    "lowercase"
  });
  styleLayout->addRow(tr("Namespace Convention:"), namespaceCombo_);

  commentCombo_ = new QComboBox();
  commentCombo_->addItems({
    "Minimal (few comments)",
    "Standard (balanced)",
    "Verbose (detailed)"
  });
  commentCombo_->setCurrentIndex(1);
  styleLayout->addRow(tr("Comment Style:"), commentCombo_);

  ros2StyleCheck_ = new QCheckBox(tr("Follow ROS2 coding style guidelines"));
  ros2StyleCheck_->setChecked(true);
  styleLayout->addRow("", ros2StyleCheck_);

  layout->addWidget(styleGroup);
  layout->addStretch();
}

void LanguageStylePage::initializePage() {
  // Nothing special needed
}

bool LanguageStylePage::useCpp() const { return cppRadio_->isChecked(); }

QString LanguageStylePage::namespaceConvention() const {
  QString text = namespaceCombo_->currentText();
  if (text.contains("CamelCase")) return "CamelCase";
  if (text.contains("lowercase")) return "lowercase";
  return "snake_case";
}

WizardGeneratorOptions::CommentStyle LanguageStylePage::commentStyle() const {
  int idx = commentCombo_->currentIndex();
  if (idx == 0) return WizardGeneratorOptions::Minimal;
  if (idx == 2) return WizardGeneratorOptions::Verbose;
  return WizardGeneratorOptions::Standard;
}

bool LanguageStylePage::ros2StyleCompliance() const { return ros2StyleCheck_->isChecked(); }

// =============================================================================
// GeneratedFilesPage (Step 5)
// =============================================================================

GeneratedFilesPage::GeneratedFilesPage(QWidget* parent)
  : QWizardPage(parent)
{
  setTitle(tr("Generated Files"));
  setSubTitle(tr("Select which files to generate."));
  setupUi();
}

void GeneratedFilesPage::setupUi() {
  QVBoxLayout* layout = new QVBoxLayout(this);

  // Required files
  QGroupBox* requiredGroup = new QGroupBox(tr("Required Files"));
  QVBoxLayout* requiredLayout = new QVBoxLayout(requiredGroup);

  cmakeCheck_ = new QCheckBox(tr("CMakeLists.txt (required for C++)"));
  cmakeCheck_->setChecked(true);
  requiredLayout->addWidget(cmakeCheck_);

  packageXmlCheck_ = new QCheckBox(tr("package.xml (required)"));
  packageXmlCheck_->setChecked(true);
  packageXmlCheck_->setEnabled(false);  // Always required
  requiredLayout->addWidget(packageXmlCheck_);

  layout->addWidget(requiredGroup);

  // Optional files
  QGroupBox* optionalGroup = new QGroupBox(tr("Optional Files"));
  QVBoxLayout* optionalLayout = new QVBoxLayout(optionalGroup);

  launchCheck_ = new QCheckBox(tr("Launch file"));
  launchCheck_->setChecked(true);
  optionalLayout->addWidget(launchCheck_);

  // Launch format selection
  QHBoxLayout* launchFormatLayout = new QHBoxLayout();
  launchFormatLayout->setContentsMargins(20, 0, 0, 0);
  launchFormatGroup_ = new QButtonGroup(this);
  launchPythonRadio_ = new QRadioButton(tr("Python (recommended)"));
  launchXmlRadio_ = new QRadioButton(tr("XML"));
  launchPythonRadio_->setChecked(true);
  launchFormatGroup_->addButton(launchPythonRadio_, 0);
  launchFormatGroup_->addButton(launchXmlRadio_, 1);
  launchFormatLayout->addWidget(launchPythonRadio_);
  launchFormatLayout->addWidget(launchXmlRadio_);
  launchFormatLayout->addStretch();
  optionalLayout->addLayout(launchFormatLayout);

  connect(launchCheck_, &QCheckBox::toggled, launchPythonRadio_, &QWidget::setEnabled);
  connect(launchCheck_, &QCheckBox::toggled, launchXmlRadio_, &QWidget::setEnabled);

  paramsCheck_ = new QCheckBox(tr("Parameters YAML (config/params.yaml)"));
  paramsCheck_->setChecked(true);
  optionalLayout->addWidget(paramsCheck_);

  readmeCheck_ = new QCheckBox(tr("README.md for generated package"));
  readmeCheck_->setChecked(false);
  optionalLayout->addWidget(readmeCheck_);

  testsCheck_ = new QCheckBox(tr("Unit test stubs"));
  testsCheck_->setChecked(false);
  optionalLayout->addWidget(testsCheck_);

  layout->addWidget(optionalGroup);
  layout->addStretch();
}

void GeneratedFilesPage::initializePage() {
  // Update CMakeLists requirement based on language
  PackageWizard* wizard = qobject_cast<PackageWizard*>(this->wizard());
  if (wizard) {
    LanguageStylePage* langPage = qobject_cast<LanguageStylePage*>(
      wizard->page(PackageWizard::Page_LanguageStyle));
    if (langPage && !langPage->useCpp()) {
      cmakeCheck_->setText(tr("CMakeLists.txt (optional for Python)"));
    } else {
      cmakeCheck_->setText(tr("CMakeLists.txt (required for C++)"));
    }
  }
}

void GeneratedFilesPage::onLanguageChanged() {
  // Called if we need to update UI based on language selection
}

bool GeneratedFilesPage::generateCMakeLists() const { return cmakeCheck_->isChecked(); }
bool GeneratedFilesPage::generatePackageXml() const { return packageXmlCheck_->isChecked(); }
bool GeneratedFilesPage::generateLaunchFile() const { return launchCheck_->isChecked(); }
bool GeneratedFilesPage::generateParamsYaml() const { return paramsCheck_->isChecked(); }
bool GeneratedFilesPage::generateReadme() const { return readmeCheck_->isChecked(); }
bool GeneratedFilesPage::generateTestStubs() const { return testsCheck_->isChecked(); }
bool GeneratedFilesPage::launchFileXml() const { return launchXmlRadio_->isChecked(); }

// =============================================================================
// ParametersConfigPage (Step 6)
// =============================================================================

ParametersConfigPage::ParametersConfigPage(const Project& project, QWidget* parent)
  : QWizardPage(parent)
  , project_(project)
{
  setTitle(tr("Parameters Configuration"));
  setSubTitle(tr("Review and edit default parameter values."));
  setupUi();
}

void ParametersConfigPage::setupUi() {
  QVBoxLayout* layout = new QVBoxLayout(this);

  // Options
  QHBoxLayout* optionsLayout = new QHBoxLayout();
  modifiedOnlyCheck_ = new QCheckBox(tr("Export only modified parameters"));
  expandButton_ = new QPushButton(tr("Expand All"));
  collapseButton_ = new QPushButton(tr("Collapse All"));

  optionsLayout->addWidget(modifiedOnlyCheck_);
  optionsLayout->addStretch();
  optionsLayout->addWidget(expandButton_);
  optionsLayout->addWidget(collapseButton_);
  layout->addLayout(optionsLayout);

  connect(expandButton_, &QPushButton::clicked, this, &ParametersConfigPage::onExpandAll);
  connect(collapseButton_, &QPushButton::clicked, this, &ParametersConfigPage::onCollapseAll);

  // Parameter tree
  paramTree_ = new QTreeWidget();
  paramTree_->setHeaderLabels({tr("Parameter"), tr("Value"), tr("Type")});
  paramTree_->setColumnWidth(0, 250);
  paramTree_->setColumnWidth(1, 200);
  paramTree_->setAlternatingRowColors(true);
  layout->addWidget(paramTree_);

  connect(paramTree_, &QTreeWidget::itemChanged,
          this, &ParametersConfigPage::onItemChanged);
}

void ParametersConfigPage::initializePage() {
  populateParameters();
}

void ParametersConfigPage::populateParameters() {
  paramTree_->clear();

  // Get selected nodes from previous page
  PackageWizard* wizard = qobject_cast<PackageWizard*>(this->wizard());
  QList<QUuid> selectedIds;
  if (wizard) {
    NodeSelectionPage* nodePage = qobject_cast<NodeSelectionPage*>(
      wizard->page(PackageWizard::Page_NodeSelection));
    if (nodePage) {
      selectedIds = nodePage->selectedNodeIds();
    }
  }

  for (const BlockData& block : project_.blocks()) {
    // Skip unselected nodes
    if (!selectedIds.isEmpty() && !selectedIds.contains(block.id)) {
      continue;
    }

    QTreeWidgetItem* nodeItem = new QTreeWidgetItem(paramTree_);
    nodeItem->setText(0, block.name);
    nodeItem->setExpanded(true);
    QFont boldFont = nodeItem->font(0);
    boldFont.setBold(true);
    nodeItem->setFont(0, boldFont);

    // Add parameters for this node
    for (const BlockParamData& param : block.parameters) {
      QTreeWidgetItem* paramItem = new QTreeWidgetItem(nodeItem);
      paramItem->setText(0, param.name);
      paramItem->setText(1, param.currentValue.toString());
      paramItem->setText(2, param.type);
      paramItem->setFlags(paramItem->flags() | Qt::ItemIsEditable);
      paramItem->setData(0, Qt::UserRole, param.name);
      paramItem->setData(0, Qt::UserRole + 1, block.id);
    }

    // If no parameters, show a note
    if (block.parameters.isEmpty()) {
      QTreeWidgetItem* emptyItem = new QTreeWidgetItem(nodeItem);
      emptyItem->setText(0, tr("(No parameters defined)"));
      emptyItem->setFlags(emptyItem->flags() & ~Qt::ItemIsEditable);
      emptyItem->setForeground(0, Qt::gray);
    }
  }

  paramTree_->expandAll();
}

void ParametersConfigPage::onItemChanged(QTreeWidgetItem* item, int column) {
  if (column == 1) {
    // Value was changed - could add validation here
    item->setData(1, Qt::UserRole, true);  // Mark as modified
  }
}

void ParametersConfigPage::onExpandAll() {
  paramTree_->expandAll();
}

void ParametersConfigPage::onCollapseAll() {
  paramTree_->collapseAll();
}

bool ParametersConfigPage::exportOnlyModifiedParams() const {
  return modifiedOnlyCheck_->isChecked();
}

// =============================================================================
// ReviewGeneratePage (Step 7)
// =============================================================================

ReviewGeneratePage::ReviewGeneratePage(const Project& project, QWidget* parent)
  : QWizardPage(parent)
  , project_(project)
  , codeGenerator_(nullptr)
  , generationComplete_(false)
  , generationSuccess_(false)
{
  setTitle(tr("Review & Generate"));
  setSubTitle(tr("Review your settings and generate the package."));
  setCommitPage(true);
  setupUi();
}

void ReviewGeneratePage::setupUi() {
  QVBoxLayout* layout = new QVBoxLayout(this);

  // Summary
  QLabel* summaryLabel = new QLabel(tr("Summary:"));
  layout->addWidget(summaryLabel);

  summaryText_ = new QTextEdit();
  summaryText_->setReadOnly(true);
  summaryText_->setMaximumHeight(180);
  layout->addWidget(summaryText_);

  // Files preview
  QLabel* filesLabel = new QLabel(tr("Files to be generated:"));
  layout->addWidget(filesLabel);

  filesPreview_ = new QTreeWidget();
  filesPreview_->setHeaderHidden(true);
  filesPreview_->setMaximumHeight(120);
  layout->addWidget(filesPreview_);

  // Progress
  progressBar_ = new QProgressBar();
  progressBar_->setVisible(false);
  layout->addWidget(progressBar_);

  statusLabel_ = new QLabel();
  layout->addWidget(statusLabel_);

  // Generate button
  QHBoxLayout* buttonLayout = new QHBoxLayout();
  buttonLayout->addStretch();
  generateButton_ = new QPushButton(tr("Generate Package"));
  generateButton_->setMinimumWidth(150);
  buttonLayout->addWidget(generateButton_);
  buttonLayout->addStretch();
  layout->addLayout(buttonLayout);

  connect(generateButton_, &QPushButton::clicked, this, &ReviewGeneratePage::onGenerate);
  connect(this, &QWizardPage::completeChanged, this, &ReviewGeneratePage::updateFinishButtonTooltip);

  // Create code generator
  codeGenerator_ = new CodeGenerator(this);
  connect(codeGenerator_, &CodeGenerator::generationProgress,
          this, &ReviewGeneratePage::onGenerationProgress);
  connect(codeGenerator_, &CodeGenerator::generationFinished,
          this, &ReviewGeneratePage::onGenerationFinished);
}

void ReviewGeneratePage::initializePage() {
  generationComplete_ = false;
  generationSuccess_ = false;
  generateButton_->setEnabled(true);
  progressBar_->setVisible(false);
  statusLabel_->clear();
  statusLabel_->setStyleSheet("");

  updateSummary();
  updateFinishButtonTooltip();
}

void ReviewGeneratePage::updateFinishButtonTooltip() {
  if (wizard()) {
    QAbstractButton* finishButton = wizard()->button(QWizard::FinishButton);
    if (finishButton) {
      if (isComplete()) {
        finishButton->setToolTip(QString());
      } else {
        finishButton->setToolTip(getIncompleteReason());
      }
    }
  }
}

QString ReviewGeneratePage::getIncompleteReason() const {
  if (!generationComplete_) {
    return tr("Click 'Generate Package' to generate the ROS2 package before finishing");
  }
  if (!generationSuccess_) {
    return tr("Package generation failed - fix the errors and try again");
  }
  return QString();
}

void ReviewGeneratePage::updateSummary() {
  QString summary;
  QTextStream stream(&summary);

  PackageWizard* wizard = qobject_cast<PackageWizard*>(this->wizard());
  if (!wizard) return;

  // Package info
  PackageInfoPage* infoPage = qobject_cast<PackageInfoPage*>(
    wizard->page(PackageWizard::Page_PackageInfo));
  if (infoPage) {
    stream << "<b>Package:</b> " << infoPage->packageName() << "<br>";
    stream << "<b>Version:</b> " << infoPage->packageVersion() << "<br>";
    stream << "<b>License:</b> " << infoPage->license() << "<br>";
    stream << "<b>ROS Distro:</b> " << infoPage->rosDistro() << "<br>";
  }

  // Output path
  OutputConfigPage* outputPage = qobject_cast<OutputConfigPage*>(
    wizard->page(PackageWizard::Page_OutputConfig));
  if (outputPage) {
    QString path = outputPage->outputPath();
    if (outputPage->createSubdirectory() && infoPage) {
      path += "/" + infoPage->packageName();
    }
    stream << "<b>Output:</b> " << path << "<br>";
  }

  // Node count
  NodeSelectionPage* nodePage = qobject_cast<NodeSelectionPage*>(
    wizard->page(PackageWizard::Page_NodeSelection));
  if (nodePage) {
    stream << "<b>Nodes:</b> " << nodePage->selectedNodeIds().size() << " selected<br>";
  }

  // Language
  LanguageStylePage* langPage = qobject_cast<LanguageStylePage*>(
    wizard->page(PackageWizard::Page_LanguageStyle));
  if (langPage) {
    stream << "<b>Language:</b> " << (langPage->useCpp() ? "C++" : "Python") << "<br>";
  }

  summaryText_->setHtml(summary);

  // Update files preview
  filesPreview_->clear();
  QTreeWidgetItem* root = new QTreeWidgetItem(filesPreview_);
  QString packageName = infoPage ? infoPage->packageName() : "package";
  root->setText(0, packageName + "/");
  root->setExpanded(true);

  GeneratedFilesPage* filesPage = qobject_cast<GeneratedFilesPage*>(
    wizard->page(PackageWizard::Page_GeneratedFiles));
  if (filesPage) {
    if (filesPage->generateCMakeLists()) {
      new QTreeWidgetItem(root, QStringList("CMakeLists.txt"));
    }
    if (filesPage->generatePackageXml()) {
      new QTreeWidgetItem(root, QStringList("package.xml"));
    }

    // Source files
    if (nodePage) {
      QTreeWidgetItem* srcItem = new QTreeWidgetItem(root, QStringList("src/"));
      srcItem->setExpanded(true);
      QString ext = langPage && langPage->useCpp() ? ".cpp" : ".py";
      for (const QUuid& id : nodePage->selectedNodeIds()) {
        for (const BlockData& block : project_.blocks()) {
          if (block.id == id) {
            new QTreeWidgetItem(srcItem, QStringList(block.name.toLower() + "_node" + ext));
            break;
          }
        }
      }
    }

    if (filesPage->generateLaunchFile()) {
      QTreeWidgetItem* launchItem = new QTreeWidgetItem(root, QStringList("launch/"));
      QString launchExt = filesPage->launchFileXml() ? ".xml" : ".py";
      new QTreeWidgetItem(launchItem, QStringList(packageName + "_launch" + launchExt));
    }
    if (filesPage->generateParamsYaml()) {
      QTreeWidgetItem* configItem = new QTreeWidgetItem(root, QStringList("config/"));
      new QTreeWidgetItem(configItem, QStringList("params.yaml"));
    }
    if (filesPage->generateReadme()) {
      new QTreeWidgetItem(root, QStringList("README.md"));
    }
  }
}

void ReviewGeneratePage::onGenerate() {
  generateButton_->setEnabled(false);
  progressBar_->setVisible(true);
  progressBar_->setValue(0);
  statusLabel_->setText(tr("Generating..."));

  emit generationStarted();

  if (!performGeneration()) {
    statusLabel_->setText(tr("Generation failed: %1").arg(codeGenerator_->lastError()));
    generateButton_->setEnabled(true);
    progressBar_->setVisible(false);
  }
}

bool ReviewGeneratePage::performGeneration() {
  PackageWizard* wizard = qobject_cast<PackageWizard*>(this->wizard());
  if (!wizard) return false;

  // Gather options from pages
  PackageInfoPage* infoPage = qobject_cast<PackageInfoPage*>(
    wizard->page(PackageWizard::Page_PackageInfo));
  OutputConfigPage* outputPage = qobject_cast<OutputConfigPage*>(
    wizard->page(PackageWizard::Page_OutputConfig));
  NodeSelectionPage* nodePage = qobject_cast<NodeSelectionPage*>(
    wizard->page(PackageWizard::Page_NodeSelection));
  LanguageStylePage* langPage = qobject_cast<LanguageStylePage*>(
    wizard->page(PackageWizard::Page_LanguageStyle));
  GeneratedFilesPage* filesPage = qobject_cast<GeneratedFilesPage*>(
    wizard->page(PackageWizard::Page_GeneratedFiles));

  if (!infoPage || !outputPage || !nodePage || !langPage || !filesPage) {
    return false;
  }

  // Build generator options
  GeneratorOptions options;
  options.packageName = infoPage->packageName();
  options.outputPath = outputPage->outputPath();
  options.maintainer = infoPage->maintainer();
  options.maintainerEmail = infoPage->maintainerEmail();
  options.license = infoPage->license();
  options.rosDistro = infoPage->rosDistro();
  options.useCppStyle = langPage->useCpp();
  options.generateLaunchFile = filesPage->generateLaunchFile();
  options.generateTests = filesPage->generateTestStubs();

  // Create filtered project with only selected nodes
  Project filteredProject;
  filteredProject.metadata() = project_.metadata();
  filteredProject.metadata().description = infoPage->description();

  QList<QUuid> selectedIds = nodePage->selectedNodeIds();
  for (const BlockData& block : project_.blocks()) {
    if (selectedIds.contains(block.id)) {
      filteredProject.addBlock(block);
    }
  }

  // Only include connections between selected nodes
  for (const ConnectionData& conn : project_.connections()) {
    if (selectedIds.contains(conn.sourceBlockId) &&
        selectedIds.contains(conn.targetBlockId)) {
      filteredProject.addConnection(conn);
    }
  }

  // Store generated path
  generatedPath_ = options.outputPath + "/" + options.packageName;

  // Generate
  return codeGenerator_->generatePackage(filteredProject, options);
}

void ReviewGeneratePage::onGenerationProgress(int percent, const QString& message) {
  progressBar_->setValue(percent);
  statusLabel_->setText(message);
}

void ReviewGeneratePage::onGenerationFinished(bool success) {
  generationComplete_ = true;
  generationSuccess_ = success;

  progressBar_->setVisible(false);

  if (success) {
    statusLabel_->setText(tr("Package generated successfully!"));
    statusLabel_->setStyleSheet("color: green; font-weight: bold;");
    generateButton_->setText(tr("Generated!"));
  } else {
    statusLabel_->setText(tr("Generation failed: %1").arg(codeGenerator_->lastError()));
    statusLabel_->setStyleSheet("color: red;");
    generateButton_->setEnabled(true);
    generateButton_->setText(tr("Retry"));
  }

  emit generationFinished(success);
  emit completeChanged();
}

bool ReviewGeneratePage::validatePage() {
  if (!generationComplete_) {
    QMessageBox::information(this, tr("Generate First"),
      tr("Please click 'Generate Package' to generate the package before finishing."));
    return false;
  }
  return generationSuccess_;
}

bool ReviewGeneratePage::isComplete() const {
  return generationComplete_ && generationSuccess_;
}

}  // namespace ros_weaver
