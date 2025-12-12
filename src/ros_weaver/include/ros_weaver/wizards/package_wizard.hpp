#ifndef ROS_WEAVER_WIZARDS_PACKAGE_WIZARD_HPP
#define ROS_WEAVER_WIZARDS_PACKAGE_WIZARD_HPP

#include <QWizard>
#include <QWizardPage>
#include <QLineEdit>
#include <QComboBox>
#include <QTextEdit>
#include <QCheckBox>
#include <QListWidget>
#include <QTreeWidget>
#include <QLabel>
#include <QRadioButton>
#include <QButtonGroup>
#include <QGroupBox>
#include <QSpinBox>
#include <QProgressBar>
#include <QPushButton>

namespace ros_weaver {

class Project;
class CodeGenerator;
struct BlockData;
struct GeneratorOptions;

// Extended generator options for wizard
struct WizardGeneratorOptions {
  // Package info (Step 1)
  QString packageName;
  QString packageVersion = "0.1.0";
  QString description;
  QString maintainer;
  QString maintainerEmail;
  QString license = "Apache-2.0";
  QString rosDistro = "humble";

  // Output config (Step 2)
  QString outputPath;
  bool createSubdirectory = true;
  enum OverwriteBehavior { Ask, Overwrite, Skip };
  OverwriteBehavior overwriteBehavior = Ask;

  // Node selection (Step 3)
  QList<QUuid> selectedNodeIds;

  // Language & Style (Step 4)
  bool useCpp = true;
  QString namespaceConvention = "snake_case";
  enum CommentStyle { Minimal, Standard, Verbose };
  CommentStyle commentStyle = Standard;
  bool ros2StyleCompliance = true;

  // Generated files (Step 5)
  bool generateCMakeLists = true;
  bool generatePackageXml = true;
  bool generateLaunchFile = true;
  bool generateParamsYaml = true;
  bool generateReadme = false;
  bool generateTestStubs = false;
  bool launchFileXml = false;  // false = Python, true = XML

  // Parameters (Step 6)
  bool exportOnlyModifiedParams = false;
};

// Forward declarations
class PackageInfoPage;
class OutputConfigPage;
class NodeSelectionPage;
class LanguageStylePage;
class GeneratedFilesPage;
class ParametersConfigPage;
class ReviewGeneratePage;

// Main wizard class
class PackageWizard : public QWizard {
  Q_OBJECT

public:
  explicit PackageWizard(const Project& project, QWidget* parent = nullptr);
  ~PackageWizard() override;

  // Get the configured options after wizard completes
  WizardGeneratorOptions options() const { return options_; }

  // Get the generated package path
  QString generatedPackagePath() const { return generatedPackagePath_; }

  enum PageId {
    Page_PackageInfo,
    Page_OutputConfig,
    Page_NodeSelection,
    Page_LanguageStyle,
    Page_GeneratedFiles,
    Page_ParametersConfig,
    Page_ReviewGenerate
  };

signals:
  void generationComplete(bool success, const QString& path);

protected:
  void accept() override;

private:
  const Project& project_;
  WizardGeneratorOptions options_;
  QString generatedPackagePath_;

  PackageInfoPage* packageInfoPage_;
  OutputConfigPage* outputConfigPage_;
  NodeSelectionPage* nodeSelectionPage_;
  LanguageStylePage* languageStylePage_;
  GeneratedFilesPage* generatedFilesPage_;
  ParametersConfigPage* parametersConfigPage_;
  ReviewGeneratePage* reviewGeneratePage_;
};

// Step 1: Package Information
class PackageInfoPage : public QWizardPage {
  Q_OBJECT

public:
  explicit PackageInfoPage(QWidget* parent = nullptr);

  void initializePage() override;
  bool validatePage() override;

  QString packageName() const;
  QString packageVersion() const;
  QString description() const;
  QString maintainer() const;
  QString maintainerEmail() const;
  QString license() const;
  QString rosDistro() const;

private slots:
  void onPackageNameChanged(const QString& text);

private:
  void setupUi();

  QLineEdit* nameEdit_;
  QLineEdit* versionEdit_;
  QTextEdit* descriptionEdit_;
  QLineEdit* maintainerEdit_;
  QLineEdit* emailEdit_;
  QComboBox* licenseCombo_;
  QComboBox* distroCombo_;
  QLabel* nameValidationLabel_;
};

// Step 2: Output Configuration
class OutputConfigPage : public QWizardPage {
  Q_OBJECT

public:
  explicit OutputConfigPage(QWidget* parent = nullptr);

  void initializePage() override;
  bool validatePage() override;

  QString outputPath() const;
  bool createSubdirectory() const;
  WizardGeneratorOptions::OverwriteBehavior overwriteBehavior() const;

private slots:
  void onBrowse();
  void updatePreview();

private:
  void setupUi();

  QLineEdit* pathEdit_;
  QPushButton* browseButton_;
  QCheckBox* subdirCheck_;
  QButtonGroup* overwriteGroup_;
  QRadioButton* askRadio_;
  QRadioButton* overwriteRadio_;
  QRadioButton* skipRadio_;
  QTreeWidget* previewTree_;
};

// Step 3: Node Selection
class NodeSelectionPage : public QWizardPage {
  Q_OBJECT

public:
  explicit NodeSelectionPage(const Project& project, QWidget* parent = nullptr);

  void initializePage() override;
  bool validatePage() override;

  QList<QUuid> selectedNodeIds() const;

private slots:
  void onSelectAll();
  void onDeselectAll();
  void updateSelectionCount();

private:
  void setupUi();

  const Project& project_;
  QListWidget* nodeList_;
  QPushButton* selectAllButton_;
  QPushButton* deselectAllButton_;
  QLabel* selectionCountLabel_;
};

// Step 4: Language & Style
class LanguageStylePage : public QWizardPage {
  Q_OBJECT

public:
  explicit LanguageStylePage(QWidget* parent = nullptr);

  void initializePage() override;

  bool useCpp() const;
  QString namespaceConvention() const;
  WizardGeneratorOptions::CommentStyle commentStyle() const;
  bool ros2StyleCompliance() const;

private:
  void setupUi();

  QButtonGroup* languageGroup_;
  QRadioButton* cppRadio_;
  QRadioButton* pythonRadio_;
  QComboBox* namespaceCombo_;
  QComboBox* commentCombo_;
  QCheckBox* ros2StyleCheck_;
};

// Step 5: Generated Files
class GeneratedFilesPage : public QWizardPage {
  Q_OBJECT

public:
  explicit GeneratedFilesPage(QWidget* parent = nullptr);

  void initializePage() override;

  bool generateCMakeLists() const;
  bool generatePackageXml() const;
  bool generateLaunchFile() const;
  bool generateParamsYaml() const;
  bool generateReadme() const;
  bool generateTestStubs() const;
  bool launchFileXml() const;

private slots:
  void onLanguageChanged();

private:
  void setupUi();

  QCheckBox* cmakeCheck_;
  QCheckBox* packageXmlCheck_;
  QCheckBox* launchCheck_;
  QCheckBox* paramsCheck_;
  QCheckBox* readmeCheck_;
  QCheckBox* testsCheck_;
  QButtonGroup* launchFormatGroup_;
  QRadioButton* launchPythonRadio_;
  QRadioButton* launchXmlRadio_;
};

// Step 6: Parameters Configuration
class ParametersConfigPage : public QWizardPage {
  Q_OBJECT

public:
  explicit ParametersConfigPage(const Project& project, QWidget* parent = nullptr);

  void initializePage() override;

  bool exportOnlyModifiedParams() const;

private slots:
  void onItemChanged(QTreeWidgetItem* item, int column);
  void onExpandAll();
  void onCollapseAll();

private:
  void setupUi();
  void populateParameters();

  const Project& project_;
  QTreeWidget* paramTree_;
  QCheckBox* modifiedOnlyCheck_;
  QPushButton* expandButton_;
  QPushButton* collapseButton_;
};

// Step 7: Review & Generate
class ReviewGeneratePage : public QWizardPage {
  Q_OBJECT

public:
  explicit ReviewGeneratePage(const Project& project, QWidget* parent = nullptr);

  void initializePage() override;
  bool validatePage() override;

  bool isComplete() const override;

signals:
  void generationStarted();
  void generationProgress(int percent, const QString& message);
  void generationFinished(bool success);

private slots:
  void onGenerate();
  void onGenerationProgress(int percent, const QString& message);
  void onGenerationFinished(bool success);

private:
  void setupUi();
  void updateSummary();
  bool performGeneration();

  const Project& project_;
  QTextEdit* summaryText_;
  QTreeWidget* filesPreview_;
  QProgressBar* progressBar_;
  QPushButton* generateButton_;
  QLabel* statusLabel_;

  CodeGenerator* codeGenerator_;
  bool generationComplete_;
  bool generationSuccess_;
  QString generatedPath_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIZARDS_PACKAGE_WIZARD_HPP
