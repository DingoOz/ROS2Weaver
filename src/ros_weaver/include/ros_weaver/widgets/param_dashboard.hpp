#ifndef ROS_WEAVER_WIDGETS_PARAM_DASHBOARD_HPP
#define ROS_WEAVER_WIDGETS_PARAM_DASHBOARD_HPP

#include <QWidget>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QComboBox>
#include <QLabel>
#include <QMap>
#include <QVariant>
#include <QToolButton>
#include <QMenu>

namespace YAML {
class Node;
}

namespace ros_weaver {

struct BlockParamData;
struct YamlFileInfo;
class PackageBlock;

// Represents a single parameter definition
struct ParamDefinition {
  QString name;
  QString type;           // "string", "int", "double", "bool", "array", "group"
  QVariant defaultValue;
  QVariant currentValue;
  QString description;
  QVariant minValue;      // For numeric types
  QVariant maxValue;      // For numeric types
  QStringList enumValues; // For enum/choice types
  QString group;          // Parameter group/namespace (e.g., "slam", "controller")
  bool isValid = true;    // Validation state
  QString validationError; // Error message if invalid
};

// Represents parameters for a node
struct NodeParams {
  QString nodeName;
  QString namespace_;
  QList<ParamDefinition> params;
};

class ParamDashboard : public QWidget {
  Q_OBJECT

public:
  explicit ParamDashboard(QWidget* parent = nullptr);
  ~ParamDashboard() override;

  // Set the block to display/edit parameters for
  void setCurrentBlock(PackageBlock* block);
  PackageBlock* currentBlock() const { return currentBlock_; }

  // Get all parameters for current block
  QList<ParamDefinition> parameters() const;

  // Set parameters
  void setParameters(const QList<ParamDefinition>& params);

  // Export parameters to YAML string
  QString toYaml() const;

  // Import parameters from YAML string
  bool fromYaml(const QString& yaml);

  // Import/export YAML files
  bool importYamlFile(const QString& filePath);
  bool exportYamlFile(const QString& filePath) const;

  // Validate all parameters
  bool validateAll();

  // YAML file management
  void setProjectYamlFiles(const QList<YamlFileInfo>& yamlFiles);
  void setProjectDirectory(const QString& projectDir);
  void clearYamlFiles();

signals:
  void parameterChanged(const QString& name, const QVariant& value);
  void parametersModified();
  void validationStateChanged(bool isValid);

private slots:
  void onItemChanged(QTreeWidgetItem* item, int column);
  void onAddParameter();
  void onRemoveParameter();
  void onResetToDefaults();
  void onImportYaml();
  void onExportYaml();
  void onAddGroup();
  void onExpandAll();
  void onCollapseAll();
  void onYamlSourceChanged(int index);
  void onSyncToBlock();
  void onContextMenu(const QPoint& pos);

private:
  void setupUi();
  void populateTree();
  void addParamToTree(const ParamDefinition& param, QTreeWidgetItem* parentItem = nullptr);
  QTreeWidgetItem* createParamItem(const ParamDefinition& param);
  QWidget* createValueEditor(const ParamDefinition& param, QTreeWidgetItem* item);
  QTreeWidgetItem* findOrCreateGroupItem(const QString& groupName);
  void updateValidationState(QTreeWidgetItem* item, bool isValid, const QString& errorMsg = QString());

  // Default parameters for known node types
  void loadDefaultsForNodeType(const QString& nodeType);

  // Validation helpers
  bool validateParameter(ParamDefinition& param);
  bool validateNumericRange(ParamDefinition& param);
  bool validateStringValue(ParamDefinition& param);

  // YAML helpers
  void parseYamlParams(const YAML::Node& node, const QString& group);

  // Conversion helpers for BlockParamData
  static ParamDefinition fromBlockParam(const BlockParamData& blockParam);
  static BlockParamData toBlockParam(const ParamDefinition& param);
  void saveParametersToBlock();
  void loadParametersFromBlock();

  // YAML file helpers
  void loadYamlFileParams(const QString& filePath);
  void updateYamlSourceCombo();
  QString resolveYamlPath(const QString& relativePath) const;
  void loadBlockParamsDefault(PackageBlock* block);
  void selectYamlSourceAutoDetect(PackageBlock* block);

  QTreeWidget* paramTree_;
  QLineEdit* searchEdit_;
  QPushButton* addButton_;
  QPushButton* removeButton_;
  QPushButton* resetButton_;
  QToolButton* importExportButton_;
  QToolButton* groupButton_;
  QLabel* nodeNameLabel_;
  QLabel* validationLabel_;

  // YAML source selector
  QComboBox* yamlSourceCombo_;
  QLabel* sourceLabel_;
  QPushButton* syncToBlockButton_;

  PackageBlock* currentBlock_;
  QList<ParamDefinition> params_;

  // Map group names to tree items
  QMap<QString, QTreeWidgetItem*> groupItems_;

  // Prevent recursive updates
  bool updatingTree_;

  // YAML file tracking
  QList<YamlFileInfo> yamlFiles_;
  QString projectDirectory_;
  QMap<QString, QList<ParamDefinition>> yamlParamsCache_;  // filepath -> params
  bool showingYamlParams_;  // true if showing YAML file, false if showing block params
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_PARAM_DASHBOARD_HPP
