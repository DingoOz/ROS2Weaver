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

namespace ros_weaver {

class PackageBlock;

// Represents a single parameter definition
struct ParamDefinition {
  QString name;
  QString type;           // "string", "int", "double", "bool", "array"
  QVariant defaultValue;
  QVariant currentValue;
  QString description;
  QVariant minValue;      // For numeric types
  QVariant maxValue;      // For numeric types
  QStringList enumValues; // For enum/choice types
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

signals:
  void parameterChanged(const QString& name, const QVariant& value);
  void parametersModified();

private slots:
  void onItemChanged(QTreeWidgetItem* item, int column);
  void onAddParameter();
  void onRemoveParameter();
  void onResetToDefaults();

private:
  void setupUi();
  void populateTree();
  void addParamToTree(const ParamDefinition& param);
  QTreeWidgetItem* createParamItem(const ParamDefinition& param);
  QWidget* createValueEditor(const ParamDefinition& param, QTreeWidgetItem* item);

  // Default parameters for known node types
  void loadDefaultsForNodeType(const QString& nodeType);

  QTreeWidget* paramTree_;
  QLineEdit* searchEdit_;
  QPushButton* addButton_;
  QPushButton* removeButton_;
  QPushButton* resetButton_;
  QLabel* nodeNameLabel_;

  PackageBlock* currentBlock_;
  QList<ParamDefinition> params_;

  // Prevent recursive updates
  bool updatingTree_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_PARAM_DASHBOARD_HPP
