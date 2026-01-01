#ifndef ROS_WEAVER_WIDGETS_REMAPPING_EDITOR_HPP
#define ROS_WEAVER_WIDGETS_REMAPPING_EDITOR_HPP

#include <QWidget>
#include <QTableWidget>
#include <QLineEdit>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QMap>

namespace ros_weaver {

class PackageBlock;

// A single remapping entry
struct Remapping {
  QString fromName;      // Original name
  QString toName;        // Remapped name
  QString type;          // "topic", "service", "action", "parameter", "node"
  bool isNamespace = false;  // True if this is a namespace remap

  bool operator==(const Remapping& other) const {
    return fromName == other.fromName && toName == other.toName && type == other.type;
  }
};

// Widget for editing namespace and remappings on a block
class RemappingEditor : public QWidget {
  Q_OBJECT

public:
  explicit RemappingEditor(QWidget* parent = nullptr);
  ~RemappingEditor() override = default;

  // Set the block to edit
  void setBlock(PackageBlock* block);

  // Get current block
  PackageBlock* currentBlock() const { return currentBlock_; }

  // Get all remappings
  QList<Remapping> remappings() const { return remappings_; }

  // Get namespace
  QString nodeNamespace() const;

  // Set namespace
  void setNodeNamespace(const QString& ns);

public slots:
  // Add a new remapping
  void addRemapping();

  // Remove selected remapping
  void removeSelectedRemapping();

  // Apply changes to the block
  void applyChanges();

  // Reset to original values
  void resetChanges();

signals:
  // Emitted when remappings change
  void remappingsChanged();

  // Emitted when namespace changes
  void namespaceChanged(const QString& ns);

private:
  void setupUi();
  void loadFromBlock();
  void updateTable();

  // UI components
  QLineEdit* namespaceEdit_;
  QTableWidget* remappingTable_;
  QComboBox* typeCombo_;
  QLineEdit* fromEdit_;
  QLineEdit* toEdit_;
  QPushButton* addButton_;
  QPushButton* removeButton_;
  QPushButton* applyButton_;
  QPushButton* resetButton_;
  QLabel* statusLabel_;

  // State
  PackageBlock* currentBlock_;
  QList<Remapping> remappings_;
  QString originalNamespace_;
  QList<Remapping> originalRemappings_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_REMAPPING_EDITOR_HPP
