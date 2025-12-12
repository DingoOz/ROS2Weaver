#ifndef ROS_WEAVER_SYSTEM_MAPPING_PANEL_HPP
#define ROS_WEAVER_SYSTEM_MAPPING_PANEL_HPP

#include <QWidget>
#include <QTreeWidget>
#include <QLabel>
#include <QPushButton>
#include <QProgressBar>
#include <QCheckBox>
#include <QSpinBox>
#include <QTimer>

#include "ros_weaver/core/canvas_mapper.hpp"

namespace ros_weaver {

class SystemDiscovery;

class SystemMappingPanel : public QWidget {
  Q_OBJECT

public:
  explicit SystemMappingPanel(QWidget* parent = nullptr);
  ~SystemMappingPanel() override;

  // Set the discovery and mapper components
  void setSystemDiscovery(SystemDiscovery* discovery);
  void setCanvasMapper(CanvasMapper* mapper);

  // Update with new mapping results
  void updateResults(const MappingResults& results);

  // Clear all results
  void clearResults();

public slots:
  void onScanClicked();
  void onAutoScanToggled(bool enabled);
  void onAutoScanIntervalChanged(int seconds);

signals:
  // Emitted when user clicks on a block row
  void blockSelected(const QUuid& blockId);

  // Emitted when user wants to add an unmatched node to canvas
  void addNodeToCanvasRequested(const QString& nodeName);

  // Emitted when user clicks the scan button
  void scanRequested();

private slots:
  void onScanStarted();
  void onScanProgress(int percent, const QString& message);
  void onScanCompleted(const SystemGraph& graph);
  void onScanFailed(const QString& error);
  void onMappingCompleted(const MappingResults& results);
  void onTreeItemClicked(QTreeWidgetItem* item, int column);
  void onTreeItemDoubleClicked(QTreeWidgetItem* item, int column);

private:
  void setupUi();
  void updateSummaryLabel();
  QString confidenceToString(MatchConfidence confidence) const;
  QString confidenceToIcon(MatchConfidence confidence) const;
  QColor confidenceToColor(MatchConfidence confidence) const;

  // UI components
  QLabel* titleLabel_;
  QLabel* summaryLabel_;
  QLabel* lastScanLabel_;
  QPushButton* scanButton_;
  QProgressBar* progressBar_;
  QCheckBox* autoScanCheck_;
  QSpinBox* autoScanIntervalSpin_;
  QTreeWidget* resultsTree_;

  // Tabs for different views
  QTreeWidgetItem* matchedNodesRoot_;
  QTreeWidgetItem* unmatchedCanvasRoot_;
  QTreeWidgetItem* unmatchedSystemRoot_;
  QTreeWidgetItem* topicsRoot_;

  // Components
  SystemDiscovery* discovery_;
  CanvasMapper* mapper_;

  // State
  MappingResults lastResults_;
  bool isScanning_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_SYSTEM_MAPPING_PANEL_HPP
