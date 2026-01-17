#ifndef ROS_WEAVER_URDF_VIEWER_PANEL_HPP
#define ROS_WEAVER_URDF_VIEWER_PANEL_HPP

#include <QWidget>
#include <QSplitter>
#include <QToolBar>
#include <QAction>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>
#include <QRadioButton>
#include <QGroupBox>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QShortcut>
#include <QFileDialog>

#include "ros_weaver/core/urdf_parser.hpp"
#include "ros_weaver/core/urdf_joint_controller.hpp"
#include "ros_weaver/widgets/urdf_3d_view.hpp"
#include "ros_weaver/widgets/urdf_tree_view.hpp"

namespace ros_weaver {

class URDFViewerPanel : public QWidget {
  Q_OBJECT

public:
  explicit URDFViewerPanel(QWidget* parent = nullptr);
  ~URDFViewerPanel() override;

  // Get the parser for external access
  URDFParser* parser() { return parser_; }
  const URDFParser* parser() const { return parser_; }

  // Get the 3D view for external access
  URDF3DView* view3D() { return view3D_; }

public slots:
  // Load/save URDF
  void loadURDF(const QString& filePath);
  void saveURDF(const QString& filePath);
  void saveURDF();  // Uses current file path

  // UI trigger slots
  void onLoadClicked();
  void onSaveClicked();
  void onSaveAsClicked();

signals:
  void urdfLoaded(const QString& filePath);
  void urdfSaved(const QString& filePath);
  void urdfModified();
  void jointSelected(const QString& jointName);

private:
  void setupUi();
  void setupToolbar();
  void setupSidePanel();
  void setupShortcuts();
  void setupConnections();

  // Selection synchronization
  void syncSelectionFrom3DView();
  void syncSelectionFromTreeView();

  // Update UI based on selection
  void updateRotationControlsState();

private slots:
  void onRenderModeChanged(int index);
  void onShadowsToggled(bool enabled);
  void onBgColorClicked();
  void onRotationModeChanged();
  void onRotateXPosClicked();
  void onRotateXNegClicked();
  void onRotateYPosClicked();
  void onRotateYNegClicked();
  void onRotateZPosClicked();
  void onRotateZNegClicked();
  void onResetOrientationClicked();
  void onAmbientLightChanged(int value);
  void onGridToggled(bool visible);
  void onJointModified(const QString& jointName);
  void on3DViewSelectionChanged(const QStringList& selectedJoints);
  void onTreeViewSelectionChanged(const QStringList& selectedJoints);
  void onJointRotated(const QString& jointName, const QQuaternion& newOrientation);

private:
  // Main layout
  QSplitter* mainSplitter_;
  URDF3DView* view3D_;
  QWidget* sidePanel_;
  URDFTreeView* treeView_;

  // Toolbar
  QToolBar* toolbar_;
  QAction* loadAction_;
  QAction* saveAction_;
  QAction* saveAsAction_;
  QComboBox* renderModeCombo_;
  QCheckBox* shadowsCheckBox_;
  QCheckBox* gridCheckBox_;
  QPushButton* bgColorButton_;

  // Rotation controls
  QGroupBox* rotationGroup_;
  QRadioButton* snap90Radio_;
  QRadioButton* freeRotationRadio_;
  QPushButton* rotateXPosButton_;
  QPushButton* rotateXNegButton_;
  QPushButton* rotateYPosButton_;
  QPushButton* rotateYNegButton_;
  QPushButton* rotateZPosButton_;
  QPushButton* rotateZNegButton_;
  QPushButton* resetOrientationButton_;
  QDoubleSpinBox* freeAngleSpinBox_;

  // Lighting controls
  QGroupBox* lightingGroup_;
  QSlider* ambientLightSlider_;
  QLabel* ambientLightLabel_;

  // Status
  QLabel* statusLabel_;
  QLabel* filePathLabel_;

  // Core components
  URDFParser* parser_;
  URDFJointController* jointController_;

  // Keyboard shortcuts
  QShortcut* rotXPosShortcut_;
  QShortcut* rotXNegShortcut_;
  QShortcut* rotYPosShortcut_;
  QShortcut* rotYNegShortcut_;
  QShortcut* rotZPosShortcut_;
  QShortcut* rotZNegShortcut_;
  QShortcut* resetShortcut_;
  QShortcut* selectAllShortcut_;
  QShortcut* escapeShortcut_;

  // State
  QString currentFilePath_;
  bool blockSelectionSync_ = false;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_URDF_VIEWER_PANEL_HPP
