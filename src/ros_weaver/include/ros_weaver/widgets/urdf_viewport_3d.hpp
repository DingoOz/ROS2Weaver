#ifndef ROS_WEAVER_WIDGETS_URDF_VIEWPORT_3D_HPP
#define ROS_WEAVER_WIDGETS_URDF_VIEWPORT_3D_HPP

#include <QWidget>
#include <QVector3D>
#include <QQuaternion>
#include <QColor>

// Forward declarations for Qt3D types
namespace Qt3DCore {
class QEntity;
class QTransform;
}

namespace Qt3DRender {
class QCamera;
class QMaterial;
class QMesh;
class QPickEvent;
class QGeometryRenderer;
}

namespace Qt3DExtras {
class Qt3DWindow;
class QOrbitCameraController;
class QPhongMaterial;
class QText2DEntity;
}

namespace ros_weaver {

class URDFModel;
class URDFGizmo;
class URDFParser;

/**
 * @brief 3D viewport for URDF visualization using Qt3D
 *
 * Features:
 * - Renders URDF links with visuals (meshes and primitives)
 * - Orbit camera with mouse control
 * - Grid floor
 * - Interactive selection
 * - Translation/rotation gizmo
 */
class URDFViewport3D : public QWidget {
  Q_OBJECT

public:
  explicit URDFViewport3D(QWidget* parent = nullptr);
  ~URDFViewport3D() override;

  void setModel(URDFModel* model);
  URDFModel* model() const { return model_; }

  // Parser for mesh path resolution
  void setParser(URDFParser* parser) { parser_ = parser; }

  // View controls
  void resetView();
  void focusOnSelected();
  void setGridVisible(bool visible);
  void setWireframeMode(bool wireframe);

  // View presets (Blender-style numpad shortcuts)
  void setViewFront();   // Numpad 1
  void setViewBack();    // Ctrl+Numpad 1
  void setViewRight();   // Numpad 3
  void setViewLeft();    // Ctrl+Numpad 3
  void setViewTop();     // Numpad 7
  void setViewBottom();  // Ctrl+Numpad 7

  // Overlay toggles
  void setLabelsVisible(bool visible);
  void setHierarchyLinesVisible(bool visible);
  void setLocalAxesVisible(bool visible);
  bool labelsVisible() const { return labelsVisible_; }
  bool hierarchyLinesVisible() const { return hierarchyLinesVisible_; }
  bool localAxesVisible() const { return localAxesVisible_; }

  // Selection
  void selectElement(const QString& name, bool isJoint);
  QString selectedName() const { return selectedName_; }
  bool isSelectedJoint() const { return selectedIsJoint_; }

  // Refresh/rebuild the scene
  void refresh();
  void rebuildScene();

signals:
  void selectionChanged(const QString& name, bool isJoint);
  void transformChanged();  // Emitted when gizmo moves something

protected:
  void resizeEvent(QResizeEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  bool eventFilter(QObject* watched, QEvent* event) override;

private slots:
  void onEntityPicked(Qt3DRender::QPickEvent* event);
  void onGizmoTransformChanged(const QVector3D& position, const QQuaternion& rotation);

private:
  void setupScene();
  void setupCamera();
  void setupLighting();
  void setupGrid();
  void setupGizmo();

  void clearScene();
  void buildModelEntities();
  void createLinkEntity(const QString& linkName, Qt3DCore::QEntity* parent);
  void createVisualEntity(const struct URDFVisual& visual, Qt3DCore::QEntity* parent);
  Qt3DCore::QEntity* createBoxEntity(const QVector3D& size, Qt3DRender::QMaterial* material);
  Qt3DCore::QEntity* createCylinderEntity(double radius, double length, Qt3DRender::QMaterial* material);
  Qt3DCore::QEntity* createSphereEntity(double radius, Qt3DRender::QMaterial* material);
  Qt3DCore::QEntity* createMeshEntity(const QString& filename, const QVector3D& scale, Qt3DRender::QMaterial* material);

  Qt3DExtras::QPhongMaterial* createMaterial(const struct URDFMaterial& urdfMaterial);

  void updateSelection();
  void highlightEntity(Qt3DCore::QEntity* entity, bool highlight);

  // Overlay helpers
  void setupLabels();
  void setupHierarchyLines();
  void setupLocalAxes();
  void updateLabelsVisibility();
  void updateHierarchyLinesVisibility();
  void updateLocalAxesVisibility();

  // Axis helper creation
  Qt3DCore::QEntity* createAxisIndicator(Qt3DCore::QEntity* parent);
  Qt3DCore::QEntity* createHierarchyLine(const QVector3D& from, const QVector3D& to, Qt3DCore::QEntity* parent);

  // Camera view helper
  void setCameraView(const QVector3D& position, const QVector3D& upVector);

  // Camera orbit/zoom (shared between mouse handlers and event filter)
  void orbitCamera(const QPoint& delta);
  void zoomCamera(int angleDeltaY);

  // Mesh path resolution
  QString resolveMeshPath(const QString& meshPath);

  // World transform calculation
  QVector3D getWorldPosition(Qt3DCore::QEntity* entity);
  QQuaternion getWorldRotation(Qt3DCore::QEntity* entity);

  URDFModel* model_ = nullptr;
  URDFParser* parser_ = nullptr;

  // Qt3D components
  Qt3DExtras::Qt3DWindow* view3d_ = nullptr;
  QWidget* view3dContainer_ = nullptr;
  Qt3DCore::QEntity* rootEntity_ = nullptr;
  Qt3DCore::QEntity* modelRoot_ = nullptr;
  Qt3DCore::QEntity* gridEntity_ = nullptr;
  Qt3DRender::QCamera* camera_ = nullptr;
  Qt3DExtras::QOrbitCameraController* cameraController_ = nullptr;

  // Gizmo
  URDFGizmo* gizmo_ = nullptr;

  // Entity tracking
  QMap<QString, Qt3DCore::QEntity*> linkEntities_;
  QMap<QString, Qt3DCore::QEntity*> jointEntities_;

  // Overlay entities
  Qt3DCore::QEntity* labelsRoot_ = nullptr;
  Qt3DCore::QEntity* hierarchyLinesRoot_ = nullptr;
  Qt3DCore::QEntity* localAxesRoot_ = nullptr;
  QMap<QString, Qt3DCore::QEntity*> labelEntities_;
  QMap<QString, Qt3DCore::QEntity*> axisEntities_;

  // Selection state
  QString selectedName_;
  bool selectedIsJoint_ = false;
  Qt3DCore::QEntity* selectedEntity_ = nullptr;

  // Visibility flags
  bool gridVisible_ = true;
  bool wireframeMode_ = false;
  bool labelsVisible_ = false;
  bool hierarchyLinesVisible_ = false;
  bool localAxesVisible_ = false;

  // Camera distance for views
  float viewDistance_ = 5.0f;

  // Keyboard modifiers for gizmo
  bool ctrlPressed_ = false;
  bool shiftPressed_ = false;

  // Mouse orbit state (middle mouse button)
  bool middleMousePressed_ = false;
  QPoint lastMousePos_;
  float orbitSpeed_ = 0.3f;

  // Saved material color for highlight restore
  QColor savedAmbientColor_{80, 80, 80};
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_URDF_VIEWPORT_3D_HPP
