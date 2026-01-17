#ifndef ROS_WEAVER_URDF_3D_VIEW_HPP
#define ROS_WEAVER_URDF_3D_VIEW_HPP

#include <QWidget>
#include <QVBoxLayout>
#include <QMap>
#include <QSet>
#include <QVector3D>
#include <QQuaternion>
#include <QMatrix4x4>
#include <QColor>
#include <QPoint>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>

#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QConeMesh>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QDirectionalLight>
#include <Qt3DRender/QPointLight>
#include <Qt3DRender/QObjectPicker>
#include <Qt3DRender/QPickEvent>

#include "ros_weaver/core/urdf_parser.hpp"

namespace ros_weaver {

// Rendering mode options
enum class RenderMode {
  Wireframe,
  BasicShading,
  FullLighting
};

// Forward declarations
class AxisIndicator;

class URDF3DView : public QWidget {
  Q_OBJECT

public:
  explicit URDF3DView(QWidget* parent = nullptr);
  ~URDF3DView() override;

  // Set the model to display
  void setModel(const URDFModel& model);
  void setParser(URDFParser* parser) { parser_ = parser; }

  // Update joint transform when model changes
  void updateJointTransform(const QString& jointName, const QQuaternion& orientation);
  void updateAllTransforms();

  // Selection management
  void selectJoint(const QString& jointName, bool addToSelection = false);
  void selectJoints(const QStringList& jointNames);
  void clearSelection();
  QStringList selectedJoints() const { return selectedJoints_; }

  // Camera controls
  void resetCamera();
  void setCameraPosition(const QVector3D& position);
  void setCameraTarget(const QVector3D& target);
  void fitToModel();

  // Rendering options
  void setRenderMode(RenderMode mode);
  RenderMode renderMode() const { return renderMode_; }
  void setShadowsEnabled(bool enabled);
  bool shadowsEnabled() const { return shadowsEnabled_; }
  void setBackgroundColor(const QColor& color);
  QColor backgroundColor() const { return backgroundColor_; }
  void setAmbientLightIntensity(float intensity);
  float ambientLightIntensity() const { return ambientIntensity_; }
  void setAxisIndicatorScale(float scale);

  // Grid visibility
  void setGridVisible(bool visible);
  bool isGridVisible() const { return gridVisible_; }

signals:
  void jointClicked(const QString& jointName, bool ctrlPressed);
  void jointDoubleClicked(const QString& jointName);
  void selectionChanged(const QStringList& selectedJoints);

protected:
  bool eventFilter(QObject* obj, QEvent* event) override;

private:
  void setupScene();
  void setupCamera();
  void setupLighting();
  void setupGrid();

  void clearModel();
  void buildModel(const URDFModel& model);
  void createLinkEntity(const URDFLink& link, Qt3DCore::QEntity* parent);
  void createJointAxisIndicator(const URDFJoint& joint);
  void createPrimitiveMesh(const QString& meshPath, Qt3DCore::QEntity* entity);

  void highlightSelectedJoints();
  QString findJointForEntity(Qt3DCore::QEntity* entity) const;

  // Mouse event handlers for Blender-style controls
  void handleMousePress(QMouseEvent* event);
  void handleMouseMove(QMouseEvent* event);
  void handleMouseRelease(QMouseEvent* event);
  void handleWheel(QWheelEvent* event);
  void handleKeyPress(QKeyEvent* event);

  // Camera orbit/pan/zoom
  void orbitCamera(int dx, int dy);
  void panCamera(int dx, int dy);
  void zoomCamera(float delta);

  // 3D window and scene
  Qt3DExtras::Qt3DWindow* view3D_;
  QWidget* container_;
  Qt3DCore::QEntity* rootEntity_;
  Qt3DCore::QEntity* modelEntity_;
  Qt3DCore::QEntity* gridEntity_;

  // Camera
  Qt3DRender::QCamera* camera_;
  QVector3D cameraTarget_;
  float cameraDistance_ = 5.0f;
  float cameraPitch_ = 30.0f;  // Degrees
  float cameraYaw_ = 45.0f;    // Degrees

  // Lighting
  Qt3DCore::QEntity* ambientLightEntity_;
  Qt3DCore::QEntity* directionalLightEntity_;
  Qt3DCore::QEntity* pointLightEntity_;
  float ambientIntensity_ = 0.3f;

  // Model entities
  QMap<QString, Qt3DCore::QEntity*> linkEntities_;
  QMap<QString, Qt3DCore::QEntity*> jointEntities_;
  QMap<QString, Qt3DCore::QTransform*> linkTransforms_;
  QMap<QString, Qt3DCore::QTransform*> jointTransforms_;
  QMap<QString, AxisIndicator*> axisIndicators_;
  QMap<Qt3DCore::QEntity*, QString> entityToJoint_;

  // Parser reference for transform computation
  URDFParser* parser_ = nullptr;

  // Selection state
  QStringList selectedJoints_;
  QColor selectionHighlightColor_ = QColor(255, 200, 0);
  QColor defaultJointColor_ = QColor(100, 100, 100);

  // Mouse state for camera control
  bool isPanning_ = false;
  bool isOrbiting_ = false;
  QPoint lastMousePos_;

  // Rendering options
  RenderMode renderMode_ = RenderMode::FullLighting;
  QColor backgroundColor_ = QColor(50, 50, 60);
  bool shadowsEnabled_ = false;
  bool gridVisible_ = true;
  float axisScale_ = 0.1f;
};

// Axis indicator for joints (RViz-style RGB coloring)
class AxisIndicator : public Qt3DCore::QEntity {
  Q_OBJECT

public:
  explicit AxisIndicator(Qt3DCore::QEntity* parent, float length = 0.1f);

  void setHighlighted(bool highlighted);
  void setScale(float scale);
  void setTransform(const QMatrix4x4& transform);

private:
  void createAxis(const QVector3D& direction, const QColor& color, float length);

  Qt3DCore::QTransform* transform_;
  Qt3DCore::QEntity* xAxisEntity_;
  Qt3DCore::QEntity* yAxisEntity_;
  Qt3DCore::QEntity* zAxisEntity_;

  // Cone tips for arrows
  Qt3DCore::QEntity* xConeEntity_;
  Qt3DCore::QEntity* yConeEntity_;
  Qt3DCore::QEntity* zConeEntity_;

  float length_;
  bool highlighted_ = false;

  // RViz convention: X=Red, Y=Green, Z=Blue
  static inline const QColor X_AXIS_COLOR = QColor(255, 0, 0);
  static inline const QColor Y_AXIS_COLOR = QColor(0, 255, 0);
  static inline const QColor Z_AXIS_COLOR = QColor(0, 0, 255);
  static inline const QColor HIGHLIGHT_COLOR = QColor(255, 255, 0);
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_URDF_3D_VIEW_HPP
