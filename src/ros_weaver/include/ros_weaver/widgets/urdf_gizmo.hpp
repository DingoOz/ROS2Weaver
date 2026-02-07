#ifndef ROS_WEAVER_WIDGETS_URDF_GIZMO_HPP
#define ROS_WEAVER_WIDGETS_URDF_GIZMO_HPP

#include <QObject>
#include <QVector3D>
#include <QQuaternion>
#include <QPoint>
#include <QColor>

// Forward declarations
namespace Qt3DCore {
class QEntity;
class QTransform;
}

namespace Qt3DRender {
class QCamera;
class QMaterial;
}

namespace ros_weaver {

/**
 * @brief Interactive gizmo for translation and rotation
 *
 * Blender-style controls:
 * - Drag to move freely
 * - Middle mouse during drag constrains to axis (X, Y, or Z)
 * - Ctrl for snapped movement (default 0.1m increments)
 * - Shift for precision (10% speed)
 * - X/Y/Z keys to constrain to specific axis
 */
class URDFGizmo : public QObject {
  Q_OBJECT

public:
  enum class Mode { Translate, Rotate };
  enum class Axis { None, X, Y, Z, Free };

  explicit URDFGizmo(QObject* parent = nullptr);
  ~URDFGizmo() override;

  // Create/destroy the gizmo entity
  void createEntity(Qt3DCore::QEntity* parent);
  void destroyEntity();

  // Gizmo state
  void setVisible(bool visible);
  bool isVisible() const { return visible_; }

  void setMode(Mode mode);
  Mode mode() const { return mode_; }

  void setPosition(const QVector3D& position);
  QVector3D position() const { return position_; }

  void setRotation(const QQuaternion& rotation);
  QQuaternion rotation() const { return rotation_; }

  void setCamera(Qt3DRender::QCamera* camera);

  // Interaction modifiers
  void setSnapEnabled(bool enabled) { snapEnabled_ = enabled; }
  bool isSnapEnabled() const { return snapEnabled_; }

  void setPrecisionEnabled(bool enabled) { precisionEnabled_ = enabled; }
  bool isPrecisionEnabled() const { return precisionEnabled_; }

  void setSnapIncrement(float increment) { snapIncrement_ = increment; }
  float snapIncrement() const { return snapIncrement_; }

  // Axis constraint
  void constrainToAxis(Axis axis);
  Axis constrainedAxis() const { return constrainedAxis_; }

  // Drag handling
  void startDrag(const QPoint& screenPos, Axis axis = Axis::Free);
  void updateDrag(const QPoint& screenPos);
  void endDrag();
  bool isDragging() const { return isDragging_; }

  // Hit testing - returns which axis was clicked, or None
  Axis hitTest(const QPoint& screenPos);

signals:
  void transformChanged(const QVector3D& position, const QQuaternion& rotation);
  void dragStarted();
  void dragEnded();

private:
  void updateEntityTransform();
  QVector3D projectToPlane(const QPoint& screenPos, const QVector3D& planeNormal);
  QVector3D screenToWorldRay(const QPoint& screenPos);
  float snapValue(float value, float increment);
  Qt3DCore::QEntity* createArrow(const QColor& color, const QVector3D& direction);

  // Gizmo entity and sub-entities
  Qt3DCore::QEntity* gizmoEntity_ = nullptr;
  Qt3DCore::QTransform* gizmoTransform_ = nullptr;

  // Arrow entities (X=red, Y=green, Z=blue)
  Qt3DCore::QEntity* xArrow_ = nullptr;
  Qt3DCore::QEntity* yArrow_ = nullptr;
  Qt3DCore::QEntity* zArrow_ = nullptr;

  // Rotation rings (for rotate mode)
  Qt3DCore::QEntity* xRing_ = nullptr;
  Qt3DCore::QEntity* yRing_ = nullptr;
  Qt3DCore::QEntity* zRing_ = nullptr;

  Qt3DRender::QCamera* camera_ = nullptr;

  // State
  Mode mode_ = Mode::Translate;
  bool visible_ = false;
  QVector3D position_;
  QQuaternion rotation_;

  // Drag state
  bool isDragging_ = false;
  QPoint dragStartPos_;
  QVector3D dragStartPosition_;
  QQuaternion dragStartRotation_;
  Axis constrainedAxis_ = Axis::Free;

  // Modifiers
  bool snapEnabled_ = false;
  bool precisionEnabled_ = false;
  float snapIncrement_ = 0.1f;  // 10cm default
  float precisionFactor_ = 0.1f;  // 10% speed
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_WIDGETS_URDF_GIZMO_HPP
