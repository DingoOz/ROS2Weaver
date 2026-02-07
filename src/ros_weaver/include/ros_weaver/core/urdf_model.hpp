#ifndef ROS_WEAVER_CORE_URDF_MODEL_HPP
#define ROS_WEAVER_CORE_URDF_MODEL_HPP

#include <QString>
#include <QVector3D>
#include <QQuaternion>
#include <QColor>
#include <QList>
#include <QMap>
#include <memory>

namespace ros_weaver {

/**
 * @brief 3D pose with position and orientation (RPY)
 */
struct URDFPose {
  QVector3D position{0.0f, 0.0f, 0.0f};
  QVector3D rpy{0.0f, 0.0f, 0.0f};  // Roll, Pitch, Yaw in radians

  QQuaternion toQuaternion() const;
  static URDFPose fromQuaternion(const QVector3D& pos, const QQuaternion& quat);

  bool operator==(const URDFPose& other) const;
  bool operator!=(const URDFPose& other) const { return !(*this == other); }
};

/**
 * @brief Inertial properties of a link
 */
struct URDFInertial {
  URDFPose origin;
  double mass = 0.0;
  // Inertia tensor (ixx, ixy, ixz, iyy, iyz, izz)
  double ixx = 0.0, ixy = 0.0, ixz = 0.0;
  double iyy = 0.0, iyz = 0.0, izz = 0.0;

  bool isValid() const { return mass > 0.0; }
};

/**
 * @brief Geometry type for visual/collision elements
 */
enum class URDFGeometryType {
  Box,
  Cylinder,
  Sphere,
  Mesh
};

/**
 * @brief Geometry specification for visual/collision
 */
struct URDFGeometry {
  URDFGeometryType type = URDFGeometryType::Box;

  // Box dimensions (x, y, z)
  QVector3D boxSize{1.0f, 1.0f, 1.0f};

  // Cylinder dimensions
  double cylinderRadius = 0.5;
  double cylinderLength = 1.0;

  // Sphere radius
  double sphereRadius = 0.5;

  // Mesh file path (can be package:// or file://)
  QString meshFilename;
  QVector3D meshScale{1.0f, 1.0f, 1.0f};
};

/**
 * @brief Material for visual appearance
 */
struct URDFMaterial {
  QString name;
  QColor color{128, 128, 128, 255};  // RGBA
  bool colorExplicitlySet = false;
  QString textureFilename;

  bool hasColor() const { return colorExplicitlySet; }
  bool hasTexture() const { return !textureFilename.isEmpty(); }
};

/**
 * @brief Visual element of a link
 */
struct URDFVisual {
  QString name;
  URDFPose origin;
  URDFGeometry geometry;
  URDFMaterial material;
};

/**
 * @brief Collision element of a link
 */
struct URDFCollision {
  QString name;
  URDFPose origin;
  URDFGeometry geometry;
};

/**
 * @brief Joint type enumeration
 */
enum class URDFJointType {
  Revolute,
  Continuous,
  Prismatic,
  Fixed,
  Floating,
  Planar
};

/**
 * @brief Joint limits for revolute and prismatic joints
 */
struct URDFJointLimits {
  double lower = 0.0;
  double upper = 0.0;
  double effort = 0.0;
  double velocity = 0.0;

  bool hasLimits() const { return lower != upper || effort > 0 || velocity > 0; }
};

/**
 * @brief Joint dynamics (damping and friction)
 */
struct URDFJointDynamics {
  double damping = 0.0;
  double friction = 0.0;
};

/**
 * @brief Safety controller for joint limits
 */
struct URDFJointSafety {
  double softLowerLimit = 0.0;
  double softUpperLimit = 0.0;
  double kPosition = 0.0;
  double kVelocity = 0.0;
};

/**
 * @brief Joint calibration
 */
struct URDFJointCalibration {
  double rising = 0.0;
  double falling = 0.0;
};

/**
 * @brief Mimic joint specification
 */
struct URDFJointMimic {
  QString joint;
  double multiplier = 1.0;
  double offset = 0.0;
};

/**
 * @brief URDF Link representation
 */
struct URDFLink {
  QString name;
  URDFInertial inertial;
  QList<URDFVisual> visuals;
  QList<URDFCollision> collisions;

  // Parent joint (empty for root link)
  QString parentJoint;
  // Child joint names
  QStringList childJoints;

  bool hasInertial() const { return inertial.isValid(); }
  bool hasVisual() const { return !visuals.isEmpty(); }
  bool hasCollision() const { return !collisions.isEmpty(); }
};

/**
 * @brief URDF Joint representation
 */
struct URDFJoint {
  QString name;
  URDFJointType type = URDFJointType::Fixed;

  URDFPose origin;
  QVector3D axis{1.0f, 0.0f, 0.0f};

  QString parentLink;
  QString childLink;

  URDFJointLimits limits;
  URDFJointDynamics dynamics;
  URDFJointSafety safety;
  URDFJointCalibration calibration;
  URDFJointMimic mimic;

  bool hasLimits() const { return limits.hasLimits(); }
  bool hasDynamics() const { return dynamics.damping != 0.0 || dynamics.friction != 0.0; }
  bool hasSafety() const { return safety.kPosition != 0.0 || safety.kVelocity != 0.0; }
  bool hasMimic() const { return !mimic.joint.isEmpty(); }

  static QString typeToString(URDFJointType type);
  static URDFJointType typeFromString(const QString& str);
};

/**
 * @brief Complete URDF model
 */
class URDFModel {
public:
  URDFModel() = default;
  ~URDFModel() = default;

  // Model metadata
  QString name() const { return name_; }
  void setName(const QString& name) { name_ = name; }

  QString filePath() const { return filePath_; }
  void setFilePath(const QString& path) { filePath_ = path; }

  // Links
  const QMap<QString, URDFLink>& links() const { return links_; }
  URDFLink* link(const QString& name);
  const URDFLink* link(const QString& name) const;
  bool hasLink(const QString& name) const { return links_.contains(name); }
  void addLink(const URDFLink& link);
  void removeLink(const QString& name);
  QStringList linkNames() const { return links_.keys(); }
  int linkCount() const { return links_.size(); }

  // Joints
  const QMap<QString, URDFJoint>& joints() const { return joints_; }
  URDFJoint* joint(const QString& name);
  const URDFJoint* joint(const QString& name) const;
  bool hasJoint(const QString& name) const { return joints_.contains(name); }
  void addJoint(const URDFJoint& joint);
  void removeJoint(const QString& name);
  QStringList jointNames() const { return joints_.keys(); }
  int jointCount() const { return joints_.size(); }

  // Materials (global materials defined at model level)
  const QMap<QString, URDFMaterial>& materials() const { return materials_; }
  URDFMaterial* material(const QString& name);
  const URDFMaterial* material(const QString& name) const;
  void addMaterial(const URDFMaterial& material);
  void removeMaterial(const QString& name);

  // Tree structure
  QString rootLinkName() const;
  URDFLink* rootLink();
  const URDFLink* rootLink() const;

  // Get children of a link
  QStringList childLinkNames(const QString& linkName) const;

  // Compute global transform for a link (from root)
  URDFPose computeGlobalPose(const QString& linkName) const;

  // Validation
  bool isValid() const;
  QStringList validationErrors() const;

  // Clear model
  void clear();

  // Check if model has been modified
  bool isModified() const { return modified_; }
  void setModified(bool modified) { modified_ = modified; }

private:
  QString name_;
  QString filePath_;

  QMap<QString, URDFLink> links_;
  QMap<QString, URDFJoint> joints_;
  QMap<QString, URDFMaterial> materials_;

  bool modified_ = false;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_URDF_MODEL_HPP
