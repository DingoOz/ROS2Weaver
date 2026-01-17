#ifndef ROS_WEAVER_URDF_PARSER_HPP
#define ROS_WEAVER_URDF_PARSER_HPP

#include <QObject>
#include <QString>
#include <QList>
#include <QMap>
#include <QVector3D>
#include <QQuaternion>
#include <QMatrix4x4>
#include <QColor>
#include <QFile>
#include <QXmlStreamReader>
#include <QXmlStreamWriter>

namespace ros_weaver {

// URDF Joint data structure
struct URDFJoint {
  QString name;
  QString parentLink;
  QString childLink;
  QString type;  // revolute, continuous, prismatic, fixed, floating, planar
  QVector3D origin;
  QQuaternion orientation;
  QVector3D axis = QVector3D(1, 0, 0);  // Default axis is X
  double lowerLimit = 0.0;
  double upperLimit = 0.0;
  double currentValue = 0.0;
  double effort = 0.0;
  double velocity = 0.0;
};

// URDF Link data structure
struct URDFLink {
  QString name;
  QString meshPath;
  QVector3D visualOrigin;
  QQuaternion visualOrientation;
  QVector3D scale = QVector3D(1, 1, 1);
  QColor color = QColor(200, 200, 200);

  // Collision geometry (simplified)
  QString collisionMeshPath;
  QVector3D collisionOrigin;
  QQuaternion collisionOrientation;

  // Inertial properties (for display purposes)
  double mass = 0.0;
  QVector3D inertialOrigin;
};

// Complete URDF Model
struct URDFModel {
  QString name;
  QList<URDFLink> links;
  QList<URDFJoint> joints;
  QString rootLink;
  QString filePath;
  bool isModified = false;
};

class URDFParser : public QObject {
  Q_OBJECT

public:
  explicit URDFParser(QObject* parent = nullptr);
  ~URDFParser() override = default;

  // Load URDF from file or string
  bool loadFromFile(const QString& filePath);
  bool loadFromString(const QString& urdfContent);

  // Export modified URDF
  QString exportToString() const;
  bool exportToFile(const QString& filePath) const;

  // Access model data
  const URDFModel& getModel() const { return model_; }
  URDFJoint* getJoint(const QString& jointName);
  const URDFJoint* getJoint(const QString& jointName) const;
  URDFLink* getLink(const QString& linkName);
  const URDFLink* getLink(const QString& linkName) const;

  // Get all joint/link names
  QStringList getJointNames() const;
  QStringList getLinkNames() const;

  // Get children of a link (joints that have this link as parent)
  QStringList getChildJoints(const QString& linkName) const;

  // Get parent chain to root
  QStringList getParentChain(const QString& linkName) const;

  // Modify joint orientation
  void setJointOrientation(const QString& jointName, const QQuaternion& orientation);
  void rotateJointBy(const QString& jointName, const QVector3D& axis, double angleDegrees);

  // Modify joint value (for revolute/prismatic joints)
  void setJointValue(const QString& jointName, double value);

  // Compute transforms
  QMatrix4x4 getGlobalTransform(const QString& linkName) const;
  QMatrix4x4 getJointTransform(const QString& jointName) const;

  // Check if model is loaded
  bool isLoaded() const { return !model_.name.isEmpty(); }
  bool isModified() const { return model_.isModified; }

  // Clear the model
  void clear();

signals:
  void modelLoaded(const URDFModel& model);
  void jointModified(const QString& jointName);
  void parseError(const QString& error);

private:
  URDFModel model_;
  QMap<QString, int> jointIndexMap_;  // joint name -> index in model_.joints
  QMap<QString, int> linkIndexMap_;   // link name -> index in model_.links

  // Parsing helpers
  void buildModelFromXml(QXmlStreamReader& reader);
  void parseRobot(QXmlStreamReader& reader);
  void parseLink(QXmlStreamReader& reader);
  void parseJoint(QXmlStreamReader& reader);
  void parseVisual(QXmlStreamReader& reader, URDFLink& link);
  void parseCollision(QXmlStreamReader& reader, URDFLink& link);
  void parseInertial(QXmlStreamReader& reader, URDFLink& link);
  void parseGeometry(QXmlStreamReader& reader, QString& meshPath, QVector3D& scale);
  void parseOrigin(QXmlStreamReader& reader, QVector3D& xyz, QQuaternion& rpy);
  void parseAxis(QXmlStreamReader& reader, QVector3D& axis);
  void parseLimit(QXmlStreamReader& reader, URDFJoint& joint);
  void parseMaterial(QXmlStreamReader& reader, QColor& color);

  // Export helpers
  void writeLink(QXmlStreamWriter& writer, const URDFLink& link) const;
  void writeJoint(QXmlStreamWriter& writer, const URDFJoint& joint) const;
  void writeOrigin(QXmlStreamWriter& writer, const QVector3D& xyz, const QQuaternion& orientation) const;

  // Build indexes after parsing
  void buildIndexes();
  void findRootLink();

  // Math helpers
  QQuaternion rpyToQuaternion(double roll, double pitch, double yaw) const;
  void quaternionToRpy(const QQuaternion& q, double& roll, double& pitch, double& yaw) const;

  // Transform computation helpers
  QMatrix4x4 computeLinkTransform(const QString& linkName, QSet<QString>& visited) const;
};

}  // namespace ros_weaver

Q_DECLARE_METATYPE(ros_weaver::URDFJoint)
Q_DECLARE_METATYPE(ros_weaver::URDFLink)
Q_DECLARE_METATYPE(ros_weaver::URDFModel)

#endif  // ROS_WEAVER_URDF_PARSER_HPP
