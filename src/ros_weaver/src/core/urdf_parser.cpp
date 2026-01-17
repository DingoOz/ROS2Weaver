#include "ros_weaver/core/urdf_parser.hpp"

#include <QDebug>
#include <QUrlQuery>
#include <QFileInfo>
#include <QDir>
#include <QtMath>
#include <cmath>

namespace ros_weaver {

URDFParser::URDFParser(QObject* parent)
  : QObject(parent) {
}

bool URDFParser::loadFromFile(const QString& filePath) {
  QFile file(filePath);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    emit parseError(tr("Failed to open file: %1").arg(filePath));
    return false;
  }

  QString content = QString::fromUtf8(file.readAll());
  file.close();

  bool success = loadFromString(content);
  if (success) {
    model_.filePath = filePath;
  }
  return success;
}

bool URDFParser::loadFromString(const QString& urdfContent) {
  clear();

  QXmlStreamReader reader(urdfContent);

  try {
    buildModelFromXml(reader);
  } catch (const std::exception& e) {
    emit parseError(tr("Parse error: %1").arg(e.what()));
    return false;
  }

  if (reader.hasError()) {
    emit parseError(tr("XML parse error: %1 at line %2")
                        .arg(reader.errorString())
                        .arg(reader.lineNumber()));
    return false;
  }

  buildIndexes();
  findRootLink();

  emit modelLoaded(model_);
  return true;
}

void URDFParser::buildModelFromXml(QXmlStreamReader& reader) {
  while (!reader.atEnd()) {
    reader.readNext();
    if (reader.isStartElement()) {
      if (reader.name() == QStringLiteral("robot")) {
        model_.name = reader.attributes().value("name").toString();
        parseRobot(reader);
      }
    }
  }
}

void URDFParser::parseRobot(QXmlStreamReader& reader) {
  while (!reader.atEnd()) {
    reader.readNext();

    if (reader.isEndElement() && reader.name() == QStringLiteral("robot")) {
      break;
    }

    if (reader.isStartElement()) {
      if (reader.name() == QStringLiteral("link")) {
        parseLink(reader);
      } else if (reader.name() == QStringLiteral("joint")) {
        parseJoint(reader);
      }
      // Skip material, gazebo, transmission tags at robot level
    }
  }
}

void URDFParser::parseLink(QXmlStreamReader& reader) {
  URDFLink link;
  link.name = reader.attributes().value("name").toString();

  while (!reader.atEnd()) {
    reader.readNext();

    if (reader.isEndElement() && reader.name() == QStringLiteral("link")) {
      break;
    }

    if (reader.isStartElement()) {
      if (reader.name() == QStringLiteral("visual")) {
        parseVisual(reader, link);
      } else if (reader.name() == QStringLiteral("collision")) {
        parseCollision(reader, link);
      } else if (reader.name() == QStringLiteral("inertial")) {
        parseInertial(reader, link);
      }
    }
  }

  model_.links.append(link);
}

void URDFParser::parseJoint(QXmlStreamReader& reader) {
  URDFJoint joint;
  joint.name = reader.attributes().value("name").toString();
  joint.type = reader.attributes().value("type").toString();

  while (!reader.atEnd()) {
    reader.readNext();

    if (reader.isEndElement() && reader.name() == QStringLiteral("joint")) {
      break;
    }

    if (reader.isStartElement()) {
      if (reader.name() == QStringLiteral("parent")) {
        joint.parentLink = reader.attributes().value("link").toString();
      } else if (reader.name() == QStringLiteral("child")) {
        joint.childLink = reader.attributes().value("link").toString();
      } else if (reader.name() == QStringLiteral("origin")) {
        parseOrigin(reader, joint.origin, joint.orientation);
      } else if (reader.name() == QStringLiteral("axis")) {
        parseAxis(reader, joint.axis);
      } else if (reader.name() == QStringLiteral("limit")) {
        parseLimit(reader, joint);
      }
    }
  }

  model_.joints.append(joint);
}

void URDFParser::parseVisual(QXmlStreamReader& reader, URDFLink& link) {
  while (!reader.atEnd()) {
    reader.readNext();

    if (reader.isEndElement() && reader.name() == QStringLiteral("visual")) {
      break;
    }

    if (reader.isStartElement()) {
      if (reader.name() == QStringLiteral("origin")) {
        parseOrigin(reader, link.visualOrigin, link.visualOrientation);
      } else if (reader.name() == QStringLiteral("geometry")) {
        parseGeometry(reader, link.meshPath, link.scale);
      } else if (reader.name() == QStringLiteral("material")) {
        parseMaterial(reader, link.color);
      }
    }
  }
}

void URDFParser::parseCollision(QXmlStreamReader& reader, URDFLink& link) {
  while (!reader.atEnd()) {
    reader.readNext();

    if (reader.isEndElement() && reader.name() == QStringLiteral("collision")) {
      break;
    }

    if (reader.isStartElement()) {
      if (reader.name() == QStringLiteral("origin")) {
        parseOrigin(reader, link.collisionOrigin, link.collisionOrientation);
      } else if (reader.name() == QStringLiteral("geometry")) {
        QVector3D unusedScale;
        parseGeometry(reader, link.collisionMeshPath, unusedScale);
      }
    }
  }
}

void URDFParser::parseInertial(QXmlStreamReader& reader, URDFLink& link) {
  while (!reader.atEnd()) {
    reader.readNext();

    if (reader.isEndElement() && reader.name() == QStringLiteral("inertial")) {
      break;
    }

    if (reader.isStartElement()) {
      if (reader.name() == QStringLiteral("origin")) {
        QQuaternion unusedOrientation;
        parseOrigin(reader, link.inertialOrigin, unusedOrientation);
      } else if (reader.name() == QStringLiteral("mass")) {
        link.mass = reader.attributes().value("value").toDouble();
      }
    }
  }
}

void URDFParser::parseGeometry(QXmlStreamReader& reader, QString& meshPath, QVector3D& scale) {
  while (!reader.atEnd()) {
    reader.readNext();

    if (reader.isEndElement() && reader.name() == QStringLiteral("geometry")) {
      break;
    }

    if (reader.isStartElement()) {
      if (reader.name() == QStringLiteral("mesh")) {
        meshPath = reader.attributes().value("filename").toString();
        QString scaleStr = reader.attributes().value("scale").toString();
        if (!scaleStr.isEmpty()) {
          QStringList parts = scaleStr.split(' ', Qt::SkipEmptyParts);
          if (parts.size() == 3) {
            scale = QVector3D(parts[0].toFloat(), parts[1].toFloat(), parts[2].toFloat());
          }
        }
      } else if (reader.name() == QStringLiteral("box")) {
        // Store box dimensions in meshPath for primitive rendering
        QString size = reader.attributes().value("size").toString();
        meshPath = QString("primitive://box?size=%1").arg(size);
      } else if (reader.name() == QStringLiteral("cylinder")) {
        double radius = reader.attributes().value("radius").toDouble();
        double length = reader.attributes().value("length").toDouble();
        meshPath = QString("primitive://cylinder?radius=%1&length=%2").arg(radius).arg(length);
      } else if (reader.name() == QStringLiteral("sphere")) {
        double radius = reader.attributes().value("radius").toDouble();
        meshPath = QString("primitive://sphere?radius=%1").arg(radius);
      }
    }
  }
}

void URDFParser::parseOrigin(QXmlStreamReader& reader, QVector3D& xyz, QQuaternion& orientation) {
  QString xyzStr = reader.attributes().value("xyz").toString();
  QString rpyStr = reader.attributes().value("rpy").toString();

  if (!xyzStr.isEmpty()) {
    QStringList parts = xyzStr.split(' ', Qt::SkipEmptyParts);
    if (parts.size() == 3) {
      xyz = QVector3D(parts[0].toFloat(), parts[1].toFloat(), parts[2].toFloat());
    }
  }

  if (!rpyStr.isEmpty()) {
    QStringList parts = rpyStr.split(' ', Qt::SkipEmptyParts);
    if (parts.size() == 3) {
      double roll = parts[0].toDouble();
      double pitch = parts[1].toDouble();
      double yaw = parts[2].toDouble();
      orientation = rpyToQuaternion(roll, pitch, yaw);
    }
  }
}

void URDFParser::parseAxis(QXmlStreamReader& reader, QVector3D& axis) {
  QString xyzStr = reader.attributes().value("xyz").toString();
  if (!xyzStr.isEmpty()) {
    QStringList parts = xyzStr.split(' ', Qt::SkipEmptyParts);
    if (parts.size() == 3) {
      axis = QVector3D(parts[0].toFloat(), parts[1].toFloat(), parts[2].toFloat());
    }
  }
}

void URDFParser::parseLimit(QXmlStreamReader& reader, URDFJoint& joint) {
  joint.lowerLimit = reader.attributes().value("lower").toDouble();
  joint.upperLimit = reader.attributes().value("upper").toDouble();
  joint.effort = reader.attributes().value("effort").toDouble();
  joint.velocity = reader.attributes().value("velocity").toDouble();
}

void URDFParser::parseMaterial(QXmlStreamReader& reader, QColor& color) {
  while (!reader.atEnd()) {
    reader.readNext();

    if (reader.isEndElement() && reader.name() == QStringLiteral("material")) {
      break;
    }

    if (reader.isStartElement()) {
      if (reader.name() == QStringLiteral("color")) {
        QString rgba = reader.attributes().value("rgba").toString();
        if (!rgba.isEmpty()) {
          QStringList parts = rgba.split(' ', Qt::SkipEmptyParts);
          if (parts.size() >= 3) {
            int r = static_cast<int>(parts[0].toFloat() * 255);
            int g = static_cast<int>(parts[1].toFloat() * 255);
            int b = static_cast<int>(parts[2].toFloat() * 255);
            int a = parts.size() > 3 ? static_cast<int>(parts[3].toFloat() * 255) : 255;
            color = QColor(r, g, b, a);
          }
        }
      }
    }
  }
}

void URDFParser::buildIndexes() {
  jointIndexMap_.clear();
  linkIndexMap_.clear();

  for (int i = 0; i < model_.joints.size(); ++i) {
    jointIndexMap_[model_.joints[i].name] = i;
  }

  for (int i = 0; i < model_.links.size(); ++i) {
    linkIndexMap_[model_.links[i].name] = i;
  }
}

void URDFParser::findRootLink() {
  // Root link is a link that is not the child of any joint
  QSet<QString> childLinks;
  for (const URDFJoint& joint : model_.joints) {
    childLinks.insert(joint.childLink);
  }

  for (const URDFLink& link : model_.links) {
    if (!childLinks.contains(link.name)) {
      model_.rootLink = link.name;
      return;
    }
  }

  // If no root found, use first link
  if (!model_.links.isEmpty()) {
    model_.rootLink = model_.links.first().name;
  }
}

QString URDFParser::exportToString() const {
  QString output;
  QXmlStreamWriter writer(&output);
  writer.setAutoFormatting(true);
  writer.setAutoFormattingIndent(2);

  writer.writeStartDocument();
  writer.writeStartElement("robot");
  writer.writeAttribute("name", model_.name);

  // Write links
  for (const URDFLink& link : model_.links) {
    writeLink(writer, link);
  }

  // Write joints
  for (const URDFJoint& joint : model_.joints) {
    writeJoint(writer, joint);
  }

  writer.writeEndElement();  // robot
  writer.writeEndDocument();

  return output;
}

bool URDFParser::exportToFile(const QString& filePath) const {
  QFile file(filePath);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    return false;
  }

  QString content = exportToString();
  file.write(content.toUtf8());
  file.close();
  return true;
}

void URDFParser::writeLink(QXmlStreamWriter& writer, const URDFLink& link) const {
  writer.writeStartElement("link");
  writer.writeAttribute("name", link.name);

  // Visual
  if (!link.meshPath.isEmpty()) {
    writer.writeStartElement("visual");
    writeOrigin(writer, link.visualOrigin, link.visualOrientation);

    writer.writeStartElement("geometry");
    if (link.meshPath.startsWith("primitive://box")) {
      writer.writeStartElement("box");
      QUrl url(link.meshPath);
      writer.writeAttribute("size", QUrlQuery(url).queryItemValue("size"));
      writer.writeEndElement();
    } else if (link.meshPath.startsWith("primitive://cylinder")) {
      writer.writeStartElement("cylinder");
      QUrl url(link.meshPath);
      QUrlQuery query(url);
      writer.writeAttribute("radius", query.queryItemValue("radius"));
      writer.writeAttribute("length", query.queryItemValue("length"));
      writer.writeEndElement();
    } else if (link.meshPath.startsWith("primitive://sphere")) {
      writer.writeStartElement("sphere");
      QUrl url(link.meshPath);
      writer.writeAttribute("radius", QUrlQuery(url).queryItemValue("radius"));
      writer.writeEndElement();
    } else {
      writer.writeStartElement("mesh");
      writer.writeAttribute("filename", link.meshPath);
      if (link.scale != QVector3D(1, 1, 1)) {
        writer.writeAttribute("scale", QString("%1 %2 %3")
                                            .arg(link.scale.x())
                                            .arg(link.scale.y())
                                            .arg(link.scale.z()));
      }
      writer.writeEndElement();
    }
    writer.writeEndElement();  // geometry

    // Material
    writer.writeStartElement("material");
    writer.writeAttribute("name", QString("material_%1").arg(link.name));
    writer.writeStartElement("color");
    writer.writeAttribute("rgba", QString("%1 %2 %3 %4")
                                      .arg(link.color.redF())
                                      .arg(link.color.greenF())
                                      .arg(link.color.blueF())
                                      .arg(link.color.alphaF()));
    writer.writeEndElement();  // color
    writer.writeEndElement();  // material

    writer.writeEndElement();  // visual
  }

  // Inertial (if mass is set)
  if (link.mass > 0) {
    writer.writeStartElement("inertial");
    QQuaternion identity;
    writeOrigin(writer, link.inertialOrigin, identity);
    writer.writeStartElement("mass");
    writer.writeAttribute("value", QString::number(link.mass));
    writer.writeEndElement();
    writer.writeEndElement();  // inertial
  }

  writer.writeEndElement();  // link
}

void URDFParser::writeJoint(QXmlStreamWriter& writer, const URDFJoint& joint) const {
  writer.writeStartElement("joint");
  writer.writeAttribute("name", joint.name);
  writer.writeAttribute("type", joint.type);

  writer.writeStartElement("parent");
  writer.writeAttribute("link", joint.parentLink);
  writer.writeEndElement();

  writer.writeStartElement("child");
  writer.writeAttribute("link", joint.childLink);
  writer.writeEndElement();

  writeOrigin(writer, joint.origin, joint.orientation);

  // Axis (only for non-fixed joints)
  if (joint.type != "fixed") {
    writer.writeStartElement("axis");
    writer.writeAttribute("xyz", QString("%1 %2 %3")
                                     .arg(joint.axis.x())
                                     .arg(joint.axis.y())
                                     .arg(joint.axis.z()));
    writer.writeEndElement();
  }

  // Limits (for revolute/prismatic joints)
  if (joint.type == "revolute" || joint.type == "prismatic") {
    writer.writeStartElement("limit");
    writer.writeAttribute("lower", QString::number(joint.lowerLimit));
    writer.writeAttribute("upper", QString::number(joint.upperLimit));
    writer.writeAttribute("effort", QString::number(joint.effort));
    writer.writeAttribute("velocity", QString::number(joint.velocity));
    writer.writeEndElement();
  }

  writer.writeEndElement();  // joint
}

void URDFParser::writeOrigin(QXmlStreamWriter& writer, const QVector3D& xyz,
                              const QQuaternion& orientation) const {
  bool hasXyz = xyz != QVector3D(0, 0, 0);
  bool hasRpy = orientation != QQuaternion();

  if (hasXyz || hasRpy) {
    writer.writeStartElement("origin");
    if (hasXyz) {
      writer.writeAttribute("xyz", QString("%1 %2 %3").arg(xyz.x()).arg(xyz.y()).arg(xyz.z()));
    }
    if (hasRpy) {
      double roll, pitch, yaw;
      quaternionToRpy(orientation, roll, pitch, yaw);
      writer.writeAttribute("rpy", QString("%1 %2 %3").arg(roll).arg(pitch).arg(yaw));
    }
    writer.writeEndElement();
  }
}

URDFJoint* URDFParser::getJoint(const QString& jointName) {
  auto it = jointIndexMap_.find(jointName);
  if (it != jointIndexMap_.end()) {
    return &model_.joints[it.value()];
  }
  return nullptr;
}

const URDFJoint* URDFParser::getJoint(const QString& jointName) const {
  auto it = jointIndexMap_.find(jointName);
  if (it != jointIndexMap_.end()) {
    return &model_.joints[it.value()];
  }
  return nullptr;
}

URDFLink* URDFParser::getLink(const QString& linkName) {
  auto it = linkIndexMap_.find(linkName);
  if (it != linkIndexMap_.end()) {
    return &model_.links[it.value()];
  }
  return nullptr;
}

const URDFLink* URDFParser::getLink(const QString& linkName) const {
  auto it = linkIndexMap_.find(linkName);
  if (it != linkIndexMap_.end()) {
    return &model_.links[it.value()];
  }
  return nullptr;
}

QStringList URDFParser::getJointNames() const {
  QStringList names;
  for (const URDFJoint& joint : model_.joints) {
    names.append(joint.name);
  }
  return names;
}

QStringList URDFParser::getLinkNames() const {
  QStringList names;
  for (const URDFLink& link : model_.links) {
    names.append(link.name);
  }
  return names;
}

QStringList URDFParser::getChildJoints(const QString& linkName) const {
  QStringList children;
  for (const URDFJoint& joint : model_.joints) {
    if (joint.parentLink == linkName) {
      children.append(joint.name);
    }
  }
  return children;
}

QStringList URDFParser::getParentChain(const QString& linkName) const {
  QStringList chain;
  QString currentLink = linkName;

  while (currentLink != model_.rootLink && !currentLink.isEmpty()) {
    // Find joint where this link is the child
    for (const URDFJoint& joint : model_.joints) {
      if (joint.childLink == currentLink) {
        chain.append(joint.name);
        currentLink = joint.parentLink;
        break;
      }
    }
  }

  return chain;
}

void URDFParser::setJointOrientation(const QString& jointName, const QQuaternion& orientation) {
  URDFJoint* joint = getJoint(jointName);
  if (joint) {
    joint->orientation = orientation;
    model_.isModified = true;
    emit jointModified(jointName);
  }
}

void URDFParser::rotateJointBy(const QString& jointName, const QVector3D& axis, double angleDegrees) {
  URDFJoint* joint = getJoint(jointName);
  if (joint) {
    QQuaternion rotation = QQuaternion::fromAxisAndAngle(axis, static_cast<float>(angleDegrees));
    joint->orientation = rotation * joint->orientation;
    model_.isModified = true;
    emit jointModified(jointName);
  }
}

void URDFParser::setJointValue(const QString& jointName, double value) {
  URDFJoint* joint = getJoint(jointName);
  if (joint) {
    // Clamp to limits for revolute/prismatic
    if (joint->type == "revolute" || joint->type == "prismatic") {
      value = qBound(joint->lowerLimit, value, joint->upperLimit);
    }
    joint->currentValue = value;
    model_.isModified = true;
    emit jointModified(jointName);
  }
}

QMatrix4x4 URDFParser::getGlobalTransform(const QString& linkName) const {
  QSet<QString> visited;
  return computeLinkTransform(linkName, visited);
}

QMatrix4x4 URDFParser::computeLinkTransform(const QString& linkName, QSet<QString>& visited) const {
  if (visited.contains(linkName)) {
    return QMatrix4x4();  // Prevent infinite recursion
  }
  visited.insert(linkName);

  if (linkName == model_.rootLink) {
    return QMatrix4x4();  // Root has identity transform
  }

  // Find joint where this link is the child
  for (const URDFJoint& joint : model_.joints) {
    if (joint.childLink == linkName) {
      // Get parent transform
      QMatrix4x4 parentTransform = computeLinkTransform(joint.parentLink, visited);

      // Build joint transform
      QMatrix4x4 jointTransform;
      jointTransform.translate(joint.origin);
      jointTransform.rotate(joint.orientation);

      // Add joint value rotation (for revolute/continuous)
      if (joint.type == "revolute" || joint.type == "continuous") {
        QQuaternion valueRotation =
            QQuaternion::fromAxisAndAngle(joint.axis, static_cast<float>(qRadiansToDegrees(joint.currentValue)));
        jointTransform.rotate(valueRotation);
      } else if (joint.type == "prismatic") {
        jointTransform.translate(joint.axis * static_cast<float>(joint.currentValue));
      }

      return parentTransform * jointTransform;
    }
  }

  return QMatrix4x4();
}

QMatrix4x4 URDFParser::getJointTransform(const QString& jointName) const {
  const URDFJoint* joint = getJoint(jointName);
  if (!joint) {
    return QMatrix4x4();
  }

  // Get parent link's global transform
  QSet<QString> visited;
  QMatrix4x4 parentTransform = computeLinkTransform(joint->parentLink, visited);

  // Add joint's local transform
  QMatrix4x4 jointTransform;
  jointTransform.translate(joint->origin);
  jointTransform.rotate(joint->orientation);

  return parentTransform * jointTransform;
}

void URDFParser::clear() {
  model_ = URDFModel();
  jointIndexMap_.clear();
  linkIndexMap_.clear();
}

QQuaternion URDFParser::rpyToQuaternion(double roll, double pitch, double yaw) const {
  // Convert from roll-pitch-yaw (XYZ fixed axes) to quaternion
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  float w = static_cast<float>(cr * cp * cy + sr * sp * sy);
  float x = static_cast<float>(sr * cp * cy - cr * sp * sy);
  float y = static_cast<float>(cr * sp * cy + sr * cp * sy);
  float z = static_cast<float>(cr * cp * sy - sr * sp * cy);

  return QQuaternion(w, x, y, z);
}

void URDFParser::quaternionToRpy(const QQuaternion& q, double& roll, double& pitch, double& yaw) const {
  // Convert quaternion to roll-pitch-yaw (XYZ fixed axes)
  float w = q.scalar();
  float x = q.x();
  float y = q.y();
  float z = q.z();

  // Roll (x-axis rotation)
  double sinr_cosp = 2.0 * (w * x + y * z);
  double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  double sinp = 2.0 * (w * y - z * x);
  if (std::abs(sinp) >= 1.0) {
    pitch = std::copysign(M_PI / 2, sinp);  // Use 90 degrees if out of range
  } else {
    pitch = std::asin(sinp);
  }

  // Yaw (z-axis rotation)
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace ros_weaver
