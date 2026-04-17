#include "ros_weaver/core/urdf_model.hpp"
#include <cmath>

namespace ros_weaver {

// ============================================================================
// URDFPose
// ============================================================================

QQuaternion URDFPose::toQuaternion() const {
  // Convert RPY (roll, pitch, yaw) to quaternion
  float roll = rpy.x();
  float pitch = rpy.y();
  float yaw = rpy.z();

  float cy = std::cos(yaw * 0.5f);
  float sy = std::sin(yaw * 0.5f);
  float cp = std::cos(pitch * 0.5f);
  float sp = std::sin(pitch * 0.5f);
  float cr = std::cos(roll * 0.5f);
  float sr = std::sin(roll * 0.5f);

  QQuaternion q;
  q.setScalar(cr * cp * cy + sr * sp * sy);
  q.setX(sr * cp * cy - cr * sp * sy);
  q.setY(cr * sp * cy + sr * cp * sy);
  q.setZ(cr * cp * sy - sr * sp * cy);

  return q;
}

URDFPose URDFPose::fromQuaternion(const QVector3D& pos, const QQuaternion& quat) {
  URDFPose pose;
  pose.position = pos;

  // Convert quaternion to RPY
  float w = quat.scalar();
  float x = quat.x();
  float y = quat.y();
  float z = quat.z();

  // Roll (x-axis rotation)
  float sinr_cosp = 2.0f * (w * x + y * z);
  float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
  pose.rpy.setX(std::atan2(sinr_cosp, cosr_cosp));

  // Pitch (y-axis rotation)
  float sinp = 2.0f * (w * y - z * x);
  if (std::abs(sinp) >= 1.0f) {
    pose.rpy.setY(std::copysign(M_PI / 2.0f, sinp));
  } else {
    pose.rpy.setY(std::asin(sinp));
  }

  // Yaw (z-axis rotation)
  float siny_cosp = 2.0f * (w * z + x * y);
  float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  pose.rpy.setZ(std::atan2(siny_cosp, cosy_cosp));

  return pose;
}

bool URDFPose::operator==(const URDFPose& other) const {
  const float epsilon = 1e-6f;
  return (position - other.position).length() < epsilon &&
         (rpy - other.rpy).length() < epsilon;
}

// ============================================================================
// URDFJoint
// ============================================================================

QString URDFJoint::typeToString(URDFJointType type) {
  switch (type) {
    case URDFJointType::Revolute:   return "revolute";
    case URDFJointType::Continuous: return "continuous";
    case URDFJointType::Prismatic:  return "prismatic";
    case URDFJointType::Fixed:      return "fixed";
    case URDFJointType::Floating:   return "floating";
    case URDFJointType::Planar:     return "planar";
  }
  return "fixed";
}

URDFJointType URDFJoint::typeFromString(const QString& str) {
  QString lower = str.toLower();
  if (lower == "revolute")   return URDFJointType::Revolute;
  if (lower == "continuous") return URDFJointType::Continuous;
  if (lower == "prismatic")  return URDFJointType::Prismatic;
  if (lower == "fixed")      return URDFJointType::Fixed;
  if (lower == "floating")   return URDFJointType::Floating;
  if (lower == "planar")     return URDFJointType::Planar;
  return URDFJointType::Fixed;
}

// ============================================================================
// URDFModel
// ============================================================================

URDFLink* URDFModel::link(const QString& name) {
  auto it = links_.find(name);
  return it != links_.end() ? &it.value() : nullptr;
}

const URDFLink* URDFModel::link(const QString& name) const {
  auto it = links_.find(name);
  return it != links_.end() ? &it.value() : nullptr;
}

void URDFModel::addLink(const URDFLink& link) {
  links_[link.name] = link;
  modified_ = true;
}

void URDFModel::removeLink(const QString& name) {
  links_.remove(name);

  // Remove joints that reference this link to keep model consistent
  QStringList jointsToRemove;
  for (auto it = joints_.constBegin(); it != joints_.constEnd(); ++it) {
    if (it->parentLink == name || it->childLink == name) {
      jointsToRemove.append(it.key());
    }
  }
  for (const QString& jointName : jointsToRemove) {
    joints_.remove(jointName);
  }

  modified_ = true;
}

URDFJoint* URDFModel::joint(const QString& name) {
  auto it = joints_.find(name);
  return it != joints_.end() ? &it.value() : nullptr;
}

const URDFJoint* URDFModel::joint(const QString& name) const {
  auto it = joints_.find(name);
  return it != joints_.end() ? &it.value() : nullptr;
}

void URDFModel::addJoint(const URDFJoint& joint) {
  joints_[joint.name] = joint;
  modified_ = true;
}

void URDFModel::removeJoint(const QString& name) {
  joints_.remove(name);
  modified_ = true;
}

URDFMaterial* URDFModel::material(const QString& name) {
  auto it = materials_.find(name);
  return it != materials_.end() ? &it.value() : nullptr;
}

const URDFMaterial* URDFModel::material(const QString& name) const {
  auto it = materials_.find(name);
  return it != materials_.end() ? &it.value() : nullptr;
}

void URDFModel::addMaterial(const URDFMaterial& material) {
  materials_[material.name] = material;
  modified_ = true;
}

void URDFModel::removeMaterial(const QString& name) {
  materials_.remove(name);
  modified_ = true;
}

QString URDFModel::rootLinkName() const {
  // Find the link that is not a child of any joint
  QSet<QString> childLinks;
  for (const auto& joint : joints_) {
    childLinks.insert(joint.childLink);
  }

  for (const auto& link : links_) {
    if (!childLinks.contains(link.name)) {
      return link.name;
    }
  }

  // Fallback: return first link
  if (!links_.isEmpty()) {
    return links_.begin().key();
  }

  return QString();
}

URDFLink* URDFModel::rootLink() {
  QString rootName = rootLinkName();
  return rootName.isEmpty() ? nullptr : link(rootName);
}

const URDFLink* URDFModel::rootLink() const {
  QString rootName = rootLinkName();
  return rootName.isEmpty() ? nullptr : link(rootName);
}

QStringList URDFModel::childLinkNames(const QString& linkName) const {
  QStringList children;
  for (const auto& joint : joints_) {
    if (joint.parentLink == linkName) {
      children.append(joint.childLink);
    }
  }
  return children;
}

URDFPose URDFModel::computeGlobalPose(const QString& linkName) const {
  URDFPose globalPose;

  // Find the chain from root to this link
  QStringList chain;
  QString current = linkName;

  QSet<QString> visited;  // Guard against cycles
  while (!current.isEmpty()) {
    if (visited.contains(current)) {
      break;  // Cycle detected, prevent infinite loop
    }
    visited.insert(current);
    chain.prepend(current);

    // Find parent joint
    QString parentJoint;
    for (const auto& joint : joints_) {
      if (joint.childLink == current) {
        parentJoint = joint.name;
        current = joint.parentLink;
        break;
      }
    }

    if (parentJoint.isEmpty()) {
      break;  // Reached root
    }
  }

  // Compute cumulative transform
  QQuaternion cumulativeRot;
  QVector3D cumulativePos;

  for (int i = 0; i < chain.size() - 1; ++i) {
    QString fromLink = chain[i];
    QString toLink = chain[i + 1];

    // Find joint connecting fromLink to toLink
    for (const auto& j : joints_) {
      if (j.parentLink == fromLink && j.childLink == toLink) {
        // Apply joint transform
        QQuaternion jointRot = j.origin.toQuaternion();
        QVector3D jointPos = j.origin.position;

        cumulativePos = cumulativePos + cumulativeRot.rotatedVector(jointPos);
        cumulativeRot = cumulativeRot * jointRot;
        break;
      }
    }
  }

  return URDFPose::fromQuaternion(cumulativePos, cumulativeRot);
}

bool URDFModel::isValid() const {
  return !name_.isEmpty() && !links_.isEmpty();
}

QStringList URDFModel::validationErrors() const {
  QStringList errors;

  if (name_.isEmpty()) {
    errors << "Model has no name";
  }

  if (links_.isEmpty()) {
    errors << "Model has no links";
  }

  // Check that all joint parent/child links exist
  for (const auto& joint : joints_) {
    if (!hasLink(joint.parentLink)) {
      errors << QString("Joint '%1' references non-existent parent link '%2'")
                    .arg(joint.name, joint.parentLink);
    }
    if (!hasLink(joint.childLink)) {
      errors << QString("Joint '%1' references non-existent child link '%2'")
                    .arg(joint.name, joint.childLink);
    }
  }

  // Check for cycles in the kinematic tree
  QSet<QString> visited;
  QString rootName = rootLinkName();
  if (!rootName.isEmpty()) {
    QStringList queue;
    queue.append(rootName);
    while (!queue.isEmpty()) {
      QString current = queue.takeFirst();
      if (visited.contains(current)) {
        errors << QString("Cycle detected in kinematic tree at link '%1'").arg(current);
        break;
      }
      visited.insert(current);
      queue.append(childLinkNames(current));
    }
  }

  return errors;
}

void URDFModel::clear() {
  name_.clear();
  filePath_.clear();
  links_.clear();
  joints_.clear();
  materials_.clear();
  modified_ = false;
}

}  // namespace ros_weaver
