#include "ros_weaver/core/urdf_parser.hpp"
#include <QFile>
#include <QFileInfo>
#include <QProcess>
#include <QProcessEnvironment>
#include <QDir>
#include <QTextStream>
#include <QRegularExpression>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>

namespace ros_weaver {

URDFParser::URDFParser(QObject* parent)
  : QObject(parent) {
}

URDFParser::~URDFParser() = default;

std::unique_ptr<URDFModel> URDFParser::loadFile(const QString& filePath) {
  QFileInfo fileInfo(filePath);
  if (!fileInfo.exists()) {
    lastError_ = tr("File does not exist: %1").arg(filePath);
    emit parseError(lastError_);
    return nullptr;
  }

  emit parseProgress(10, tr("Loading file..."));

  QString xmlContent;

  if (isXacroFile(filePath)) {
    // Expand xacro first
    emit parseProgress(20, tr("Expanding xacro..."));
    xmlContent = expandXacro(filePath);
    if (xmlContent.isEmpty()) {
      return nullptr;  // Error already set by expandXacro
    }
  } else {
    // Read URDF directly
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      lastError_ = tr("Failed to open file: %1").arg(file.errorString());
      emit parseError(lastError_);
      return nullptr;
    }
    xmlContent = QString::fromUtf8(file.readAll());
    file.close();
  }

  emit parseProgress(50, tr("Parsing XML..."));

  auto model = loadFromString(xmlContent);
  if (model) {
    model->setFilePath(filePath);
  }

  emit parseProgress(100, tr("Done"));
  return model;
}

std::unique_ptr<URDFModel> URDFParser::loadFromString(const QString& xml) {
  QDomDocument doc;
  QString errorMsg;
  int errorLine, errorCol;

  if (!doc.setContent(xml, &errorMsg, &errorLine, &errorCol)) {
    lastError_ = tr("XML parse error at line %1, column %2: %3")
                     .arg(errorLine).arg(errorCol).arg(errorMsg);
    emit parseError(lastError_);
    return nullptr;
  }

  QDomElement root = doc.documentElement();
  if (root.tagName() != "robot") {
    lastError_ = tr("Root element is not 'robot' (found '%1')").arg(root.tagName());
    emit parseError(lastError_);
    return nullptr;
  }

  auto model = std::make_unique<URDFModel>();
  if (!parseRobot(root, *model)) {
    return nullptr;  // Error already set
  }

  return model;
}

bool URDFParser::saveFile(const URDFModel& model, const QString& filePath, bool createBackup) {
  // Create backup if requested and file exists
  if (createBackup) {
    QFile existingFile(filePath);
    if (existingFile.exists()) {
      QString backupPath = filePath + ".backup";
      QFile::remove(backupPath);  // Remove old backup if exists
      if (!existingFile.copy(backupPath)) {
        lastError_ = tr("Failed to create backup file: %1").arg(backupPath);
        emit parseError(lastError_);
        return false;
      }
    }
  }

  QString xml = toXmlString(model);
  if (xml.isEmpty()) {
    return false;  // Error already set
  }

  QFile file(filePath);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    lastError_ = tr("Failed to open file for writing: %1").arg(file.errorString());
    emit parseError(lastError_);
    return false;
  }

  QTextStream stream(&file);
  stream << xml;

  if (stream.status() != QTextStream::Ok) {
    lastError_ = tr("Write error: stream status indicates failure");
    emit parseError(lastError_);
    file.close();
    return false;
  }

  file.close();

  if (file.error() != QFile::NoError) {
    lastError_ = tr("Write error: %1").arg(file.errorString());
    emit parseError(lastError_);
    return false;
  }

  return true;
}

QString URDFParser::toXmlString(const URDFModel& model) {
  QDomDocument doc;

  // XML declaration
  QDomProcessingInstruction xmlDecl = doc.createProcessingInstruction(
      "xml", "version=\"1.0\" encoding=\"UTF-8\"");
  doc.appendChild(xmlDecl);

  // Robot element
  QDomElement robot = doc.createElement("robot");
  robot.setAttribute("name", model.name());
  doc.appendChild(robot);

  // Materials (global)
  for (const auto& material : model.materials()) {
    robot.appendChild(createMaterialElement(doc, material));
  }

  // Links
  for (const auto& link : model.links()) {
    robot.appendChild(createLinkElement(doc, link));
  }

  // Joints
  for (const auto& joint : model.joints()) {
    robot.appendChild(createJointElement(doc, joint));
  }

  return doc.toString(2);  // 2-space indentation
}

QString URDFParser::findWorkspaceRoot(const QString& filePath) {
  QFileInfo fileInfo(filePath);
  QDir searchDir = fileInfo.absoluteDir();

  // Search up the directory tree for a workspace
  // A ROS2 workspace has src/ AND typically install/ or build/ directories
  for (int i = 0; i < 15; ++i) {  // Limit search depth
    QString basePath = searchDir.absolutePath();

    // Check if this directory has src/ subdirectory
    QDir srcDir(basePath + "/src");
    if (srcDir.exists()) {
      // Verify this is actually a workspace by checking for install/ or build/
      // This avoids falsely detecting package-level src/ directories
      QDir installDir(basePath + "/install");
      QDir buildDir(basePath + "/build");
      if (installDir.exists() || buildDir.exists()) {
        return basePath;
      }
    }

    if (!searchDir.cdUp()) {
      break;
    }
  }

  return QString();
}

bool URDFParser::isWorkspaceBuilt(const QString& workspaceRoot) {
  if (workspaceRoot.isEmpty()) {
    return false;
  }

  // Check if install/setup.bash or install/local_setup.bash exists
  QString setupPath = workspaceRoot + "/install/setup.bash";
  QString localSetupPath = workspaceRoot + "/install/local_setup.bash";

  return QFile::exists(setupPath) || QFile::exists(localSetupPath);
}

QString URDFParser::expandXacro(const QString& xacroPath, const QMap<QString, QString>& args) {
  // First, find the workspace and check if it's built
  QString workspaceRoot = findWorkspaceRoot(xacroPath);
  QString workspaceInstall;

  if (!workspaceRoot.isEmpty()) {
    workspaceInstall = workspaceRoot + "/install";

    // Check if workspace is built
    if (!isWorkspaceBuilt(workspaceRoot)) {
      QString buildCommand = QString("cd %1 && colcon build").arg(workspaceRoot);
      lastError_ = tr("Workspace not built");
      emit workspaceNotBuilt(workspaceRoot, buildCommand);
      return QString();
    }
  }

  QProcess process;

  // Build the xacro command
  QString xacroCmd = QString("ros2 run xacro xacro '%1'").arg(xacroPath);

  // Add xacro arguments with shell escaping
  for (auto it = args.constBegin(); it != args.constEnd(); ++it) {
    // Escape single quotes in key and value to prevent shell injection
    QString safeKey = it.key();
    safeKey.replace("'", "'\\''");
    QString safeValue = it.value();
    safeValue.replace("'", "'\\''");
    xacroCmd += QString(" '%1':='%2'").arg(safeKey, safeValue);
  }

  // If we have a workspace, source it before running xacro
  // This properly sets up AMENT_PREFIX_PATH with all package paths
  QString fullCmd;
  if (!workspaceInstall.isEmpty() && QFile::exists(workspaceInstall + "/setup.bash")) {
    fullCmd = QString("source '%1/setup.bash' && %2").arg(workspaceInstall, xacroCmd);
  } else {
    fullCmd = xacroCmd;
  }

  process.start("bash", QStringList() << "-c" << fullCmd);
  if (!process.waitForStarted(5000)) {
    lastError_ = tr("Failed to start xacro process: %1").arg(process.errorString());
    emit parseError(lastError_);
    return QString();
  }

  if (!process.waitForFinished(30000)) {
    process.kill();
    lastError_ = tr("Xacro process timed out");
    emit parseError(lastError_);
    return QString();
  }

  if (process.exitCode() != 0) {
    QString stderr = QString::fromUtf8(process.readAllStandardError());

    // Check for package not found errors - might need rebuild or missing dependency
    if (stderr.contains("PackageNotFoundError") || (stderr.contains("package") && stderr.contains("not found"))) {
      // Extract the package name if possible
      QRegularExpression pkgRegex("package '([^']+)' not found");
      QRegularExpressionMatch match = pkgRegex.match(stderr);
      QString missingPkg = match.hasMatch() ? match.captured(1) : tr("unknown");

      if (!workspaceRoot.isEmpty()) {
        QString buildCommand = QString("cd %1 && colcon build").arg(workspaceRoot);
        emit workspaceNotBuilt(workspaceRoot, buildCommand);
        lastError_ = tr("Package '%1' not found. The workspace may need to be rebuilt.").arg(missingPkg);
      } else {
        lastError_ = tr("Package '%1' not found.\n\n"
                        "No ROS workspace found containing this file.\n"
                        "Make sure to:\n"
                        "1. Build your workspace (colcon build)\n"
                        "2. Source your workspace (source install/setup.bash)\n"
                        "3. Launch ROS2Weaver from the sourced terminal").arg(missingPkg);
      }
    } else {
      lastError_ = tr("Xacro expansion failed: %1").arg(stderr);
    }
    emit parseError(lastError_);
    return QString();
  }

  return QString::fromUtf8(process.readAllStandardOutput());
}

QString URDFParser::resolvePackageUri(const QString& uri) {
  if (!uri.startsWith("package://")) {
    return uri;  // Not a package URI
  }

  // Parse package://package_name/path
  QString withoutPrefix = uri.mid(10);  // Remove "package://"
  int slashIndex = withoutPrefix.indexOf('/');
  if (slashIndex == -1) {
    lastError_ = tr("Invalid package URI: %1").arg(uri);
    return QString();
  }

  QString packageName = withoutPrefix.left(slashIndex);
  QString relativePath = withoutPrefix.mid(slashIndex + 1);

  // Check cache first
  if (!packagePathCache_.contains(packageName)) {
    try {
      std::string shareDir = ament_index_cpp::get_package_share_directory(packageName.toStdString());
      packagePathCache_[packageName] = QString::fromStdString(shareDir);
    } catch (const std::exception& e) {
      lastError_ = tr("Failed to find package '%1': %2").arg(packageName, e.what());
      return QString();
    }
  }

  return packagePathCache_[packageName] + "/" + relativePath;
}

bool URDFParser::isXacroFile(const QString& filePath) {
  return filePath.endsWith(".xacro", Qt::CaseInsensitive) ||
         filePath.endsWith(".urdf.xacro", Qt::CaseInsensitive);
}

// ============================================================================
// XML Parsing
// ============================================================================

bool URDFParser::parseRobot(const QDomElement& robotElement, URDFModel& model) {
  model.setName(robotElement.attribute("name"));

  QDomElement child = robotElement.firstChildElement();
  while (!child.isNull()) {
    QString tagName = child.tagName();

    if (tagName == "material") {
      URDFMaterial material;
      if (parseMaterial(child, material)) {
        model.addMaterial(material);
      }
    } else if (tagName == "link") {
      URDFLink link;
      if (parseLink(child, link)) {
        model.addLink(link);
      } else {
        return false;
      }
    } else if (tagName == "joint") {
      URDFJoint joint;
      if (parseJoint(child, joint)) {
        model.addJoint(joint);
      } else {
        return false;
      }
    }
    // Ignore unknown elements (gazebo plugins, etc.)

    child = child.nextSiblingElement();
  }

  return true;
}

bool URDFParser::parseLink(const QDomElement& linkElement, URDFLink& link) {
  link.name = linkElement.attribute("name");
  if (link.name.isEmpty()) {
    lastError_ = tr("Link element missing 'name' attribute");
    emit parseError(lastError_);
    return false;
  }

  QDomElement child = linkElement.firstChildElement();
  while (!child.isNull()) {
    QString tagName = child.tagName();

    if (tagName == "inertial") {
      parseInertial(child, link.inertial);
    } else if (tagName == "visual") {
      URDFVisual visual;
      if (parseVisual(child, visual)) {
        link.visuals.append(visual);
      }
    } else if (tagName == "collision") {
      URDFCollision collision;
      if (parseCollision(child, collision)) {
        link.collisions.append(collision);
      }
    }

    child = child.nextSiblingElement();
  }

  return true;
}

bool URDFParser::parseJoint(const QDomElement& jointElement, URDFJoint& joint) {
  joint.name = jointElement.attribute("name");
  if (joint.name.isEmpty()) {
    lastError_ = tr("Joint element missing 'name' attribute");
    emit parseError(lastError_);
    return false;
  }

  joint.type = URDFJoint::typeFromString(jointElement.attribute("type", "fixed"));

  QDomElement child = jointElement.firstChildElement();
  while (!child.isNull()) {
    QString tagName = child.tagName();

    if (tagName == "origin") {
      parsePose(child, joint.origin);
    } else if (tagName == "parent") {
      joint.parentLink = child.attribute("link");
    } else if (tagName == "child") {
      joint.childLink = child.attribute("link");
    } else if (tagName == "axis") {
      bool ok;
      joint.axis = parseVector3(child.attribute("xyz", "1 0 0"), &ok);
    } else if (tagName == "limit") {
      joint.limits.lower = child.attribute("lower", "0").toDouble();
      joint.limits.upper = child.attribute("upper", "0").toDouble();
      joint.limits.effort = child.attribute("effort", "0").toDouble();
      joint.limits.velocity = child.attribute("velocity", "0").toDouble();
    } else if (tagName == "dynamics") {
      joint.dynamics.damping = child.attribute("damping", "0").toDouble();
      joint.dynamics.friction = child.attribute("friction", "0").toDouble();
    } else if (tagName == "safety_controller") {
      joint.safety.softLowerLimit = child.attribute("soft_lower_limit", "0").toDouble();
      joint.safety.softUpperLimit = child.attribute("soft_upper_limit", "0").toDouble();
      joint.safety.kPosition = child.attribute("k_position", "0").toDouble();
      joint.safety.kVelocity = child.attribute("k_velocity", "0").toDouble();
    } else if (tagName == "calibration") {
      joint.calibration.rising = child.attribute("rising", "0").toDouble();
      joint.calibration.falling = child.attribute("falling", "0").toDouble();
    } else if (tagName == "mimic") {
      joint.mimic.joint = child.attribute("joint");
      joint.mimic.multiplier = child.attribute("multiplier", "1").toDouble();
      joint.mimic.offset = child.attribute("offset", "0").toDouble();
    }

    child = child.nextSiblingElement();
  }

  return true;
}

bool URDFParser::parseMaterial(const QDomElement& materialElement, URDFMaterial& material) {
  material.name = materialElement.attribute("name");

  QDomElement child = materialElement.firstChildElement();
  while (!child.isNull()) {
    if (child.tagName() == "color") {
      bool ok;
      material.color = parseColor(child.attribute("rgba"), &ok);
      if (ok) {
        material.colorExplicitlySet = true;
      }
    } else if (child.tagName() == "texture") {
      material.textureFilename = child.attribute("filename");
    }
    child = child.nextSiblingElement();
  }

  return true;
}

bool URDFParser::parseInertial(const QDomElement& element, URDFInertial& inertial) {
  QDomElement child = element.firstChildElement();
  while (!child.isNull()) {
    if (child.tagName() == "origin") {
      parsePose(child, inertial.origin);
    } else if (child.tagName() == "mass") {
      inertial.mass = child.attribute("value", "0").toDouble();
    } else if (child.tagName() == "inertia") {
      inertial.ixx = child.attribute("ixx", "0").toDouble();
      inertial.ixy = child.attribute("ixy", "0").toDouble();
      inertial.ixz = child.attribute("ixz", "0").toDouble();
      inertial.iyy = child.attribute("iyy", "0").toDouble();
      inertial.iyz = child.attribute("iyz", "0").toDouble();
      inertial.izz = child.attribute("izz", "0").toDouble();
    }
    child = child.nextSiblingElement();
  }
  return true;
}

bool URDFParser::parseVisual(const QDomElement& element, URDFVisual& visual) {
  visual.name = element.attribute("name");

  QDomElement child = element.firstChildElement();
  while (!child.isNull()) {
    if (child.tagName() == "origin") {
      parsePose(child, visual.origin);
    } else if (child.tagName() == "geometry") {
      parseGeometry(child, visual.geometry);
    } else if (child.tagName() == "material") {
      parseMaterial(child, visual.material);
    }
    child = child.nextSiblingElement();
  }
  return true;
}

bool URDFParser::parseCollision(const QDomElement& element, URDFCollision& collision) {
  collision.name = element.attribute("name");

  QDomElement child = element.firstChildElement();
  while (!child.isNull()) {
    if (child.tagName() == "origin") {
      parsePose(child, collision.origin);
    } else if (child.tagName() == "geometry") {
      parseGeometry(child, collision.geometry);
    }
    child = child.nextSiblingElement();
  }
  return true;
}

bool URDFParser::parseGeometry(const QDomElement& element, URDFGeometry& geometry) {
  QDomElement child = element.firstChildElement();
  if (child.isNull()) {
    return false;
  }

  QString tagName = child.tagName();
  if (tagName == "box") {
    geometry.type = URDFGeometryType::Box;
    bool ok;
    geometry.boxSize = parseVector3(child.attribute("size", "1 1 1"), &ok);
  } else if (tagName == "cylinder") {
    geometry.type = URDFGeometryType::Cylinder;
    geometry.cylinderRadius = child.attribute("radius", "0.5").toDouble();
    geometry.cylinderLength = child.attribute("length", "1").toDouble();
  } else if (tagName == "sphere") {
    geometry.type = URDFGeometryType::Sphere;
    geometry.sphereRadius = child.attribute("radius", "0.5").toDouble();
  } else if (tagName == "mesh") {
    geometry.type = URDFGeometryType::Mesh;
    geometry.meshFilename = child.attribute("filename");
    QString scaleStr = child.attribute("scale");
    if (!scaleStr.isEmpty()) {
      bool ok;
      geometry.meshScale = parseVector3(scaleStr, &ok);
    }
  }

  return true;
}

bool URDFParser::parsePose(const QDomElement& element, URDFPose& pose) {
  QString xyzStr = element.attribute("xyz");
  QString rpyStr = element.attribute("rpy");

  bool ok;
  if (!xyzStr.isEmpty()) {
    pose.position = parseVector3(xyzStr, &ok);
  }
  if (!rpyStr.isEmpty()) {
    pose.rpy = parseVector3(rpyStr, &ok);
  }

  return true;
}

// ============================================================================
// XML Generation
// ============================================================================

QDomElement URDFParser::createLinkElement(QDomDocument& doc, const URDFLink& link) {
  QDomElement elem = doc.createElement("link");
  elem.setAttribute("name", link.name);

  if (link.hasInertial()) {
    elem.appendChild(createInertialElement(doc, link.inertial));
  }

  for (const auto& visual : link.visuals) {
    elem.appendChild(createVisualElement(doc, visual));
  }

  for (const auto& collision : link.collisions) {
    elem.appendChild(createCollisionElement(doc, collision));
  }

  return elem;
}

QDomElement URDFParser::createJointElement(QDomDocument& doc, const URDFJoint& joint) {
  QDomElement elem = doc.createElement("joint");
  elem.setAttribute("name", joint.name);
  elem.setAttribute("type", URDFJoint::typeToString(joint.type));

  // Origin (if non-zero)
  if (joint.origin.position.length() > 1e-6 || joint.origin.rpy.length() > 1e-6) {
    elem.appendChild(createPoseElement(doc, "origin", joint.origin));
  }

  // Parent link
  QDomElement parent = doc.createElement("parent");
  parent.setAttribute("link", joint.parentLink);
  elem.appendChild(parent);

  // Child link
  QDomElement child = doc.createElement("child");
  child.setAttribute("link", joint.childLink);
  elem.appendChild(child);

  // Axis (if not default)
  if (joint.type != URDFJointType::Fixed &&
      (joint.axis.x() != 1.0f || joint.axis.y() != 0.0f || joint.axis.z() != 0.0f)) {
    QDomElement axis = doc.createElement("axis");
    axis.setAttribute("xyz", formatVector3(joint.axis));
    elem.appendChild(axis);
  }

  // Limits
  if (joint.hasLimits()) {
    QDomElement limit = doc.createElement("limit");
    limit.setAttribute("lower", QString::number(joint.limits.lower));
    limit.setAttribute("upper", QString::number(joint.limits.upper));
    if (joint.limits.effort > 0) {
      limit.setAttribute("effort", QString::number(joint.limits.effort));
    }
    if (joint.limits.velocity > 0) {
      limit.setAttribute("velocity", QString::number(joint.limits.velocity));
    }
    elem.appendChild(limit);
  }

  // Dynamics
  if (joint.hasDynamics()) {
    QDomElement dynamics = doc.createElement("dynamics");
    dynamics.setAttribute("damping", QString::number(joint.dynamics.damping));
    dynamics.setAttribute("friction", QString::number(joint.dynamics.friction));
    elem.appendChild(dynamics);
  }

  // Safety controller
  if (joint.hasSafety()) {
    QDomElement safety = doc.createElement("safety_controller");
    safety.setAttribute("soft_lower_limit", QString::number(joint.safety.softLowerLimit));
    safety.setAttribute("soft_upper_limit", QString::number(joint.safety.softUpperLimit));
    safety.setAttribute("k_position", QString::number(joint.safety.kPosition));
    safety.setAttribute("k_velocity", QString::number(joint.safety.kVelocity));
    elem.appendChild(safety);
  }

  // Mimic
  if (joint.hasMimic()) {
    QDomElement mimic = doc.createElement("mimic");
    mimic.setAttribute("joint", joint.mimic.joint);
    if (joint.mimic.multiplier != 1.0) {
      mimic.setAttribute("multiplier", QString::number(joint.mimic.multiplier));
    }
    if (joint.mimic.offset != 0.0) {
      mimic.setAttribute("offset", QString::number(joint.mimic.offset));
    }
    elem.appendChild(mimic);
  }

  return elem;
}

QDomElement URDFParser::createMaterialElement(QDomDocument& doc, const URDFMaterial& material) {
  QDomElement elem = doc.createElement("material");
  elem.setAttribute("name", material.name);

  if (material.hasColor()) {
    QDomElement color = doc.createElement("color");
    color.setAttribute("rgba", formatColor(material.color));
    elem.appendChild(color);
  }

  if (material.hasTexture()) {
    QDomElement texture = doc.createElement("texture");
    texture.setAttribute("filename", material.textureFilename);
    elem.appendChild(texture);
  }

  return elem;
}

QDomElement URDFParser::createInertialElement(QDomDocument& doc, const URDFInertial& inertial) {
  QDomElement elem = doc.createElement("inertial");

  if (inertial.origin.position.length() > 1e-6 || inertial.origin.rpy.length() > 1e-6) {
    elem.appendChild(createPoseElement(doc, "origin", inertial.origin));
  }

  QDomElement mass = doc.createElement("mass");
  mass.setAttribute("value", QString::number(inertial.mass));
  elem.appendChild(mass);

  QDomElement inertia = doc.createElement("inertia");
  inertia.setAttribute("ixx", QString::number(inertial.ixx));
  inertia.setAttribute("ixy", QString::number(inertial.ixy));
  inertia.setAttribute("ixz", QString::number(inertial.ixz));
  inertia.setAttribute("iyy", QString::number(inertial.iyy));
  inertia.setAttribute("iyz", QString::number(inertial.iyz));
  inertia.setAttribute("izz", QString::number(inertial.izz));
  elem.appendChild(inertia);

  return elem;
}

QDomElement URDFParser::createVisualElement(QDomDocument& doc, const URDFVisual& visual) {
  QDomElement elem = doc.createElement("visual");
  if (!visual.name.isEmpty()) {
    elem.setAttribute("name", visual.name);
  }

  if (visual.origin.position.length() > 1e-6 || visual.origin.rpy.length() > 1e-6) {
    elem.appendChild(createPoseElement(doc, "origin", visual.origin));
  }

  elem.appendChild(createGeometryElement(doc, visual.geometry));

  if (!visual.material.name.isEmpty() || visual.material.hasColor()) {
    elem.appendChild(createMaterialElement(doc, visual.material));
  }

  return elem;
}

QDomElement URDFParser::createCollisionElement(QDomDocument& doc, const URDFCollision& collision) {
  QDomElement elem = doc.createElement("collision");
  if (!collision.name.isEmpty()) {
    elem.setAttribute("name", collision.name);
  }

  if (collision.origin.position.length() > 1e-6 || collision.origin.rpy.length() > 1e-6) {
    elem.appendChild(createPoseElement(doc, "origin", collision.origin));
  }

  elem.appendChild(createGeometryElement(doc, collision.geometry));

  return elem;
}

QDomElement URDFParser::createGeometryElement(QDomDocument& doc, const URDFGeometry& geometry) {
  QDomElement elem = doc.createElement("geometry");

  switch (geometry.type) {
    case URDFGeometryType::Box: {
      QDomElement box = doc.createElement("box");
      box.setAttribute("size", formatVector3(geometry.boxSize));
      elem.appendChild(box);
      break;
    }
    case URDFGeometryType::Cylinder: {
      QDomElement cylinder = doc.createElement("cylinder");
      cylinder.setAttribute("radius", QString::number(geometry.cylinderRadius));
      cylinder.setAttribute("length", QString::number(geometry.cylinderLength));
      elem.appendChild(cylinder);
      break;
    }
    case URDFGeometryType::Sphere: {
      QDomElement sphere = doc.createElement("sphere");
      sphere.setAttribute("radius", QString::number(geometry.sphereRadius));
      elem.appendChild(sphere);
      break;
    }
    case URDFGeometryType::Mesh: {
      QDomElement mesh = doc.createElement("mesh");
      mesh.setAttribute("filename", geometry.meshFilename);
      if (geometry.meshScale != QVector3D(1, 1, 1)) {
        mesh.setAttribute("scale", formatVector3(geometry.meshScale));
      }
      elem.appendChild(mesh);
      break;
    }
  }

  return elem;
}

QDomElement URDFParser::createPoseElement(QDomDocument& doc, const QString& tagName, const URDFPose& pose) {
  QDomElement elem = doc.createElement(tagName);
  elem.setAttribute("xyz", formatVector3(pose.position));
  elem.setAttribute("rpy", formatVector3(pose.rpy));
  return elem;
}

// ============================================================================
// Utility Methods
// ============================================================================

QVector3D URDFParser::parseVector3(const QString& str, bool* ok) {
  QVector3D result;
  QStringList parts = str.simplified().split(' ', Qt::SkipEmptyParts);
  if (parts.size() >= 3) {
    bool xOk, yOk, zOk;
    result.setX(parts[0].toFloat(&xOk));
    result.setY(parts[1].toFloat(&yOk));
    result.setZ(parts[2].toFloat(&zOk));
    if (ok) *ok = xOk && yOk && zOk;
  } else {
    if (ok) *ok = false;
  }
  return result;
}

QColor URDFParser::parseColor(const QString& str, bool* ok) {
  QColor result;
  QStringList parts = str.simplified().split(' ', Qt::SkipEmptyParts);
  if (parts.size() >= 4) {
    bool rOk, gOk, bOk, aOk;
    float r = parts[0].toFloat(&rOk);
    float g = parts[1].toFloat(&gOk);
    float b = parts[2].toFloat(&bOk);
    float a = parts[3].toFloat(&aOk);
    result.setRgbF(r, g, b, a);
    if (ok) *ok = rOk && gOk && bOk && aOk;
  } else {
    if (ok) *ok = false;
  }
  return result;
}

QString URDFParser::formatVector3(const QVector3D& vec) {
  return QString("%1 %2 %3")
      .arg(static_cast<double>(vec.x()), 0, 'g', 6)
      .arg(static_cast<double>(vec.y()), 0, 'g', 6)
      .arg(static_cast<double>(vec.z()), 0, 'g', 6);
}

QString URDFParser::formatColor(const QColor& color) {
  return QString("%1 %2 %3 %4")
      .arg(color.redF(), 0, 'g', 4)
      .arg(color.greenF(), 0, 'g', 4)
      .arg(color.blueF(), 0, 'g', 4)
      .arg(color.alphaF(), 0, 'g', 4);
}

}  // namespace ros_weaver
