#ifndef ROS_WEAVER_CORE_URDF_PARSER_HPP
#define ROS_WEAVER_CORE_URDF_PARSER_HPP

#include "ros_weaver/core/urdf_model.hpp"
#include <QObject>
#include <QString>
#include <QDomDocument>
#include <memory>

namespace ros_weaver {

/**
 * @brief Parser for URDF and xacro files
 *
 * Handles:
 * - Direct URDF XML parsing
 * - Xacro expansion via ros2 run xacro xacro
 * - Package path resolution (package:// URIs)
 * - Mesh file path resolution
 */
class URDFParser : public QObject {
  Q_OBJECT

public:
  explicit URDFParser(QObject* parent = nullptr);
  ~URDFParser() override;

  /**
   * @brief Load a URDF or xacro file
   * @param filePath Path to the .urdf or .xacro file
   * @return Parsed URDF model, or nullptr on failure
   */
  std::unique_ptr<URDFModel> loadFile(const QString& filePath);

  /**
   * @brief Load URDF from XML string
   * @param xml URDF XML content
   * @return Parsed URDF model, or nullptr on failure
   */
  std::unique_ptr<URDFModel> loadFromString(const QString& xml);

  /**
   * @brief Save a URDF model to file
   * @param model The model to save
   * @param filePath Output file path
   * @param createBackup If true, backup existing file before overwriting
   * @return true on success
   */
  bool saveFile(const URDFModel& model, const QString& filePath, bool createBackup = true);

  /**
   * @brief Export model to URDF XML string
   * @param model The model to export
   * @return URDF XML content
   */
  QString toXmlString(const URDFModel& model);

  /**
   * @brief Expand a xacro file to URDF
   * @param xacroPath Path to the .xacro file
   * @param args Optional xacro arguments as key=value pairs
   * @return Expanded URDF XML content, or empty string on failure
   */
  QString expandXacro(const QString& xacroPath, const QMap<QString, QString>& args = {});

  /**
   * @brief Resolve a package:// URI to an absolute path
   * @param uri The package:// URI
   * @return Resolved absolute path, or empty string if resolution fails
   */
  QString resolvePackageUri(const QString& uri);

  /**
   * @brief Get the last error message
   */
  QString lastError() const { return lastError_; }

  /**
   * @brief Check if a file is a xacro file
   */
  static bool isXacroFile(const QString& filePath);

  /**
   * @brief Find the ROS workspace containing a file
   * @param filePath Path to a file in the workspace
   * @return Workspace root path (containing src/ and install/), or empty if not found
   */
  static QString findWorkspaceRoot(const QString& filePath);

  /**
   * @brief Check if a workspace has been built
   * @param workspaceRoot Root path of the workspace
   * @return true if install/setup.bash exists
   */
  static bool isWorkspaceBuilt(const QString& workspaceRoot);

signals:
  void parseProgress(int percent, const QString& message);
  void parseError(const QString& error);
  void workspaceNotBuilt(const QString& workspacePath, const QString& buildCommand);

private:
  // XML parsing helpers
  bool parseRobot(const QDomElement& robotElement, URDFModel& model);
  bool parseLink(const QDomElement& linkElement, URDFLink& link);
  bool parseJoint(const QDomElement& jointElement, URDFJoint& joint);
  bool parseMaterial(const QDomElement& materialElement, URDFMaterial& material);

  bool parseInertial(const QDomElement& element, URDFInertial& inertial);
  bool parseVisual(const QDomElement& element, URDFVisual& visual);
  bool parseCollision(const QDomElement& element, URDFCollision& collision);
  bool parseGeometry(const QDomElement& element, URDFGeometry& geometry);
  bool parsePose(const QDomElement& element, URDFPose& pose);

  // XML generation helpers
  QDomElement createLinkElement(QDomDocument& doc, const URDFLink& link);
  QDomElement createJointElement(QDomDocument& doc, const URDFJoint& joint);
  QDomElement createMaterialElement(QDomDocument& doc, const URDFMaterial& material);
  QDomElement createInertialElement(QDomDocument& doc, const URDFInertial& inertial);
  QDomElement createVisualElement(QDomDocument& doc, const URDFVisual& visual);
  QDomElement createCollisionElement(QDomDocument& doc, const URDFCollision& collision);
  QDomElement createGeometryElement(QDomDocument& doc, const URDFGeometry& geometry);
  QDomElement createPoseElement(QDomDocument& doc, const QString& tagName, const URDFPose& pose);

  // Utility methods
  QVector3D parseVector3(const QString& str, bool* ok = nullptr);
  QColor parseColor(const QString& str, bool* ok = nullptr);
  QString formatVector3(const QVector3D& vec);
  QString formatColor(const QColor& color);

  QString lastError_;

  // Cache of resolved package paths
  QMap<QString, QString> packagePathCache_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_URDF_PARSER_HPP
