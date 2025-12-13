#ifndef ROS_WEAVER_CORE_ROS_PACKAGE_INDEX_HPP
#define ROS_WEAVER_CORE_ROS_PACKAGE_INDEX_HPP

#include <QObject>
#include <QString>
#include <QList>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>

namespace ros_weaver {

// Represents a ROS package from the index
struct RosPackageInfo {
  QString name;
  QString description;
  QString version;
  QString repository;
  QString license;
  QStringList maintainers;
  QStringList dependencies;
  QString distro;  // e.g., "humble", "iron", "jazzy"

  // For local packages
  bool isLocal = false;
  QString localPath = {};
};

// Represents a message type
struct RosMessageInfo {
  QString package;      // e.g., "std_msgs"
  QString name;         // e.g., "String"
  QString fullName;     // e.g., "std_msgs/msg/String"
  QStringList fields;   // Field definitions
};

class RosPackageIndex : public QObject {
  Q_OBJECT

public:
  explicit RosPackageIndex(QObject* parent = nullptr);
  ~RosPackageIndex() override;

  // Search for packages by name or keyword
  void searchPackages(const QString& query, const QString& distro = "humble");

  // Get package details
  void getPackageDetails(const QString& packageName, const QString& distro = "humble");

  // Search for message types
  void searchMessageTypes(const QString& query);

  // Scan local workspace for packages
  void scanLocalWorkspace(const QString& workspacePath);

  // Find the source path for a ROS2 package
  // Returns empty string if not found
  QString findPackagePath(const QString& packageName);

  // Get common message types (cached)
  QList<RosMessageInfo> commonMessageTypes() const;

  // Set the ROS distro to use
  void setDistro(const QString& distro) { distro_ = distro; }
  QString distro() const { return distro_; }

signals:
  void searchResultsReady(const QList<RosPackageInfo>& packages);
  void packageDetailsReady(const RosPackageInfo& package);
  void messageTypesReady(const QList<RosMessageInfo>& messages);
  void localPackagesReady(const QList<RosPackageInfo>& packages);
  void searchError(const QString& error);

private slots:
  void onSearchReplyFinished();
  void onDetailsReplyFinished();

private:
  void initCommonMessageTypes();

  QNetworkAccessManager* networkManager_;
  QString distro_;
  QList<RosMessageInfo> commonMessages_;

  // Track pending requests
  QNetworkReply* pendingSearchReply_;
  QNetworkReply* pendingDetailsReply_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_ROS_PACKAGE_INDEX_HPP
