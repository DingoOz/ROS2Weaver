#include "ros_weaver/core/ros_package_index.hpp"
#include <QDir>
#include <QFile>
#include <QDirIterator>
#include <QXmlStreamReader>
#include <QUrl>
#include <QUrlQuery>

namespace ros_weaver {

RosPackageIndex::RosPackageIndex(QObject* parent)
  : QObject(parent)
  , networkManager_(new QNetworkAccessManager(this))
  , distro_("humble")
  , pendingSearchReply_(nullptr)
  , pendingDetailsReply_(nullptr)
{
  initCommonMessageTypes();
}

RosPackageIndex::~RosPackageIndex() = default;

void RosPackageIndex::initCommonMessageTypes() {
  // Initialize common ROS2 message types for quick access
  commonMessages_ = {
    // std_msgs
    {"std_msgs", "String", "std_msgs/msg/String", {"string data"}},
    {"std_msgs", "Int32", "std_msgs/msg/Int32", {"int32 data"}},
    {"std_msgs", "Int64", "std_msgs/msg/Int64", {"int64 data"}},
    {"std_msgs", "Float32", "std_msgs/msg/Float32", {"float32 data"}},
    {"std_msgs", "Float64", "std_msgs/msg/Float64", {"float64 data"}},
    {"std_msgs", "Bool", "std_msgs/msg/Bool", {"bool data"}},
    {"std_msgs", "Header", "std_msgs/msg/Header", {"builtin_interfaces/Time stamp", "string frame_id"}},

    // geometry_msgs
    {"geometry_msgs", "Point", "geometry_msgs/msg/Point", {"float64 x", "float64 y", "float64 z"}},
    {"geometry_msgs", "Pose", "geometry_msgs/msg/Pose", {"Point position", "Quaternion orientation"}},
    {"geometry_msgs", "PoseStamped", "geometry_msgs/msg/PoseStamped", {"Header header", "Pose pose"}},
    {"geometry_msgs", "Twist", "geometry_msgs/msg/Twist", {"Vector3 linear", "Vector3 angular"}},
    {"geometry_msgs", "TwistStamped", "geometry_msgs/msg/TwistStamped", {"Header header", "Twist twist"}},
    {"geometry_msgs", "Vector3", "geometry_msgs/msg/Vector3", {"float64 x", "float64 y", "float64 z"}},
    {"geometry_msgs", "Quaternion", "geometry_msgs/msg/Quaternion", {"float64 x", "float64 y", "float64 z", "float64 w"}},
    {"geometry_msgs", "Transform", "geometry_msgs/msg/Transform", {"Vector3 translation", "Quaternion rotation"}},
    {"geometry_msgs", "TransformStamped", "geometry_msgs/msg/TransformStamped", {"Header header", "string child_frame_id", "Transform transform"}},

    // sensor_msgs
    {"sensor_msgs", "Image", "sensor_msgs/msg/Image", {"Header header", "uint32 height", "uint32 width", "string encoding", "uint8 is_bigendian", "uint32 step", "uint8[] data"}},
    {"sensor_msgs", "CameraInfo", "sensor_msgs/msg/CameraInfo", {"Header header", "uint32 height", "uint32 width", "string distortion_model", "float64[] d", "float64[9] k", "float64[9] r", "float64[12] p"}},
    {"sensor_msgs", "LaserScan", "sensor_msgs/msg/LaserScan", {"Header header", "float32 angle_min", "float32 angle_max", "float32 angle_increment", "float32 time_increment", "float32 scan_time", "float32 range_min", "float32 range_max", "float32[] ranges", "float32[] intensities"}},
    {"sensor_msgs", "PointCloud2", "sensor_msgs/msg/PointCloud2", {"Header header", "uint32 height", "uint32 width", "PointField[] fields", "bool is_bigendian", "uint32 point_step", "uint32 row_step", "uint8[] data", "bool is_dense"}},
    {"sensor_msgs", "Imu", "sensor_msgs/msg/Imu", {"Header header", "Quaternion orientation", "float64[9] orientation_covariance", "Vector3 angular_velocity", "float64[9] angular_velocity_covariance", "Vector3 linear_acceleration", "float64[9] linear_acceleration_covariance"}},
    {"sensor_msgs", "JointState", "sensor_msgs/msg/JointState", {"Header header", "string[] name", "float64[] position", "float64[] velocity", "float64[] effort"}},

    // nav_msgs
    {"nav_msgs", "Odometry", "nav_msgs/msg/Odometry", {"Header header", "string child_frame_id", "PoseWithCovariance pose", "TwistWithCovariance twist"}},
    {"nav_msgs", "Path", "nav_msgs/msg/Path", {"Header header", "PoseStamped[] poses"}},
    {"nav_msgs", "OccupancyGrid", "nav_msgs/msg/OccupancyGrid", {"Header header", "MapMetaData info", "int8[] data"}},

    // tf2_msgs
    {"tf2_msgs", "TFMessage", "tf2_msgs/msg/TFMessage", {"TransformStamped[] transforms"}},
  };
}

void RosPackageIndex::searchPackages(const QString& query, const QString& distro) {
  // Cancel any pending search
  if (pendingSearchReply_) {
    pendingSearchReply_->abort();
    pendingSearchReply_->deleteLater();
    pendingSearchReply_ = nullptr;
  }

  // Use ROS Index API
  // The ROS Index provides a search API at https://index.ros.org
  QUrl url("https://index.ros.org/api/v1/packages");
  QUrlQuery urlQuery;
  urlQuery.addQueryItem("search", query);
  urlQuery.addQueryItem("distro", distro);
  url.setQuery(urlQuery);

  QNetworkRequest request(url);
  request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

  pendingSearchReply_ = networkManager_->get(request);
  connect(pendingSearchReply_, &QNetworkReply::finished,
          this, &RosPackageIndex::onSearchReplyFinished);
}

void RosPackageIndex::onSearchReplyFinished() {
  QNetworkReply* reply = pendingSearchReply_;
  pendingSearchReply_ = nullptr;

  if (!reply) return;

  QList<RosPackageInfo> results;

  if (reply->error() != QNetworkReply::NoError) {
    // If network fails, provide some built-in packages as fallback
    // This is useful for offline mode or when the API is unavailable
    QString query = reply->request().url().query();

    // Provide common ROS2 packages as fallback results
    QList<RosPackageInfo> fallbackPackages = {
      {"rclcpp", "ROS Client Library for C++", "latest", "https://github.com/ros2/rclcpp", "Apache-2.0", {"Open Robotics"}, {}, distro_},
      {"rclpy", "ROS Client Library for Python", "latest", "https://github.com/ros2/rclpy", "Apache-2.0", {"Open Robotics"}, {}, distro_},
      {"std_msgs", "Standard ROS Messages", "latest", "https://github.com/ros2/common_interfaces", "Apache-2.0", {"Open Robotics"}, {}, distro_},
      {"geometry_msgs", "Geometry Messages", "latest", "https://github.com/ros2/common_interfaces", "Apache-2.0", {"Open Robotics"}, {}, distro_},
      {"sensor_msgs", "Sensor Messages", "latest", "https://github.com/ros2/common_interfaces", "Apache-2.0", {"Open Robotics"}, {}, distro_},
      {"nav_msgs", "Navigation Messages", "latest", "https://github.com/ros2/common_interfaces", "Apache-2.0", {"Open Robotics"}, {}, distro_},
      {"nav2_bringup", "Nav2 Bringup", "latest", "https://github.com/ros-planning/navigation2", "Apache-2.0", {"Open Robotics"}, {}, distro_},
      {"slam_toolbox", "SLAM Toolbox", "latest", "https://github.com/SteveMacenski/slam_toolbox", "Apache-2.0", {"Steve Macenski"}, {}, distro_},
      {"turtlebot3", "TurtleBot3 Packages", "latest", "https://github.com/ROBOTIS-GIT/turtlebot3", "Apache-2.0", {"ROBOTIS"}, {}, distro_},
      {"gazebo_ros_pkgs", "Gazebo ROS Packages", "latest", "https://github.com/ros-simulation/gazebo_ros_pkgs", "Apache-2.0", {"Open Robotics"}, {}, distro_},
      {"rviz2", "RViz2 Visualization", "latest", "https://github.com/ros2/rviz", "BSD-3-Clause", {"Open Robotics"}, {}, distro_},
      {"tf2_ros", "TF2 ROS", "latest", "https://github.com/ros2/geometry2", "BSD-3-Clause", {"Open Robotics"}, {}, distro_},
      {"image_transport", "Image Transport", "latest", "https://github.com/ros-perception/image_common", "BSD-3-Clause", {"Open Robotics"}, {}, distro_},
      {"cv_bridge", "CV Bridge", "latest", "https://github.com/ros-perception/vision_opencv", "BSD-3-Clause", {"Open Robotics"}, {}, distro_},
      {"robot_state_publisher", "Robot State Publisher", "latest", "https://github.com/ros/robot_state_publisher", "BSD-3-Clause", {"Open Robotics"}, {}, distro_},
    };

    // Filter by query if present
    QString searchTerm = QUrlQuery(reply->request().url().query()).queryItemValue("search").toLower();
    if (!searchTerm.isEmpty()) {
      for (const auto& pkg : fallbackPackages) {
        if (pkg.name.toLower().contains(searchTerm) ||
            pkg.description.toLower().contains(searchTerm)) {
          results.append(pkg);
        }
      }
    } else {
      results = fallbackPackages;
    }

    emit searchResultsReady(results);
    reply->deleteLater();
    return;
  }

  // Parse JSON response
  QByteArray data = reply->readAll();
  QJsonDocument doc = QJsonDocument::fromJson(data);

  if (doc.isArray()) {
    QJsonArray packages = doc.array();
    for (const auto& pkgVal : packages) {
      QJsonObject pkgObj = pkgVal.toObject();
      RosPackageInfo info;
      info.name = pkgObj["name"].toString();
      info.description = pkgObj["description"].toString();
      info.version = pkgObj["version"].toString();
      info.repository = pkgObj["repository"].toString();
      info.license = pkgObj["license"].toString();
      info.distro = distro_;
      results.append(info);
    }
  } else if (doc.isObject()) {
    // Some APIs return an object with a "packages" array
    QJsonObject root = doc.object();
    QJsonArray packages = root["packages"].toArray();
    for (const auto& pkgVal : packages) {
      QJsonObject pkgObj = pkgVal.toObject();
      RosPackageInfo info;
      info.name = pkgObj["name"].toString();
      info.description = pkgObj["description"].toString();
      info.version = pkgObj["version"].toString();
      info.repository = pkgObj["repository"].toString();
      info.license = pkgObj["license"].toString();
      info.distro = distro_;
      results.append(info);
    }
  }

  emit searchResultsReady(results);
  reply->deleteLater();
}

void RosPackageIndex::getPackageDetails(const QString& packageName, const QString& distro) {
  // Cancel any pending details request
  if (pendingDetailsReply_) {
    pendingDetailsReply_->abort();
    pendingDetailsReply_->deleteLater();
    pendingDetailsReply_ = nullptr;
  }

  QUrl url(QString("https://index.ros.org/api/v1/packages/%1").arg(packageName));
  QUrlQuery urlQuery;
  urlQuery.addQueryItem("distro", distro);
  url.setQuery(urlQuery);

  QNetworkRequest request(url);
  request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

  pendingDetailsReply_ = networkManager_->get(request);
  connect(pendingDetailsReply_, &QNetworkReply::finished,
          this, &RosPackageIndex::onDetailsReplyFinished);
}

void RosPackageIndex::onDetailsReplyFinished() {
  QNetworkReply* reply = pendingDetailsReply_;
  pendingDetailsReply_ = nullptr;

  if (!reply) return;

  RosPackageInfo info;

  if (reply->error() != QNetworkReply::NoError) {
    emit searchError(reply->errorString());
    reply->deleteLater();
    return;
  }

  QByteArray data = reply->readAll();
  QJsonDocument doc = QJsonDocument::fromJson(data);
  QJsonObject pkgObj = doc.object();

  info.name = pkgObj["name"].toString();
  info.description = pkgObj["description"].toString();
  info.version = pkgObj["version"].toString();
  info.repository = pkgObj["repository"].toString();
  info.license = pkgObj["license"].toString();

  // Parse dependencies if available
  QJsonArray deps = pkgObj["dependencies"].toArray();
  for (const auto& dep : deps) {
    info.dependencies.append(dep.toString());
  }

  // Parse maintainers if available
  QJsonArray maintainers = pkgObj["maintainers"].toArray();
  for (const auto& m : maintainers) {
    info.maintainers.append(m.toString());
  }

  emit packageDetailsReady(info);
  reply->deleteLater();
}

void RosPackageIndex::searchMessageTypes(const QString& query) {
  QList<RosMessageInfo> results;
  QString lowerQuery = query.toLower();

  for (const auto& msg : commonMessages_) {
    if (msg.name.toLower().contains(lowerQuery) ||
        msg.fullName.toLower().contains(lowerQuery) ||
        msg.package.toLower().contains(lowerQuery)) {
      results.append(msg);
    }
  }

  emit messageTypesReady(results);
}

void RosPackageIndex::scanLocalWorkspace(const QString& workspacePath) {
  QList<RosPackageInfo> packages;

  // Look for package.xml files in the workspace src directory
  QString srcPath = workspacePath + "/src";
  QDir srcDir(srcPath);

  if (!srcDir.exists()) {
    // Try the workspace path directly
    srcDir.setPath(workspacePath);
  }

  QDirIterator it(srcDir.absolutePath(), QStringList() << "package.xml",
                  QDir::Files, QDirIterator::Subdirectories);

  while (it.hasNext()) {
    QString packageXmlPath = it.next();
    QFile file(packageXmlPath);

    if (!file.open(QIODevice::ReadOnly)) continue;

    RosPackageInfo info;
    info.isLocal = true;
    info.localPath = QFileInfo(packageXmlPath).absolutePath();

    QXmlStreamReader xml(&file);
    while (!xml.atEnd() && !xml.hasError()) {
      QXmlStreamReader::TokenType token = xml.readNext();

      if (token == QXmlStreamReader::StartElement) {
        if (xml.name() == QString("name")) {
          info.name = xml.readElementText();
        } else if (xml.name() == QString("description")) {
          info.description = xml.readElementText();
        } else if (xml.name() == QString("version")) {
          info.version = xml.readElementText();
        } else if (xml.name() == QString("license")) {
          info.license = xml.readElementText();
        } else if (xml.name() == QString("maintainer")) {
          info.maintainers.append(xml.readElementText());
        } else if (xml.name() == QString("depend") ||
                   xml.name() == QString("build_depend") ||
                   xml.name() == QString("exec_depend")) {
          info.dependencies.append(xml.readElementText());
        }
      }
    }

    file.close();

    if (!info.name.isEmpty()) {
      packages.append(info);
    }
  }

  emit localPackagesReady(packages);
}

QList<RosMessageInfo> RosPackageIndex::commonMessageTypes() const {
  return commonMessages_;
}

}  // namespace ros_weaver
