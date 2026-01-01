#ifndef ROS_WEAVER_CORE_ROS_DOCS_PROVIDER_HPP
#define ROS_WEAVER_CORE_ROS_DOCS_PROVIDER_HPP

#include <QObject>
#include <QString>
#include <QMap>
#include <QProcess>
#include <QMutex>
#include <QDateTime>

namespace ros_weaver {

// Information about a ROS2 message/service/action field
struct FieldInfo {
  QString name;
  QString type;
  QString defaultValue;
  QString comment;
  int depth = 0;  // Nesting level for sub-messages
};

// Complete documentation for a ROS2 interface type
struct InterfaceDoc {
  enum class Type { Message, Service, Action, Unknown };

  QString fullName;          // e.g., "std_msgs/msg/String"
  QString packageName;       // e.g., "std_msgs"
  QString typeName;          // e.g., "String"
  Type interfaceType = Type::Unknown;
  QString rawDefinition;     // Raw output from ros2 interface show
  QList<FieldInfo> fields;   // Parsed field information
  QString description;       // Description if available
  QDateTime fetchedAt;       // When this was cached
  bool isValid = false;
};

// Information about a ROS2 package
struct PackageDoc {
  QString name;
  QString version;
  QString description;
  QString maintainer;
  QString license;
  QStringList dependencies;
  QString buildType;
  QString xmlPath;           // Path to package.xml
  QString readmePath;        // Path to README.md if exists
  QDateTime fetchedAt;
  bool isValid = false;
};

// ROS2 Documentation Provider - Singleton that caches interface documentation
class RosDocsProvider : public QObject {
  Q_OBJECT

public:
  static RosDocsProvider& instance();

  // Prevent copying
  RosDocsProvider(const RosDocsProvider&) = delete;
  RosDocsProvider& operator=(const RosDocsProvider&) = delete;

  // Get interface documentation (messages, services, actions)
  // Returns cached version if available and not expired
  InterfaceDoc getInterfaceDoc(const QString& fullName);

  // Get package documentation
  PackageDoc getPackageDoc(const QString& packageName);

  // Check if documentation is cached
  bool hasInterfaceDoc(const QString& fullName) const;
  bool hasPackageDoc(const QString& packageName) const;

  // Clear all caches
  void clearCache();

  // Cache configuration
  void setCacheExpirationMinutes(int minutes);
  int cacheExpirationMinutes() const { return cacheExpirationMinutes_; }

  // Force refresh of specific documentation
  InterfaceDoc refreshInterfaceDoc(const QString& fullName);
  PackageDoc refreshPackageDoc(const QString& packageName);

  // Get list of available interfaces of a given type
  QStringList getAvailableMessages();
  QStringList getAvailableServices();
  QStringList getAvailableActions();

  // Get list of available packages
  QStringList getAvailablePackages();

  // Format documentation as rich text for display
  QString formatInterfaceAsHtml(const InterfaceDoc& doc) const;
  QString formatPackageAsHtml(const PackageDoc& doc) const;

signals:
  // Emitted when documentation is fetched (useful for async operations)
  void interfaceDocFetched(const QString& fullName, const InterfaceDoc& doc);
  void packageDocFetched(const QString& packageName, const PackageDoc& doc);
  void errorOccurred(const QString& message);

private:
  RosDocsProvider();
  ~RosDocsProvider() override = default;

  // Internal fetch methods
  InterfaceDoc fetchInterfaceDoc(const QString& fullName);
  PackageDoc fetchPackageDoc(const QString& packageName);

  // Parse ros2 interface show output
  QList<FieldInfo> parseInterfaceDefinition(const QString& definition);

  // Parse package.xml
  PackageDoc parsePackageXml(const QString& xmlPath);

  // Execute ros2 command and return output
  QString executeRos2Command(const QStringList& args, int timeoutMs = 5000);

  // Check if cached entry is expired
  bool isCacheExpired(const QDateTime& fetchedAt) const;

  // Cache storage
  QMap<QString, InterfaceDoc> interfaceCache_;
  QMap<QString, PackageDoc> packageCache_;

  // Cache for available interfaces/packages (refreshed periodically)
  QStringList availableMessages_;
  QStringList availableServices_;
  QStringList availableActions_;
  QStringList availablePackages_;
  QDateTime lastInterfaceListRefresh_;
  QDateTime lastPackageListRefresh_;

  // Configuration
  int cacheExpirationMinutes_ = 60;  // Default 1 hour cache

  // Thread safety
  mutable QMutex mutex_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_ROS_DOCS_PROVIDER_HPP
