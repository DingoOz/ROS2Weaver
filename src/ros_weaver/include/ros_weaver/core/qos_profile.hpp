#ifndef ROS_WEAVER_CORE_QOS_PROFILE_HPP
#define ROS_WEAVER_CORE_QOS_PROFILE_HPP

#include <QString>
#include <QVariant>
#include <QMap>

namespace ros_weaver {

/**
 * @brief QoS Reliability policy
 */
enum class QosReliability {
  BestEffort,   // Messages may be lost
  Reliable      // Messages guaranteed to be delivered
};

/**
 * @brief QoS Durability policy
 */
enum class QosDurability {
  Volatile,         // Messages not stored for late joiners
  TransientLocal    // Messages stored for late-joining subscribers
};

/**
 * @brief QoS History policy
 */
enum class QosHistory {
  KeepLast,    // Only keep last N messages
  KeepAll      // Keep all messages
};

/**
 * @brief QoS Liveliness policy
 */
enum class QosLiveliness {
  Automatic,        // System handles liveliness
  ManualByNode,     // Node must assert liveliness
  ManualByTopic     // Topic must be asserted per topic
};

/**
 * @brief QoS preset types
 */
enum class QosPreset {
  Default,        // ROS2 default QoS
  SensorData,     // Best effort, keep last
  Services,       // Reliable, volatile
  Parameters,     // Reliable, transient local
  SystemDefault,  // System default
  Custom          // User-defined
};

/**
 * @brief Complete QoS profile configuration
 */
struct QosProfile {
  QosPreset preset = QosPreset::Default;

  // Core policies
  QosReliability reliability = QosReliability::Reliable;
  QosDurability durability = QosDurability::Volatile;
  QosHistory history = QosHistory::KeepLast;
  int historyDepth = 10;

  // Time-based policies (in nanoseconds, -1 = infinite)
  qint64 deadline = -1;       // Expected message arrival interval
  qint64 lifespan = -1;       // Message validity duration
  qint64 leaseDuration = -1;  // Liveliness lease duration

  // Liveliness
  QosLiveliness liveliness = QosLiveliness::Automatic;

  // Return profile name
  QString name() const {
    switch (preset) {
      case QosPreset::Default: return "Default";
      case QosPreset::SensorData: return "Sensor Data";
      case QosPreset::Services: return "Services";
      case QosPreset::Parameters: return "Parameters";
      case QosPreset::SystemDefault: return "System Default";
      case QosPreset::Custom: return "Custom";
    }
    return "Unknown";
  }

  // Check if two QoS profiles are compatible
  static bool areCompatible(const QosProfile& publisher, const QosProfile& subscriber);

  // Get compatibility issues as a list of warnings
  static QStringList compatibilityIssues(const QosProfile& publisher, const QosProfile& subscriber);

  // Generate C++ code snippet
  QString toCppCode() const;

  // Generate Python code snippet
  QString toPythonCode() const;

  // Preset factory methods
  static QosProfile defaultProfile();
  static QosProfile sensorDataProfile();
  static QosProfile servicesProfile();
  static QosProfile parametersProfile();
  static QosProfile systemDefaultProfile();

  // Serialization
  QVariantMap toVariantMap() const;
  static QosProfile fromVariantMap(const QVariantMap& map);

  bool operator==(const QosProfile& other) const;
  bool operator!=(const QosProfile& other) const { return !(*this == other); }
};

// Helper functions for string conversion
QString qosReliabilityToString(QosReliability r);
QString qosDurabilityToString(QosDurability d);
QString qosHistoryToString(QosHistory h);
QString qosLivelinessToString(QosLiveliness l);
QString qosPresetToString(QosPreset p);

QosReliability stringToQosReliability(const QString& s);
QosDurability stringToQosDurability(const QString& s);
QosHistory stringToQosHistory(const QString& s);
QosLiveliness stringToQosLiveliness(const QString& s);
QosPreset stringToQosPreset(const QString& s);

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_QOS_PROFILE_HPP
