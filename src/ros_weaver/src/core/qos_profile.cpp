#include "ros_weaver/core/qos_profile.hpp"

namespace ros_weaver {

bool QosProfile::areCompatible(const QosProfile& publisher, const QosProfile& subscriber) {
  return compatibilityIssues(publisher, subscriber).isEmpty();
}

QStringList QosProfile::compatibilityIssues(const QosProfile& publisher, const QosProfile& subscriber) {
  QStringList issues;

  // Reliability: Reliable publisher can work with any subscriber
  // Best effort publisher + Reliable subscriber = incompatible
  if (publisher.reliability == QosReliability::BestEffort &&
      subscriber.reliability == QosReliability::Reliable) {
    issues << "Publisher uses BestEffort but subscriber requires Reliable - messages may be lost";
  }

  // Durability: TransientLocal publisher can work with any subscriber
  // Volatile publisher + TransientLocal subscriber = incompatible
  if (publisher.durability == QosDurability::Volatile &&
      subscriber.durability == QosDurability::TransientLocal) {
    issues << "Publisher uses Volatile but subscriber expects TransientLocal - late joiners won't receive historical messages";
  }

  // Deadline: Publisher deadline should be <= subscriber deadline
  if (publisher.deadline != -1 && subscriber.deadline != -1) {
    if (publisher.deadline > subscriber.deadline) {
      issues << QString("Publisher deadline (%1ns) exceeds subscriber deadline (%2ns)")
                .arg(publisher.deadline).arg(subscriber.deadline);
    }
  }

  // Lifespan: If subscriber has deadline, publisher lifespan should be compatible
  if (publisher.lifespan != -1 && subscriber.deadline != -1) {
    if (publisher.lifespan < subscriber.deadline) {
      issues << "Publisher message lifespan may expire before subscriber deadline";
    }
  }

  // Liveliness compatibility
  if (publisher.liveliness == QosLiveliness::Automatic &&
      subscriber.liveliness != QosLiveliness::Automatic) {
    issues << "Publisher uses Automatic liveliness but subscriber expects manual assertion";
  }

  return issues;
}

QString QosProfile::toCppCode() const {
  QString code;
  code += "rclcpp::QoS qos(rclcpp::KeepLast{" + QString::number(historyDepth) + "});\n";

  if (reliability == QosReliability::BestEffort) {
    code += "qos.best_effort();\n";
  } else {
    code += "qos.reliable();\n";
  }

  if (durability == QosDurability::TransientLocal) {
    code += "qos.transient_local();\n";
  } else {
    code += "qos.durability_volatile();\n";
  }

  if (history == QosHistory::KeepAll) {
    code += "qos.keep_all();\n";
  }

  if (deadline != -1) {
    code += QString("qos.deadline(std::chrono::nanoseconds{%1});\n").arg(deadline);
  }

  if (lifespan != -1) {
    code += QString("qos.lifespan(std::chrono::nanoseconds{%1});\n").arg(lifespan);
  }

  if (liveliness != QosLiveliness::Automatic) {
    if (liveliness == QosLiveliness::ManualByNode) {
      code += "qos.liveliness(rclcpp::LivelinessPolicy::ManualByNode);\n";
    } else {
      code += "qos.liveliness(rclcpp::LivelinessPolicy::ManualByTopic);\n";
    }
    if (leaseDuration != -1) {
      code += QString("qos.liveliness_lease_duration(std::chrono::nanoseconds{%1});\n").arg(leaseDuration);
    }
  }

  return code;
}

QString QosProfile::toPythonCode() const {
  QString code;
  code += "from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy\n\n";
  code += "qos = QoSProfile(\n";
  code += QString("    depth=%1,\n").arg(historyDepth);

  if (history == QosHistory::KeepAll) {
    code += "    history=QoSHistoryPolicy.KEEP_ALL,\n";
  } else {
    code += "    history=QoSHistoryPolicy.KEEP_LAST,\n";
  }

  if (reliability == QosReliability::BestEffort) {
    code += "    reliability=QoSReliabilityPolicy.BEST_EFFORT,\n";
  } else {
    code += "    reliability=QoSReliabilityPolicy.RELIABLE,\n";
  }

  if (durability == QosDurability::TransientLocal) {
    code += "    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,\n";
  } else {
    code += "    durability=QoSDurabilityPolicy.VOLATILE,\n";
  }

  code += ")\n";

  return code;
}

QosProfile QosProfile::defaultProfile() {
  QosProfile profile;
  profile.preset = QosPreset::Default;
  profile.reliability = QosReliability::Reliable;
  profile.durability = QosDurability::Volatile;
  profile.history = QosHistory::KeepLast;
  profile.historyDepth = 10;
  profile.liveliness = QosLiveliness::Automatic;
  return profile;
}

QosProfile QosProfile::sensorDataProfile() {
  QosProfile profile;
  profile.preset = QosPreset::SensorData;
  profile.reliability = QosReliability::BestEffort;
  profile.durability = QosDurability::Volatile;
  profile.history = QosHistory::KeepLast;
  profile.historyDepth = 5;
  profile.liveliness = QosLiveliness::Automatic;
  return profile;
}

QosProfile QosProfile::servicesProfile() {
  QosProfile profile;
  profile.preset = QosPreset::Services;
  profile.reliability = QosReliability::Reliable;
  profile.durability = QosDurability::Volatile;
  profile.history = QosHistory::KeepLast;
  profile.historyDepth = 10;
  profile.liveliness = QosLiveliness::Automatic;
  return profile;
}

QosProfile QosProfile::parametersProfile() {
  QosProfile profile;
  profile.preset = QosPreset::Parameters;
  profile.reliability = QosReliability::Reliable;
  profile.durability = QosDurability::TransientLocal;
  profile.history = QosHistory::KeepLast;
  profile.historyDepth = 1000;
  profile.liveliness = QosLiveliness::Automatic;
  return profile;
}

QosProfile QosProfile::systemDefaultProfile() {
  QosProfile profile;
  profile.preset = QosPreset::SystemDefault;
  profile.reliability = QosReliability::Reliable;
  profile.durability = QosDurability::Volatile;
  profile.history = QosHistory::KeepLast;
  profile.historyDepth = 10;
  profile.liveliness = QosLiveliness::Automatic;
  return profile;
}

QVariantMap QosProfile::toVariantMap() const {
  QVariantMap map;
  map["preset"] = qosPresetToString(preset);
  map["reliability"] = qosReliabilityToString(reliability);
  map["durability"] = qosDurabilityToString(durability);
  map["history"] = qosHistoryToString(history);
  map["historyDepth"] = historyDepth;
  map["deadline"] = deadline;
  map["lifespan"] = lifespan;
  map["leaseDuration"] = leaseDuration;
  map["liveliness"] = qosLivelinessToString(liveliness);
  return map;
}

QosProfile QosProfile::fromVariantMap(const QVariantMap& map) {
  QosProfile profile;
  profile.preset = stringToQosPreset(map.value("preset", "default").toString());
  profile.reliability = stringToQosReliability(map.value("reliability", "reliable").toString());
  profile.durability = stringToQosDurability(map.value("durability", "volatile").toString());
  profile.history = stringToQosHistory(map.value("history", "keep_last").toString());
  profile.historyDepth = map.value("historyDepth", 10).toInt();
  profile.deadline = map.value("deadline", -1).toLongLong();
  profile.lifespan = map.value("lifespan", -1).toLongLong();
  profile.leaseDuration = map.value("leaseDuration", -1).toLongLong();
  profile.liveliness = stringToQosLiveliness(map.value("liveliness", "automatic").toString());
  return profile;
}

bool QosProfile::operator==(const QosProfile& other) const {
  return preset == other.preset &&
         reliability == other.reliability &&
         durability == other.durability &&
         history == other.history &&
         historyDepth == other.historyDepth &&
         deadline == other.deadline &&
         lifespan == other.lifespan &&
         leaseDuration == other.leaseDuration &&
         liveliness == other.liveliness;
}

// String conversion helpers

QString qosReliabilityToString(QosReliability r) {
  switch (r) {
    case QosReliability::BestEffort: return "best_effort";
    case QosReliability::Reliable: return "reliable";
  }
  return "reliable";
}

QString qosDurabilityToString(QosDurability d) {
  switch (d) {
    case QosDurability::Volatile: return "volatile";
    case QosDurability::TransientLocal: return "transient_local";
  }
  return "volatile";
}

QString qosHistoryToString(QosHistory h) {
  switch (h) {
    case QosHistory::KeepLast: return "keep_last";
    case QosHistory::KeepAll: return "keep_all";
  }
  return "keep_last";
}

QString qosLivelinessToString(QosLiveliness l) {
  switch (l) {
    case QosLiveliness::Automatic: return "automatic";
    case QosLiveliness::ManualByNode: return "manual_by_node";
    case QosLiveliness::ManualByTopic: return "manual_by_topic";
  }
  return "automatic";
}

QString qosPresetToString(QosPreset p) {
  switch (p) {
    case QosPreset::Default: return "default";
    case QosPreset::SensorData: return "sensor_data";
    case QosPreset::Services: return "services";
    case QosPreset::Parameters: return "parameters";
    case QosPreset::SystemDefault: return "system_default";
    case QosPreset::Custom: return "custom";
  }
  return "default";
}

QosReliability stringToQosReliability(const QString& s) {
  if (s == "best_effort") return QosReliability::BestEffort;
  return QosReliability::Reliable;
}

QosDurability stringToQosDurability(const QString& s) {
  if (s == "transient_local") return QosDurability::TransientLocal;
  return QosDurability::Volatile;
}

QosHistory stringToQosHistory(const QString& s) {
  if (s == "keep_all") return QosHistory::KeepAll;
  return QosHistory::KeepLast;
}

QosLiveliness stringToQosLiveliness(const QString& s) {
  if (s == "manual_by_node") return QosLiveliness::ManualByNode;
  if (s == "manual_by_topic") return QosLiveliness::ManualByTopic;
  return QosLiveliness::Automatic;
}

QosPreset stringToQosPreset(const QString& s) {
  if (s == "sensor_data") return QosPreset::SensorData;
  if (s == "services") return QosPreset::Services;
  if (s == "parameters") return QosPreset::Parameters;
  if (s == "system_default") return QosPreset::SystemDefault;
  if (s == "custom") return QosPreset::Custom;
  return QosPreset::Default;
}

}  // namespace ros_weaver
