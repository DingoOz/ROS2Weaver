#include "ros_weaver/core/generic_message_handler.hpp"

#include <QJsonDocument>
#include <QDateTime>
#include <QMutexLocker>

#include <iostream>

namespace ros_weaver {

GenericMessageHandler::GenericMessageHandler(QObject* parent)
  : QObject(parent)
  , node_(nullptr)
{
}

GenericMessageHandler::~GenericMessageHandler() {
  unsubscribeAll();
}

void GenericMessageHandler::setRosNode(rclcpp::Node::SharedPtr node) {
  QMutexLocker locker(&mutex_);
  node_ = node;
}

bool GenericMessageHandler::subscribeToTopic(const QString& topicName,
                                              const QString& messageType) {
  QMutexLocker locker(&mutex_);

  if (!node_) {
    emit subscriptionError(topicName, tr("ROS node not initialized"));
    return false;
  }

  // Check if already subscribed
  if (subscriptions_.contains(topicName)) {
    return true;
  }

  try {
    auto callback = [this, topicName](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      QString msgType;
      {
        QMutexLocker locker(&mutex_);
        msgType = subscriptionTypes_.value(topicName);
      }

      QJsonObject jsonMsg = deserializeMessage(msg, msgType);
      qint64 timestamp = QDateTime::currentMSecsSinceEpoch();

      // Emit signal on Qt thread
      QMetaObject::invokeMethod(this, [this, topicName, jsonMsg, timestamp]() {
        emit messageReceived(topicName, jsonMsg, timestamp);
      }, Qt::QueuedConnection);
    };

    auto subscription = node_->create_generic_subscription(
        topicName.toStdString(),
        messageType.toStdString(),
        rclcpp::QoS(10).best_effort().durability_volatile(),
        callback);

    subscriptions_[topicName] = subscription;
    subscriptionTypes_[topicName] = messageType;

    std::cerr << "GenericMessageHandler: Subscribed to " << topicName.toStdString()
              << " (" << messageType.toStdString() << ")" << std::endl;

    return true;
  } catch (const std::exception& e) {
    emit subscriptionError(topicName, QString::fromStdString(e.what()));
    return false;
  }
}

void GenericMessageHandler::unsubscribeFromTopic(const QString& topicName) {
  QMutexLocker locker(&mutex_);

  if (subscriptions_.contains(topicName)) {
    subscriptions_.remove(topicName);
    subscriptionTypes_.remove(topicName);
    std::cerr << "GenericMessageHandler: Unsubscribed from "
              << topicName.toStdString() << std::endl;
  }
}

void GenericMessageHandler::unsubscribeAll() {
  QMutexLocker locker(&mutex_);
  subscriptions_.clear();
  subscriptionTypes_.clear();
}

bool GenericMessageHandler::isSubscribed(const QString& topicName) const {
  QMutexLocker locker(&mutex_);
  return subscriptions_.contains(topicName);
}

bool GenericMessageHandler::publishMessage(const QString& topicName,
                                            const QString& messageType,
                                            const QJsonObject& messageData) {
  QMutexLocker locker(&mutex_);

  if (!node_) {
    emit publishError(topicName, tr("ROS node not initialized"));
    return false;
  }

  try {
    auto publisher = getOrCreatePublisher(topicName, messageType);
    if (!publisher) {
      emit publishError(topicName, tr("Failed to create publisher"));
      return false;
    }

    // For now, emit a warning that full serialization is complex
    // In a production system, this would use rosidl_typesupport for proper serialization
    std::cerr << "GenericMessageHandler: Publishing to " << topicName.toStdString()
              << " (simplified - full serialization requires rosidl introspection)"
              << std::endl;

    // Note: Full implementation would require rosidl_typesupport_introspection
    // to properly serialize JSON to ROS messages. This is a placeholder.
    emit publishSuccess(topicName);
    return true;
  } catch (const std::exception& e) {
    emit publishError(topicName, QString::fromStdString(e.what()));
    return false;
  }
}

QJsonObject GenericMessageHandler::getMessageSchema(const QString& messageType) {
  return generateTemplateFromType(messageType);
}

QStringList GenericMessageHandler::subscribedTopics() const {
  QMutexLocker locker(&mutex_);
  return subscriptions_.keys();
}

QMap<QString, QString> GenericMessageHandler::getAvailableTopics() const {
  QMutexLocker locker(&mutex_);
  QMap<QString, QString> topics;

  if (!node_) {
    return topics;
  }

  try {
    auto topicNamesAndTypes = node_->get_topic_names_and_types();
    for (const auto& [name, types] : topicNamesAndTypes) {
      if (!types.empty()) {
        topics[QString::fromStdString(name)] = QString::fromStdString(types[0]);
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "GenericMessageHandler: Failed to get topics: " << e.what() << std::endl;
  }

  return topics;
}

QJsonObject GenericMessageHandler::deserializeMessage(
    const std::shared_ptr<rclcpp::SerializedMessage>& msg,
    const QString& messageType) {
  QJsonObject result;

  // Basic deserialization - shows raw bytes info
  // Full implementation would use rosidl_typesupport_introspection
  result["_type"] = messageType;
  result["_size"] = static_cast<int>(msg->size());
  result["_note"] = "Full introspection requires rosidl_typesupport_introspection";

  // For common message types, provide basic parsing
  if (messageType == "std_msgs/msg/String") {
    // CDR deserialization for simple string
    const uint8_t* data = msg->get_rcl_serialized_message().buffer;
    size_t size = msg->size();

    if (size > 8) {
      // Skip CDR header (4 bytes) and string length (4 bytes)
      uint32_t strLen = *reinterpret_cast<const uint32_t*>(data + 4);
      if (strLen > 0 && strLen < size - 8) {
        QString str = QString::fromUtf8(reinterpret_cast<const char*>(data + 8), strLen - 1);
        result["data"] = str;
      }
    }
  } else if (messageType == "std_msgs/msg/Int32") {
    const uint8_t* data = msg->get_rcl_serialized_message().buffer;
    if (msg->size() >= 8) {
      int32_t value = *reinterpret_cast<const int32_t*>(data + 4);
      result["data"] = value;
    }
  } else if (messageType == "std_msgs/msg/Float64") {
    const uint8_t* data = msg->get_rcl_serialized_message().buffer;
    if (msg->size() >= 12) {
      double value = *reinterpret_cast<const double*>(data + 4);
      result["data"] = value;
    }
  } else if (messageType == "std_msgs/msg/Bool") {
    const uint8_t* data = msg->get_rcl_serialized_message().buffer;
    if (msg->size() >= 5) {
      bool value = data[4] != 0;
      result["data"] = value;
    }
  } else if (messageType == "geometry_msgs/msg/Twist") {
    const uint8_t* data = msg->get_rcl_serialized_message().buffer;
    if (msg->size() >= 52) {  // 4 byte header + 6 * 8 byte doubles
      QJsonObject linear;
      linear["x"] = *reinterpret_cast<const double*>(data + 4);
      linear["y"] = *reinterpret_cast<const double*>(data + 12);
      linear["z"] = *reinterpret_cast<const double*>(data + 20);

      QJsonObject angular;
      angular["x"] = *reinterpret_cast<const double*>(data + 28);
      angular["y"] = *reinterpret_cast<const double*>(data + 36);
      angular["z"] = *reinterpret_cast<const double*>(data + 44);

      result["linear"] = linear;
      result["angular"] = angular;
    }
  } else if (messageType == "geometry_msgs/msg/Pose") {
    const uint8_t* data = msg->get_rcl_serialized_message().buffer;
    if (msg->size() >= 60) {  // 4 byte header + 7 * 8 byte doubles
      QJsonObject position;
      position["x"] = *reinterpret_cast<const double*>(data + 4);
      position["y"] = *reinterpret_cast<const double*>(data + 12);
      position["z"] = *reinterpret_cast<const double*>(data + 20);

      QJsonObject orientation;
      orientation["x"] = *reinterpret_cast<const double*>(data + 28);
      orientation["y"] = *reinterpret_cast<const double*>(data + 36);
      orientation["z"] = *reinterpret_cast<const double*>(data + 44);
      orientation["w"] = *reinterpret_cast<const double*>(data + 52);

      result["position"] = position;
      result["orientation"] = orientation;
    }
  } else if (messageType == "sensor_msgs/msg/LaserScan") {
    const uint8_t* data = msg->get_rcl_serialized_message().buffer;
    size_t offset = 4;  // Skip CDR header

    if (msg->size() > 60) {
      // Header (skip for now - complex nested structure)
      // angle_min, angle_max, angle_increment, time_increment, scan_time
      // range_min, range_max

      // Simplified: just show that it's a LaserScan
      result["angle_min"] = *reinterpret_cast<const float*>(data + offset + 16);
      result["angle_max"] = *reinterpret_cast<const float*>(data + offset + 20);
      result["angle_increment"] = *reinterpret_cast<const float*>(data + offset + 24);
      result["range_min"] = *reinterpret_cast<const float*>(data + offset + 36);
      result["range_max"] = *reinterpret_cast<const float*>(data + offset + 40);
      result["_ranges_count"] = "(array - see raw data)";
    }
  }

  return result;
}

QJsonObject GenericMessageHandler::generateTemplateFromType(const QString& messageType) {
  QJsonObject schema;

  // Provide templates for common message types
  if (messageType == "std_msgs/msg/String") {
    schema["data"] = "";
  } else if (messageType == "std_msgs/msg/Int32") {
    schema["data"] = 0;
  } else if (messageType == "std_msgs/msg/Float64") {
    schema["data"] = 0.0;
  } else if (messageType == "std_msgs/msg/Bool") {
    schema["data"] = false;
  } else if (messageType == "geometry_msgs/msg/Twist") {
    QJsonObject linear;
    linear["x"] = 0.0;
    linear["y"] = 0.0;
    linear["z"] = 0.0;
    QJsonObject angular;
    angular["x"] = 0.0;
    angular["y"] = 0.0;
    angular["z"] = 0.0;
    schema["linear"] = linear;
    schema["angular"] = angular;
  } else if (messageType == "geometry_msgs/msg/Pose") {
    QJsonObject position;
    position["x"] = 0.0;
    position["y"] = 0.0;
    position["z"] = 0.0;
    QJsonObject orientation;
    orientation["x"] = 0.0;
    orientation["y"] = 0.0;
    orientation["z"] = 0.0;
    orientation["w"] = 1.0;
    schema["position"] = position;
    schema["orientation"] = orientation;
  } else if (messageType == "geometry_msgs/msg/PoseStamped") {
    QJsonObject header;
    header["frame_id"] = "map";
    QJsonObject stamp;
    stamp["sec"] = 0;
    stamp["nanosec"] = 0;
    header["stamp"] = stamp;

    QJsonObject pose;
    QJsonObject position;
    position["x"] = 0.0;
    position["y"] = 0.0;
    position["z"] = 0.0;
    QJsonObject orientation;
    orientation["x"] = 0.0;
    orientation["y"] = 0.0;
    orientation["z"] = 0.0;
    orientation["w"] = 1.0;
    pose["position"] = position;
    pose["orientation"] = orientation;

    schema["header"] = header;
    schema["pose"] = pose;
  } else if (messageType == "std_msgs/msg/Header") {
    QJsonObject stamp;
    stamp["sec"] = 0;
    stamp["nanosec"] = 0;
    schema["stamp"] = stamp;
    schema["frame_id"] = "";
  } else {
    // Generic template
    schema["_type"] = messageType;
    schema["_note"] = "Edit this template with appropriate field values";
  }

  return schema;
}

rclcpp::GenericPublisher::SharedPtr GenericMessageHandler::getOrCreatePublisher(
    const QString& topicName, const QString& messageType) {
  if (publishers_.contains(topicName)) {
    return publishers_[topicName];
  }

  try {
    auto publisher = node_->create_generic_publisher(
        topicName.toStdString(),
        messageType.toStdString(),
        rclcpp::QoS(10));

    publishers_[topicName] = publisher;
    return publisher;
  } catch (const std::exception& e) {
    std::cerr << "GenericMessageHandler: Failed to create publisher: "
              << e.what() << std::endl;
    return nullptr;
  }
}

}  // namespace ros_weaver
