#ifndef ROS_WEAVER_CORE_GENERIC_MESSAGE_HANDLER_HPP
#define ROS_WEAVER_CORE_GENERIC_MESSAGE_HANDLER_HPP

#include <QObject>
#include <QString>
#include <QJsonObject>
#include <QJsonArray>
#include <QMap>
#include <QList>
#include <QMutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/serialized_message.hpp>

namespace ros_weaver {

/**
 * @brief Handles dynamic message subscription and publishing for any ROS2 topic
 *
 * This class provides functionality to:
 * - Subscribe to any topic dynamically using generic subscriptions
 * - Publish messages to any topic
 * - Deserialize ROS2 messages to JSON for inspection
 * - Serialize JSON back to ROS2 messages for publishing
 */
class GenericMessageHandler : public QObject {
  Q_OBJECT

public:
  explicit GenericMessageHandler(QObject* parent = nullptr);
  ~GenericMessageHandler() override;

  /**
   * @brief Initialize with a ROS node
   */
  void setRosNode(rclcpp::Node::SharedPtr node);

  /**
   * @brief Subscribe to a topic
   * @param topicName The name of the topic to subscribe to
   * @param messageType The message type (e.g., "std_msgs/msg/String")
   * @return true if subscription was successful
   */
  bool subscribeToTopic(const QString& topicName, const QString& messageType);

  /**
   * @brief Unsubscribe from a topic
   * @param topicName The name of the topic to unsubscribe from
   */
  void unsubscribeFromTopic(const QString& topicName);

  /**
   * @brief Unsubscribe from all topics
   */
  void unsubscribeAll();

  /**
   * @brief Check if currently subscribed to a topic
   */
  bool isSubscribed(const QString& topicName) const;

  /**
   * @brief Publish a message to a topic
   * @param topicName The topic name
   * @param messageType The message type
   * @param messageData JSON representation of the message
   * @return true if publish was successful
   */
  bool publishMessage(const QString& topicName, const QString& messageType,
                      const QJsonObject& messageData);

  /**
   * @brief Get the message schema/template for a message type
   * @param messageType The message type
   * @return JSON object representing the schema with default values
   */
  QJsonObject getMessageSchema(const QString& messageType);

  /**
   * @brief Get list of currently subscribed topics
   */
  QStringList subscribedTopics() const;

  /**
   * @brief Get available topics from the ROS graph
   */
  QMap<QString, QString> getAvailableTopics() const;

signals:
  /**
   * @brief Emitted when a message is received on a subscribed topic
   * @param topicName The topic the message was received on
   * @param message The message content as JSON
   * @param timestamp Timestamp when message was received (ms since epoch)
   */
  void messageReceived(const QString& topicName, const QJsonObject& message,
                       qint64 timestamp);

  /**
   * @brief Emitted when a subscription error occurs
   */
  void subscriptionError(const QString& topicName, const QString& error);

  /**
   * @brief Emitted when a publish error occurs
   */
  void publishError(const QString& topicName, const QString& error);

  /**
   * @brief Emitted when a publish succeeds
   */
  void publishSuccess(const QString& topicName);

private:
  /**
   * @brief Deserialize a serialized message to JSON
   */
  QJsonObject deserializeMessage(const std::shared_ptr<rclcpp::SerializedMessage>& msg,
                                  const QString& messageType);

  /**
   * @brief Convert primitive ROS types to JSON values
   */
  QJsonValue primitiveToJson(const uint8_t* data, const QString& typeName,
                              size_t& offset);

  /**
   * @brief Generate a template JSON object for a message type
   */
  QJsonObject generateTemplateFromType(const QString& messageType);

  /**
   * @brief Get or create a publisher for a topic
   */
  rclcpp::GenericPublisher::SharedPtr getOrCreatePublisher(
      const QString& topicName, const QString& messageType);

  rclcpp::Node::SharedPtr node_;
  QMap<QString, rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  QMap<QString, QString> subscriptionTypes_;  // topicName -> messageType
  QMap<QString, rclcpp::GenericPublisher::SharedPtr> publishers_;
  mutable QMutex mutex_;
};

}  // namespace ros_weaver

#endif  // ROS_WEAVER_CORE_GENERIC_MESSAGE_HANDLER_HPP
