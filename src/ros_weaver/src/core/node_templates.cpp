#include "ros_weaver/core/node_templates.hpp"

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

namespace ros_weaver {

// =============================================================================
// NodeTemplate Factory Methods
// =============================================================================

NodeTemplate NodeTemplate::createBasicPublisher() {
  NodeTemplate tmpl;
  tmpl.id = QUuid::createUuid();
  tmpl.name = "basic_publisher";
  tmpl.displayName = "Basic Publisher";
  tmpl.description = "A simple ROS2 publisher node that publishes messages at a fixed rate";
  tmpl.category = TemplateCategory::Basic;
  tmpl.iconName = "publish";

  tmpl.outputPins = {
    {"output", "std_msgs/msg/String", true, "Published message output"}
  };

  tmpl.parameters = {
    {"topic_name", "/output_topic", "string", "Topic to publish on", true},
    {"publish_rate", 10.0, "double", "Publishing rate in Hz", false},
    {"queue_size", 10, "int", "Publisher queue size", false}
  };

  tmpl.baseClass = "rclcpp::Node";
  tmpl.requiredPackages = QStringList{"rclcpp", "std_msgs"};
  tmpl.author = "ROS Weaver";
  tmpl.version = "1.0";
  tmpl.tags = QStringList{"basic", "publisher", "output"};

  return tmpl;
}

NodeTemplate NodeTemplate::createBasicSubscriber() {
  NodeTemplate tmpl;
  tmpl.id = QUuid::createUuid();
  tmpl.name = "basic_subscriber";
  tmpl.displayName = "Basic Subscriber";
  tmpl.description = "A simple ROS2 subscriber node that receives and processes messages";
  tmpl.category = TemplateCategory::Basic;
  tmpl.iconName = "subscribe";

  tmpl.inputPins = {
    {"input", "std_msgs/msg/String", false, "Incoming message input"}
  };

  tmpl.parameters = {
    {"topic_name", "/input_topic", "string", "Topic to subscribe to", true},
    {"queue_size", 10, "int", "Subscription queue size", false}
  };

  tmpl.baseClass = "rclcpp::Node";
  tmpl.requiredPackages = QStringList{"rclcpp", "std_msgs"};
  tmpl.author = "ROS Weaver";
  tmpl.version = "1.0";
  tmpl.tags = QStringList{"basic", "subscriber", "input"};

  return tmpl;
}

NodeTemplate NodeTemplate::createSensorFusion() {
  NodeTemplate tmpl;
  tmpl.id = QUuid::createUuid();
  tmpl.name = "sensor_fusion";
  tmpl.displayName = "Sensor Fusion Node";
  tmpl.description = "Fuses multiple sensor inputs using configurable algorithms (EKF, UKF, complementary)";
  tmpl.category = TemplateCategory::SensorFusion;
  tmpl.iconName = "merge";

  tmpl.inputPins = {
    {"imu", "sensor_msgs/msg/Imu", false, "IMU sensor data"},
    {"gps", "sensor_msgs/msg/NavSatFix", false, "GPS position data"},
    {"odom", "nav_msgs/msg/Odometry", false, "Odometry data"}
  };

  tmpl.outputPins = {
    {"fused_odom", "nav_msgs/msg/Odometry", true, "Fused odometry output"},
    {"pose", "geometry_msgs/msg/PoseStamped", true, "Estimated pose"}
  };

  tmpl.parameters = {
    {"fusion_algorithm", "ekf", "string", "Fusion algorithm: ekf, ukf, complementary", true},
    {"process_noise", 0.01, "double", "Process noise covariance", false},
    {"measurement_noise", 0.1, "double", "Measurement noise covariance", false},
    {"publish_rate", 50.0, "double", "Output rate in Hz", false}
  };

  tmpl.baseClass = "rclcpp::Node";
  tmpl.requiredPackages = QStringList{"rclcpp", "sensor_msgs", "nav_msgs", "geometry_msgs", "tf2_ros"};
  tmpl.author = "ROS Weaver";
  tmpl.version = "1.0";
  tmpl.tags = QStringList{"sensor", "fusion", "localization", "ekf"};

  return tmpl;
}

NodeTemplate NodeTemplate::createStateMachine() {
  NodeTemplate tmpl;
  tmpl.id = QUuid::createUuid();
  tmpl.name = "state_machine";
  tmpl.displayName = "State Machine Node";
  tmpl.description = "A finite state machine node with configurable states and transitions";
  tmpl.category = TemplateCategory::StateMachine;
  tmpl.iconName = "state";

  tmpl.inputPins = {
    {"events", "std_msgs/msg/String", false, "Incoming state transition events"},
    {"trigger", "std_msgs/msg/Bool", false, "External trigger input"}
  };

  tmpl.outputPins = {
    {"current_state", "std_msgs/msg/String", true, "Current state name"},
    {"state_changed", "std_msgs/msg/Bool", true, "State change notification"}
  };

  tmpl.parameters = {
    {"initial_state", "idle", "string", "Initial state name", true},
    {"states", QVariant(QStringList{"idle", "running", "paused", "stopped"}), "array", "List of valid states", true},
    {"auto_transitions", false, "bool", "Enable automatic timed transitions", false}
  };

  tmpl.baseClass = "rclcpp::Node";
  tmpl.requiredPackages = QStringList{"rclcpp", "std_msgs"};
  tmpl.author = "ROS Weaver";
  tmpl.version = "1.0";
  tmpl.tags = QStringList{"state", "machine", "fsm", "control"};

  return tmpl;
}

NodeTemplate NodeTemplate::createBehaviorTreeExecutor() {
  NodeTemplate tmpl;
  tmpl.id = QUuid::createUuid();
  tmpl.name = "behavior_tree_executor";
  tmpl.displayName = "Behavior Tree Executor";
  tmpl.description = "Executes behavior trees for robot task planning and execution";
  tmpl.category = TemplateCategory::BehaviorTree;
  tmpl.iconName = "tree";

  tmpl.inputPins = {
    {"goal", "std_msgs/msg/String", false, "Goal/mission input"},
    {"feedback", "std_msgs/msg/String", false, "External feedback"}
  };

  tmpl.outputPins = {
    {"status", "std_msgs/msg/String", true, "Execution status"},
    {"result", "std_msgs/msg/Bool", true, "Execution result"}
  };

  tmpl.parameters = {
    {"tree_file", "", "string", "Path to behavior tree XML file", true},
    {"tick_rate", 10.0, "double", "Tree tick rate in Hz", false},
    {"plugins_path", "", "string", "Path to BT plugins", false}
  };

  tmpl.baseClass = "rclcpp::Node";
  tmpl.requiredPackages = QStringList{"rclcpp", "std_msgs", "behaviortree_cpp_v3"};
  tmpl.author = "ROS Weaver";
  tmpl.version = "1.0";
  tmpl.tags = QStringList{"behavior", "tree", "bt", "planning"};

  return tmpl;
}

NodeTemplate NodeTemplate::createRosBridge() {
  NodeTemplate tmpl;
  tmpl.id = QUuid::createUuid();
  tmpl.name = "ros_bridge";
  tmpl.displayName = "ROS1-ROS2 Bridge";
  tmpl.description = "Bridge node for communication between ROS1 and ROS2 topics";
  tmpl.category = TemplateCategory::Bridge;
  tmpl.iconName = "bridge";

  tmpl.inputPins = {
    {"ros1_input", "std_msgs/msg/String", false, "ROS1 side input"},
    {"ros2_input", "std_msgs/msg/String", false, "ROS2 side input"}
  };

  tmpl.outputPins = {
    {"ros1_output", "std_msgs/msg/String", true, "ROS1 side output"},
    {"ros2_output", "std_msgs/msg/String", true, "ROS2 side output"}
  };

  tmpl.parameters = {
    {"ros1_topic", "/ros1/topic", "string", "ROS1 topic name", true},
    {"ros2_topic", "/ros2/topic", "string", "ROS2 topic name", true},
    {"bidirectional", true, "bool", "Enable bidirectional bridging", false}
  };

  tmpl.baseClass = "rclcpp::Node";
  tmpl.requiredPackages = QStringList{"rclcpp", "ros1_bridge", "std_msgs"};
  tmpl.author = "ROS Weaver";
  tmpl.version = "1.0";
  tmpl.tags = QStringList{"bridge", "ros1", "ros2", "interop"};

  return tmpl;
}

NodeTemplate NodeTemplate::createLifecycleNode() {
  NodeTemplate tmpl;
  tmpl.id = QUuid::createUuid();
  tmpl.name = "lifecycle_node";
  tmpl.displayName = "Lifecycle Node";
  tmpl.description = "A managed lifecycle node with configurable state transitions";
  tmpl.category = TemplateCategory::Lifecycle;
  tmpl.iconName = "lifecycle";

  tmpl.inputPins = {
    {"input", "std_msgs/msg/String", false, "Data input (active when configured)"}
  };

  tmpl.outputPins = {
    {"output", "std_msgs/msg/String", true, "Data output (active when active)"},
    {"lifecycle_state", "lifecycle_msgs/msg/State", true, "Current lifecycle state"}
  };

  tmpl.parameters = {
    {"auto_activate", false, "bool", "Automatically transition to active", false},
    {"heartbeat_rate", 1.0, "double", "Heartbeat publishing rate", false}
  };

  tmpl.baseClass = "rclcpp_lifecycle::LifecycleNode";
  tmpl.requiredPackages = QStringList{"rclcpp", "rclcpp_lifecycle", "lifecycle_msgs", "std_msgs"};
  tmpl.author = "ROS Weaver";
  tmpl.version = "1.0";
  tmpl.tags = QStringList{"lifecycle", "managed", "node"};

  return tmpl;
}

NodeTemplate NodeTemplate::createComponentNode() {
  NodeTemplate tmpl;
  tmpl.id = QUuid::createUuid();
  tmpl.name = "component_node";
  tmpl.displayName = "Component Node";
  tmpl.description = "A composable node that can be loaded into a component container";
  tmpl.category = TemplateCategory::Component;
  tmpl.iconName = "component";

  tmpl.inputPins = {
    {"input", "std_msgs/msg/String", false, "Component input"}
  };

  tmpl.outputPins = {
    {"output", "std_msgs/msg/String", true, "Component output"}
  };

  tmpl.parameters = {
    {"use_intra_process", true, "bool", "Enable intra-process communication", false}
  };

  tmpl.baseClass = "rclcpp::Node";
  tmpl.requiredPackages = QStringList{"rclcpp", "rclcpp_components", "std_msgs"};
  tmpl.author = "ROS Weaver";
  tmpl.version = "1.0";
  tmpl.tags = QStringList{"component", "composable", "container"};

  return tmpl;
}

NodeTemplate NodeTemplate::createNav2Controller() {
  NodeTemplate tmpl;
  tmpl.id = QUuid::createUuid();
  tmpl.name = "nav2_controller";
  tmpl.displayName = "Nav2 Controller";
  tmpl.description = "Navigation controller node compatible with Nav2 stack";
  tmpl.category = TemplateCategory::Navigation;
  tmpl.iconName = "navigation";

  tmpl.inputPins = {
    {"goal_pose", "geometry_msgs/msg/PoseStamped", false, "Navigation goal"},
    {"odom", "nav_msgs/msg/Odometry", false, "Robot odometry"},
    {"scan", "sensor_msgs/msg/LaserScan", false, "Laser scan data"}
  };

  tmpl.outputPins = {
    {"cmd_vel", "geometry_msgs/msg/Twist", true, "Velocity commands"},
    {"path", "nav_msgs/msg/Path", true, "Planned path"},
    {"status", "action_msgs/msg/GoalStatusArray", true, "Navigation status"}
  };

  tmpl.parameters = {
    {"controller_plugin", "dwb_core::DWBLocalPlanner", "string", "Controller plugin name", true},
    {"max_vel_x", 0.5, "double", "Maximum linear velocity", false},
    {"max_vel_theta", 1.0, "double", "Maximum angular velocity", false},
    {"goal_tolerance", 0.1, "double", "Goal tolerance in meters", false}
  };

  tmpl.baseClass = "rclcpp_lifecycle::LifecycleNode";
  tmpl.requiredPackages = QStringList{"rclcpp", "rclcpp_lifecycle", "nav2_core", "geometry_msgs", "nav_msgs"};
  tmpl.author = "ROS Weaver";
  tmpl.version = "1.0";
  tmpl.tags = QStringList{"navigation", "nav2", "controller", "path"};

  return tmpl;
}

NodeTemplate NodeTemplate::createImageProcessor() {
  NodeTemplate tmpl;
  tmpl.id = QUuid::createUuid();
  tmpl.name = "image_processor";
  tmpl.displayName = "Image Processor";
  tmpl.description = "Image processing node with configurable pipeline stages";
  tmpl.category = TemplateCategory::Perception;
  tmpl.iconName = "image";

  tmpl.inputPins = {
    {"image_raw", "sensor_msgs/msg/Image", false, "Raw camera image"},
    {"camera_info", "sensor_msgs/msg/CameraInfo", false, "Camera calibration info"}
  };

  tmpl.outputPins = {
    {"image_processed", "sensor_msgs/msg/Image", true, "Processed image"},
    {"detections", "vision_msgs/msg/Detection2DArray", true, "Object detections"}
  };

  tmpl.parameters = {
    {"pipeline_stages", QVariant(QStringList{"resize", "normalize", "detect"}), "array", "Processing pipeline stages", true},
    {"target_width", 640, "int", "Target image width", false},
    {"target_height", 480, "int", "Target image height", false},
    {"detection_threshold", 0.5, "double", "Detection confidence threshold", false}
  };

  tmpl.baseClass = "rclcpp::Node";
  tmpl.requiredPackages = QStringList{"rclcpp", "sensor_msgs", "vision_msgs", "cv_bridge", "image_transport"};
  tmpl.author = "ROS Weaver";
  tmpl.version = "1.0";
  tmpl.tags = QStringList{"image", "perception", "vision", "detection"};

  return tmpl;
}

QString NodeTemplate::categoryToString(TemplateCategory category) {
  switch (category) {
    case TemplateCategory::Basic: return "Basic";
    case TemplateCategory::SensorFusion: return "Sensor Fusion";
    case TemplateCategory::StateMachine: return "State Machine";
    case TemplateCategory::BehaviorTree: return "Behavior Tree";
    case TemplateCategory::Bridge: return "Bridge";
    case TemplateCategory::Lifecycle: return "Lifecycle";
    case TemplateCategory::Component: return "Component";
    case TemplateCategory::Navigation: return "Navigation";
    case TemplateCategory::Perception: return "Perception";
    case TemplateCategory::Control: return "Control";
    case TemplateCategory::Custom: return "Custom";
  }
  return "Unknown";
}

TemplateCategory NodeTemplate::stringToCategory(const QString& str) {
  if (str == "Basic") return TemplateCategory::Basic;
  if (str == "Sensor Fusion") return TemplateCategory::SensorFusion;
  if (str == "State Machine") return TemplateCategory::StateMachine;
  if (str == "Behavior Tree") return TemplateCategory::BehaviorTree;
  if (str == "Bridge") return TemplateCategory::Bridge;
  if (str == "Lifecycle") return TemplateCategory::Lifecycle;
  if (str == "Component") return TemplateCategory::Component;
  if (str == "Navigation") return TemplateCategory::Navigation;
  if (str == "Perception") return TemplateCategory::Perception;
  if (str == "Control") return TemplateCategory::Control;
  return TemplateCategory::Custom;
}

QString NodeTemplate::categoryToIcon(TemplateCategory category) {
  switch (category) {
    case TemplateCategory::Basic: return QString::fromUtf8("\xF0\x9F\x93\xA6");        // Package
    case TemplateCategory::SensorFusion: return QString::fromUtf8("\xF0\x9F\x94\x80"); // Shuffle
    case TemplateCategory::StateMachine: return QString::fromUtf8("\xE2\x9A\x99");     // Gear
    case TemplateCategory::BehaviorTree: return QString::fromUtf8("\xF0\x9F\x8C\xB3"); // Tree
    case TemplateCategory::Bridge: return QString::fromUtf8("\xF0\x9F\x8C\x89");       // Bridge
    case TemplateCategory::Lifecycle: return QString::fromUtf8("\xF0\x9F\x94\x84");    // Cycle
    case TemplateCategory::Component: return QString::fromUtf8("\xF0\x9F\xA7\xA9");    // Puzzle
    case TemplateCategory::Navigation: return QString::fromUtf8("\xF0\x9F\xA7\xAD");   // Compass
    case TemplateCategory::Perception: return QString::fromUtf8("\xF0\x9F\x91\x81");   // Eye
    case TemplateCategory::Control: return QString::fromUtf8("\xF0\x9F\x8E\x9B");      // Slider
    case TemplateCategory::Custom: return QString::fromUtf8("\xE2\x9C\xA8");           // Sparkles
  }
  return QString::fromUtf8("\xF0\x9F\x93\xA6");
}

// =============================================================================
// NodeTemplatesManager
// =============================================================================

NodeTemplatesManager& NodeTemplatesManager::instance() {
  static NodeTemplatesManager instance;
  return instance;
}

NodeTemplatesManager::NodeTemplatesManager(QObject* parent)
  : QObject(parent)
{
  createBuiltInTemplates();
}

void NodeTemplatesManager::createBuiltInTemplates() {
  templates_.clear();

  // Add all built-in templates
  templates_.append(NodeTemplate::createBasicPublisher());
  templates_.append(NodeTemplate::createBasicSubscriber());
  templates_.append(NodeTemplate::createSensorFusion());
  templates_.append(NodeTemplate::createStateMachine());
  templates_.append(NodeTemplate::createBehaviorTreeExecutor());
  templates_.append(NodeTemplate::createRosBridge());
  templates_.append(NodeTemplate::createLifecycleNode());
  templates_.append(NodeTemplate::createComponentNode());
  templates_.append(NodeTemplate::createNav2Controller());
  templates_.append(NodeTemplate::createImageProcessor());

  emit templatesChanged();
}

QList<NodeTemplate> NodeTemplatesManager::templatesByCategory(TemplateCategory category) const {
  QList<NodeTemplate> result;
  for (const auto& tmpl : templates_) {
    if (tmpl.category == category) {
      result.append(tmpl);
    }
  }
  return result;
}

NodeTemplate* NodeTemplatesManager::findTemplate(const QUuid& id) {
  for (auto& tmpl : templates_) {
    if (tmpl.id == id) {
      return &tmpl;
    }
  }
  return nullptr;
}

NodeTemplate* NodeTemplatesManager::findTemplateByName(const QString& name) {
  for (auto& tmpl : templates_) {
    if (tmpl.name == name) {
      return &tmpl;
    }
  }
  return nullptr;
}

void NodeTemplatesManager::addTemplate(const NodeTemplate& tmpl) {
  templates_.append(tmpl);
  emit templateAdded(tmpl);
  emit templatesChanged();
}

void NodeTemplatesManager::removeTemplate(const QUuid& id) {
  for (int i = 0; i < templates_.size(); ++i) {
    if (templates_[i].id == id) {
      templates_.removeAt(i);
      emit templateRemoved(id);
      emit templatesChanged();
      return;
    }
  }
}

void NodeTemplatesManager::updateTemplate(const NodeTemplate& tmpl) {
  for (auto& existing : templates_) {
    if (existing.id == tmpl.id) {
      existing = tmpl;
      emit templateUpdated(tmpl);
      emit templatesChanged();
      return;
    }
  }
}

QList<TemplateCategory> NodeTemplatesManager::categories() const {
  QList<TemplateCategory> result;
  QSet<TemplateCategory> seen;
  for (const auto& tmpl : templates_) {
    if (!seen.contains(tmpl.category)) {
      seen.insert(tmpl.category);
      result.append(tmpl.category);
    }
  }
  return result;
}

int NodeTemplatesManager::templateCountForCategory(TemplateCategory category) const {
  int count = 0;
  for (const auto& tmpl : templates_) {
    if (tmpl.category == category) {
      ++count;
    }
  }
  return count;
}

QList<NodeTemplate> NodeTemplatesManager::searchTemplates(const QString& query) const {
  QList<NodeTemplate> result;
  QString lowerQuery = query.toLower();

  for (const auto& tmpl : templates_) {
    if (tmpl.name.toLower().contains(lowerQuery) ||
        tmpl.displayName.toLower().contains(lowerQuery) ||
        tmpl.description.toLower().contains(lowerQuery)) {
      result.append(tmpl);
      continue;
    }

    // Search tags
    for (const auto& tag : tmpl.tags) {
      if (tag.toLower().contains(lowerQuery)) {
        result.append(tmpl);
        break;
      }
    }
  }

  return result;
}

bool NodeTemplatesManager::saveCustomTemplates(const QString& filePath) {
  QJsonArray templatesArray;

  for (const auto& tmpl : templates_) {
    if (tmpl.category == TemplateCategory::Custom) {
      QJsonObject obj;
      obj["id"] = tmpl.id.toString();
      obj["name"] = tmpl.name;
      obj["displayName"] = tmpl.displayName;
      obj["description"] = tmpl.description;
      obj["category"] = NodeTemplate::categoryToString(tmpl.category);
      obj["iconName"] = tmpl.iconName;
      obj["baseClass"] = tmpl.baseClass;
      obj["author"] = tmpl.author;
      obj["version"] = tmpl.version;

      // Input pins
      QJsonArray inputPinsArray;
      for (const auto& pin : tmpl.inputPins) {
        QJsonObject pinObj;
        pinObj["name"] = pin.name;
        pinObj["messageType"] = pin.messageType;
        pinObj["isOutput"] = pin.isOutput;
        pinObj["description"] = pin.description;
        inputPinsArray.append(pinObj);
      }
      obj["inputPins"] = inputPinsArray;

      // Output pins
      QJsonArray outputPinsArray;
      for (const auto& pin : tmpl.outputPins) {
        QJsonObject pinObj;
        pinObj["name"] = pin.name;
        pinObj["messageType"] = pin.messageType;
        pinObj["isOutput"] = pin.isOutput;
        pinObj["description"] = pin.description;
        outputPinsArray.append(pinObj);
      }
      obj["outputPins"] = outputPinsArray;

      // Parameters
      QJsonArray paramsArray;
      for (const auto& param : tmpl.parameters) {
        QJsonObject paramObj;
        paramObj["name"] = param.name;
        paramObj["defaultValue"] = QJsonValue::fromVariant(param.defaultValue);
        paramObj["type"] = param.type;
        paramObj["description"] = param.description;
        paramObj["required"] = param.required;
        paramsArray.append(paramObj);
      }
      obj["parameters"] = paramsArray;

      // Required packages
      QJsonArray packagesArray;
      for (const auto& pkg : tmpl.requiredPackages) {
        packagesArray.append(pkg);
      }
      obj["requiredPackages"] = packagesArray;

      // Tags
      QJsonArray tagsArray;
      for (const auto& tag : tmpl.tags) {
        tagsArray.append(tag);
      }
      obj["tags"] = tagsArray;

      templatesArray.append(obj);
    }
  }

  QJsonDocument doc(templatesArray);
  QFile file(filePath);
  if (!file.open(QIODevice::WriteOnly)) {
    return false;
  }
  file.write(doc.toJson(QJsonDocument::Indented));
  file.close();
  return true;
}

bool NodeTemplatesManager::loadCustomTemplates(const QString& filePath) {
  QFile file(filePath);
  if (!file.open(QIODevice::ReadOnly)) {
    return false;
  }

  QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
  file.close();

  if (!doc.isArray()) {
    return false;
  }

  QJsonArray templatesArray = doc.array();
  for (const auto& value : templatesArray) {
    if (!value.isObject()) continue;

    QJsonObject obj = value.toObject();
    NodeTemplate tmpl;

    tmpl.id = QUuid(obj["id"].toString());
    if (tmpl.id.isNull()) {
      tmpl.id = QUuid::createUuid();
    }
    tmpl.name = obj["name"].toString();
    tmpl.displayName = obj["displayName"].toString();
    tmpl.description = obj["description"].toString();
    tmpl.category = NodeTemplate::stringToCategory(obj["category"].toString());
    tmpl.iconName = obj["iconName"].toString();
    tmpl.baseClass = obj["baseClass"].toString();
    tmpl.author = obj["author"].toString();
    tmpl.version = obj["version"].toString();

    // Input pins
    QJsonArray inputPinsArray = obj["inputPins"].toArray();
    for (const auto& pinValue : inputPinsArray) {
      QJsonObject pinObj = pinValue.toObject();
      TemplatePinDef pin;
      pin.name = pinObj["name"].toString();
      pin.messageType = pinObj["messageType"].toString();
      pin.isOutput = pinObj["isOutput"].toBool();
      pin.description = pinObj["description"].toString();
      tmpl.inputPins.append(pin);
    }

    // Output pins
    QJsonArray outputPinsArray = obj["outputPins"].toArray();
    for (const auto& pinValue : outputPinsArray) {
      QJsonObject pinObj = pinValue.toObject();
      TemplatePinDef pin;
      pin.name = pinObj["name"].toString();
      pin.messageType = pinObj["messageType"].toString();
      pin.isOutput = pinObj["isOutput"].toBool();
      pin.description = pinObj["description"].toString();
      tmpl.outputPins.append(pin);
    }

    // Parameters
    QJsonArray paramsArray = obj["parameters"].toArray();
    for (const auto& paramValue : paramsArray) {
      QJsonObject paramObj = paramValue.toObject();
      TemplateParamDef param;
      param.name = paramObj["name"].toString();
      param.defaultValue = paramObj["defaultValue"].toVariant();
      param.type = paramObj["type"].toString();
      param.description = paramObj["description"].toString();
      param.required = paramObj["required"].toBool();
      tmpl.parameters.append(param);
    }

    // Required packages
    QJsonArray packagesArray = obj["requiredPackages"].toArray();
    for (const auto& pkg : packagesArray) {
      tmpl.requiredPackages.append(pkg.toString());
    }

    // Tags
    QJsonArray tagsArray = obj["tags"].toArray();
    for (const auto& tag : tagsArray) {
      tmpl.tags.append(tag.toString());
    }

    addTemplate(tmpl);
  }

  return true;
}

}  // namespace ros_weaver
