#include "ros_weaver/core/architecture_generator.hpp"
#include "ros_weaver/core/ollama_manager.hpp"
#include "ros_weaver/core/project.hpp"

#include <QJsonDocument>
#include <QJsonArray>
#include <QRegularExpression>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace ros_weaver {

ArchitectureGenerator& ArchitectureGenerator::instance() {
  static ArchitectureGenerator instance;
  return instance;
}

ArchitectureGenerator::ArchitectureGenerator()
    : QObject(nullptr)
{
}

bool ArchitectureGenerator::isAIAvailable() const {
  return OllamaManager::instance().isOllamaRunning();
}

QStringList ArchitectureGenerator::availableTemplates() const {
  return {"slam_stack", "navigation2", "perception", "manipulation", "teleop"};
}

QString ArchitectureGenerator::templateDescription(ArchitectureTemplate templateType) const {
  switch (templateType) {
    case ArchitectureTemplate::SlamStack:
      return tr("SLAM (Simultaneous Localization and Mapping) stack with sensor "
                "preprocessing, mapping, and localization nodes.");
    case ArchitectureTemplate::Navigation2:
      return tr("Nav2 navigation stack with path planning, behavior trees, "
                "costmaps, and velocity controllers.");
    case ArchitectureTemplate::Perception:
      return tr("Perception pipeline with image processing, object detection, "
                "and point cloud processing nodes.");
    case ArchitectureTemplate::Manipulation:
      return tr("Manipulation stack with MoveIt2 integration, grasp planning, "
                "and arm control nodes.");
    case ArchitectureTemplate::Teleop:
      return tr("Teleoperation setup with joystick/keyboard input, velocity "
                "commands, and safety monitors.");
    default:
      return tr("Custom architecture generated from description.");
  }
}

ArchitectureGenerationResult ArchitectureGenerator::generateFromTemplate(
    ArchitectureTemplate templateType,
    const QMap<QString, QString>& options) {

  emit generationStarted(templateDescription(templateType));

  ArchitectureGenerationResult result;

  switch (templateType) {
    case ArchitectureTemplate::SlamStack:
      result = generateSlamStack(options);
      break;
    case ArchitectureTemplate::Navigation2:
      result = generateNav2Stack(options);
      break;
    case ArchitectureTemplate::Perception:
      result = generatePerceptionStack(options);
      break;
    case ArchitectureTemplate::Manipulation:
      result = generateManipulationStack(options);
      break;
    case ArchitectureTemplate::Teleop:
      result = generateTeleopStack(options);
      break;
    default:
      result.success = false;
      result.errorMessage = tr("Unknown template type");
      break;
  }

  if (result.success) {
    calculateLayout(result);
    emit generationCompleted(result);
  } else {
    emit generationFailed(result.errorMessage);
  }

  return result;
}

ArchitectureGenerationResult ArchitectureGenerator::generateSlamStack(
    const QMap<QString, QString>& options) {

  ArchitectureGenerationResult result;
  result.success = true;
  result.architectureName = "SLAM Stack";
  result.description = "SLAM mapping and localization pipeline";

  QString slamMethod = options.value("slam_method", "slam_toolbox");
  bool useLidar = options.value("use_lidar", "true") == "true";
  bool useDepthCamera = options.value("use_depth_camera", "false") == "true";

  // Laser scan source
  if (useLidar) {
    GeneratedNode lidar;
    lidar.name = "lidar_driver";
    lidar.packageName = "velodyne_driver";
    lidar.description = "LiDAR sensor driver";
    lidar.outputPins = {{"topic", "/scan"}};
    lidar.parameters = {{"frame_id", "laser_frame"}, {"range_max", "30.0"}};
    result.nodes.append(lidar);
  }

  if (useDepthCamera) {
    GeneratedNode depthCamera;
    depthCamera.name = "depth_camera";
    depthCamera.packageName = "realsense2_camera";
    depthCamera.description = "Depth camera driver";
    depthCamera.outputPins = {{"topic", "/camera/depth/image_raw"},
                               {"topic", "/camera/depth/points"}};
    result.nodes.append(depthCamera);

    GeneratedNode depthToLaser;
    depthToLaser.name = "depthimage_to_laserscan";
    depthToLaser.packageName = "depthimage_to_laserscan";
    depthToLaser.description = "Convert depth image to laser scan";
    depthToLaser.inputPins = {{"topic", "/camera/depth/image_raw"}};
    depthToLaser.outputPins = {{"topic", "/scan"}};
    result.nodes.append(depthToLaser);
  }

  // SLAM node
  GeneratedNode slam;
  slam.name = slamMethod;
  slam.packageName = slamMethod;
  slam.description = "SLAM mapping and localization";
  slam.inputPins = {{"topic", "/scan"}, {"topic", "/odom"}};
  slam.outputPins = {{"topic", "/map"}, {"topic", "/tf"}};
  slam.parameters = {{"resolution", "0.05"}, {"max_laser_range", "25.0"}};
  result.nodes.append(slam);

  // Map server
  GeneratedNode mapServer;
  mapServer.name = "map_server";
  mapServer.packageName = "nav2_map_server";
  mapServer.description = "Map server for saving/loading maps";
  mapServer.inputPins = {{"topic", "/map"}};
  mapServer.outputPins = {{"service", "/map_server/save_map"}};
  result.nodes.append(mapServer);

  // Odometry (assumed from robot)
  GeneratedNode odom;
  odom.name = "robot_state_publisher";
  odom.packageName = "robot_state_publisher";
  odom.description = "Robot state and TF publisher";
  odom.outputPins = {{"topic", "/odom"}, {"topic", "/tf"}};
  result.nodes.append(odom);

  // Connections
  if (useLidar) {
    result.connections.append({"lidar_driver", 0, slamMethod, 0, "/scan", "sensor_msgs/LaserScan"});
  } else if (useDepthCamera) {
    result.connections.append({"depth_camera", 0, "depthimage_to_laserscan", 0,
                               "/camera/depth/image_raw", "sensor_msgs/Image"});
    result.connections.append({"depthimage_to_laserscan", 0, slamMethod, 0,
                               "/scan", "sensor_msgs/LaserScan"});
  }
  result.connections.append({"robot_state_publisher", 0, slamMethod, 1, "/odom", "nav_msgs/Odometry"});
  result.connections.append({slamMethod, 0, "map_server", 0, "/map", "nav_msgs/OccupancyGrid"});

  // Groups
  GeneratedGroup sensorGroup;
  sensorGroup.title = "Sensors";
  if (useLidar) sensorGroup.nodeNames.append("lidar_driver");
  if (useDepthCamera) {
    sensorGroup.nodeNames.append("depth_camera");
    sensorGroup.nodeNames.append("depthimage_to_laserscan");
  }
  sensorGroup.color = "blue";
  if (!sensorGroup.nodeNames.isEmpty()) {
    result.groups.append(sensorGroup);
  }

  GeneratedGroup slamGroup;
  slamGroup.title = "SLAM";
  slamGroup.nodeNames = {slamMethod, "map_server"};
  slamGroup.color = "green";
  result.groups.append(slamGroup);

  return result;
}

ArchitectureGenerationResult ArchitectureGenerator::generateNav2Stack(
    const QMap<QString, QString>& options) {

  ArchitectureGenerationResult result;
  result.success = true;
  result.architectureName = "Navigation2 Stack";
  result.description = "ROS2 Navigation2 autonomous navigation pipeline";

  QString planner = options.value("planner", "navfn");
  QString controller = options.value("controller", "dwb_controller");

  // BT Navigator
  GeneratedNode btNav;
  btNav.name = "bt_navigator";
  btNav.packageName = "nav2_bt_navigator";
  btNav.description = "Behavior tree navigator";
  btNav.inputPins = {{"action", "/navigate_to_pose"}};
  btNav.outputPins = {{"topic", "/behavior_tree_log"}};
  result.nodes.append(btNav);

  // Planner server
  GeneratedNode plannerNode;
  plannerNode.name = "planner_server";
  plannerNode.packageName = "nav2_planner";
  plannerNode.description = "Global path planner";
  plannerNode.inputPins = {{"topic", "/map"}, {"topic", "/goal_pose"}};
  plannerNode.outputPins = {{"topic", "/plan"}};
  plannerNode.parameters = {{"planner_plugin", planner}};
  result.nodes.append(plannerNode);

  // Controller server
  GeneratedNode controllerNode;
  controllerNode.name = "controller_server";
  controllerNode.packageName = "nav2_controller";
  controllerNode.description = "Local trajectory controller";
  controllerNode.inputPins = {{"topic", "/plan"}, {"topic", "/odom"}};
  controllerNode.outputPins = {{"topic", "/cmd_vel"}};
  controllerNode.parameters = {{"controller_plugin", controller}};
  result.nodes.append(controllerNode);

  // Costmap nodes
  GeneratedNode globalCostmap;
  globalCostmap.name = "global_costmap";
  globalCostmap.packageName = "nav2_costmap_2d";
  globalCostmap.description = "Global costmap for planning";
  globalCostmap.inputPins = {{"topic", "/map"}, {"topic", "/scan"}};
  globalCostmap.outputPins = {{"topic", "/global_costmap/costmap"}};
  result.nodes.append(globalCostmap);

  GeneratedNode localCostmap;
  localCostmap.name = "local_costmap";
  localCostmap.packageName = "nav2_costmap_2d";
  localCostmap.description = "Local costmap for obstacle avoidance";
  localCostmap.inputPins = {{"topic", "/scan"}, {"topic", "/odom"}};
  localCostmap.outputPins = {{"topic", "/local_costmap/costmap"}};
  result.nodes.append(localCostmap);

  // Recovery server
  GeneratedNode recovery;
  recovery.name = "recovery_server";
  recovery.packageName = "nav2_recoveries";
  recovery.description = "Recovery behaviors (spin, backup, wait)";
  recovery.inputPins = {{"topic", "/odom"}};
  recovery.outputPins = {{"topic", "/cmd_vel"}};
  result.nodes.append(recovery);

  // Lifecycle manager
  GeneratedNode lifecycle;
  lifecycle.name = "lifecycle_manager";
  lifecycle.packageName = "nav2_lifecycle_manager";
  lifecycle.description = "Manages Nav2 node lifecycles";
  lifecycle.outputPins = {{"service", "/lifecycle_manager/manage_nodes"}};
  result.nodes.append(lifecycle);

  // Connections
  result.connections.append({"planner_server", 0, "controller_server", 0,
                             "/plan", "nav_msgs/Path"});

  // Groups
  GeneratedGroup planningGroup;
  planningGroup.title = "Planning";
  planningGroup.nodeNames = {"bt_navigator", "planner_server", "global_costmap"};
  planningGroup.color = "purple";
  result.groups.append(planningGroup);

  GeneratedGroup controlGroup;
  controlGroup.title = "Control";
  controlGroup.nodeNames = {"controller_server", "local_costmap", "recovery_server"};
  controlGroup.color = "orange";
  result.groups.append(controlGroup);

  return result;
}

ArchitectureGenerationResult ArchitectureGenerator::generatePerceptionStack(
    const QMap<QString, QString>& options) {

  ArchitectureGenerationResult result;
  result.success = true;
  result.architectureName = "Perception Stack";
  result.description = "Computer vision and perception pipeline";

  bool useObjectDetection = options.value("object_detection", "true") == "true";
  bool usePointCloud = options.value("pointcloud", "true") == "true";

  // Camera driver
  GeneratedNode camera;
  camera.name = "camera_driver";
  camera.packageName = "v4l2_camera";
  camera.description = "Camera driver";
  camera.outputPins = {{"topic", "/camera/image_raw"},
                        {"topic", "/camera/camera_info"}};
  result.nodes.append(camera);

  // Image processing
  GeneratedNode imageProc;
  imageProc.name = "image_proc";
  imageProc.packageName = "image_proc";
  imageProc.description = "Image rectification and processing";
  imageProc.inputPins = {{"topic", "/camera/image_raw"},
                          {"topic", "/camera/camera_info"}};
  imageProc.outputPins = {{"topic", "/camera/image_rect"}};
  result.nodes.append(imageProc);

  result.connections.append({"camera_driver", 0, "image_proc", 0,
                             "/camera/image_raw", "sensor_msgs/Image"});
  result.connections.append({"camera_driver", 1, "image_proc", 1,
                             "/camera/camera_info", "sensor_msgs/CameraInfo"});

  if (useObjectDetection) {
    GeneratedNode detector;
    detector.name = "object_detector";
    detector.packageName = "vision_msgs";
    detector.description = "Object detection (YOLO/SSD)";
    detector.inputPins = {{"topic", "/camera/image_rect"}};
    detector.outputPins = {{"topic", "/detections"},
                            {"topic", "/detection_image"}};
    result.nodes.append(detector);

    result.connections.append({"image_proc", 0, "object_detector", 0,
                               "/camera/image_rect", "sensor_msgs/Image"});
  }

  if (usePointCloud) {
    GeneratedNode depthCamera;
    depthCamera.name = "depth_camera";
    depthCamera.packageName = "realsense2_camera";
    depthCamera.description = "Depth camera driver";
    depthCamera.outputPins = {{"topic", "/camera/depth/points"}};
    result.nodes.append(depthCamera);

    GeneratedNode pclFilter;
    pclFilter.name = "pointcloud_filter";
    pclFilter.packageName = "pcl_ros";
    pclFilter.description = "Point cloud filtering";
    pclFilter.inputPins = {{"topic", "/camera/depth/points"}};
    pclFilter.outputPins = {{"topic", "/filtered_points"}};
    result.nodes.append(pclFilter);

    result.connections.append({"depth_camera", 0, "pointcloud_filter", 0,
                               "/camera/depth/points", "sensor_msgs/PointCloud2"});
  }

  // Groups
  GeneratedGroup cameraGroup;
  cameraGroup.title = "RGB Camera";
  cameraGroup.nodeNames = {"camera_driver", "image_proc"};
  if (useObjectDetection) cameraGroup.nodeNames.append("object_detector");
  cameraGroup.color = "blue";
  result.groups.append(cameraGroup);

  if (usePointCloud) {
    GeneratedGroup depthGroup;
    depthGroup.title = "Depth Processing";
    depthGroup.nodeNames = {"depth_camera", "pointcloud_filter"};
    depthGroup.color = "green";
    result.groups.append(depthGroup);
  }

  return result;
}

ArchitectureGenerationResult ArchitectureGenerator::generateManipulationStack(
    const QMap<QString, QString>& options) {

  ArchitectureGenerationResult result;
  result.success = true;
  result.architectureName = "Manipulation Stack";
  result.description = "MoveIt2 manipulation pipeline";

  QString robotName = options.value("robot_name", "arm");

  // MoveIt2 move_group
  GeneratedNode moveGroup;
  moveGroup.name = "move_group";
  moveGroup.packageName = "moveit_ros_move_group";
  moveGroup.description = "MoveIt2 motion planning";
  moveGroup.inputPins = {{"topic", "/joint_states"}, {"action", "/move_action"}};
  moveGroup.outputPins = {{"topic", "/display_planned_path"},
                           {"topic", "/trajectory_execution"}};
  result.nodes.append(moveGroup);

  // Robot state publisher
  GeneratedNode robotState;
  robotState.name = "robot_state_publisher";
  robotState.packageName = "robot_state_publisher";
  robotState.description = "Robot URDF state publisher";
  robotState.outputPins = {{"topic", "/robot_description"}, {"topic", "/tf"}};
  result.nodes.append(robotState);

  // Joint state publisher
  GeneratedNode jointState;
  jointState.name = "joint_state_publisher";
  jointState.packageName = "joint_state_publisher";
  jointState.description = "Joint state aggregator";
  jointState.inputPins = {{"topic", "/arm/joint_states"}};
  jointState.outputPins = {{"topic", "/joint_states"}};
  result.nodes.append(jointState);

  // Controller manager
  GeneratedNode controllerManager;
  controllerManager.name = "controller_manager";
  controllerManager.packageName = "controller_manager";
  controllerManager.description = "ros2_control controller manager";
  controllerManager.inputPins = {{"topic", "/trajectory_execution"}};
  controllerManager.outputPins = {{"topic", "/arm/joint_states"}};
  result.nodes.append(controllerManager);

  // Grasp planner (optional)
  GeneratedNode graspPlanner;
  graspPlanner.name = "grasp_planner";
  graspPlanner.packageName = "moveit_task_constructor";
  graspPlanner.description = "Grasp planning and pick-place";
  graspPlanner.inputPins = {{"topic", "/target_pose"}};
  graspPlanner.outputPins = {{"action", "/move_action"}};
  result.nodes.append(graspPlanner);

  // Connections
  result.connections.append({"controller_manager", 0, "joint_state_publisher", 0,
                             "/arm/joint_states", "sensor_msgs/JointState"});
  result.connections.append({"joint_state_publisher", 0, "move_group", 0,
                             "/joint_states", "sensor_msgs/JointState"});
  result.connections.append({"move_group", 1, "controller_manager", 0,
                             "/trajectory_execution", "trajectory_msgs/JointTrajectory"});
  result.connections.append({"grasp_planner", 0, "move_group", 1,
                             "/move_action", "moveit_msgs/MoveGroupAction"});

  // Groups
  GeneratedGroup moveitGroup;
  moveitGroup.title = "MoveIt2";
  moveitGroup.nodeNames = {"move_group", "grasp_planner"};
  moveitGroup.color = "purple";
  result.groups.append(moveitGroup);

  GeneratedGroup hardwareGroup;
  hardwareGroup.title = "Hardware Interface";
  hardwareGroup.nodeNames = {"controller_manager", "joint_state_publisher", "robot_state_publisher"};
  hardwareGroup.color = "orange";
  result.groups.append(hardwareGroup);

  return result;
}

ArchitectureGenerationResult ArchitectureGenerator::generateTeleopStack(
    const QMap<QString, QString>& options) {

  ArchitectureGenerationResult result;
  result.success = true;
  result.architectureName = "Teleop Stack";
  result.description = "Teleoperation and remote control setup";

  QString inputType = options.value("input_type", "keyboard");

  // Input node
  GeneratedNode input;
  if (inputType == "joystick") {
    input.name = "joy_node";
    input.packageName = "joy";
    input.description = "Joystick driver";
    input.outputPins = {{"topic", "/joy"}};
    input.parameters = {{"device_id", "0"}};
  } else {
    input.name = "teleop_keyboard";
    input.packageName = "teleop_twist_keyboard";
    input.description = "Keyboard teleoperation";
    input.outputPins = {{"topic", "/cmd_vel"}};
  }
  result.nodes.append(input);

  // Joy to twist converter (for joystick)
  if (inputType == "joystick") {
    GeneratedNode joyTwist;
    joyTwist.name = "teleop_twist_joy";
    joyTwist.packageName = "teleop_twist_joy";
    joyTwist.description = "Convert joystick to velocity commands";
    joyTwist.inputPins = {{"topic", "/joy"}};
    joyTwist.outputPins = {{"topic", "/cmd_vel"}};
    joyTwist.parameters = {{"linear_scale", "0.5"}, {"angular_scale", "1.0"}};
    result.nodes.append(joyTwist);

    result.connections.append({"joy_node", 0, "teleop_twist_joy", 0,
                               "/joy", "sensor_msgs/Joy"});
  }

  // Velocity smoother
  GeneratedNode smoother;
  smoother.name = "velocity_smoother";
  smoother.packageName = "nav2_velocity_smoother";
  smoother.description = "Smooth velocity commands";
  smoother.inputPins = {{"topic", "/cmd_vel"}};
  smoother.outputPins = {{"topic", "/cmd_vel_smoothed"}};
  smoother.parameters = {{"smoothing_frequency", "20.0"}};
  result.nodes.append(smoother);

  // Safety monitor
  GeneratedNode safety;
  safety.name = "safety_monitor";
  safety.packageName = "twist_mux";
  safety.description = "Safety monitoring and velocity mux";
  safety.inputPins = {{"topic", "/cmd_vel_smoothed"}, {"topic", "/e_stop"}};
  safety.outputPins = {{"topic", "/cmd_vel_safe"}};
  result.nodes.append(safety);

  // Connections
  if (inputType == "joystick") {
    result.connections.append({"teleop_twist_joy", 0, "velocity_smoother", 0,
                               "/cmd_vel", "geometry_msgs/Twist"});
  } else {
    result.connections.append({"teleop_keyboard", 0, "velocity_smoother", 0,
                               "/cmd_vel", "geometry_msgs/Twist"});
  }
  result.connections.append({"velocity_smoother", 0, "safety_monitor", 0,
                             "/cmd_vel_smoothed", "geometry_msgs/Twist"});

  // Groups
  GeneratedGroup inputGroup;
  inputGroup.title = "Input";
  if (inputType == "joystick") {
    inputGroup.nodeNames = {"joy_node", "teleop_twist_joy"};
  } else {
    inputGroup.nodeNames = {"teleop_keyboard"};
  }
  inputGroup.color = "blue";
  result.groups.append(inputGroup);

  GeneratedGroup safetyGroup;
  safetyGroup.title = "Safety & Control";
  safetyGroup.nodeNames = {"velocity_smoother", "safety_monitor"};
  safetyGroup.color = "red";
  result.groups.append(safetyGroup);

  return result;
}

ArchitectureGenerationResult ArchitectureGenerator::generateFromDescription(
    const QString& description,
    const QString& context) {

  emit generationStarted(description);

  ArchitectureGenerationResult result;

  if (!isAIAvailable()) {
    result.success = false;
    result.errorMessage = tr("AI backend (Ollama) is not available");
    emit generationFailed(result.errorMessage);
    return result;
  }

  emit generationProgress(10, tr("Building AI prompt..."));

  QString prompt = buildPrompt(description, context);

  emit generationProgress(30, tr("Waiting for AI response..."));

  QString response = sendToAI(prompt);

  if (response.isEmpty()) {
    result.success = false;
    result.errorMessage = tr("No response from AI");
    emit generationFailed(result.errorMessage);
    return result;
  }

  emit generationProgress(70, tr("Parsing architecture..."));

  result = parseAIResponse(response);

  if (result.success) {
    calculateLayout(result);
    emit generationProgress(100, tr("Generation complete"));
    emit generationCompleted(result);
  } else {
    emit generationFailed(result.errorMessage);
  }

  return result;
}

ArchitectureGenerationResult ArchitectureGenerator::suggestArchitecture(
    const QString& robotType,
    const QString& useCase,
    const QStringList& sensors) {

  QString description = QString(
      "Create a ROS2 architecture for a %1 robot that will be used for %2. "
      "The robot has the following sensors: %3."
  ).arg(robotType, useCase, sensors.join(", "));

  return generateFromDescription(description);
}

QString ArchitectureGenerator::buildPrompt(const QString& description,
                                            const QString& context) const {
  QString prompt = QString(
      "You are a ROS2 robotics architecture expert. Design a ROS2 node architecture "
      "based on this description:\n\n"
      "%1\n\n"
      "%2"
      "Provide the architecture as a JSON object with this structure:\n"
      "```json\n"
      "{\n"
      "  \"name\": \"Architecture Name\",\n"
      "  \"description\": \"Brief description\",\n"
      "  \"nodes\": [\n"
      "    {\n"
      "      \"name\": \"node_name\",\n"
      "      \"package\": \"ros_package_name\",\n"
      "      \"description\": \"What this node does\",\n"
      "      \"inputs\": [{\"type\": \"topic\", \"name\": \"/topic_name\"}],\n"
      "      \"outputs\": [{\"type\": \"topic\", \"name\": \"/output_topic\"}],\n"
      "      \"parameters\": {\"param_name\": \"value\"}\n"
      "    }\n"
      "  ],\n"
      "  \"connections\": [\n"
      "    {\"from\": \"node1\", \"from_pin\": 0, \"to\": \"node2\", \"to_pin\": 0, "
      "\"topic\": \"/topic\", \"msg_type\": \"std_msgs/String\"}\n"
      "  ],\n"
      "  \"groups\": [\n"
      "    {\"title\": \"Group Name\", \"nodes\": [\"node1\", \"node2\"], \"color\": \"blue\"}\n"
      "  ],\n"
      "  \"rationale\": \"Explanation of design decisions\"\n"
      "}\n"
      "```\n"
      "Use real ROS2 package names where possible. Include common nodes like "
      "robot_state_publisher, joint_state_publisher where appropriate."
  ).arg(description, context.isEmpty() ? "" : QString("Context: %1\n\n").arg(context));

  return prompt;
}

ArchitectureGenerationResult ArchitectureGenerator::parseAIResponse(const QString& response) {
  ArchitectureGenerationResult result;

  // Try to find JSON in the response
  static QRegularExpression jsonPattern(R"(\{[\s\S]*"nodes"[\s\S]*\})");
  QRegularExpressionMatch match = jsonPattern.match(response);

  if (!match.hasMatch()) {
    result.success = false;
    result.errorMessage = tr("Could not parse AI response as architecture JSON");
    return result;
  }

  QString jsonStr = match.captured(0);
  QJsonParseError error;
  QJsonDocument doc = QJsonDocument::fromJson(jsonStr.toUtf8(), &error);

  if (error.error != QJsonParseError::NoError) {
    result.success = false;
    result.errorMessage = tr("JSON parse error: %1").arg(error.errorString());
    return result;
  }

  if (!doc.isObject()) {
    result.success = false;
    result.errorMessage = tr("Expected JSON object");
    return result;
  }

  QJsonObject obj = doc.object();

  result.architectureName = obj["name"].toString("Generated Architecture");
  result.description = obj["description"].toString();
  result.aiRationale = obj["rationale"].toString();

  // Parse nodes
  QJsonArray nodesArray = obj["nodes"].toArray();
  for (const QJsonValue& nodeVal : nodesArray) {
    QJsonObject nodeObj = nodeVal.toObject();
    GeneratedNode node;
    node.name = nodeObj["name"].toString();
    node.packageName = nodeObj["package"].toString();
    node.description = nodeObj["description"].toString();

    QJsonArray inputs = nodeObj["inputs"].toArray();
    for (const QJsonValue& pinVal : inputs) {
      QJsonObject pinObj = pinVal.toObject();
      node.inputPins.append({pinObj["type"].toString("topic"),
                             pinObj["name"].toString()});
    }

    QJsonArray outputs = nodeObj["outputs"].toArray();
    for (const QJsonValue& pinVal : outputs) {
      QJsonObject pinObj = pinVal.toObject();
      node.outputPins.append({pinObj["type"].toString("topic"),
                              pinObj["name"].toString()});
    }

    QJsonObject params = nodeObj["parameters"].toObject();
    for (const QString& key : params.keys()) {
      node.parameters[key] = params[key].toString();
    }

    result.nodes.append(node);
  }

  // Parse connections
  QJsonArray connsArray = obj["connections"].toArray();
  for (const QJsonValue& connVal : connsArray) {
    QJsonObject connObj = connVal.toObject();
    GeneratedConnection conn;
    conn.sourceNode = connObj["from"].toString();
    conn.sourcePin = connObj["from_pin"].toInt();
    conn.targetNode = connObj["to"].toString();
    conn.targetPin = connObj["to_pin"].toInt();
    conn.topicName = connObj["topic"].toString();
    conn.messageType = connObj["msg_type"].toString();
    result.connections.append(conn);
  }

  // Parse groups
  QJsonArray groupsArray = obj["groups"].toArray();
  for (const QJsonValue& groupVal : groupsArray) {
    QJsonObject groupObj = groupVal.toObject();
    GeneratedGroup group;
    group.title = groupObj["title"].toString();
    QJsonArray nodeNames = groupObj["nodes"].toArray();
    for (const QJsonValue& nameVal : nodeNames) {
      group.nodeNames.append(nameVal.toString());
    }
    group.color = groupObj["color"].toString("blue");
    result.groups.append(group);
  }

  result.success = true;
  return result;
}

QString ArchitectureGenerator::sendToAI(const QString& prompt) {
  Q_UNUSED(prompt);
  OllamaManager& ollama = OllamaManager::instance();

  if (!ollama.isOllamaRunning()) {
    RCLCPP_WARN(rclcpp::get_logger("ros_weaver"),
                "Ollama not running, cannot generate architecture");
    return "";
  }

  RCLCPP_INFO(rclcpp::get_logger("ros_weaver"),
              "Sending architecture generation request to AI...");

  // Return empty for now - actual implementation would use async signals
  return "";
}

void ArchitectureGenerator::calculateLayout(ArchitectureGenerationResult& result) {
  // Simple grid layout with groups considered
  const double nodeWidth = 200.0;
  const double nodeHeight = 100.0;
  const double horizontalSpacing = 80.0;
  const double verticalSpacing = 60.0;

  // Build dependency graph to determine node ordering
  QMap<QString, int> nodeDepth;
  QSet<QString> nodeNames;

  for (const GeneratedNode& node : result.nodes) {
    nodeNames.insert(node.name);
    nodeDepth[node.name] = 0;
  }

  // Calculate depths based on connections
  bool changed = true;
  while (changed) {
    changed = false;
    for (const GeneratedConnection& conn : result.connections) {
      if (nodeDepth.contains(conn.sourceNode) && nodeDepth.contains(conn.targetNode)) {
        int newDepth = nodeDepth[conn.sourceNode] + 1;
        if (newDepth > nodeDepth[conn.targetNode]) {
          nodeDepth[conn.targetNode] = newDepth;
          changed = true;
        }
      }
    }
  }

  // Group nodes by depth
  QMap<int, QList<QString>> depthGroups;
  for (auto it = nodeDepth.begin(); it != nodeDepth.end(); ++it) {
    depthGroups[it.value()].append(it.key());
  }

  // Assign positions
  QMap<QString, QPointF> nodePositions;
  for (auto it = depthGroups.begin(); it != depthGroups.end(); ++it) {
    int depth = it.key();
    const QList<QString>& nodesAtDepth = it.value();

    double x = depth * (nodeWidth + horizontalSpacing);
    double startY = -(nodesAtDepth.size() - 1) * (nodeHeight + verticalSpacing) / 2.0;

    for (int i = 0; i < nodesAtDepth.size(); ++i) {
      double y = startY + i * (nodeHeight + verticalSpacing);
      nodePositions[nodesAtDepth[i]] = QPointF(x, y);
    }
  }

  // Apply positions to nodes
  for (GeneratedNode& node : result.nodes) {
    if (nodePositions.contains(node.name)) {
      node.position = nodePositions[node.name];
    }
  }
}

bool ArchitectureGenerator::applyToProject(const ArchitectureGenerationResult& result,
                                            Project& project) {
  if (!result.success) {
    return false;
  }

  // Add nodes
  for (const GeneratedNode& genNode : result.nodes) {
    BlockData block;
    block.id = QUuid::createUuid();
    block.name = genNode.name;
    block.position = genNode.position;

    for (const auto& pin : genNode.inputPins) {
      PinData pinData;
      pinData.type = "input";
      pinData.dataType = pin.first;
      pinData.name = pin.second;
      block.inputPins.append(pinData);
    }

    for (const auto& pin : genNode.outputPins) {
      PinData pinData;
      pinData.type = "output";
      pinData.dataType = pin.first;
      pinData.name = pin.second;
      block.outputPins.append(pinData);
    }

    for (auto it = genNode.parameters.begin(); it != genNode.parameters.end(); ++it) {
      BlockParamData param;
      param.name = it.key();
      param.currentValue = it.value();
      param.type = "string";
      block.parameters.append(param);
    }

    project.addBlock(block);
  }

  // Add connections
  for (const GeneratedConnection& genConn : result.connections) {
    ConnectionData conn;
    conn.id = QUuid::createUuid();

    // Find source and target block IDs
    for (const BlockData& block : project.blocks()) {
      if (block.name == genConn.sourceNode) {
        conn.sourceBlockId = block.id;
        conn.sourcePinIndex = genConn.sourcePin;
      }
      if (block.name == genConn.targetNode) {
        conn.targetBlockId = block.id;
        conn.targetPinIndex = genConn.targetPin;
      }
    }

    project.addConnection(conn);
  }

  // Add groups
  for (const GeneratedGroup& genGroup : result.groups) {
    NodeGroupData group;
    group.id = QUuid::createUuid();
    group.title = genGroup.title;

    // Find member block IDs
    for (const BlockData& block : project.blocks()) {
      if (genGroup.nodeNames.contains(block.name)) {
        group.containedNodeIds.append(block.id);
      }
    }

    if (genGroup.color == "blue") group.color = QColor(80, 120, 180, 180);
    else if (genGroup.color == "green") group.color = QColor(80, 160, 100, 180);
    else if (genGroup.color == "red") group.color = QColor(180, 80, 80, 180);
    else if (genGroup.color == "orange") group.color = QColor(200, 140, 60, 180);
    else if (genGroup.color == "purple") group.color = QColor(140, 80, 180, 180);
    else group.color = QColor(80, 120, 180, 180);

    project.addNodeGroup(group);
  }

  return true;
}

}  // namespace ros_weaver
