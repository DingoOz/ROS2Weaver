#include "ros_weaver/core/code_generator.hpp"
#include "ros_weaver/core/project.hpp"
#include "ros_weaver/widgets/param_dashboard.hpp"

#include <QFile>
#include <QTextStream>
#include <QRegularExpression>
#include <QDateTime>

namespace ros_weaver {

CodeGenerator::CodeGenerator(QObject* parent)
  : QObject(parent)
{
}

CodeGenerator::~CodeGenerator() = default;

bool CodeGenerator::generatePackage(const Project& project, const GeneratorOptions& options) {
  emit generationStarted();

  QString packagePath = options.outputPath + "/" + options.packageName;

  // Create directory structure
  emit generationProgress(10, "Creating directory structure...");

  if (!createDirectory(packagePath)) {
    lastError_ = "Failed to create package directory: " + packagePath;
    emit generationFinished(false);
    return false;
  }

  if (!createDirectory(packagePath + "/src")) {
    lastError_ = "Failed to create src directory";
    emit generationFinished(false);
    return false;
  }

  if (!createDirectory(packagePath + "/include/" + options.packageName)) {
    lastError_ = "Failed to create include directory";
    emit generationFinished(false);
    return false;
  }

  if (options.generateLaunchFile) {
    if (!createDirectory(packagePath + "/launch")) {
      lastError_ = "Failed to create launch directory";
      emit generationFinished(false);
      return false;
    }
  }

  if (!createDirectory(packagePath + "/config")) {
    lastError_ = "Failed to create config directory";
    emit generationFinished(false);
    return false;
  }

  // Generate CMakeLists.txt
  emit generationProgress(20, "Generating CMakeLists.txt...");
  QString cmakeLists = generateCMakeLists(project, options);
  if (!writeFile(packagePath + "/CMakeLists.txt", cmakeLists)) {
    lastError_ = "Failed to write CMakeLists.txt";
    emit generationFinished(false);
    return false;
  }
  emit fileGenerated(packagePath + "/CMakeLists.txt");

  // Generate package.xml
  emit generationProgress(30, "Generating package.xml...");
  QString packageXml = generatePackageXml(project, options);
  if (!writeFile(packagePath + "/package.xml", packageXml)) {
    lastError_ = "Failed to write package.xml";
    emit generationFinished(false);
    return false;
  }
  emit fileGenerated(packagePath + "/package.xml");

  // Generate node source files
  emit generationProgress(40, "Generating node source files...");
  int nodeIndex = 0;
  for (const BlockData& block : project.blocks()) {
    QString nodeCode;

    // Convert BlockParamData to ParamDefinition
    QList<ParamDefinition> params;
    for (const BlockParamData& bpd : block.parameters) {
      ParamDefinition pd;
      pd.name = bpd.name;
      pd.type = bpd.type;
      pd.defaultValue = bpd.defaultValue;
      pd.currentValue = bpd.currentValue;
      pd.description = bpd.description;
      pd.minValue = bpd.minValue;
      pd.maxValue = bpd.maxValue;
      pd.enumValues = bpd.enumValues;
      pd.group = bpd.group;
      params.append(pd);
    }

    if (options.useCppStyle) {
      nodeCode = generateNodeCpp(block, project.connections(), params);
      QString fileName = toSnakeCase(block.name) + "_node.cpp";
      if (!writeFile(packagePath + "/src/" + fileName, nodeCode)) {
        lastError_ = "Failed to write " + fileName;
        emit generationFinished(false);
        return false;
      }
      emit fileGenerated(packagePath + "/src/" + fileName);
    } else {
      nodeCode = generateNodePython(block, project.connections(), params);
      QString fileName = toSnakeCase(block.name) + "_node.py";
      if (!writeFile(packagePath + "/src/" + fileName, nodeCode)) {
        lastError_ = "Failed to write " + fileName;
        emit generationFinished(false);
        return false;
      }
      emit fileGenerated(packagePath + "/src/" + fileName);
    }

    nodeIndex++;
    int progress = 40 + (nodeIndex * 30 / project.blocks().size());
    emit generationProgress(progress, QString("Generated %1...").arg(block.name));
  }

  // Generate launch file
  if (options.generateLaunchFile) {
    emit generationProgress(80, "Generating launch file...");
    QString launchFile = generateLaunchFile(project, options);
    QString launchFileName = options.packageName + "_launch.py";
    if (!writeFile(packagePath + "/launch/" + launchFileName, launchFile)) {
      lastError_ = "Failed to write launch file";
      emit generationFinished(false);
      return false;
    }
    emit fileGenerated(packagePath + "/launch/" + launchFileName);
  }

  // Generate params YAML
  emit generationProgress(90, "Generating parameters YAML...");
  QString paramsYaml = generateParamsYaml(project);
  if (!writeFile(packagePath + "/config/params.yaml", paramsYaml)) {
    lastError_ = "Failed to write params.yaml";
    emit generationFinished(false);
    return false;
  }
  emit fileGenerated(packagePath + "/config/params.yaml");

  emit generationProgress(100, "Done!");
  emit generationFinished(true);
  return true;
}

QString CodeGenerator::generateCMakeLists(const Project& project, const GeneratorOptions& options) {
  QString content;
  QTextStream stream(&content);

  stream << "cmake_minimum_required(VERSION 3.8)\n";
  stream << "project(" << options.packageName << ")\n\n";

  stream << "# Default to C++17\n";
  stream << "if(NOT CMAKE_CXX_STANDARD)\n";
  stream << "  set(CMAKE_CXX_STANDARD 17)\n";
  stream << "endif()\n\n";

  stream << "if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES \"Clang\")\n";
  stream << "  add_compile_options(-Wall -Wextra -Wpedantic)\n";
  stream << "endif()\n\n";

  stream << "# Find dependencies\n";
  stream << "find_package(ament_cmake REQUIRED)\n";
  stream << "find_package(rclcpp REQUIRED)\n";

  // Collect unique message packages from all blocks
  QSet<QString> messagePackages;
  for (const BlockData& block : project.blocks()) {
    for (const PinData& pin : block.inputPins) {
      if (!pin.messageType.isEmpty()) {
        QString pkg = pin.messageType.split("/").first();
        messagePackages.insert(pkg);
      }
    }
    for (const PinData& pin : block.outputPins) {
      if (!pin.messageType.isEmpty()) {
        QString pkg = pin.messageType.split("/").first();
        messagePackages.insert(pkg);
      }
    }
  }

  for (const QString& pkg : messagePackages) {
    stream << "find_package(" << pkg << " REQUIRED)\n";
  }
  stream << "\n";

  // Generate executables for each node
  stream << "# Executables\n";
  for (const BlockData& block : project.blocks()) {
    QString nodeName = toSnakeCase(block.name);
    stream << "add_executable(" << nodeName << "_node src/" << nodeName << "_node.cpp)\n";
    stream << "ament_target_dependencies(" << nodeName << "_node\n";
    stream << "  rclcpp\n";
    for (const QString& pkg : messagePackages) {
      stream << "  " << pkg << "\n";
    }
    stream << ")\n\n";
  }

  // Install targets
  stream << "# Install targets\n";
  stream << "install(TARGETS\n";
  for (const BlockData& block : project.blocks()) {
    QString nodeName = toSnakeCase(block.name);
    stream << "  " << nodeName << "_node\n";
  }
  stream << "  DESTINATION lib/${PROJECT_NAME}\n";
  stream << ")\n\n";

  // Install launch and config files
  stream << "# Install launch files\n";
  stream << "install(DIRECTORY launch/\n";
  stream << "  DESTINATION share/${PROJECT_NAME}/launch\n";
  stream << ")\n\n";

  stream << "# Install config files\n";
  stream << "install(DIRECTORY config/\n";
  stream << "  DESTINATION share/${PROJECT_NAME}/config\n";
  stream << ")\n\n";

  if (options.generateTests) {
    stream << "if(BUILD_TESTING)\n";
    stream << "  find_package(ament_lint_auto REQUIRED)\n";
    stream << "  ament_lint_auto_find_test_dependencies()\n";
    stream << "endif()\n\n";
  }

  stream << "ament_package()\n";

  return content;
}

QString CodeGenerator::generatePackageXml(const Project& project, const GeneratorOptions& options) {
  QString content;
  QTextStream stream(&content);

  stream << "<?xml version=\"1.0\"?>\n";
  stream << "<?xml-model href=\"http://download.ros.org/schema/package_format3.xsd\" schematypens=\"http://www.w3.org/2001/XMLSchema\"?>\n";
  stream << "<package format=\"3\">\n";
  stream << "  <name>" << options.packageName << "</name>\n";
  stream << "  <version>0.1.0</version>\n";
  stream << "  <description>" << project.metadata().description << "</description>\n";
  stream << "  <maintainer email=\"" << options.maintainerEmail << "\">" << options.maintainer << "</maintainer>\n";
  stream << "  <license>" << options.license << "</license>\n\n";

  stream << "  <buildtool_depend>ament_cmake</buildtool_depend>\n\n";

  stream << "  <depend>rclcpp</depend>\n";

  // Collect unique message packages
  QSet<QString> messagePackages;
  for (const BlockData& block : project.blocks()) {
    for (const PinData& pin : block.inputPins) {
      if (!pin.messageType.isEmpty()) {
        QString pkg = pin.messageType.split("/").first();
        messagePackages.insert(pkg);
      }
    }
    for (const PinData& pin : block.outputPins) {
      if (!pin.messageType.isEmpty()) {
        QString pkg = pin.messageType.split("/").first();
        messagePackages.insert(pkg);
      }
    }
  }

  for (const QString& pkg : messagePackages) {
    stream << "  <depend>" << pkg << "</depend>\n";
  }

  stream << "\n";

  if (options.generateTests) {
    stream << "  <test_depend>ament_lint_auto</test_depend>\n";
    stream << "  <test_depend>ament_lint_common</test_depend>\n\n";
  }

  stream << "  <export>\n";
  stream << "    <build_type>ament_cmake</build_type>\n";
  stream << "  </export>\n";
  stream << "</package>\n";

  return content;
}

QString CodeGenerator::generateNodeCpp(const BlockData& block,
                                        [[maybe_unused]] const QList<ConnectionData>& connections,
                                        const QList<ParamDefinition>& params) {
  QString content;
  QTextStream stream(&content);

  QString className = toCamelCase(block.name) + "Node";
  QString nodeName = toSnakeCase(block.name);

  // Includes
  stream << "#include <rclcpp/rclcpp.hpp>\n";

  // Message includes
  QSet<QString> includes;
  for (const PinData& pin : block.inputPins) {
    if (!pin.messageType.isEmpty()) {
      includes.insert(getMessageInclude(pin.messageType));
    }
  }
  for (const PinData& pin : block.outputPins) {
    if (!pin.messageType.isEmpty()) {
      includes.insert(getMessageInclude(pin.messageType));
    }
  }
  for (const QString& inc : includes) {
    stream << "#include <" << inc << ">\n";
  }

  stream << "\n";
  stream << "#include <memory>\n";
  stream << "#include <chrono>\n";
  stream << "\n";
  stream << "using namespace std::chrono_literals;\n";
  stream << "\n";

  // Class definition
  stream << "class " << className << " : public rclcpp::Node {\n";
  stream << "public:\n";
  stream << "  " << className << "() : Node(\"" << nodeName << "\") {\n";

  // Declare parameters
  if (!params.isEmpty()) {
    stream << "    // Declare parameters\n";
    for (const ParamDefinition& param : params) {
      stream << "    this->declare_parameter(\"" << param.name << "\", ";
      if (param.type == "string") {
        stream << "\"" << param.defaultValue.toString() << "\"";
      } else if (param.type == "bool") {
        stream << (param.defaultValue.toBool() ? "true" : "false");
      } else {
        stream << param.defaultValue.toString();
      }
      stream << ");\n";
    }
    stream << "\n";
  }

  // Create publishers
  if (!block.outputPins.isEmpty()) {
    stream << "    // Create publishers\n";
    int pubIndex = 0;
    for (const PinData& pin : block.outputPins) {
      if (pin.dataType == "topic" && !pin.messageType.isEmpty()) {
        QString msgType = getMessageType(pin.messageType);
        stream << "    publisher_" << pubIndex << "_ = this->create_publisher<"
               << msgType << ">(\"" << pin.name << "\", 10);\n";
        pubIndex++;
      }
    }
    stream << "\n";
  }

  // Create subscribers
  if (!block.inputPins.isEmpty()) {
    stream << "    // Create subscribers\n";
    int subIndex = 0;
    for (const PinData& pin : block.inputPins) {
      if (pin.dataType == "topic" && !pin.messageType.isEmpty()) {
        QString msgType = getMessageType(pin.messageType);
        stream << "    subscription_" << subIndex << "_ = this->create_subscription<"
               << msgType << ">(\n";
        stream << "      \"" << pin.name << "\", 10,\n";
        stream << "      std::bind(&" << className << "::on_" << toSnakeCase(pin.name)
               << ", this, std::placeholders::_1));\n";
        subIndex++;
      }
    }
    stream << "\n";
  }

  // Create timer for publishers if no subscribers (source node)
  if (block.inputPins.isEmpty() && !block.outputPins.isEmpty()) {
    stream << "    // Create timer for publishing\n";
    stream << "    timer_ = this->create_wall_timer(\n";
    stream << "      100ms, std::bind(&" << className << "::timer_callback, this));\n\n";
  }

  stream << "    RCLCPP_INFO(this->get_logger(), \"" << className << " initialized\");\n";
  stream << "  }\n\n";

  stream << "private:\n";

  // Timer callback
  if (block.inputPins.isEmpty() && !block.outputPins.isEmpty()) {
    stream << "  void timer_callback() {\n";
    stream << "    // TODO: Implement publishing logic\n";
    int pubIndex = 0;
    for (const PinData& pin : block.outputPins) {
      if (pin.dataType == "topic" && !pin.messageType.isEmpty()) {
        QString msgType = getMessageType(pin.messageType);
        stream << "    auto message = " << msgType << "();\n";
        stream << "    publisher_" << pubIndex << "_->publish(message);\n";
        pubIndex++;
        break;  // Just one for now
      }
    }
    stream << "  }\n\n";
  }

  // Subscriber callbacks
  int subIndex = 0;
  for (const PinData& pin : block.inputPins) {
    if (pin.dataType == "topic" && !pin.messageType.isEmpty()) {
      QString msgType = getMessageType(pin.messageType);
      stream << "  void on_" << toSnakeCase(pin.name) << "(const " << msgType
             << "::SharedPtr msg) {\n";
      stream << "    // TODO: Implement callback logic\n";
      stream << "    (void)msg;  // Suppress unused warning\n";

      // If there are output pins, show how to publish
      if (!block.outputPins.isEmpty()) {
        stream << "\n    // Forward/process and publish\n";
        int pubIndex = 0;
        for (const PinData& outPin : block.outputPins) {
          if (outPin.dataType == "topic" && !outPin.messageType.isEmpty()) {
            QString outMsgType = getMessageType(outPin.messageType);
            stream << "    auto out_msg = " << outMsgType << "();\n";
            stream << "    publisher_" << pubIndex << "_->publish(out_msg);\n";
            pubIndex++;
            break;  // Just one for now
          }
        }
      }
      stream << "  }\n\n";
      subIndex++;
    }
  }

  // Member variables - timer
  if (block.inputPins.isEmpty() && !block.outputPins.isEmpty()) {
    stream << "  rclcpp::TimerBase::SharedPtr timer_;\n";
  }

  // Member variables - publishers
  int pubIndex = 0;
  for (const PinData& pin : block.outputPins) {
    if (pin.dataType == "topic" && !pin.messageType.isEmpty()) {
      QString msgType = getMessageType(pin.messageType);
      stream << "  rclcpp::Publisher<" << msgType << ">::SharedPtr publisher_"
             << pubIndex << "_;\n";
      pubIndex++;
    }
  }

  // Member variables - subscribers
  subIndex = 0;
  for (const PinData& pin : block.inputPins) {
    if (pin.dataType == "topic" && !pin.messageType.isEmpty()) {
      QString msgType = getMessageType(pin.messageType);
      stream << "  rclcpp::Subscription<" << msgType << ">::SharedPtr subscription_"
             << subIndex << "_;\n";
      subIndex++;
    }
  }

  stream << "};\n\n";

  // Main function
  stream << "int main(int argc, char* argv[]) {\n";
  stream << "  rclcpp::init(argc, argv);\n";
  stream << "  rclcpp::spin(std::make_shared<" << className << ">());\n";
  stream << "  rclcpp::shutdown();\n";
  stream << "  return 0;\n";
  stream << "}\n";

  return content;
}

QString CodeGenerator::generateNodePython(const BlockData& block,
                                           const QList<ConnectionData>& connections,
                                           const QList<ParamDefinition>& params) {
  Q_UNUSED(connections)

  QString content;
  QTextStream stream(&content);

  QString className = toCamelCase(block.name) + "Node";
  QString nodeName = toSnakeCase(block.name);

  stream << "#!/usr/bin/env python3\n";
  stream << "\n";
  stream << "import rclpy\n";
  stream << "from rclpy.node import Node\n";

  // Message imports
  QSet<QString> imports;
  for (const PinData& pin : block.inputPins) {
    if (!pin.messageType.isEmpty()) {
      QStringList parts = pin.messageType.split("/");
      if (parts.size() >= 3) {
        imports.insert(QString("from %1.%2 import %3").arg(parts[0], parts[1], parts[2]));
      }
    }
  }
  for (const PinData& pin : block.outputPins) {
    if (!pin.messageType.isEmpty()) {
      QStringList parts = pin.messageType.split("/");
      if (parts.size() >= 3) {
        imports.insert(QString("from %1.%2 import %3").arg(parts[0], parts[1], parts[2]));
      }
    }
  }
  for (const QString& imp : imports) {
    stream << imp << "\n";
  }

  stream << "\n\n";

  // Class definition
  stream << "class " << className << "(Node):\n";
  stream << "    def __init__(self):\n";
  stream << "        super().__init__('" << nodeName << "')\n";
  stream << "\n";

  // Declare parameters
  if (!params.isEmpty()) {
    stream << "        # Declare parameters\n";
    for (const ParamDefinition& param : params) {
      stream << "        self.declare_parameter('" << param.name << "', ";
      if (param.type == "string") {
        stream << "'" << param.defaultValue.toString() << "'";
      } else if (param.type == "bool") {
        stream << (param.defaultValue.toBool() ? "True" : "False");
      } else {
        stream << param.defaultValue.toString();
      }
      stream << ")\n";
    }
    stream << "\n";
  }

  // Create publishers
  if (!block.outputPins.isEmpty()) {
    stream << "        # Create publishers\n";
    int pubIndex = 0;
    for (const PinData& pin : block.outputPins) {
      if (pin.dataType == "topic" && !pin.messageType.isEmpty()) {
        QStringList parts = pin.messageType.split("/");
        if (parts.size() >= 3) {
          stream << "        self.publisher_" << pubIndex << " = self.create_publisher("
                 << parts[2] << ", '" << pin.name << "', 10)\n";
        }
        pubIndex++;
      }
    }
    stream << "\n";
  }

  // Create subscribers
  if (!block.inputPins.isEmpty()) {
    stream << "        # Create subscribers\n";
    int subIndex = 0;
    for (const PinData& pin : block.inputPins) {
      if (pin.dataType == "topic" && !pin.messageType.isEmpty()) {
        QStringList parts = pin.messageType.split("/");
        if (parts.size() >= 3) {
          stream << "        self.subscription_" << subIndex << " = self.create_subscription(\n";
          stream << "            " << parts[2] << ", '" << pin.name << "', self.on_"
                 << toSnakeCase(pin.name) << ", 10)\n";
        }
        subIndex++;
      }
    }
    stream << "\n";
  }

  // Create timer for publishers if no subscribers
  if (block.inputPins.isEmpty() && !block.outputPins.isEmpty()) {
    stream << "        # Create timer for publishing\n";
    stream << "        self.timer = self.create_timer(0.1, self.timer_callback)\n\n";
  }

  stream << "        self.get_logger().info('" << className << " initialized')\n";
  stream << "\n";

  // Timer callback
  if (block.inputPins.isEmpty() && !block.outputPins.isEmpty()) {
    stream << "    def timer_callback(self):\n";
    stream << "        # TODO: Implement publishing logic\n";
    int pubIndex = 0;
    for (const PinData& pin : block.outputPins) {
      if (pin.dataType == "topic" && !pin.messageType.isEmpty()) {
        QStringList parts = pin.messageType.split("/");
        if (parts.size() >= 3) {
          stream << "        msg = " << parts[2] << "()\n";
          stream << "        self.publisher_" << pubIndex << ".publish(msg)\n";
        }
        pubIndex++;
        break;
      }
    }
    stream << "\n";
  }

  // Subscriber callbacks
  for (const PinData& pin : block.inputPins) {
    if (pin.dataType == "topic" && !pin.messageType.isEmpty()) {
      stream << "    def on_" << toSnakeCase(pin.name) << "(self, msg):\n";
      stream << "        # TODO: Implement callback logic\n";
      stream << "        pass\n\n";
    }
  }

  // Main function
  stream << "\n";
  stream << "def main(args=None):\n";
  stream << "    rclpy.init(args=args)\n";
  stream << "    node = " << className << "()\n";
  stream << "    rclpy.spin(node)\n";
  stream << "    node.destroy_node()\n";
  stream << "    rclpy.shutdown()\n";
  stream << "\n\n";
  stream << "if __name__ == '__main__':\n";
  stream << "    main()\n";

  return content;
}

QString CodeGenerator::generateLaunchFile(const Project& project, const GeneratorOptions& options) {
  QString content;
  QTextStream stream(&content);

  stream << "from launch import LaunchDescription\n";
  stream << "from launch_ros.actions import Node\n";
  stream << "from launch.actions import DeclareLaunchArgument\n";
  stream << "from launch.substitutions import LaunchConfiguration\n";
  stream << "from ament_index_python.packages import get_package_share_directory\n";
  stream << "import os\n";
  stream << "\n\n";

  stream << "def generate_launch_description():\n";
  stream << "    # Get the package share directory\n";
  stream << "    pkg_dir = get_package_share_directory('" << options.packageName << "')\n";
  stream << "    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')\n";
  stream << "\n";

  stream << "    # Declare launch arguments\n";
  stream << "    use_sim_time_arg = DeclareLaunchArgument(\n";
  stream << "        'use_sim_time',\n";
  stream << "        default_value='false',\n";
  stream << "        description='Use simulation time')\n";
  stream << "\n";

  stream << "    # Create node list\n";
  stream << "    nodes = []\n";
  stream << "\n";

  // Build a map of block ID to BlockData for quick lookup
  QMap<QUuid, const BlockData*> blockMap;
  for (const BlockData& block : project.blocks()) {
    blockMap[block.id] = &block;
  }

  // Generate node entries
  for (const BlockData& block : project.blocks()) {
    QString nodeName = toSnakeCase(block.name);

    // Build remappings for this node based on connections
    QList<QPair<QString, QString>> remappings;

    for (const ConnectionData& conn : project.connections()) {
      // Check if this block is the target (subscriber) of a connection
      if (conn.targetBlockId == block.id) {
        const BlockData* sourceBlock = blockMap.value(conn.sourceBlockId);
        if (sourceBlock && conn.sourcePinIndex < sourceBlock->outputPins.size() &&
            conn.targetPinIndex < block.inputPins.size()) {
          QString sourceTopic = sourceBlock->outputPins[conn.sourcePinIndex].name;
          QString targetTopic = block.inputPins[conn.targetPinIndex].name;

          // If topic names differ, we need a remapping
          if (sourceTopic != targetTopic) {
            remappings.append(qMakePair(targetTopic, sourceTopic));
          }
        }
      }

      // Check if this block is the source (publisher) of a connection
      if (conn.sourceBlockId == block.id) {
        const BlockData* targetBlock = blockMap.value(conn.targetBlockId);
        if (targetBlock && conn.sourcePinIndex < block.outputPins.size() &&
            conn.targetPinIndex < targetBlock->inputPins.size()) {
          QString sourceTopic = block.outputPins[conn.sourcePinIndex].name;
          QString targetTopic = targetBlock->inputPins[conn.targetPinIndex].name;

          // If topic names differ, remap publisher to match what subscriber expects
          // This is typically handled by remapping subscriber, so only add if not already handled
          if (sourceTopic != targetTopic) {
            // Check if we should remap publisher or subscriber
            // Convention: remap subscriber to match publisher
            // But if multiple subscribers, may need to remap publisher
            // For simplicity, we prefer remapping subscriber side
          }
        }
      }
    }

    stream << "    # " << block.name << " node\n";
    stream << "    " << nodeName << "_node = Node(\n";
    stream << "        package='" << options.packageName << "',\n";
    stream << "        executable='" << nodeName << "_node',\n";
    stream << "        name='" << nodeName << "',\n";
    stream << "        parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],\n";

    // Add remappings if any
    if (!remappings.isEmpty()) {
      stream << "        remappings=[\n";
      for (int i = 0; i < remappings.size(); ++i) {
        stream << "            ('" << remappings[i].first << "', '" << remappings[i].second << "')";
        if (i < remappings.size() - 1) {
          stream << ",";
        }
        stream << "\n";
      }
      stream << "        ],\n";
    }

    stream << "        output='screen'\n";
    stream << "    )\n";
    stream << "    nodes.append(" << nodeName << "_node)\n";
    stream << "\n";
  }

  stream << "    return LaunchDescription([\n";
  stream << "        use_sim_time_arg,\n";
  stream << "        *nodes\n";
  stream << "    ])\n";

  return content;
}

QString CodeGenerator::generateParamsYaml(const Project& project) {
  QString content;
  QTextStream stream(&content);

  stream << "# Auto-generated parameters file\n";
  stream << "# Generated by ROS Weaver\n";
  stream << "\n";

  for (const BlockData& block : project.blocks()) {
    QString nodeName = toSnakeCase(block.name);
    stream << nodeName << ":\n";
    stream << "  ros__parameters:\n";
    stream << "    use_sim_time: false\n";

    // Group parameters by their group field
    QMap<QString, QList<const BlockParamData*>> groupedParams;
    for (const BlockParamData& param : block.parameters) {
      QString group = param.group.isEmpty() ? "" : param.group;
      groupedParams[group].append(&param);
    }

    // Output ungrouped parameters first
    if (groupedParams.contains("")) {
      for (const BlockParamData* param : groupedParams[""]) {
        writeParamYaml(stream, *param, 4);
      }
      groupedParams.remove("");
    }

    // Output grouped parameters
    for (auto it = groupedParams.begin(); it != groupedParams.end(); ++it) {
      stream << "    # " << it.key() << " parameters\n";
      for (const BlockParamData* param : it.value()) {
        writeParamYaml(stream, *param, 4);
      }
    }

    stream << "\n";
  }

  return content;
}

bool CodeGenerator::createDirectory(const QString& path) {
  QDir dir(path);
  if (!dir.exists()) {
    return dir.mkpath(".");
  }
  return true;
}

bool CodeGenerator::writeFile(const QString& path, const QString& content) {
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    return false;
  }
  QTextStream stream(&file);
  stream << content;
  file.close();
  return true;
}

QString CodeGenerator::indentCode(const QString& code, int spaces) {
  QString indent(spaces, ' ');
  QString result;
  QStringList lines = code.split('\n');
  for (const QString& line : lines) {
    result += indent + line + '\n';
  }
  return result;
}

QString CodeGenerator::toCamelCase(const QString& name) {
  QString result;
  bool capitalizeNext = true;

  for (const QChar& c : name) {
    if (c == '_' || c == '-' || c == ' ') {
      capitalizeNext = true;
    } else {
      if (capitalizeNext) {
        result += c.toUpper();
        capitalizeNext = false;
      } else {
        result += c;
      }
    }
  }

  return result;
}

QString CodeGenerator::toSnakeCase(const QString& name) {
  QString result;

  for (int i = 0; i < name.length(); ++i) {
    QChar c = name[i];
    if (c.isUpper()) {
      if (i > 0) {
        result += '_';
      }
      result += c.toLower();
    } else if (c == '-' || c == ' ') {
      result += '_';
    } else {
      result += c;
    }
  }

  return result;
}

QString CodeGenerator::getMessageInclude(const QString& messageType) {
  // Convert "std_msgs/msg/String" to "std_msgs/msg/string.hpp"
  QStringList parts = messageType.split("/");
  if (parts.size() >= 3) {
    QString package = parts[0];
    QString type = parts[1];  // "msg" or "srv"
    QString name = parts[2];

    // Convert CamelCase to snake_case for include
    QString snakeName;
    for (int i = 0; i < name.length(); ++i) {
      QChar c = name[i];
      if (c.isUpper() && i > 0) {
        snakeName += '_';
      }
      snakeName += c.toLower();
    }

    return package + "/" + type + "/" + snakeName + ".hpp";
  }
  return messageType;
}

QString CodeGenerator::getMessageType(const QString& fullType) {
  // Convert "std_msgs/msg/String" to "std_msgs::msg::String"
  QString result = fullType;
  result.replace("/", "::");
  return result;
}

void CodeGenerator::writeParamYaml(QTextStream& stream, const BlockParamData& param, int indentSpaces) {
  QString indent(indentSpaces, ' ');
  QString value = formatYamlValue(param.currentValue.isValid() ? param.currentValue : param.defaultValue, param.type);

  // Add description as comment if available
  if (!param.description.isEmpty()) {
    stream << indent << "# " << param.description << "\n";
  }

  stream << indent << param.name << ": " << value << "\n";
}

QString CodeGenerator::formatYamlValue(const QVariant& value, const QString& type) {
  if (!value.isValid()) {
    return "null";
  }

  if (type == "string") {
    QString strVal = value.toString();
    // Quote strings that contain special YAML characters or are empty
    if (strVal.isEmpty() || strVal.contains(':') || strVal.contains('#') ||
        strVal.contains('\n') || strVal.startsWith(' ') || strVal.endsWith(' ')) {
      return "\"" + strVal.replace("\"", "\\\"") + "\"";
    }
    return strVal;
  } else if (type == "bool") {
    return value.toBool() ? "true" : "false";
  } else if (type == "int") {
    return QString::number(value.toInt());
  } else if (type == "double") {
    return QString::number(value.toDouble(), 'g', 10);
  } else if (type == "array") {
    QStringList list = value.toStringList();
    if (list.isEmpty()) {
      return "[]";
    }
    QString result = "[";
    for (int i = 0; i < list.size(); ++i) {
      if (i > 0) result += ", ";
      // Try to detect if items are numeric
      bool isNumber = false;
      list[i].toDouble(&isNumber);
      if (isNumber) {
        result += list[i];
      } else {
        result += "\"" + list[i] + "\"";
      }
    }
    result += "]";
    return result;
  }

  // Default: return as string
  return value.toString();
}

}  // namespace ros_weaver
