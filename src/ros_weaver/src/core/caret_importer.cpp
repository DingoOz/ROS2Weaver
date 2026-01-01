#include "ros_weaver/core/caret_importer.hpp"
#include "ros_weaver/canvas/weaver_canvas.hpp"
#include "ros_weaver/canvas/package_block.hpp"
#include "ros_weaver/canvas/node_group.hpp"

#include <QFile>
#include <QTextStream>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

namespace ros_weaver {

CaretImporter& CaretImporter::instance() {
  static CaretImporter instance;
  return instance;
}

CaretImporter::CaretImporter() : QObject(nullptr) {
}

CaretArchitecture CaretImporter::parseFile(const QString& filePath) {
  CaretArchitecture arch;

  QFile file(filePath);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    lastError_ = QString("Could not open file: %1").arg(filePath);
    arch.errors.append(lastError_);
    emit importFailed(lastError_);
    return arch;
  }

  QTextStream in(&file);
  QString content = in.readAll();
  file.close();

  return parseString(content);
}

CaretArchitecture CaretImporter::parseString(const QString& content) {
  CaretArchitecture arch;
  emit importStarted();

  try {
    YAML::Node yaml = YAML::Load(content.toStdString());

    // Get architecture name
    if (yaml["architecture_name"]) {
      arch.name = QString::fromStdString(yaml["architecture_name"].as<std::string>());
    } else if (yaml["name"]) {
      arch.name = QString::fromStdString(yaml["name"].as<std::string>());
    }

    // Parse nodes
    if (yaml["nodes"]) {
      for (const auto& nodeEntry : yaml["nodes"]) {
        CaretNode node;

        if (nodeEntry["name"]) {
          node.name = QString::fromStdString(nodeEntry["name"].as<std::string>());
        } else if (nodeEntry["node_name"]) {
          node.name = QString::fromStdString(nodeEntry["node_name"].as<std::string>());
        }

        if (nodeEntry["executor_type"]) {
          node.executorType = QString::fromStdString(nodeEntry["executor_type"].as<std::string>());
        }

        if (nodeEntry["callback_groups"]) {
          for (const auto& cbg : nodeEntry["callback_groups"]) {
            node.callbackGroups.append(QString::fromStdString(cbg.as<std::string>()));
          }
        }

        if (nodeEntry["parameters"]) {
          for (const auto& param : nodeEntry["parameters"]) {
            QString key = QString::fromStdString(param.first.as<std::string>());
            // Simple value extraction
            if (param.second.IsScalar()) {
              node.parameters[key] = QString::fromStdString(param.second.as<std::string>());
            }
          }
        }

        if (!node.name.isEmpty()) {
          arch.nodes.append(node);
        }
      }
    }

    // Parse callbacks
    if (yaml["callbacks"]) {
      for (const auto& cbEntry : yaml["callbacks"]) {
        CaretCallback cb;

        if (cbEntry["callback_name"]) {
          cb.name = QString::fromStdString(cbEntry["callback_name"].as<std::string>());
        }
        if (cbEntry["callback_type"]) {
          cb.type = QString::fromStdString(cbEntry["callback_type"].as<std::string>());
        }
        if (cbEntry["topic_name"]) {
          cb.topicName = QString::fromStdString(cbEntry["topic_name"].as<std::string>());
        }
        if (cbEntry["service_name"]) {
          cb.serviceName = QString::fromStdString(cbEntry["service_name"].as<std::string>());
        }
        if (cbEntry["period_ns"]) {
          cb.period = cbEntry["period_ns"].as<double>() / 1000000.0;  // Convert ns to ms
        }

        if (!cb.name.isEmpty()) {
          arch.callbacks.append(cb);
        }
      }
    }

    // Parse communication paths
    if (yaml["paths"] || yaml["target_paths"]) {
      YAML::Node pathsNode = yaml["paths"] ? yaml["paths"] : yaml["target_paths"];
      for (const auto& pathEntry : pathsNode) {
        CaretPath path;

        if (pathEntry["path_name"]) {
          path.name = QString::fromStdString(pathEntry["path_name"].as<std::string>());
        }
        if (pathEntry["node_chain"]) {
          for (const auto& node : pathEntry["node_chain"]) {
            path.nodeNames.append(QString::fromStdString(node.as<std::string>()));
          }
        }
        if (pathEntry["topic_chain"]) {
          for (const auto& topic : pathEntry["topic_chain"]) {
            path.topicNames.append(QString::fromStdString(topic.as<std::string>()));
          }
        }

        if (!path.nodeNames.isEmpty()) {
          arch.paths.append(path);
        }
      }
    }

    // Parse executors
    if (yaml["executors"]) {
      for (const auto& execEntry : yaml["executors"]) {
        CaretExecutor executor;

        if (execEntry["executor_name"]) {
          executor.name = QString::fromStdString(execEntry["executor_name"].as<std::string>());
        }
        if (execEntry["executor_type"]) {
          executor.type = QString::fromStdString(execEntry["executor_type"].as<std::string>());
        }
        if (execEntry["nodes"]) {
          for (const auto& node : execEntry["nodes"]) {
            executor.nodeNames.append(QString::fromStdString(node.as<std::string>()));
          }
        }

        if (!executor.name.isEmpty()) {
          arch.executors.append(executor);
        }
      }
    }

    arch.isValid = !arch.nodes.isEmpty();

    RCLCPP_INFO(rclcpp::get_logger("ros_weaver"),
                "Parsed CARET architecture '%s': %d nodes, %d callbacks, %d paths",
                arch.name.toStdString().c_str(),
                static_cast<int>(arch.nodes.size()),
                static_cast<int>(arch.callbacks.size()),
                static_cast<int>(arch.paths.size()));

  } catch (const YAML::Exception& e) {
    lastError_ = QString("YAML parsing error: %1").arg(e.what());
    arch.errors.append(lastError_);
    emit importFailed(lastError_);
  }

  return arch;
}

int CaretImporter::importToCanvas(const CaretArchitecture& arch, WeaverCanvas* canvas) {
  if (!canvas || !arch.isValid) {
    return 0;
  }

  emit importStarted();

  QMap<QString, PackageBlock*> nodeToBlock;
  int nodesCreated = 0;
  int connectionsCreated = 0;

  // Position calculation
  double xOffset = 100.0;
  double yOffset = 100.0;
  double xSpacing = 250.0;
  double ySpacing = 180.0;
  int row = 0;
  int col = 0;
  int nodesPerRow = 4;

  // Group nodes by executor for layout
  QMap<QString, QList<QString>> executorNodes;
  for (const CaretExecutor& exec : arch.executors) {
    for (const QString& nodeName : exec.nodeNames) {
      executorNodes[exec.name].append(nodeName);
    }
  }

  // Create blocks for each node
  for (int i = 0; i < arch.nodes.size(); ++i) {
    const CaretNode& node = arch.nodes[i];

    QPointF pos(xOffset + col * xSpacing, yOffset + row * ySpacing);
    col++;
    if (col >= nodesPerRow) {
      col = 0;
      row++;
    }

    // Create the block
    QString blockName = node.name;
    if (blockName.startsWith('/')) {
      blockName = blockName.mid(1);
    }

    PackageBlock* block = canvas->addPackageBlock(blockName, pos);
    if (block) {
      nodeToBlock[node.name] = block;
      nodesCreated++;

      // Add output pins based on callbacks (timer callbacks often publish)
      for (const CaretCallback& cb : arch.callbacks) {
        if (!cb.topicName.isEmpty()) {
          // This callback uses a topic - could be pub or sub
          // For simplicity, add as output for timer callbacks
          if (cb.type == "timer_callback") {
            QString topicName = cb.topicName;
            if (topicName.startsWith('/')) topicName = topicName.mid(1);
            int lastSlash = topicName.lastIndexOf('/');
            if (lastSlash > 0) topicName = topicName.mid(lastSlash + 1);
            block->addOutputPin(topicName, Pin::DataType::Topic, "");
          }
        }
      }
    }

    emit importProgress(50 * nodesCreated / arch.nodes.size(),
                        QString("Creating node: %1").arg(blockName));
  }

  // Create connections based on paths
  for (const CaretPath& path : arch.paths) {
    for (int i = 0; i < path.nodeNames.size() - 1; ++i) {
      QString sourceName = path.nodeNames[i];
      QString targetName = path.nodeNames[i + 1];

      PackageBlock* sourceBlock = nodeToBlock.value(sourceName);
      PackageBlock* targetBlock = nodeToBlock.value(targetName);

      if (sourceBlock && targetBlock) {
        // Get topic name if available
        QString topicName;
        if (i < path.topicNames.size()) {
          topicName = path.topicNames[i];
          if (topicName.startsWith('/')) topicName = topicName.mid(1);
          int lastSlash = topicName.lastIndexOf('/');
          if (lastSlash > 0) topicName = topicName.mid(lastSlash + 1);
        }

        // Add pins if needed
        int outputPinIndex = -1;
        for (int p = 0; p < sourceBlock->outputPins().size(); ++p) {
          if (sourceBlock->outputPins()[p].name == topicName) {
            outputPinIndex = p;
            break;
          }
        }
        if (outputPinIndex < 0 && !topicName.isEmpty()) {
          sourceBlock->addOutputPin(topicName, Pin::DataType::Topic, "");
          outputPinIndex = sourceBlock->outputPins().size() - 1;
        }
        if (outputPinIndex < 0) {
          outputPinIndex = 0;
        }

        int inputPinIndex = -1;
        for (int p = 0; p < targetBlock->inputPins().size(); ++p) {
          if (targetBlock->inputPins()[p].name == topicName) {
            inputPinIndex = p;
            break;
          }
        }
        if (inputPinIndex < 0 && !topicName.isEmpty()) {
          targetBlock->addInputPin(topicName, Pin::DataType::Topic, "");
          inputPinIndex = targetBlock->inputPins().size() - 1;
        }
        if (inputPinIndex < 0) {
          inputPinIndex = 0;
        }

        if (outputPinIndex >= 0 && inputPinIndex >= 0) {
          canvas->createConnection(sourceBlock, outputPinIndex, targetBlock, inputPinIndex);
          connectionsCreated++;
        }
      }
    }
  }

  // Create groups for executors
  for (const CaretExecutor& exec : arch.executors) {
    QList<PackageBlock*> executorBlocks;
    for (const QString& nodeName : exec.nodeNames) {
      if (nodeToBlock.contains(nodeName)) {
        executorBlocks.append(nodeToBlock[nodeName]);
      }
    }

    if (executorBlocks.size() > 1) {
      // Select these blocks and create a group
      for (PackageBlock* block : executorBlocks) {
        block->setSelected(true);
      }
      NodeGroup* group = canvas->createGroupFromSelection(exec.name);
      if (group) {
        group->setSelected(false);
      }
      // Deselect all
      for (PackageBlock* block : executorBlocks) {
        block->setSelected(false);
      }
    }
  }

  emit importProgress(100, "Import complete");
  emit importCompleted(nodesCreated, connectionsCreated);

  RCLCPP_INFO(rclcpp::get_logger("ros_weaver"),
              "CARET import complete: %d nodes, %d connections",
              nodesCreated, connectionsCreated);

  return nodesCreated;
}

}  // namespace ros_weaver
