// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, ROS Weaver Contributors

#include "ros_weaver/core/mcp_providers.hpp"
#include "ros_weaver/core/mcp_manager.hpp"
#include <QProcess>
#include <QJsonArray>
#include <QJsonDocument>
#include <QFileInfo>
#include <QDirIterator>
#include <QRegularExpression>
#include <QStorageInfo>
#include <QElapsedTimer>
#include <fstream>
#include <sstream>

namespace ros_weaver {

// ============================================================================
// Factory function
// ============================================================================

std::shared_ptr<MCPServer> createMCPServer(const MCPServerConfig& config) {
    if (config.serverType == "ros_logs") {
        return std::make_shared<ROSLogsMCPServer>(config);
    } else if (config.serverType == "ros_topics") {
        return std::make_shared<ROSTopicsMCPServer>(config);
    } else if (config.serverType == "rosbag") {
        return std::make_shared<RosbagMCPServer>(config);
    } else if (config.serverType == "ros_control") {
        return std::make_shared<ROSControlMCPServer>(config);
    } else if (config.serverType == "system_stats") {
        return std::make_shared<SystemStatsMCPServer>(config);
    } else if (config.serverType == "files") {
        return std::make_shared<FilesMCPServer>(config);
    }
    return nullptr;
}

// ============================================================================
// ROSLogsMCPServer
// ============================================================================

ROSLogsMCPServer::ROSLogsMCPServer(const MCPServerConfig& config, QObject* parent)
    : MCPServer(config, parent)
{
    maxLines_ = config.settings.value("maxLines").toInt(1000);
    severityFilter_ = config.settings.value("severityFilter").toString("debug");
}

ROSLogsMCPServer::~ROSLogsMCPServer() {
    onStop();
}

QJsonObject ROSLogsMCPServer::LogEntry::toJson() const {
    QJsonObject obj;
    obj["timestamp"] = timestamp.toString(Qt::ISODate);
    obj["level"] = level;
    obj["node"] = nodeName;
    obj["message"] = message;
    if (!function.isEmpty()) obj["function"] = function;
    if (!file.isEmpty()) obj["file"] = file;
    if (line > 0) obj["line"] = line;
    return obj;
}

void ROSLogsMCPServer::onStart() {
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    rosNode_ = std::make_shared<rclcpp::Node>("ros_weaver_logs_mcp");

    subscription_ = rosNode_->create_subscription<rcl_interfaces::msg::Log>(
        "/rosout", 100,
        [this](const rcl_interfaces::msg::Log::SharedPtr msg) {
            processLogMessage(msg);
        });

    spinning_ = true;
    spinThread_ = std::make_unique<std::thread>([this]() {
        while (spinning_ && rclcpp::ok()) {
            rclcpp::spin_some(rosNode_);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
}

void ROSLogsMCPServer::onStop() {
    spinning_ = false;
    if (spinThread_ && spinThread_->joinable()) {
        spinThread_->join();
    }
    subscription_.reset();
    rosNode_.reset();
}

QString ROSLogsMCPServer::severityToString(int severity) const {
    switch (severity) {
        case rcl_interfaces::msg::Log::DEBUG: return "DEBUG";
        case rcl_interfaces::msg::Log::INFO: return "INFO";
        case rcl_interfaces::msg::Log::WARN: return "WARN";
        case rcl_interfaces::msg::Log::ERROR: return "ERROR";
        case rcl_interfaces::msg::Log::FATAL: return "FATAL";
        default: return "UNKNOWN";
    }
}

void ROSLogsMCPServer::processLogMessage(const rcl_interfaces::msg::Log::SharedPtr msg) {
    LogEntry entry;
    entry.timestamp = QDateTime::currentDateTime();
    entry.level = severityToString(msg->level);
    entry.nodeName = QString::fromStdString(msg->name);
    entry.message = QString::fromStdString(msg->msg);
    entry.function = QString::fromStdString(msg->function);
    entry.file = QString::fromStdString(msg->file);
    entry.line = msg->line;

    {
        QMutexLocker locker(&logMutex_);
        logBuffer_.push_back(entry);
        while (static_cast<int>(logBuffer_.size()) > maxLines_) {
            logBuffer_.pop_front();
        }
    }

    recordBytesReceived(entry.message.size());
}

QList<MCPTool> ROSLogsMCPServer::availableTools() const {
    QList<MCPTool> tools;

    MCPTool getLogs;
    getLogs.name = "get_logs";
    getLogs.description = "Get recent ROS 2 log messages, optionally filtered by severity or node name";
    getLogs.inputSchema = QJsonObject{
        {"type", "object"},
        {"properties", QJsonObject{
            {"count", QJsonObject{{"type", "integer"}, {"description", "Number of recent logs to return (default 50)"}}},
            {"level", QJsonObject{{"type", "string"}, {"description", "Minimum severity level: debug, info, warn, error, fatal"}}},
            {"node", QJsonObject{{"type", "string"}, {"description", "Filter by node name (substring match)"}}}
        }}
    };
    tools.append(getLogs);

    MCPTool clearLogs;
    clearLogs.name = "clear_logs";
    clearLogs.description = "Clear the log buffer";
    clearLogs.inputSchema = QJsonObject{{"type", "object"}};
    tools.append(clearLogs);

    MCPTool searchLogs;
    searchLogs.name = "search_logs";
    searchLogs.description = "Search logs for a pattern";
    searchLogs.inputSchema = QJsonObject{
        {"type", "object"},
        {"properties", QJsonObject{
            {"pattern", QJsonObject{{"type", "string"}, {"description", "Search pattern (regex supported)"}}}
        }},
        {"required", QJsonArray{"pattern"}}
    };
    tools.append(searchLogs);

    return tools;
}

QList<MCPResource> ROSLogsMCPServer::availableResources() const {
    QList<MCPResource> resources;

    MCPResource recentLogs;
    recentLogs.uri = "logs/recent";
    recentLogs.name = "Recent Logs";
    recentLogs.description = "Most recent ROS 2 log entries";
    recentLogs.mimeType = "application/json";
    resources.append(recentLogs);

    return resources;
}

MCPToolResult ROSLogsMCPServer::callTool(const QString& toolName, const QJsonObject& params) {
    MCPToolResult result;
    recordToolCall();
    QElapsedTimer timer;
    timer.start();

    if (toolName == "get_logs") {
        int count = params.value("count").toInt(50);
        QString level = params.value("level").toString().toUpper();
        QString nodeFilter = params.value("node").toString();

        // Severity level mapping
        QMap<QString, int> severityOrder = {
            {"DEBUG", 0}, {"INFO", 1}, {"WARN", 2}, {"ERROR", 3}, {"FATAL", 4}
        };
        int minSeverity = severityOrder.value(level, 0);

        QJsonArray logsArray;
        {
            QMutexLocker locker(&logMutex_);
            int added = 0;
            for (auto it = logBuffer_.rbegin(); it != logBuffer_.rend() && added < count; ++it) {
                int logSeverity = severityOrder.value(it->level, 0);
                if (logSeverity >= minSeverity) {
                    if (nodeFilter.isEmpty() || it->nodeName.contains(nodeFilter, Qt::CaseInsensitive)) {
                        logsArray.append(it->toJson());
                        added++;
                    }
                }
            }
        }

        result.success = true;
        result.data["logs"] = logsArray;
        result.data["count"] = logsArray.size();
        result.message = QString("Retrieved %1 log entries").arg(logsArray.size());

    } else if (toolName == "clear_logs") {
        QMutexLocker locker(&logMutex_);
        logBuffer_.clear();
        result.success = true;
        result.message = "Log buffer cleared";

    } else if (toolName == "search_logs") {
        QString pattern = params.value("pattern").toString();
        if (pattern.isEmpty()) {
            result.success = false;
            result.message = "Pattern is required";
        } else {
            QRegularExpression regex(pattern, QRegularExpression::CaseInsensitiveOption);
            QJsonArray matchesArray;

            {
                QMutexLocker locker(&logMutex_);
                for (const auto& entry : logBuffer_) {
                    if (regex.match(entry.message).hasMatch() ||
                        regex.match(entry.nodeName).hasMatch()) {
                        matchesArray.append(entry.toJson());
                    }
                }
            }

            result.success = true;
            result.data["matches"] = matchesArray;
            result.data["count"] = matchesArray.size();
            result.message = QString("Found %1 matching log entries").arg(matchesArray.size());
        }

    } else {
        result.success = false;
        result.message = QString("Unknown tool: %1").arg(toolName);
    }

    result.executionTimeMs = timer.elapsed();
    recordBytesSent(QJsonDocument(result.data).toJson().size());
    return result;
}

MCPResourceContent ROSLogsMCPServer::readResource(const QString& uri) {
    MCPResourceContent content;
    content.uri = uri;
    recordResourceRead();

    if (uri == "logs/recent") {
        QJsonArray logsArray;
        {
            QMutexLocker locker(&logMutex_);
            int count = 0;
            for (auto it = logBuffer_.rbegin(); it != logBuffer_.rend() && count < 100; ++it, ++count) {
                logsArray.append(it->toJson());
            }
        }

        QJsonObject obj;
        obj["logs"] = logsArray;
        content.textContent = QJsonDocument(obj).toJson(QJsonDocument::Indented);
        content.mimeType = "application/json";
    } else {
        content.textContent = QString("Unknown resource: %1").arg(uri);
    }

    recordBytesSent(content.textContent.size());
    return content;
}

// ============================================================================
// ROSTopicsMCPServer
// ============================================================================

ROSTopicsMCPServer::ROSTopicsMCPServer(const MCPServerConfig& config, QObject* parent)
    : MCPServer(config, parent)
{
}

ROSTopicsMCPServer::~ROSTopicsMCPServer() {
    onStop();
}

void ROSTopicsMCPServer::onStart() {
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    rosNode_ = std::make_shared<rclcpp::Node>("ros_weaver_topics_mcp");

    spinning_ = true;
    spinThread_ = std::make_unique<std::thread>([this]() {
        while (spinning_ && rclcpp::ok()) {
            rclcpp::spin_some(rosNode_);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });
}

void ROSTopicsMCPServer::onStop() {
    spinning_ = false;
    if (spinThread_ && spinThread_->joinable()) {
        spinThread_->join();
    }
    rosNode_.reset();
}

QString ROSTopicsMCPServer::runRosCommand(const QStringList& args) {
    QProcess process;
    process.start("ros2", args);
    process.waitForFinished(5000);
    return process.readAllStandardOutput();
}

QList<MCPTool> ROSTopicsMCPServer::availableTools() const {
    QList<MCPTool> tools;

    MCPTool listTopics;
    listTopics.name = "list_topics";
    listTopics.description = "List all active ROS 2 topics with their message types";
    listTopics.inputSchema = QJsonObject{{"type", "object"}};
    tools.append(listTopics);

    MCPTool topicInfo;
    topicInfo.name = "topic_info";
    topicInfo.description = "Get detailed information about a specific topic";
    topicInfo.inputSchema = QJsonObject{
        {"type", "object"},
        {"properties", QJsonObject{
            {"topic", QJsonObject{{"type", "string"}, {"description", "Topic name"}}}
        }},
        {"required", QJsonArray{"topic"}}
    };
    tools.append(topicInfo);

    MCPTool echoTopic;
    echoTopic.name = "echo_topic";
    echoTopic.description = "Echo messages from a topic";
    echoTopic.inputSchema = QJsonObject{
        {"type", "object"},
        {"properties", QJsonObject{
            {"topic", QJsonObject{{"type", "string"}, {"description", "Topic name"}}},
            {"count", QJsonObject{{"type", "integer"}, {"description", "Number of messages to capture (default 1)"}}}
        }},
        {"required", QJsonArray{"topic"}}
    };
    tools.append(echoTopic);

    MCPTool listNodes;
    listNodes.name = "list_nodes";
    listNodes.description = "List all active ROS 2 nodes";
    listNodes.inputSchema = QJsonObject{{"type", "object"}};
    tools.append(listNodes);

    MCPTool nodeInfo;
    nodeInfo.name = "node_info";
    nodeInfo.description = "Get detailed information about a specific node";
    nodeInfo.inputSchema = QJsonObject{
        {"type", "object"},
        {"properties", QJsonObject{
            {"node", QJsonObject{{"type", "string"}, {"description", "Node name"}}}
        }},
        {"required", QJsonArray{"node"}}
    };
    tools.append(nodeInfo);

    return tools;
}

QList<MCPResource> ROSTopicsMCPServer::availableResources() const {
    QList<MCPResource> resources;

    MCPResource topicList;
    topicList.uri = "topics/list";
    topicList.name = "Topic List";
    topicList.description = "Current list of all ROS 2 topics";
    topicList.mimeType = "application/json";
    resources.append(topicList);

    MCPResource nodeList;
    nodeList.uri = "nodes/list";
    nodeList.name = "Node List";
    nodeList.description = "Current list of all ROS 2 nodes";
    nodeList.mimeType = "application/json";
    resources.append(nodeList);

    return resources;
}

MCPToolResult ROSTopicsMCPServer::listTopics() {
    MCPToolResult result;

    if (!rosNode_) {
        result.success = false;
        result.message = "ROS node not initialized";
        return result;
    }

    auto topicNamesAndTypes = rosNode_->get_topic_names_and_types();

    QJsonArray topicsArray;
    for (const auto& [name, types] : topicNamesAndTypes) {
        QJsonObject topicObj;
        topicObj["name"] = QString::fromStdString(name);
        QJsonArray typesArray;
        for (const auto& type : types) {
            typesArray.append(QString::fromStdString(type));
        }
        topicObj["types"] = typesArray;
        topicsArray.append(topicObj);
    }

    result.success = true;
    result.data["topics"] = topicsArray;
    result.data["count"] = topicsArray.size();
    result.message = QString("Found %1 topics").arg(topicsArray.size());

    return result;
}

MCPToolResult ROSTopicsMCPServer::topicInfo(const QString& topicName) {
    MCPToolResult result;

    QString output = runRosCommand({"topic", "info", topicName, "-v"});
    recordBytesReceived(output.size());

    if (output.isEmpty()) {
        result.success = false;
        result.message = QString("Topic '%1' not found or no info available").arg(topicName);
    } else {
        result.success = true;
        result.data["topic"] = topicName;
        result.data["info"] = output;
        result.message = QString("Retrieved info for topic %1").arg(topicName);
    }

    return result;
}

MCPToolResult ROSTopicsMCPServer::echoTopic(const QString& topicName, int count) {
    MCPToolResult result;

    QProcess process;
    process.start("ros2", {"topic", "echo", topicName, "--once"});

    QStringList messages;
    for (int i = 0; i < count && process.waitForReadyRead(2000); ++i) {
        QString msg = process.readAllStandardOutput();
        if (!msg.isEmpty()) {
            messages.append(msg);
        }
    }
    process.kill();

    if (messages.isEmpty()) {
        // Try to get at least one message with timeout
        process.start("ros2", {"topic", "echo", topicName, "--once"});
        if (process.waitForFinished(3000)) {
            QString msg = process.readAllStandardOutput();
            if (!msg.isEmpty()) {
                messages.append(msg);
            }
        }
    }

    if (messages.isEmpty()) {
        result.success = false;
        result.message = QString("No messages received from topic '%1'").arg(topicName);
    } else {
        result.success = true;
        QJsonArray msgArray;
        for (const QString& msg : messages) {
            msgArray.append(msg);
        }
        result.data["topic"] = topicName;
        result.data["messages"] = msgArray;
        result.data["count"] = messages.size();
        result.message = QString("Captured %1 messages from %2").arg(messages.size()).arg(topicName);

        int totalSize = 0;
        for (const QString& msg : messages) totalSize += msg.size();
        recordBytesReceived(totalSize);
    }

    return result;
}

MCPToolResult ROSTopicsMCPServer::listNodes() {
    MCPToolResult result;

    if (!rosNode_) {
        result.success = false;
        result.message = "ROS node not initialized";
        return result;
    }

    auto nodeNames = rosNode_->get_node_names();

    QJsonArray nodesArray;
    for (const auto& name : nodeNames) {
        nodesArray.append(QString::fromStdString(name));
    }

    result.success = true;
    result.data["nodes"] = nodesArray;
    result.data["count"] = nodesArray.size();
    result.message = QString("Found %1 nodes").arg(nodesArray.size());

    return result;
}

MCPToolResult ROSTopicsMCPServer::nodeInfo(const QString& nodeName) {
    MCPToolResult result;

    QString output = runRosCommand({"node", "info", nodeName});
    recordBytesReceived(output.size());

    if (output.isEmpty()) {
        result.success = false;
        result.message = QString("Node '%1' not found or no info available").arg(nodeName);
    } else {
        result.success = true;
        result.data["node"] = nodeName;
        result.data["info"] = output;
        result.message = QString("Retrieved info for node %1").arg(nodeName);
    }

    return result;
}

MCPToolResult ROSTopicsMCPServer::callTool(const QString& toolName, const QJsonObject& params) {
    MCPToolResult result;
    recordToolCall();
    QElapsedTimer timer;
    timer.start();

    if (toolName == "list_topics") {
        result = listTopics();
    } else if (toolName == "topic_info") {
        result = topicInfo(params.value("topic").toString());
    } else if (toolName == "echo_topic") {
        result = echoTopic(params.value("topic").toString(), params.value("count").toInt(1));
    } else if (toolName == "list_nodes") {
        result = listNodes();
    } else if (toolName == "node_info") {
        result = nodeInfo(params.value("node").toString());
    } else {
        result.success = false;
        result.message = QString("Unknown tool: %1").arg(toolName);
    }

    result.executionTimeMs = timer.elapsed();
    recordBytesSent(QJsonDocument(result.data).toJson().size());
    return result;
}

MCPResourceContent ROSTopicsMCPServer::readResource(const QString& uri) {
    MCPResourceContent content;
    content.uri = uri;
    content.mimeType = "application/json";
    recordResourceRead();

    if (uri == "topics/list") {
        auto result = listTopics();
        content.textContent = QJsonDocument(result.data).toJson(QJsonDocument::Indented);
    } else if (uri == "nodes/list") {
        auto result = listNodes();
        content.textContent = QJsonDocument(result.data).toJson(QJsonDocument::Indented);
    } else {
        content.textContent = QString("Unknown resource: %1").arg(uri);
    }

    recordBytesSent(content.textContent.size());
    return content;
}

// ============================================================================
// RosbagMCPServer
// ============================================================================

RosbagMCPServer::RosbagMCPServer(const MCPServerConfig& config, QObject* parent)
    : MCPServer(config, parent)
{
}

RosbagMCPServer::~RosbagMCPServer() {
    onStop();
}

void RosbagMCPServer::onStart() {
    // No persistent resources needed
}

void RosbagMCPServer::onStop() {
    // Cleanup
}

void RosbagMCPServer::setRecordingState(bool recording, const QString& bagPath) {
    isRecording_ = recording;
    currentBagPath_ = bagPath;
    if (recording) {
        recordingStartTime_ = QDateTime::currentDateTime();
    }
}

QList<MCPTool> RosbagMCPServer::availableTools() const {
    QList<MCPTool> tools;

    MCPTool getStatus;
    getStatus.name = "get_recording_status";
    getStatus.description = "Get current rosbag recording status";
    getStatus.inputSchema = QJsonObject{{"type", "object"}};
    tools.append(getStatus);

    MCPTool getBagInfo;
    getBagInfo.name = "get_bag_info";
    getBagInfo.description = "Get metadata about a rosbag file";
    getBagInfo.inputSchema = QJsonObject{
        {"type", "object"},
        {"properties", QJsonObject{
            {"path", QJsonObject{{"type", "string"}, {"description", "Path to the bag file/directory"}}}
        }},
        {"required", QJsonArray{"path"}}
    };
    tools.append(getBagInfo);

    MCPTool listBags;
    listBags.name = "list_bags";
    listBags.description = "List rosbag files in a directory";
    listBags.inputSchema = QJsonObject{
        {"type", "object"},
        {"properties", QJsonObject{
            {"directory", QJsonObject{{"type", "string"}, {"description", "Directory to search for bags"}}}
        }}
    };
    tools.append(listBags);

    return tools;
}

QList<MCPResource> RosbagMCPServer::availableResources() const {
    QList<MCPResource> resources;

    MCPResource status;
    status.uri = "recording/status";
    status.name = "Recording Status";
    status.description = "Current rosbag recording status";
    status.mimeType = "application/json";
    resources.append(status);

    return resources;
}

MCPToolResult RosbagMCPServer::getRecordingStatus() {
    MCPToolResult result;
    result.success = true;

    result.data["recording"] = isRecording_;
    if (isRecording_) {
        result.data["bagPath"] = currentBagPath_;
        result.data["startTime"] = recordingStartTime_.toString(Qt::ISODate);
        result.data["duration"] = recordingStartTime_.secsTo(QDateTime::currentDateTime());
        if (!recordingTopics_.isEmpty()) {
            QJsonArray topics;
            for (const QString& t : recordingTopics_) topics.append(t);
            result.data["topics"] = topics;
        }
        result.message = "Recording in progress";
    } else {
        result.message = "Not recording";
    }

    return result;
}

MCPToolResult RosbagMCPServer::getBagInfo(const QString& bagPath) {
    MCPToolResult result;

    QProcess process;
    process.start("ros2", {"bag", "info", bagPath});
    process.waitForFinished(10000);

    QString output = process.readAllStandardOutput();
    QString error = process.readAllStandardError();

    if (!error.isEmpty() || output.isEmpty()) {
        result.success = false;
        result.message = error.isEmpty() ? "Could not read bag info" : error;
    } else {
        result.success = true;
        result.data["path"] = bagPath;
        result.data["info"] = output;
        result.message = "Retrieved bag info";
        recordBytesReceived(output.size());
    }

    return result;
}

MCPToolResult RosbagMCPServer::listBagsInDirectory(const QString& directory) {
    MCPToolResult result;

    QString dir = directory.isEmpty() ? QDir::currentPath() : directory;
    QDir qdir(dir);

    if (!qdir.exists()) {
        result.success = false;
        result.message = QString("Directory does not exist: %1").arg(dir);
        return result;
    }

    QJsonArray bagsArray;

    // Look for directories with metadata.yaml (rosbag2 format)
    QDirIterator it(dir, QDir::Dirs | QDir::NoDotAndDotDot);
    while (it.hasNext()) {
        QString path = it.next();
        if (QFile::exists(path + "/metadata.yaml")) {
            QJsonObject bagObj;
            bagObj["path"] = path;
            bagObj["name"] = QFileInfo(path).fileName();

            // Get size
            qint64 size = 0;
            QDirIterator sizeIt(path, QDir::Files);
            while (sizeIt.hasNext()) {
                sizeIt.next();
                size += QFileInfo(sizeIt.filePath()).size();
            }
            bagObj["sizeBytes"] = size;

            bagsArray.append(bagObj);
        }
    }

    result.success = true;
    result.data["directory"] = dir;
    result.data["bags"] = bagsArray;
    result.data["count"] = bagsArray.size();
    result.message = QString("Found %1 bags in %2").arg(bagsArray.size()).arg(dir);

    return result;
}

MCPToolResult RosbagMCPServer::callTool(const QString& toolName, const QJsonObject& params) {
    MCPToolResult result;
    recordToolCall();
    QElapsedTimer timer;
    timer.start();

    if (toolName == "get_recording_status") {
        result = getRecordingStatus();
    } else if (toolName == "get_bag_info") {
        result = getBagInfo(params.value("path").toString());
    } else if (toolName == "list_bags") {
        result = listBagsInDirectory(params.value("directory").toString());
    } else {
        result.success = false;
        result.message = QString("Unknown tool: %1").arg(toolName);
    }

    result.executionTimeMs = timer.elapsed();
    recordBytesSent(QJsonDocument(result.data).toJson().size());
    return result;
}

MCPResourceContent RosbagMCPServer::readResource(const QString& uri) {
    MCPResourceContent content;
    content.uri = uri;
    content.mimeType = "application/json";
    recordResourceRead();

    if (uri == "recording/status") {
        auto result = getRecordingStatus();
        content.textContent = QJsonDocument(result.data).toJson(QJsonDocument::Indented);
    } else {
        content.textContent = QString("Unknown resource: %1").arg(uri);
    }

    recordBytesSent(content.textContent.size());
    return content;
}

// ============================================================================
// ROSControlMCPServer
// ============================================================================

ROSControlMCPServer::ROSControlMCPServer(const MCPServerConfig& config, QObject* parent)
    : MCPServer(config, parent)
{
    controllerManagerName_ = config.settings.value("controllerManager").toString("/controller_manager");
}

ROSControlMCPServer::~ROSControlMCPServer() {
    onStop();
}

void ROSControlMCPServer::onStart() {
    // No persistent resources needed
}

void ROSControlMCPServer::onStop() {
    // Cleanup
}

QString ROSControlMCPServer::runRosControlCommand(const QStringList& args) {
    QProcess process;
    QStringList fullArgs = {"control"};
    fullArgs.append(args);
    if (!controllerManagerName_.isEmpty() && controllerManagerName_ != "/controller_manager") {
        fullArgs.append("-c");
        fullArgs.append(controllerManagerName_);
    }

    process.start("ros2", fullArgs);
    process.waitForFinished(5000);
    return process.readAllStandardOutput();
}

QList<MCPTool> ROSControlMCPServer::availableTools() const {
    QList<MCPTool> tools;

    MCPTool listControllers;
    listControllers.name = "list_controllers";
    listControllers.description = "List all loaded ros2_control controllers and their states";
    listControllers.inputSchema = QJsonObject{{"type", "object"}};
    tools.append(listControllers);

    MCPTool controllerInfo;
    controllerInfo.name = "controller_info";
    controllerInfo.description = "Get detailed information about a specific controller";
    controllerInfo.inputSchema = QJsonObject{
        {"type", "object"},
        {"properties", QJsonObject{
            {"name", QJsonObject{{"type", "string"}, {"description", "Controller name"}}}
        }},
        {"required", QJsonArray{"name"}}
    };
    tools.append(controllerInfo);

    MCPTool listHWInterfaces;
    listHWInterfaces.name = "list_hardware_interfaces";
    listHWInterfaces.description = "List all hardware interfaces";
    listHWInterfaces.inputSchema = QJsonObject{{"type", "object"}};
    tools.append(listHWInterfaces);

    MCPTool switchController;
    switchController.name = "switch_controller";
    switchController.description = "Activate or deactivate controllers";
    switchController.inputSchema = QJsonObject{
        {"type", "object"},
        {"properties", QJsonObject{
            {"activate", QJsonObject{{"type", "string"}, {"description", "Controller to activate"}}},
            {"deactivate", QJsonObject{{"type", "string"}, {"description", "Controller to deactivate"}}}
        }}
    };
    tools.append(switchController);

    return tools;
}

QList<MCPResource> ROSControlMCPServer::availableResources() const {
    QList<MCPResource> resources;

    MCPResource controllers;
    controllers.uri = "controllers/list";
    controllers.name = "Controller List";
    controllers.description = "Current ros2_control controllers";
    controllers.mimeType = "application/json";
    resources.append(controllers);

    MCPResource hwInterfaces;
    hwInterfaces.uri = "hardware/interfaces";
    hwInterfaces.name = "Hardware Interfaces";
    hwInterfaces.description = "Available hardware interfaces";
    hwInterfaces.mimeType = "application/json";
    resources.append(hwInterfaces);

    return resources;
}

MCPToolResult ROSControlMCPServer::listControllers() {
    MCPToolResult result;

    QString output = runRosControlCommand({"list_controllers"});
    recordBytesReceived(output.size());

    if (output.isEmpty()) {
        result.success = false;
        result.message = "No controller manager found or no controllers loaded";
    } else {
        result.success = true;
        result.data["output"] = output;

        // Parse output into structured data
        QJsonArray controllersArray;
        QStringList lines = output.split('\n', Qt::SkipEmptyParts);
        for (const QString& line : lines) {
            QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
            if (parts.size() >= 2) {
                QJsonObject ctrl;
                ctrl["name"] = parts[0];
                ctrl["type"] = parts.size() > 1 ? parts[1] : "";
                ctrl["state"] = parts.size() > 2 ? parts[2] : "";
                controllersArray.append(ctrl);
            }
        }
        result.data["controllers"] = controllersArray;
        result.data["count"] = controllersArray.size();
        result.message = QString("Found %1 controllers").arg(controllersArray.size());
    }

    return result;
}

MCPToolResult ROSControlMCPServer::controllerInfo(const QString& controllerName) {
    MCPToolResult result;

    // There's no direct "controller info" command, so we list and filter
    QString output = runRosControlCommand({"list_controllers", "-v"});
    recordBytesReceived(output.size());

    if (output.contains(controllerName)) {
        result.success = true;
        result.data["name"] = controllerName;
        result.data["info"] = output;
        result.message = QString("Retrieved info for controller %1").arg(controllerName);
    } else {
        result.success = false;
        result.message = QString("Controller '%1' not found").arg(controllerName);
    }

    return result;
}

MCPToolResult ROSControlMCPServer::listHardwareInterfaces() {
    MCPToolResult result;

    QString output = runRosControlCommand({"list_hardware_interfaces"});
    recordBytesReceived(output.size());

    if (output.isEmpty()) {
        result.success = false;
        result.message = "No hardware interfaces found";
    } else {
        result.success = true;
        result.data["output"] = output;
        result.message = "Retrieved hardware interfaces";
    }

    return result;
}

MCPToolResult ROSControlMCPServer::switchController(const QString& activate, const QString& deactivate) {
    MCPToolResult result;

    QStringList args = {"switch_controllers"};
    if (!activate.isEmpty()) {
        args << "--activate" << activate;
    }
    if (!deactivate.isEmpty()) {
        args << "--deactivate" << deactivate;
    }

    QString output = runRosControlCommand(args);

    result.success = true;  // If command ran
    result.data["output"] = output;
    result.message = "Controller switch command executed";

    return result;
}

MCPToolResult ROSControlMCPServer::callTool(const QString& toolName, const QJsonObject& params) {
    MCPToolResult result;
    recordToolCall();
    QElapsedTimer timer;
    timer.start();

    if (toolName == "list_controllers") {
        result = listControllers();
    } else if (toolName == "controller_info") {
        result = controllerInfo(params.value("name").toString());
    } else if (toolName == "list_hardware_interfaces") {
        result = listHardwareInterfaces();
    } else if (toolName == "switch_controller") {
        result = switchController(params.value("activate").toString(),
                                 params.value("deactivate").toString());
    } else {
        result.success = false;
        result.message = QString("Unknown tool: %1").arg(toolName);
    }

    result.executionTimeMs = timer.elapsed();
    recordBytesSent(QJsonDocument(result.data).toJson().size());
    return result;
}

MCPResourceContent ROSControlMCPServer::readResource(const QString& uri) {
    MCPResourceContent content;
    content.uri = uri;
    content.mimeType = "application/json";
    recordResourceRead();

    if (uri == "controllers/list") {
        auto result = listControllers();
        content.textContent = QJsonDocument(result.data).toJson(QJsonDocument::Indented);
    } else if (uri == "hardware/interfaces") {
        auto result = listHardwareInterfaces();
        content.textContent = QJsonDocument(result.data).toJson(QJsonDocument::Indented);
    } else {
        content.textContent = QString("Unknown resource: %1").arg(uri);
    }

    recordBytesSent(content.textContent.size());
    return content;
}

// ============================================================================
// SystemStatsMCPServer
// ============================================================================

SystemStatsMCPServer::SystemStatsMCPServer(const MCPServerConfig& config, QObject* parent)
    : MCPServer(config, parent)
    , updateTimer_(new QTimer(this))
{
    updateIntervalMs_ = config.settings.value("updateIntervalMs").toInt(1000);
    includeTemperatures_ = config.settings.value("includeTemperatures").toBool(true);
}

SystemStatsMCPServer::~SystemStatsMCPServer() {
    onStop();
}

void SystemStatsMCPServer::onStart() {
    // Initialize CPU tracking
    readCPUStats();
}

void SystemStatsMCPServer::onStop() {
    updateTimer_->stop();
}

SystemStatsMCPServer::CPUStats SystemStatsMCPServer::readCPUStats() {
    CPUStats stats;
    stats.coreCount = std::thread::hardware_concurrency();

    // Read from /proc/stat
    std::ifstream statFile("/proc/stat");
    if (statFile.is_open()) {
        std::string line;
        std::getline(statFile, line);  // First line is total CPU

        // Parse: cpu user nice system idle iowait irq softirq
        std::istringstream iss(line);
        std::string cpu;
        qint64 user, nice, system, idle, iowait, irq, softirq, steal = 0;
        iss >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

        qint64 idleTime = idle + iowait;
        qint64 totalTime = user + nice + system + idle + iowait + irq + softirq + steal;

        if (prevTotalTime_ > 0) {
            qint64 deltaIdle = idleTime - prevIdleTime_;
            qint64 deltaTotal = totalTime - prevTotalTime_;
            if (deltaTotal > 0) {
                stats.usagePercent = 100.0 * (1.0 - (double)deltaIdle / deltaTotal);
            }
        }

        prevIdleTime_ = idleTime;
        prevTotalTime_ = totalTime;

        // Read per-core stats
        while (std::getline(statFile, line)) {
            if (line.substr(0, 3) == "cpu" && line[3] != ' ') {
                std::istringstream coreIss(line);
                coreIss >> cpu >> user >> nice >> system >> idle;
                // Simplified per-core usage (would need tracking like total)
                double coreUsage = 100.0 * (user + nice + system) / (user + nice + system + idle + 1);
                stats.perCoreUsage.append(coreUsage);
            } else {
                break;
            }
        }
    }

    // Read load average
    std::ifstream loadFile("/proc/loadavg");
    if (loadFile.is_open()) {
        loadFile >> stats.loadAverage1 >> stats.loadAverage5 >> stats.loadAverage15;
    }

    return stats;
}

SystemStatsMCPServer::MemoryStats SystemStatsMCPServer::readMemoryStats() {
    MemoryStats stats = {};

    std::ifstream memFile("/proc/meminfo");
    if (memFile.is_open()) {
        std::string line;
        while (std::getline(memFile, line)) {
            std::string key;
            qint64 value;
            std::istringstream iss(line);
            iss >> key >> value;

            // Remove colon from key
            if (!key.empty() && key.back() == ':') {
                key.pop_back();
            }

            // Values are in kB
            value *= 1024;

            if (key == "MemTotal") stats.totalBytes = value;
            else if (key == "MemAvailable") stats.availableBytes = value;
            else if (key == "SwapTotal") stats.swapTotalBytes = value;
            else if (key == "SwapFree") stats.swapUsedBytes = stats.swapTotalBytes - value;
        }
        stats.usedBytes = stats.totalBytes - stats.availableBytes;
    }

    return stats;
}

QList<SystemStatsMCPServer::DiskStats> SystemStatsMCPServer::readDiskStats() {
    QList<DiskStats> diskList;

    for (const QStorageInfo& storage : QStorageInfo::mountedVolumes()) {
        if (storage.isValid() && storage.isReady()) {
            // Skip virtual filesystems
            QString fsType = QString::fromUtf8(storage.fileSystemType());
            if (fsType.startsWith("tmp") || fsType.startsWith("devtmp") ||
                fsType.startsWith("overlay") || fsType.startsWith("squash")) {
                continue;
            }

            DiskStats disk;
            disk.mountPoint = storage.rootPath();
            disk.filesystem = fsType;
            disk.totalBytes = storage.bytesTotal();
            disk.availableBytes = storage.bytesAvailable();
            disk.usedBytes = disk.totalBytes - disk.availableBytes;
            disk.usagePercent = disk.totalBytes > 0 ?
                100.0 * disk.usedBytes / disk.totalBytes : 0;

            diskList.append(disk);
        }
    }

    return diskList;
}

QList<SystemStatsMCPServer::TempReading> SystemStatsMCPServer::readTemperatures() {
    QList<TempReading> temps;

    // Read from /sys/class/thermal
    QDir thermalDir("/sys/class/thermal");
    QStringList zones = thermalDir.entryList(QStringList("thermal_zone*"), QDir::Dirs);

    for (const QString& zone : zones) {
        QString basePath = "/sys/class/thermal/" + zone;

        TempReading reading;
        reading.sensor = zone;

        // Read type
        QFile typeFile(basePath + "/type");
        if (typeFile.open(QIODevice::ReadOnly)) {
            reading.label = typeFile.readAll().trimmed();
            typeFile.close();
        }

        // Read temp (in millidegrees)
        QFile tempFile(basePath + "/temp");
        if (tempFile.open(QIODevice::ReadOnly)) {
            bool ok;
            int milliDegrees = tempFile.readAll().trimmed().toInt(&ok);
            if (ok) {
                reading.temperatureCelsius = milliDegrees / 1000.0;
            }
            tempFile.close();
        }

        temps.append(reading);
    }

    // Also try hwmon sensors
    QDir hwmonDir("/sys/class/hwmon");
    QStringList hwmons = hwmonDir.entryList(QStringList("hwmon*"), QDir::Dirs);

    for (const QString& hwmon : hwmons) {
        QString basePath = "/sys/class/hwmon/" + hwmon;

        // Find temp inputs
        QDir hwmonSubDir(basePath);
        QStringList tempFiles = hwmonSubDir.entryList(QStringList("temp*_input"), QDir::Files);

        for (const QString& tempFile : tempFiles) {
            QString prefix = tempFile.left(tempFile.indexOf("_input"));

            TempReading reading;
            reading.sensor = hwmon + "/" + prefix;

            // Read label
            QFile labelFile(basePath + "/" + prefix + "_label");
            if (labelFile.open(QIODevice::ReadOnly)) {
                reading.label = labelFile.readAll().trimmed();
                labelFile.close();
            }

            // Read temp
            QFile tFile(basePath + "/" + tempFile);
            if (tFile.open(QIODevice::ReadOnly)) {
                bool ok;
                int milliDegrees = tFile.readAll().trimmed().toInt(&ok);
                if (ok) {
                    reading.temperatureCelsius = milliDegrees / 1000.0;
                }
                tFile.close();
            }

            // Read critical temp
            QFile critFile(basePath + "/" + prefix + "_crit");
            if (critFile.open(QIODevice::ReadOnly)) {
                bool ok;
                int milliDegrees = critFile.readAll().trimmed().toInt(&ok);
                if (ok) {
                    reading.criticalTemp = milliDegrees / 1000.0;
                }
                critFile.close();
            }

            temps.append(reading);
        }
    }

    return temps;
}

MCPToolResult SystemStatsMCPServer::getCPUStats() {
    MCPToolResult result;
    auto stats = readCPUStats();

    result.success = true;
    result.data["usagePercent"] = stats.usagePercent;
    result.data["coreCount"] = stats.coreCount;

    QJsonArray perCore;
    for (double usage : stats.perCoreUsage) {
        perCore.append(usage);
    }
    result.data["perCoreUsage"] = perCore;

    result.data["loadAverage1"] = stats.loadAverage1;
    result.data["loadAverage5"] = stats.loadAverage5;
    result.data["loadAverage15"] = stats.loadAverage15;

    result.message = QString("CPU usage: %.1f%%").arg(stats.usagePercent);

    return result;
}

MCPToolResult SystemStatsMCPServer::getMemoryStats() {
    MCPToolResult result;
    auto stats = readMemoryStats();

    result.success = true;
    result.data["totalBytes"] = stats.totalBytes;
    result.data["usedBytes"] = stats.usedBytes;
    result.data["availableBytes"] = stats.availableBytes;
    result.data["usagePercent"] = stats.totalBytes > 0 ?
        100.0 * stats.usedBytes / stats.totalBytes : 0;
    result.data["swapTotalBytes"] = stats.swapTotalBytes;
    result.data["swapUsedBytes"] = stats.swapUsedBytes;
    result.data["swapUsagePercent"] = stats.swapTotalBytes > 0 ?
        100.0 * stats.swapUsedBytes / stats.swapTotalBytes : 0;

    // Human readable
    result.data["totalGB"] = stats.totalBytes / (1024.0 * 1024.0 * 1024.0);
    result.data["usedGB"] = stats.usedBytes / (1024.0 * 1024.0 * 1024.0);
    result.data["availableGB"] = stats.availableBytes / (1024.0 * 1024.0 * 1024.0);

    result.message = QString("Memory: %.1f GB / %.1f GB (%.1f%% used)")
        .arg(stats.usedBytes / (1024.0 * 1024.0 * 1024.0))
        .arg(stats.totalBytes / (1024.0 * 1024.0 * 1024.0))
        .arg(100.0 * stats.usedBytes / stats.totalBytes);

    return result;
}

MCPToolResult SystemStatsMCPServer::getDiskStats() {
    MCPToolResult result;
    auto diskList = readDiskStats();

    result.success = true;

    QJsonArray disksArray;
    for (const auto& disk : diskList) {
        QJsonObject diskObj;
        diskObj["mountPoint"] = disk.mountPoint;
        diskObj["filesystem"] = disk.filesystem;
        diskObj["totalBytes"] = disk.totalBytes;
        diskObj["usedBytes"] = disk.usedBytes;
        diskObj["availableBytes"] = disk.availableBytes;
        diskObj["usagePercent"] = disk.usagePercent;
        diskObj["totalGB"] = disk.totalBytes / (1024.0 * 1024.0 * 1024.0);
        diskObj["availableGB"] = disk.availableBytes / (1024.0 * 1024.0 * 1024.0);
        disksArray.append(diskObj);
    }

    result.data["disks"] = disksArray;
    result.data["count"] = disksArray.size();
    result.message = QString("Found %1 mounted filesystems").arg(diskList.size());

    return result;
}

MCPToolResult SystemStatsMCPServer::getTemperatures() {
    MCPToolResult result;
    auto temps = readTemperatures();

    result.success = true;

    QJsonArray tempsArray;
    for (const auto& temp : temps) {
        QJsonObject tempObj;
        tempObj["sensor"] = temp.sensor;
        tempObj["label"] = temp.label;
        tempObj["temperatureCelsius"] = temp.temperatureCelsius;
        if (temp.criticalTemp > 0) {
            tempObj["criticalCelsius"] = temp.criticalTemp;
        }
        tempsArray.append(tempObj);
    }

    result.data["temperatures"] = tempsArray;
    result.data["count"] = tempsArray.size();
    result.message = QString("Found %1 temperature sensors").arg(temps.size());

    return result;
}

MCPToolResult SystemStatsMCPServer::getAllStats() {
    MCPToolResult result;
    result.success = true;

    auto cpuResult = getCPUStats();
    auto memResult = getMemoryStats();
    auto diskResult = getDiskStats();

    result.data["cpu"] = cpuResult.data;
    result.data["memory"] = memResult.data;
    result.data["disk"] = diskResult.data;

    if (includeTemperatures_) {
        auto tempResult = getTemperatures();
        result.data["temperatures"] = tempResult.data;
    }

    result.message = "Retrieved all system stats";

    return result;
}

QList<MCPTool> SystemStatsMCPServer::availableTools() const {
    QList<MCPTool> tools;

    MCPTool getCPU;
    getCPU.name = "get_cpu_stats";
    getCPU.description = "Get CPU usage, load average, and per-core stats";
    getCPU.inputSchema = QJsonObject{{"type", "object"}};
    tools.append(getCPU);

    MCPTool getMemory;
    getMemory.name = "get_memory_stats";
    getMemory.description = "Get RAM and swap usage";
    getMemory.inputSchema = QJsonObject{{"type", "object"}};
    tools.append(getMemory);

    MCPTool getDisk;
    getDisk.name = "get_disk_stats";
    getDisk.description = "Get disk usage for all mounted filesystems";
    getDisk.inputSchema = QJsonObject{{"type", "object"}};
    tools.append(getDisk);

    MCPTool getTemps;
    getTemps.name = "get_temperatures";
    getTemps.description = "Get CPU and system temperatures";
    getTemps.inputSchema = QJsonObject{{"type", "object"}};
    tools.append(getTemps);

    MCPTool getAll;
    getAll.name = "get_all_stats";
    getAll.description = "Get all system statistics (CPU, memory, disk, temperatures)";
    getAll.inputSchema = QJsonObject{{"type", "object"}};
    tools.append(getAll);

    return tools;
}

QList<MCPResource> SystemStatsMCPServer::availableResources() const {
    QList<MCPResource> resources;

    MCPResource allStats;
    allStats.uri = "system/stats";
    allStats.name = "System Stats";
    allStats.description = "Current system resource statistics";
    allStats.mimeType = "application/json";
    resources.append(allStats);

    return resources;
}

MCPToolResult SystemStatsMCPServer::callTool(const QString& toolName, const QJsonObject& params) {
    Q_UNUSED(params);
    MCPToolResult result;
    recordToolCall();
    QElapsedTimer timer;
    timer.start();

    if (toolName == "get_cpu_stats") {
        result = getCPUStats();
    } else if (toolName == "get_memory_stats") {
        result = getMemoryStats();
    } else if (toolName == "get_disk_stats") {
        result = getDiskStats();
    } else if (toolName == "get_temperatures") {
        result = getTemperatures();
    } else if (toolName == "get_all_stats") {
        result = getAllStats();
    } else {
        result.success = false;
        result.message = QString("Unknown tool: %1").arg(toolName);
    }

    result.executionTimeMs = timer.elapsed();
    recordBytesSent(QJsonDocument(result.data).toJson().size());
    return result;
}

MCPResourceContent SystemStatsMCPServer::readResource(const QString& uri) {
    MCPResourceContent content;
    content.uri = uri;
    content.mimeType = "application/json";
    recordResourceRead();

    if (uri == "system/stats") {
        auto result = getAllStats();
        content.textContent = QJsonDocument(result.data).toJson(QJsonDocument::Indented);
    } else {
        content.textContent = QString("Unknown resource: %1").arg(uri);
    }

    recordBytesSent(content.textContent.size());
    return content;
}

// ============================================================================
// FilesMCPServer
// ============================================================================

FilesMCPServer::FilesMCPServer(const MCPServerConfig& config, QObject* parent)
    : MCPServer(config, parent)
{
    // Parse allowed paths from settings
    QJsonArray paths = config.settings.value("allowedPaths").toArray();
    for (const auto& p : paths) {
        allowedPaths_.append(p.toString());
    }
    if (allowedPaths_.isEmpty()) {
        allowedPaths_.append(QDir::homePath());
    }

    maxFileSize_ = static_cast<qint64>(config.settings.value("maxFileSize").toDouble(10485760));
}

FilesMCPServer::~FilesMCPServer() {
    onStop();
}

void FilesMCPServer::onStart() {
    // No initialization needed
}

void FilesMCPServer::onStop() {
    // No cleanup needed
}

bool FilesMCPServer::isPathAllowed(const QString& path) const {
    QString normalizedPath = normalizePath(path);
    for (const QString& allowed : allowedPaths_) {
        QString normalizedAllowed = normalizePath(allowed);
        if (normalizedPath.startsWith(normalizedAllowed)) {
            return true;
        }
    }
    return false;
}

QString FilesMCPServer::normalizePath(const QString& path) const {
    QFileInfo info(path);
    return info.absoluteFilePath();
}

MCPToolResult FilesMCPServer::listDirectory(const QString& path, bool showHidden) {
    MCPToolResult result;

    QString dirPath = path.isEmpty() ? QDir::currentPath() : path;

    if (!isPathAllowed(dirPath)) {
        result.success = false;
        result.message = QString("Access denied: %1 is not in allowed paths").arg(dirPath);
        return result;
    }

    QDir dir(dirPath);
    if (!dir.exists()) {
        result.success = false;
        result.message = QString("Directory does not exist: %1").arg(dirPath);
        return result;
    }

    QDir::Filters filters = QDir::AllEntries | QDir::NoDotAndDotDot;
    if (showHidden) {
        filters |= QDir::Hidden;
    }

    QFileInfoList entries = dir.entryInfoList(filters, QDir::DirsFirst | QDir::Name);

    QJsonArray entriesArray;
    for (const QFileInfo& info : entries) {
        QJsonObject entry;
        entry["name"] = info.fileName();
        entry["path"] = info.absoluteFilePath();
        entry["isDir"] = info.isDir();
        entry["isFile"] = info.isFile();
        entry["isSymlink"] = info.isSymLink();
        entry["size"] = info.size();
        entry["permissions"] = static_cast<int>(info.permissions());
        entry["lastModified"] = info.lastModified().toString(Qt::ISODate);

        if (info.isDir()) {
            entry["type"] = "directory";
        } else if (info.isSymLink()) {
            entry["type"] = "symlink";
            entry["symlinkTarget"] = info.symLinkTarget();
        } else {
            entry["type"] = "file";
            entry["suffix"] = info.suffix();
        }

        entriesArray.append(entry);
    }

    result.success = true;
    result.data["path"] = dirPath;
    result.data["entries"] = entriesArray;
    result.data["count"] = entriesArray.size();
    result.message = QString("Listed %1 entries in %2").arg(entriesArray.size()).arg(dirPath);

    return result;
}

MCPToolResult FilesMCPServer::readFile(const QString& path, int maxBytes) {
    MCPToolResult result;

    if (!isPathAllowed(path)) {
        result.success = false;
        result.message = QString("Access denied: %1 is not in allowed paths").arg(path);
        return result;
    }

    QFile file(path);
    QFileInfo info(path);

    if (!file.exists()) {
        result.success = false;
        result.message = QString("File does not exist: %1").arg(path);
        return result;
    }

    if (info.size() > maxFileSize_) {
        result.success = false;
        result.message = QString("File too large: %1 bytes (max %2 bytes)")
            .arg(info.size()).arg(maxFileSize_);
        return result;
    }

    if (!file.open(QIODevice::ReadOnly)) {
        result.success = false;
        result.message = QString("Cannot open file: %1").arg(file.errorString());
        return result;
    }

    qint64 bytesToRead = maxBytes > 0 ? qMin((qint64)maxBytes, info.size()) : info.size();
    QByteArray data = file.read(bytesToRead);
    file.close();

    // Try to determine if binary or text
    bool isBinary = false;
    for (int i = 0; i < qMin(1000, data.size()); ++i) {
        char c = data[i];
        if (c == 0 || (c < 32 && c != '\n' && c != '\r' && c != '\t')) {
            isBinary = true;
            break;
        }
    }

    result.success = true;
    result.data["path"] = path;
    result.data["size"] = info.size();
    result.data["bytesRead"] = data.size();
    result.data["isBinary"] = isBinary;

    if (isBinary) {
        result.data["contentBase64"] = QString::fromLatin1(data.toBase64());
        result.binaryData = data;
        result.mimeType = "application/octet-stream";
    } else {
        result.data["content"] = QString::fromUtf8(data);
    }

    result.message = QString("Read %1 bytes from %2").arg(data.size()).arg(path);
    recordBytesReceived(data.size());

    return result;
}

MCPToolResult FilesMCPServer::fileInfo(const QString& path) {
    MCPToolResult result;

    if (!isPathAllowed(path)) {
        result.success = false;
        result.message = QString("Access denied: %1 is not in allowed paths").arg(path);
        return result;
    }

    QFileInfo info(path);

    if (!info.exists()) {
        result.success = false;
        result.message = QString("Path does not exist: %1").arg(path);
        return result;
    }

    result.success = true;
    result.data["path"] = info.absoluteFilePath();
    result.data["name"] = info.fileName();
    result.data["exists"] = info.exists();
    result.data["isFile"] = info.isFile();
    result.data["isDir"] = info.isDir();
    result.data["isSymlink"] = info.isSymLink();
    result.data["isReadable"] = info.isReadable();
    result.data["isWritable"] = info.isWritable();
    result.data["isExecutable"] = info.isExecutable();
    result.data["size"] = info.size();
    result.data["created"] = info.birthTime().toString(Qt::ISODate);
    result.data["lastModified"] = info.lastModified().toString(Qt::ISODate);
    result.data["lastRead"] = info.lastRead().toString(Qt::ISODate);
    result.data["owner"] = info.owner();
    result.data["group"] = info.group();

    if (info.isSymLink()) {
        result.data["symlinkTarget"] = info.symLinkTarget();
    }

    result.message = QString("Retrieved info for %1").arg(path);

    return result;
}

MCPToolResult FilesMCPServer::searchFiles(const QString& directory, const QString& pattern) {
    MCPToolResult result;

    QString dir = directory.isEmpty() ? QDir::currentPath() : directory;

    if (!isPathAllowed(dir)) {
        result.success = false;
        result.message = QString("Access denied: %1 is not in allowed paths").arg(dir);
        return result;
    }

    QDir qdir(dir);
    if (!qdir.exists()) {
        result.success = false;
        result.message = QString("Directory does not exist: %1").arg(dir);
        return result;
    }

    QJsonArray matchesArray;
    QDirIterator it(dir, QStringList(pattern),
                   QDir::Files | QDir::NoSymLinks,
                   QDirIterator::Subdirectories);

    int count = 0;
    int maxResults = 100;  // Limit results

    while (it.hasNext() && count < maxResults) {
        QString filePath = it.next();
        QFileInfo info(filePath);

        QJsonObject fileObj;
        fileObj["path"] = info.absoluteFilePath();
        fileObj["name"] = info.fileName();
        fileObj["size"] = info.size();
        fileObj["lastModified"] = info.lastModified().toString(Qt::ISODate);
        matchesArray.append(fileObj);
        count++;
    }

    result.success = true;
    result.data["directory"] = dir;
    result.data["pattern"] = pattern;
    result.data["matches"] = matchesArray;
    result.data["count"] = matchesArray.size();
    result.data["truncated"] = count >= maxResults;
    result.message = QString("Found %1 files matching '%2'").arg(count).arg(pattern);

    return result;
}

QList<MCPTool> FilesMCPServer::availableTools() const {
    QList<MCPTool> tools;

    MCPTool listDir;
    listDir.name = "list_directory";
    listDir.description = "List contents of a directory";
    listDir.inputSchema = QJsonObject{
        {"type", "object"},
        {"properties", QJsonObject{
            {"path", QJsonObject{{"type", "string"}, {"description", "Directory path (default: current directory)"}}},
            {"showHidden", QJsonObject{{"type", "boolean"}, {"description", "Include hidden files"}}}
        }}
    };
    tools.append(listDir);

    MCPTool readFile;
    readFile.name = "read_file";
    readFile.description = "Read contents of a file";
    readFile.inputSchema = QJsonObject{
        {"type", "object"},
        {"properties", QJsonObject{
            {"path", QJsonObject{{"type", "string"}, {"description", "File path"}}},
            {"maxBytes", QJsonObject{{"type", "integer"}, {"description", "Maximum bytes to read"}}}
        }},
        {"required", QJsonArray{"path"}}
    };
    tools.append(readFile);

    MCPTool fileInfo;
    fileInfo.name = "file_info";
    fileInfo.description = "Get detailed information about a file or directory";
    fileInfo.inputSchema = QJsonObject{
        {"type", "object"},
        {"properties", QJsonObject{
            {"path", QJsonObject{{"type", "string"}, {"description", "File or directory path"}}}
        }},
        {"required", QJsonArray{"path"}}
    };
    tools.append(fileInfo);

    MCPTool search;
    search.name = "search_files";
    search.description = "Search for files matching a pattern";
    search.inputSchema = QJsonObject{
        {"type", "object"},
        {"properties", QJsonObject{
            {"directory", QJsonObject{{"type", "string"}, {"description", "Directory to search in"}}},
            {"pattern", QJsonObject{{"type", "string"}, {"description", "Filename pattern (e.g., *.txt, *log*)"}}}
        }},
        {"required", QJsonArray{"pattern"}}
    };
    tools.append(search);

    return tools;
}

QList<MCPResource> FilesMCPServer::availableResources() const {
    QList<MCPResource> resources;

    MCPResource cwd;
    cwd.uri = "files/cwd";
    cwd.name = "Current Directory";
    cwd.description = "List of files in current working directory";
    cwd.mimeType = "application/json";
    resources.append(cwd);

    return resources;
}

MCPToolResult FilesMCPServer::callTool(const QString& toolName, const QJsonObject& params) {
    MCPToolResult result;
    recordToolCall();
    QElapsedTimer timer;
    timer.start();

    if (toolName == "list_directory") {
        result = listDirectory(params.value("path").toString(),
                              params.value("showHidden").toBool(false));
    } else if (toolName == "read_file") {
        result = readFile(params.value("path").toString(),
                         params.value("maxBytes").toInt(-1));
    } else if (toolName == "file_info") {
        result = fileInfo(params.value("path").toString());
    } else if (toolName == "search_files") {
        result = searchFiles(params.value("directory").toString(),
                            params.value("pattern").toString());
    } else {
        result.success = false;
        result.message = QString("Unknown tool: %1").arg(toolName);
    }

    result.executionTimeMs = timer.elapsed();
    recordBytesSent(QJsonDocument(result.data).toJson().size());
    return result;
}

MCPResourceContent FilesMCPServer::readResource(const QString& uri) {
    MCPResourceContent content;
    content.uri = uri;
    content.mimeType = "application/json";
    recordResourceRead();

    if (uri == "files/cwd") {
        auto result = listDirectory(QDir::currentPath(), false);
        content.textContent = QJsonDocument(result.data).toJson(QJsonDocument::Indented);
    } else {
        content.textContent = QString("Unknown resource: %1").arg(uri);
    }

    recordBytesSent(content.textContent.size());
    return content;
}

}  // namespace ros_weaver
