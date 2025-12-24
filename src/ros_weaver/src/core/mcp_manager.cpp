// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, ROS Weaver Contributors

#include "ros_weaver/core/mcp_manager.hpp"
#include "ros_weaver/core/mcp_providers.hpp"
#include <QSettings>
#include <QJsonDocument>
#include <QJsonArray>
#include <iostream>

namespace ros_weaver {

MCPManager& MCPManager::instance() {
    static MCPManager instance;
    return instance;
}

MCPManager::MCPManager()
    : QObject(nullptr)
    , statsUpdateTimer_(new QTimer(this))
{
    connect(statsUpdateTimer_, &QTimer::timeout, this, &MCPManager::allStatsUpdated);
    statsUpdateTimer_->setInterval(1000);
    statsUpdateTimer_->start();
}

MCPManager::~MCPManager() {
    std::cerr << "MCPManager destructor: starting (singleton)" << std::endl;
    stopAll();
    std::cerr << "MCPManager destructor: complete" << std::endl;
}

void MCPManager::registerServer(std::shared_ptr<MCPServer> server) {
    if (!server) return;

    servers_[server->id()] = server;
    connectServerSignals(server);
    emit serverRegistered(server->id());

    // Auto-start if configured
    if (server->config().autoStart && server->config().enabled) {
        server->start();
    }
}

void MCPManager::unregisterServer(const QUuid& serverId) {
    auto it = servers_.find(serverId);
    if (it != servers_.end()) {
        (*it)->stop();
        servers_.erase(it);
        emit serverUnregistered(serverId);
    }
}

std::shared_ptr<MCPServer> MCPManager::server(const QUuid& id) const {
    auto it = servers_.find(id);
    return it != servers_.end() ? *it : nullptr;
}

std::shared_ptr<MCPServer> MCPManager::serverByName(const QString& name) const {
    for (const auto& server : servers_) {
        if (server->name() == name) {
            return server;
        }
    }
    return nullptr;
}

std::shared_ptr<MCPServer> MCPManager::serverByType(const QString& type) const {
    for (const auto& server : servers_) {
        if (server->serverType() == type) {
            return server;
        }
    }
    return nullptr;
}

QList<std::shared_ptr<MCPServer>> MCPManager::allServers() const {
    return servers_.values();
}

QList<std::shared_ptr<MCPServer>> MCPManager::activeServers() const {
    QList<std::shared_ptr<MCPServer>> active;
    for (const auto& server : servers_) {
        if (server->isConnected()) {
            active.append(server);
        }
    }
    return active;
}

QStringList MCPManager::availableServerTypes() const {
    return {
        "ros_logs",
        "ros_topics",
        "rosbag",
        "ros_control",
        "system_stats",
        "files"
    };
}

QString MCPManager::serverTypeDescription(const QString& type) const {
    static const QMap<QString, QString> descriptions = {
        {"ros_logs", "ROS 2 log messages via rosout subscription"},
        {"ros_topics", "ROS 2 topic listing, info, and echo capabilities"},
        {"rosbag", "ROSbag recording status and metadata"},
        {"ros_control", "ros2_control lifecycle state and management"},
        {"system_stats", "System resource monitoring (CPU, RAM, disk, temperatures)"},
        {"files", "File system listing and basic file operations"}
    };
    return descriptions.value(type, "Unknown server type");
}

MCPServerConfig MCPManager::defaultConfigForType(const QString& type) const {
    MCPServerConfig config;
    config.id = QUuid::createUuid();
    config.serverType = type;
    config.enabled = true;
    config.autoStart = true;

    if (type == "ros_logs") {
        config.name = "ROS Logs";
        config.description = "Access ROS 2 log messages";
        config.settings["maxLines"] = 1000;
        config.settings["severityFilter"] = "debug";
    } else if (type == "ros_topics") {
        config.name = "ROS Topics";
        config.description = "Access ROS 2 topics and messages";
        config.settings["maxMessageHistory"] = 100;
    } else if (type == "rosbag") {
        config.name = "ROSbag";
        config.description = "ROSbag recording and status";
        config.settings["defaultPath"] = "";
    } else if (type == "ros_control") {
        config.name = "ROS Control";
        config.description = "ros2_control lifecycle management";
        config.settings["controllerManager"] = "/controller_manager";
    } else if (type == "system_stats") {
        config.name = "System Stats";
        config.description = "System resource monitoring";
        config.settings["updateIntervalMs"] = 1000;
        config.settings["includeTemperatures"] = true;
    } else if (type == "files") {
        config.name = "Files";
        config.description = "File system access";
        config.settings["allowedPaths"] = QJsonArray({QDir::homePath()});
        config.settings["maxFileSize"] = 10485760;  // 10MB
    }

    return config;
}

void MCPManager::startAll() {
    for (auto& server : servers_) {
        if (server->config().enabled) {
            server->start();
        }
    }
}

void MCPManager::stopAll() {
    std::cerr << "MCPManager::stopAll() - stopping " << servers_.size() << " servers" << std::endl;
    for (auto& server : servers_) {
        std::cerr << "MCPManager: stopping server " << server->name().toStdString() << std::endl;
        server->stop();
        std::cerr << "MCPManager: server " << server->name().toStdString() << " stopped" << std::endl;
    }
    std::cerr << "MCPManager::stopAll() complete" << std::endl;
}

void MCPManager::startServer(const QUuid& id) {
    if (auto srv = server(id)) {
        srv->start();
    }
}

void MCPManager::stopServer(const QUuid& id) {
    if (auto srv = server(id)) {
        srv->stop();
    }
}

QList<MCPTool> MCPManager::allTools() const {
    QList<MCPTool> tools;
    for (const auto& server : servers_) {
        if (server->isConnected()) {
            // Prefix tools with server name for disambiguation
            for (MCPTool tool : server->availableTools()) {
                tool.name = server->name() + "/" + tool.name;
                tools.append(tool);
            }
        }
    }
    return tools;
}

QList<MCPResource> MCPManager::allResources() const {
    QList<MCPResource> resources;
    for (const auto& server : servers_) {
        if (server->isConnected()) {
            // Prefix URIs with server name
            for (MCPResource res : server->availableResources()) {
                res.uri = server->name() + "://" + res.uri;
                resources.append(res);
            }
        }
    }
    return resources;
}

MCPToolResult MCPManager::callTool(const QString& serverName, const QString& toolName,
                                   const QJsonObject& params) {
    auto srv = serverByName(serverName);
    if (!srv) {
        MCPToolResult result;
        result.success = false;
        result.message = QString("Server '%1' not found").arg(serverName);
        return result;
    }

    if (!srv->isConnected()) {
        MCPToolResult result;
        result.success = false;
        result.message = QString("Server '%1' is not connected").arg(serverName);
        return result;
    }

    // Try to find matching tool name with fuzzy matching
    QString actualToolName = toolName;
    QList<MCPTool> tools = srv->availableTools();

    // Helper to normalize tool names
    auto normalize = [](const QString& name) -> QString {
        return name.toLower().remove('_').remove('-');
    };

    // Check if exact match exists
    bool exactMatch = false;
    for (const auto& tool : tools) {
        if (tool.name == toolName) {
            exactMatch = true;
            break;
        }
    }

    // If no exact match, try fuzzy matching
    if (!exactMatch) {
        QString normalizedInput = normalize(toolName);
        for (const auto& tool : tools) {
            if (normalize(tool.name) == normalizedInput) {
                actualToolName = tool.name;
                break;
            }
        }

        // Additional heuristics: map common variations
        if (actualToolName == toolName) {  // Still no match
            // Map "getsystemstats" -> "get_all_stats"
            if (normalizedInput.contains("systemstats") || normalizedInput.contains("allstats")) {
                for (const auto& tool : tools) {
                    if (tool.name.contains("all_stats") || tool.name.contains("system")) {
                        actualToolName = tool.name;
                        break;
                    }
                }
            }
            // Map "getmemory" -> "get_memory_stats"
            if (normalizedInput.contains("memory") && !normalizedInput.contains("stats")) {
                for (const auto& tool : tools) {
                    if (tool.name.contains("memory")) {
                        actualToolName = tool.name;
                        break;
                    }
                }
            }
            // Map "getcpu" -> "get_cpu_stats"
            if (normalizedInput.contains("cpu") && !normalizedInput.contains("stats")) {
                for (const auto& tool : tools) {
                    if (tool.name.contains("cpu")) {
                        actualToolName = tool.name;
                        break;
                    }
                }
            }
        }
    }

    return srv->callTool(actualToolName, params);
}

MCPResourceContent MCPManager::readResource(const QString& serverName, const QString& uri) {
    auto srv = serverByName(serverName);
    if (!srv) {
        MCPResourceContent content;
        content.textContent = QString("Server '%1' not found").arg(serverName);
        return content;
    }

    return srv->readResource(uri);
}

MCPServerStats MCPManager::aggregateStats() const {
    MCPServerStats aggregate;
    for (const auto& server : servers_) {
        auto stats = server->stats();
        aggregate.bytesReceived += stats.bytesReceived;
        aggregate.bytesSent += stats.bytesSent;
        aggregate.toolCallCount += stats.toolCallCount;
        aggregate.resourceReadCount += stats.resourceReadCount;
        aggregate.errorCount += stats.errorCount;
        aggregate.cpuUsagePercent += stats.cpuUsagePercent;
        aggregate.memoryUsageBytes += stats.memoryUsageBytes;
    }
    return aggregate;
}

qint64 MCPManager::totalBytesTransferred() const {
    return aggregateStats().totalBytesTransferred();
}

qint64 MCPManager::totalToolCalls() const {
    return aggregateStats().toolCallCount;
}

void MCPManager::saveConfiguration() {
    QSettings settings;
    settings.beginGroup("MCP");

    QJsonArray serversArray;
    for (const auto& server : servers_) {
        serversArray.append(server->config().toJson());
    }

    settings.setValue("servers", QJsonDocument(serversArray).toJson());
    settings.endGroup();
}

void MCPManager::loadConfiguration() {
    QSettings settings;
    settings.beginGroup("MCP");

    QByteArray data = settings.value("servers").toByteArray();
    if (!data.isEmpty()) {
        QJsonDocument doc = QJsonDocument::fromJson(data);
        QJsonArray serversArray = doc.array();

        for (const auto& val : serversArray) {
            MCPServerConfig config = MCPServerConfig::fromJson(val.toObject());
            auto server = createServer(config);
            if (server) {
                registerServer(server);
            }
        }
    }

    settings.endGroup();

    // If no servers loaded, set up defaults
    if (servers_.isEmpty()) {
        setupBuiltInServers();
    }
}

void MCPManager::setCanvas(WeaverCanvas* canvas) {
    canvas_ = canvas;
}

MCPManager::MCPSuggestion MCPManager::suggestServer(const QString& userQuery) const {
    MCPSuggestion suggestion;
    QString queryLower = userQuery.toLower();

    // Simple keyword matching for suggestions
    if (queryLower.contains("log") || queryLower.contains("rosout") ||
        queryLower.contains("error") || queryLower.contains("warning")) {
        suggestion.serverType = "ros_logs";
        suggestion.reason = "Your query mentions logs - the ROS Logs MCP server can help monitor and filter ROS 2 log messages.";
        suggestion.suggestedConfig = defaultConfigForType("ros_logs");
    }
    else if (queryLower.contains("topic") || queryLower.contains("message") ||
             queryLower.contains("publish") || queryLower.contains("subscrib")) {
        suggestion.serverType = "ros_topics";
        suggestion.reason = "Your query is about topics - the ROS Topics MCP server can list, monitor, and echo topic data.";
        suggestion.suggestedConfig = defaultConfigForType("ros_topics");
    }
    else if (queryLower.contains("bag") || queryLower.contains("record") ||
             queryLower.contains("playback") || queryLower.contains("rosbag")) {
        suggestion.serverType = "rosbag";
        suggestion.reason = "Your query is about bag files - the ROSbag MCP server can manage recordings.";
        suggestion.suggestedConfig = defaultConfigForType("rosbag");
    }
    else if (queryLower.contains("controller") || queryLower.contains("lifecycle") ||
             queryLower.contains("hardware") || queryLower.contains("ros_control") ||
             queryLower.contains("ros2_control")) {
        suggestion.serverType = "ros_control";
        suggestion.reason = "Your query is about controllers - the ROS Control MCP server can monitor and manage ros2_control.";
        suggestion.suggestedConfig = defaultConfigForType("ros_control");
    }
    else if (queryLower.contains("cpu") || queryLower.contains("memory") ||
             queryLower.contains("ram") || queryLower.contains("disk") ||
             queryLower.contains("temperature") || queryLower.contains("system")) {
        suggestion.serverType = "system_stats";
        suggestion.reason = "Your query is about system resources - the System Stats MCP server can monitor CPU, memory, disk, and temperatures.";
        suggestion.suggestedConfig = defaultConfigForType("system_stats");
    }
    else if (queryLower.contains("file") || queryLower.contains("directory") ||
             queryLower.contains("folder") || queryLower.contains("path")) {
        suggestion.serverType = "files";
        suggestion.reason = "Your query is about files - the Files MCP server can list and read file contents.";
        suggestion.suggestedConfig = defaultConfigForType("files");
    }

    return suggestion;
}

void MCPManager::setupBuiltInServers() {
    // Create and register all built-in servers with default configs
    for (const QString& type : availableServerTypes()) {
        auto config = defaultConfigForType(type);
        auto server = createServer(config);
        if (server) {
            registerServer(server);
        }
    }
}

void MCPManager::connectServerSignals(std::shared_ptr<MCPServer> server) {
    QUuid serverId = server->id();

    connect(server.get(), &MCPServer::stateChanged,
            this, [this, serverId](MCPServerState state) {
                emit serverStateChanged(serverId, state);
            });

    connect(server.get(), &MCPServer::dataReceived,
            this, [this, serverId](qint64 bytes) {
                emit serverDataActivity(serverId, MCPDataDirection::Receiving, bytes);
            });

    connect(server.get(), &MCPServer::dataSent,
            this, [this, serverId](qint64 bytes) {
                emit serverDataActivity(serverId, MCPDataDirection::Sending, bytes);
            });
}

// Factory method - delegates to mcp_providers.cpp
std::shared_ptr<MCPServer> MCPManager::createServer(const MCPServerConfig& config) {
    return createMCPServer(config);
}

}  // namespace ros_weaver
